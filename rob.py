
import sys
import argparse
import random
import time
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

# TODO: use this instead of tuples as particles
class RobotParticle:
    def __init__(self, x, y, theta, out_left=0, out_right=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.out_left = out_left
        self.out_right = out_right


# TODO: might be of use
class Wall:
    pass
    
    
class Rob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, particle_count, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

        self.diameter = 1
        self.out_left = 0
        self.out_right = 0
        self.noise = 1   # TODO: use gaussian noise of 1.5% (?)
        self.sensor_angle = 60  # range that that defines each sensor's "opening", to form a cone shape

        # initialize particles
        # assuming that starting rotation is always 0
        self.particles = []
        particles_per_cell = particle_count // (CELLCOLS*CELLROWS)
        [
            # multiplying by 2 as each cell is twice the diameter of the robot, and summing diameter to get center of cell
            self.particles.extend([(x*2+self.diameter, y*2+self.diameter, 0)]*particles_per_cell)
            for x in range(CELLCOLS)
            for y in range(CELLROWS)
        ]
        # randomize remaining particles across cells
        leftover_particles = particle_count % (CELLCOLS*CELLROWS)
        [
            self.particles.append((
                random.randint(0, CELLCOLS-1)*2+self.diameter,
                random.randint(0, CELLROWS-1)*2+self.diameter,
                0
            ))
            for _ in range(leftover_particles)
        ]
        

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap


    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()


            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.cycle()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
    

    def cycle(self):
        ts_start = time.time()
        distances = { sid: 1/m for sid, m in enumerate(self.measures.irSensor) }
        self.updateParticlesWithMeasures(distances)
        print("time:", time.time() - ts_start)
        self.wander()
        print("Avg Distances:", distances)
        print("Most likely current location:", self.getMostLikelyPosition())


    def updateParticlesWithMeasures(self, distances):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        sensor_ids = [center_id, left_id, right_id, back_id]

        unique_particles = list(set(self.particles))
        weights = []

        for (x, y, theta) in unique_particles:
            theta_rad = theta*pi/180

            # weight is starts as a small value rather than 0, as some uncertainty is desired
            weight = 0.1

            # for each sensor
            for sid in sensor_ids:
                # convert to radians
                sensor_ang_rad = self.angs[sid]*pi/180
                sensor_range = (self.sensor_angle/2)*pi/180

                # location of sensor
                sensor_x = x + self.diameter/2*cos(theta_rad + sensor_ang_rad)
                sensor_y = y + self.diameter/2*sin(theta_rad + sensor_ang_rad)
                
                # TODO: find wall corners and wall intersections with cone of sight to get list of points to access
                # TODO: find smallest difference between measured value and distances to points
                # TODO: increase weight based on smallest difference found (the smaller the difference the better)
        
            weights.append(weight)
        
        # resample particles according to weights
        self.particles = random.choices(population=unique_particles, weights=weights, k=len(self.particles))

        with open("localization.out", "a") as fp:
            fp.write("Measurement\n")
            [ fp.write(f"{x} {y} {theta}\n") for x,y,theta in self.particles ]
        

    def driveAndUpdateParticles(self, in1, in2):
        self.driveMotors(in1, in2)

        self.out_left = (in1+self.out_left)/2*self.noise
        self.out_right = (in2+self.out_right)/2*self.noise
        lin = (self.out_left+self.out_right)/2

        def getNewPose(px, py, ptheta):
            x = px + lin*cos(ptheta*pi/180)
            y = py + lin*sin(ptheta*pi/180)

            # perform calculations in radians and convert back to degrees
            # unsure if i could just do the math in degrees
            theta = (ptheta*pi/180 + (self.out_right - self.out_left)/self.diameter) / pi*180
            # keep orientation in [-180, 180] range
            if theta > 180:
                theta = theta % (-180)
            elif theta < -180:
                theta = theta % 180

            return (x, y, theta)
        
        new_particles = [ getNewPose(x,y,theta) for x,y,theta in self.particles ]
        self.particles = new_particles

        with open("localization.out", "a") as fp:
            fp.write("Motion\n")
            [ fp.write(f"{x} {y} {theta}\n") for x,y,theta in self.particles ]


    def getMostLikelyPosition(self):
        return max(set(self.particles), key = self.particles.count)


    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        if   self.measures.irSensor[center_id] > 5.0\
          or self.measures.irSensor[left_id]   > 5.0\
          or self.measures.irSensor[right_id]  > 5.0\
          or self.measures.irSensor[back_id]   > 5.0:
           print('Rotate left')
           self.driveAndUpdateParticles(-0.1, +0.1)
        elif self.measures.irSensor[left_id]> 2.7:
           print('Rotate slowly right')
           self.driveAndUpdateParticles(0.1, 0.0)
        elif self.measures.irSensor[right_id]> 2.7:
           print('Rotate slowly left')
           self.driveAndUpdateParticles(0.0, 0.1)
        else:
           print('Go')
           self.driveAndUpdateParticles(0.1, 0.1)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1



if __name__ == '__main__':

    argparser = argparse.ArgumentParser(add_help=False)
    argparser.add_argument("--robot_name", "-r", default="pClient1")
    argparser.add_argument("--host", "-h", default="localhost")
    argparser.add_argument("--map", "-m")
    argparser.add_argument("--number_of_particles", "-n", type=int, default=500)
    argparser.add_argument("--position", "-p", type=int, default=1)

    args = vars(argparser.parse_args())

    rob=Rob(
        args["robot_name"],
        args["position"],
        [0.0,60.0,-60.0,180.0],
        args["number_of_particles"],
        args["host"]
    )

    mapc = Map(args["map"])
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
