
import sys
from croblink import *
from math import *

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    def run(self):
        if self.status != 0:
            print "Connection refused or error"
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print self.rob_name + " exiting"
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.groundReady and self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
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
            
            if self.measures.time % 2 == 0:
                self.requestSensors(['IRSensor0','IRSensor1','IRSensor2','Ground'])
            else:
                self.requestSensors(['IRSensor1','IRSensor2','Compass','GPS'])


    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if self.measures.irSensorReady[center_id] and self.measures.irSensor[center_id]> 5.0\
           or self.measures.irSensorReady[left_id] and self.measures.irSensor[left_id]> 5.0\
           or self.measures.irSensorReady[right_id] and self.measures.irSensor[right_id]> 5.0\
           or self.measures.irSensorReady[back_id] and self.measures.irSensor[back_id]> 5.0:
        #    print 'Rotate'
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensorReady[left_id] and self.measures.irSensor[left_id]> 0.7:
        #    print 'Rotate slowly'
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensorReady[right_id] and self.measures.irSensor[right_id]> 0.7:
            self.driveMotors(0.0,0.1)
        else:
         #    print 'Go'
            self.driveMotors(0.1,0.1)


rob_name = "pClient1"
host = "localhost"
pos = 1

for i in range(0, len(sys.argv)):
    if sys.argv[i] == "-host" and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    if sys.argv[i] == "-pos" and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    if sys.argv[i] == "-robname" and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]

rob=MyRob(rob_name,pos,[0.0,-60.0,60.0,180.0],host)
rob.run()
