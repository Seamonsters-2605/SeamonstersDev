__author__ = "jacobvanthoog"

import wpilib
import seamonsters.fix2017
from robotpy_ext.common_drivers.navx import AHRS
from seamonsters.logging import LogState

class Test(wpilib.IterativeRobot):

    def robotInit(self):
        self.ahrs = AHRS.create_spi() # the NavX
        #self.ahrs = AHRS.create_i2c() # other communication protocol option
        
        self.yawLog = LogState("Yaw")
        self.angleLog = LogState("Angle")
        
        #wpilib.SmartDashboard.putString("thisIsAKey", "this is a value!")
        
    def teleopPeriodic(self):
        # angles are floats, in degrees clockwise

        # yaw ranges from -180 to 180
        self.yawLog.update(self.ahrs.getYaw())

        # angle starts at 0 and accumulates rotations. After 2 full rotations
        # clockwise it will be 720
        self.angleLog.update(self.ahrs.getAngle())
        
if __name__ == "__main__":
    wpilib.run(Test)
