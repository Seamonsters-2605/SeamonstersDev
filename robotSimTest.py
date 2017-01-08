__author__ = "jacobvanthoog"

import wpilib
import seamonsters.fix2017

# Based off the example robot from
# github.com/robotpy/pyfrc/blob/master/samples/physics-mecanum/src/robot.py

class MyRobot(wpilib.IterativeRobot):
    
    def robotInit(self):
        self.lJoy = wpilib.Joystick(0)
        self.rJoy = wpilib.Joystick(1)
        
        self.FL = wpilib.CANTalon(2)
        self.FR = wpilib.CANTalon(1)
        self.BL = wpilib.CANTalon(0)
        self.BR = wpilib.CANTalon(3)
        
        # Position gets automatically updated as robot moves
        self.gyro = wpilib.AnalogGyro(1)
        
    def autonomousPeriodic(self):
        # spin all motors at half speed
        self.FL.set(0.5)
        self.FR.set(0.5)
        self.BL.set(0.5)
        self.BR.set(0.5)
        
    def teleopPeriodic(self):
        # tank drive
        lSpeed = self.lJoy.getY()
        rSpeed = self.rJoy.getY()
        self.FL.set(lSpeed)
        self.FR.set(rSpeed)
        self.BL.set(lSpeed)
        self.BR.set(rSpeed)

if __name__ == '__main__':
    wpilib.run(MyRobot, physics_enabled=True)

