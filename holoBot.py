__author__ = "jacobvanthoog"

import wpilib
import seamonsters.fix2017
from seamonsters.wpilib_sim import simulate
from seamonsters.utilityBots.driveTest import DriveTest
from seamonsters.drive import DriveInterface
from seamonsters.drive import AccelerationFilterDrive
from seamonsters.drive import FieldOrientedDrive
from seamonsters.holonomicDrive import HolonomicDrive

from robotpy_ext.common_drivers.navx import AHRS
import math

class SwerveBot(DriveTest):
    
    def robotInit(self):
        DriveTest.robotInit(self, normalScale = .3, fastScale = .5,
                            slowScale = .05)
        
        fl = wpilib.CANTalon(2)
        fr = wpilib.CANTalon(1)
        bl = wpilib.CANTalon(0)
        br = wpilib.CANTalon(3)
        fl.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        fr.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        bl.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        br.setFeedbackDevice(wpilib.CANTalon.FeedbackDevice.QuadEncoder)
        
        # 4156 ticks per wheel rotation
        # encoder has 100 raw ticks -- with a QuadEncoder that makes 400 ticks
        # the motor gear has 18 teeth and the wheel has 187 teeth
        # 187 / 18 * 400 = 4155.5556 = ~4156
        drive = HolonomicDrive(fl, fr, bl, br, 4156)
        drive.invertDrive(True)
        # TODO: move magic number to constant
        drive.setWheelOffset(math.radians(22.5)) #angle of rollers
        
        filterDrive = AccelerationFilterDrive(drive)

        self.ahrs = AHRS.create_spi() # the NavX
        fieldDrive = FieldOrientedDrive(filterDrive, self.ahrs)
        
        DriveTest.initDrive(self, fieldDrive,
                            driveMode=DriveInterface.DriveMode.POSITION,
                            talons=[fl, fr, bl, br],
                            normalPID=(10.0, 0.0, 3.0, 0.0),
                            slowPID=(30.0, 0.0, 3.0, 0.0) )

    def teleopPeriodic(self):
        super().teleopPeriodic()
        print(self.ahrs.getYaw())
        
        
if __name__ == "__main__":
    wpilib.run(SwerveBot)
