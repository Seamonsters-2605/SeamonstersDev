__author__ = "jacobvanthoog"

from seamonsters.wpilib_sim import simulate
from seamonsters.utilityBots.driveTest import DriveTest
from seamonsters.drive import DriveInterface
from seamonsters.drive import AccelerationFilterDrive
from seamonsters.swerveDrive import SwerveDrive
import wpilib
import ctre

class StingrayDrive(DriveTest):
    
    def robotInit(self):
        super().robotInit()
        
        # Front Left wheel
        flDrive = ctre.CANTalon(0)
        flDrive.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
        flDrive.setPID(1.0, 0.0, 3.0, 0.0)
        
        flRotate = ctre.CANTalon(1)
        flRotate.reverseOutput(True)
        flRotate.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
        flRotate.setPID(1.0, 0.0, 3.0, 0.0)
        
        
        # Back Right wheel
        brDrive = ctre.CANTalon(2)
        brDrive.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
        brDrive.setPID(1.0, 0.0, 3.0, 0.0)
        
        brRotate = ctre.CANTalon(3)
        brRotate.reverseOutput(True)
        brRotate.setFeedbackDevice(ctre.CANTalon.FeedbackDevice.QuadEncoder)
        brRotate.setPID(1.0, 0.0, 3.0, 0.0)
        
        # Drive controller
        drive = SwerveDrive()
        
        # 104 gear teeth / 18 gear teeth * 280 ticks per rotation * 4 (quad)
        # then divide by 2 for some reason
        drive.addWheel(-1.0, 1.0, flDrive, flRotate, -104/18*280*4/2)
        drive.addWheel(1.0, -1.0, brDrive, brRotate, -104/18*280*4/2)
        
        filterDrive = AccelerationFilterDrive(drive)
        
        DriveTest.initDrive(self, filterDrive,
            driveMode=DriveInterface.DriveMode.VOLTAGE)
        
        
if __name__ == "__main__":
    wpilib.run(StingrayDrive)
