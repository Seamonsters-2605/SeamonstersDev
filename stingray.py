__author__ = "jacobvanthoog"

import wpilib
from seamonsters.wpilib_sim import simulate
from seamonsters.modularRobot import Module
from stingray.drive import StingrayDrive
from stingray.shooter import StingrayShooter

class StingrayBot(Module):
    
    def __init__(self):
        super().__init__()
        self.addModule(StingrayDrive(initSuper=False))
        self.addModule(StingrayShooter(initSuper=False))
        
if __name__ == "__main__":
    wpilib.run(StingrayBot)
