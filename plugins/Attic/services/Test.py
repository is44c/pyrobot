# A Simple Service

from pyro.robot.service import Service

def INIT(robot):
    return {"test": Service()}
