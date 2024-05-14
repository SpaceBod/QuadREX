import numpy as np

def checkDomain(D):
    if D > 1 or D < -1:
        print("D-Clip!")
        D = np.clip(D, -0.99, 0.99)
    return D


def solveLeg(coord, coxa, femur, tibia, coord1Sign=1, coord2Sign=1, coxaSign=1, gammaSign=1, alphaAdjustment=np.pi/4.0, gammaAdjustment=-np.pi/2.0):
    D = (coord[1] ** 2 + (coord2Sign * coord[2]) ** 2 - coxa**2 + coord[0] ** 2 - femur**2 - tibia**2) / (2 * tibia * femur)
    D = checkDomain(D)
    gamma = np.arctan2(gammaSign * np.sqrt(1 - D**2), D)
    tetta = -np.arctan2(-coord[2], coord1Sign * coord[1]) - np.arctan2(np.sqrt(coord[1] ** 2 + (coord2Sign * coord[2]) ** 2 - coxa**2), coxaSign * coxa)
    alpha = np.arctan2(coord[0], np.sqrt(coord[1] ** 2 + (coord2Sign * coord[2]) ** 2 - coxa**2)) - np.arctan2(tibia * np.sin(gamma), femur + tibia * np.cos(gamma))
    angles = np.array([tetta, alpha + alphaAdjustment, gamma + gammaAdjustment])
    return angles

def solveFR(coord, coxa, femur, tibia):
    return solveLeg(coord, coxa, femur, tibia, coord1Sign=-1, coord2Sign=1, coxaSign=1, gammaSign=-1, alphaAdjustment=-np.pi/4.0, gammaAdjustment=np.pi/2.0)

def solveFL(coord, coxa, femur, tibia):
    return solveLeg(coord, coxa, femur, tibia, coord1Sign=1, coord2Sign=-1, coxaSign=1, gammaSign=1, alphaAdjustment=np.pi/4.0, gammaAdjustment=-np.pi/2.0)

def solveBR(coord, coxa, femur, tibia):
    return solveLeg(coord, coxa, femur, tibia, coord1Sign=1, coord2Sign=1, coxaSign=-1, gammaSign=-1, alphaAdjustment=-np.pi/4.0, gammaAdjustment=np.pi/2.0)

def solveBL(coord, coxa, femur, tibia):
    return solveLeg(coord, coxa, femur, tibia, coord1Sign=-1, coord2Sign=-1, coxaSign=-1, gammaSign=1, alphaAdjustment=np.pi/4.0, gammaAdjustment=-np.pi/2.0)
