import numpy as np


# IK equations written in pybullet frame.
def IK(coord, coxa, femur, tibia, sign):
    D = (
        coord[1] ** 2 + coord[2] ** 2 - coxa**2 + coord[0] ** 2 - femur**2 - tibia**2
    ) / (2 * tibia * femur)
    if D > 1 or D < -1:
        if D > 1:
            D = 0.99
        elif D < -1:
            D = -0.99
    gamma = sign * np.arctan2(np.sqrt(1 - D**2), D)
    tetta = -np.arctan2(-coord[2], coord[1]) - np.arctan2(
        np.sqrt(coord[1] ** 2 + coord[2] ** 2 - coxa**2), -coxa
    )
    alpha = np.arctan2(
        coord[0], np.sqrt(coord[1] ** 2 + coord[2] ** 2 - coxa**2)
    ) - np.arctan2(tibia * np.sin(gamma), femur + tibia * np.cos(gamma))
    angles = np.array([tetta, alpha, gamma])
    return angles


def legs_IK(FRcoord, FLcoord, BRcoord, BLcoord, shoulder, femur, tibia):
    signFront = 1.0
    signBack = 1.0

    FR_angles = IK(FRcoord, shoulder, femur, tibia, signFront)
    FL_angles = IK(FLcoord, -shoulder, femur, tibia, signFront)
    BR_angles = IK(BRcoord, shoulder, femur, tibia, signBack)
    BL_angles = IK(BLcoord, -shoulder, femur, tibia, signBack)

    angles = np.matrix(
        [
            [FR_angles[0], FR_angles[1] - np.pi / 4.0, FR_angles[2] - np.pi / 2.0],
            [FL_angles[0], FL_angles[1] - np.pi / 4.0, FL_angles[2] - np.pi / 2.0],
            [BR_angles[0], BR_angles[1] - np.pi / 4.0, BR_angles[2] + np.pi / 2.0],
            [BL_angles[0], BL_angles[1] - np.pi / 4.0, BL_angles[2] + np.pi / 2.0],
        ]
    )
    angles = np.matrix(
        [
            [FR_angles[0] - np.pi / 2, 0, 0],
            [FL_angles[0] + np.pi / 2, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
    )
    print(FL_angles[0] + np.pi)
    return angles


def Rx(roll):
    """Rotation matrix arround x (roll)"""
    #    roll = np.radians(roll)
    return np.matrix(
        [
            [1, 0, 0, 0],
            [0, np.cos(roll), -np.sin(roll), 0],
            [0, np.sin(roll), np.cos(roll), 0],
            [0, 0, 0, 1],
        ]
    )


def Ry(pitch):
    """Rotation matrix arround y (pitch)"""
    #    pitch = np.radians(pitch)
    return np.matrix(
        [
            [np.cos(pitch), 0, np.sin(pitch), 0],
            [0, 1, 0, 0],
            [-np.sin(pitch), 0, np.cos(pitch), 0],
            [0, 0, 0, 1],
        ]
    )


def Rz(yaw):
    """Rotation matrix arround z (yaw)"""
    #    yaw = np.radians(yaw)
    return np.matrix(
        [
            [np.cos(yaw), -np.sin(yaw), 0, 0],
            [np.sin(yaw), np.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def Rxyz(roll, pitch, yaw):
    if roll != 0.0 or pitch != 0.0 or yaw != 0.0:
        R = Rx(roll) * Ry(pitch) * Rz(yaw)
        return R
    else:
        return np.identity(4)


def RTmatrix(orientation, position):
    """compose translation and rotation"""
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]
    x0 = position[0]
    y0 = position[1]
    z0 = position[2]

    translation = np.matrix(
        [
            [1, 0, 0, x0],  # Translation matrix
            [0, 1, 0, y0],
            [0, 0, 1, z0],
            [0, 0, 0, 1],
        ]
    )
    rotation = Rxyz(roll, pitch, yaw)  # rotation matrix

    return rotation * translation


def transform(coord, rotation, translation):
    """transforms a vector to a desire rotation and translation"""
    vector = np.array([[coord[0]], [coord[1]], [coord[2]], [1]])

    tranformVector = RTmatrix(rotation, translation) * vector
    return np.array([tranformVector[0, 0], tranformVector[1, 0], tranformVector[2, 0]])


class robotKinematics:
    def __init__(self):
        """in meter"""
        self.L = 0.18  # length of robot joints
        self.W = 0.064  # width of robot joints
        self.coxa = 0.0375  # coxa length
        self.femur = 0.08  # femur length
        self.tibia = 0.08  # tibia length
        """initial foot position"""
        # foot separation (0.182 -> tetta=0) and distance to floor
        self.Ydist = 11.0
        self.Xdist = self.L
        self.height = 15.0
        # body frame to coxa frame vector
        self.bodytoFR0 = np.array([self.L / 2, -self.W / 2, 0])
        self.bodytoFL0 = np.array([self.L / 2, self.W / 2, 0])
        self.bodytoBR0 = np.array([-self.L / 2, -self.W / 2, 0])
        self.bodytoBL0 = np.array([-self.L / 2, self.W / 2, 0])
        # body frame to foot frame vector
        self.bodytoFR4 = np.array([self.Xdist / 2, -self.Ydist / 2, -self.height])
        self.bodytoFL4 = np.array([self.Xdist / 2, self.Ydist / 2, -self.height])
        self.bodytoBR4 = np.array([-self.Xdist / 2, -self.Ydist / 2, -self.height])
        self.bodytoBL4 = np.array([-self.Xdist / 2, self.Ydist / 2, -self.height])

    def solve(self, orn, pos, bodytoFeet):
        bodytoFR4 = np.asarray([bodytoFeet[0, 0], bodytoFeet[0, 1], bodytoFeet[0, 2]])
        bodytoFL4 = np.asarray([bodytoFeet[1, 0], bodytoFeet[1, 1], bodytoFeet[1, 2]])
        bodytoBR4 = np.asarray([bodytoFeet[2, 0], bodytoFeet[2, 1], bodytoFeet[2, 2]])
        bodytoBL4 = np.asarray([bodytoFeet[3, 0], bodytoFeet[3, 1], bodytoFeet[3, 2]])

        """defines 4 vertices which rotates with the body"""
        _bodytoFR0 = transform(self.bodytoFR0, orn, pos)
        _bodytoFL0 = transform(self.bodytoFL0, orn, pos)
        _bodytoBR0 = transform(self.bodytoBR0, orn, pos)
        _bodytoBL0 = transform(self.bodytoBL0, orn, pos)
        """defines coxa_frame to foot_frame leg vector neccesary for IK"""
        FRcoord = bodytoFR4 - _bodytoFR0
        FLcoord = bodytoFL4 - _bodytoFL0
        BRcoord = bodytoBR4 - _bodytoBR0
        BLcoord = bodytoBL4 - _bodytoBL0
        """undo transformation of leg vector to keep feet still"""
        undoOrn = -np.array(orn)
        undoPos = -np.array(pos)
        _FRcoord = transform(FRcoord, undoOrn, undoPos)
        _FLcoord = transform(FLcoord, undoOrn, undoPos)
        _BRcoord = transform(BRcoord, undoOrn, undoPos)
        _BLcoord = transform(BLcoord, undoOrn, undoPos)

        angles = legs_IK(
            _FRcoord,
            _FLcoord,
            _BRcoord,
            _BLcoord,
            self.coxa,
            self.femur,
            self.tibia,
        )

        _bodytofeetFR = _bodytoFR0 + _FRcoord
        _bodytofeetFL = _bodytoFL0 + _FLcoord
        _bodytofeetBR = _bodytoBR0 + _BRcoord
        _bodytofeetBL = _bodytoBL0 + _BLcoord
        _bodytofeet = np.matrix(
            [
                [_bodytofeetFR[0], _bodytofeetFR[1], _bodytofeetFR[2]],
                [_bodytofeetFL[0], _bodytofeetFL[1], _bodytofeetFL[2]],
                [_bodytofeetBR[0], _bodytofeetBR[1], _bodytofeetBR[2]],
                [_bodytofeetBL[0], _bodytofeetBL[1], _bodytofeetBL[2]],
            ]
        )

        return angles, _bodytofeet
