import numpy as np
import time


def f(n, k):
    return np.math.factorial(n) / (np.math.factorial(k) * np.math.factorial(n - k))


def b(t, k, point):
    n = 9
    return point * f(n, k) * np.power(t, k) * np.power(1 - t, n - k)


# Gait planner to move all feet
class walkGait:
    def __init__(self):
        self.bodytoFeet = np.zeros([4, 3])
        self.phi = 0.0
        self.phiStance = 0.0
        self.lastTime = 0.0
        self.alpha = 0.0
        self.s = False

    def calculateStance(self, phi_st, V, angle):
        c = np.cos(np.deg2rad(angle))
        s = np.sin(np.deg2rad(angle))
        A = 0.00
        halfL = 0.05
        p_stance = halfL * (1 - 2 * phi_st)
        stanceX = c * p_stance * np.abs(V)
        stanceY = -s * p_stance * np.abs(V)
        stanceZ = -A * np.cos(np.pi / (2 * halfL) * p_stance)
        return stanceX, stanceY, stanceZ

    def calculateBezier_swing(self, phi_sw, V, angle):
        c = np.cos(np.deg2rad(angle))
        s = np.sin(np.deg2rad(angle))
        X = (
            np.abs(V)
            * c
            * np.array([-0.05, -0.06, -0.07, -0.07, 0.0, 0.0, 0.07, 0.07, 0.06, 0.05])
        )
        Y = (
            np.abs(V)
            * s
            * np.array([0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05])
        )
        Z = np.abs(V) * np.array(
            [0.0, 0.0, 0.05, 0.05, 0.05, 0.06, 0.06, 0.06, 0.0, 0.0]
        )
        swingX, swingY, swingZ = 0.0, 0.0, 0.0
        for i in range(10):  # sum all terms of the curve
            swingX += b(phi_sw, i, X[i])
            swingY += b(phi_sw, i, Y[i])
            swingZ += b(phi_sw, i, Z[i])
        return swingX, swingY, swingZ

    def stepTrajectory(self, phi, V, angle, Wrot, centerToFoot, stepOffset=0.75):
        if phi >= 1:
            phi -= 1.0
        r = np.sqrt(centerToFoot[0] ** 2 + centerToFoot[1] ** 2)
        footAngle = np.arctan2(centerToFoot[1], centerToFoot[0])

        if Wrot >= 0.0:
            circleTrayectory = 90.0 - np.rad2deg(footAngle - self.alpha)
        else:
            circleTrayectory = 270.0 - np.rad2deg(footAngle - self.alpha)

        if phi <= stepOffset:
            phiStance = phi / stepOffset
            stepX_long, stepY_long, stepZ_long = self.calculateStance(
                phiStance, V, angle
            )
            stepX_rot, stepY_rot, stepZ_rot = self.calculateStance(
                phiStance, Wrot, circleTrayectory
            )
        else:
            phiSwing = (phi - stepOffset) / (1 - stepOffset)
            stepX_long, stepY_long, stepZ_long = self.calculateBezier_swing(
                phiSwing, V, angle
            )
            stepX_rot, stepY_rot, stepZ_rot = self.calculateBezier_swing(
                phiSwing, Wrot, circleTrayectory
            )

        if centerToFoot[1] > 0:
            if stepX_rot < 0:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)
        else:
            if stepX_rot < 0:
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2), r)

        x = stepX_long + stepX_rot
        y = stepY_long + stepY_rot
        z = stepZ_long + stepZ_rot

        return x, y, z

    def simulateCycle(
        self,
        V,
        angle,
        Wrot,
        offset,
        bodytoFeet_,
        step_asym=0.0,
        duty_cycle=0.5,
        steps=100,
    ):

        adjusted_duty_cycle = duty_cycle * np.exp(step_asym)

        trajectories = {i: {"x": [], "y": [], "z": []} for i in range(4)}
        bodytoFeet = np.asarray(bodytoFeet_)
        for step in range(steps):
            self.phi = step / steps
            for i in range(4):
                step_coord = self.stepTrajectory(
                    self.phi + offset[i],
                    V,
                    angle,
                    Wrot,
                    bodytoFeet[i, :],
                    adjusted_duty_cycle,
                )
                self.bodytoFeet[i, 0] = bodytoFeet_[i, 0] + step_coord[0]
                self.bodytoFeet[i, 1] = bodytoFeet_[i, 1] + step_coord[1]
                self.bodytoFeet[i, 2] = bodytoFeet_[i, 2] - step_coord[2]
                trajectories[i]["x"].append(self.bodytoFeet[i, 0])
                trajectories[i]["y"].append(self.bodytoFeet[i, 1])
                trajectories[i]["z"].append(self.bodytoFeet[i, 2])
        return trajectories

    def loop(
        self, V, angle, Wrot, T, offset, bodytoFeet_, step_asym=0.0, duty_cycle=0.5
    ):

        if T <= 0.01:
            T = 0.01

        if self.phi >= 0.99:
            self.lastTime = time.time()
        self.phi = (time.time() - self.lastTime) / T
        #        print(self.phi)
        # now it calculates step trajectory for every foot
        step_coord = self.stepTrajectory(
            self.phi + offset[0],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodytoFeet_[0, :])),
            duty_cycle * np.exp(step_asym),
        )  # FR
        self.bodytoFeet[0, 0] = bodytoFeet_[0, 0] + step_coord[0]
        self.bodytoFeet[0, 1] = bodytoFeet_[0, 1] + step_coord[1]
        self.bodytoFeet[0, 2] = bodytoFeet_[0, 2] - step_coord[2]

        step_coord = self.stepTrajectory(
            self.phi + offset[1],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodytoFeet_[1, :])),
            duty_cycle * np.exp(step_asym),
        )  # FL
        self.bodytoFeet[1, 0] = bodytoFeet_[1, 0] + step_coord[0]
        self.bodytoFeet[1, 1] = bodytoFeet_[1, 1] + step_coord[1]
        self.bodytoFeet[1, 2] = bodytoFeet_[1, 2] - step_coord[2]

        step_coord = self.stepTrajectory(
            self.phi + offset[2],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodytoFeet_[2, :])),
            duty_cycle * np.exp(step_asym),
        )  # BR
        self.bodytoFeet[2, 0] = bodytoFeet_[2, 0] + step_coord[0]
        self.bodytoFeet[2, 1] = bodytoFeet_[2, 1] + step_coord[1]
        self.bodytoFeet[2, 2] = bodytoFeet_[2, 2] - step_coord[2]

        step_coord = self.stepTrajectory(
            self.phi + offset[3],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodytoFeet_[3, :])),
            duty_cycle * np.exp(step_asym),
        )  # BL
        self.bodytoFeet[3, 0] = bodytoFeet_[3, 0] + step_coord[0]
        self.bodytoFeet[3, 1] = bodytoFeet_[3, 1] + step_coord[1]
        self.bodytoFeet[3, 2] = bodytoFeet_[3, 2] - step_coord[2]
        #

        return self.bodytoFeet
