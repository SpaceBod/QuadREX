import time
import numpy as np

class TrotGait:
    """
    Gait planner for moving all feet.
    """
    def __init__(self):
        self.bodyToFeet = np.zeros([4, 3])
        self.phi = 0.0
        self.phiStance = 0.0
        self.lastTime = 0.0
        self.alpha = 0.0
        self.s = False

    def bezierCurve(self, t, k, point):
        """
        Computes the Bezier curve.

        Parameters:
        t (float): Time.
        k (int): Control point index.
        point (float): Control point value.

        Returns:
        float: Bezier curve value.
        """
        n = 9
        return point * self.binomialCoeff(n, k) * np.power(t, k) * np.power(1 - t, n - k)

    def binomialCoeff(self, n, k):
        """
        Calculates binomial factor (n k).

        Parameters:
        n (int): Total.
        k (int): Subset.

        Returns:
        float: Binomial factor.
        """
        return np.math.factorial(n) / (np.math.factorial(k) * np.math.factorial(n - k))
    
    def calculate_stance(self, phiSt, V, angle):
        """
        Calculates the stance phase for a foot.

        Parameters:
        phiSt (float): Phase between [0, 1).
        V (float): Velocity.
        angle (float): Angle in degrees.

        Returns:
        tuple: X, Y, Z coordinates of the stance phase.
        """
        c = np.cos(np.deg2rad(angle))
        s = np.sin(np.deg2rad(angle))
        A = 0.002
        halfL = 0.05
        pStance = halfL * (1 - 2 * phiSt)
        stanceX = c * pStance * np.abs(V)
        stanceY = -s * pStance * np.abs(V)
        stanceZ = -A * np.cos(np.pi / (2 * halfL) * pStance)
        return stanceX, stanceY, stanceZ

    def calculate_bezier_swing(self, phiSw, V, angle):
        """
        Calculates the swing phase using Bezier curve.

        Parameters:
        phiSw (float): Phase between [0, 1).
        V (float): Velocity.
        angle (float): Angle in degrees.

        Returns:
        tuple: X, Y, Z coordinates of the swing phase.
        """
        c = np.cos(np.deg2rad(angle))
        s = np.sin(np.deg2rad(angle))
        X = np.abs(V) * c * np.array([-0.05, -0.06, -0.07, -0.07, 0.0, 0.0, 0.07, 0.07, 0.06, 0.05])
        Y = np.abs(V) * s * np.array([0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05])
        Z = np.abs(V) * np.array([0.0, 0.0, 0.05, 0.05, 0.05, 0.06, 0.06, 0.06, 0.0, 0.0])
        swingX = 0.0
        swingY = 0.0
        swingZ = 0.0
        for i in range(10):
            swingX += self.bezierCurve(phiSw, i, X[i])
            swingY += self.bezierCurve(phiSw, i, Y[i])
            swingZ += self.bezierCurve(phiSw, i, Z[i])
        return swingX, swingY, swingZ

    def step_trajectory(self, phi, V, angle, Wrot, centerToFoot, stepOffset=0.75):
        """
        Computes step trajectory for a foot.

        Parameters:
        phi (float): Phase between [0, 1).
        V (float): Velocity.
        angle (float): Angle in degrees.
        Wrot (float): Rotation.
        centerToFoot (numpy.ndarray): Vector from the center of the robot to foot.
        stepOffset (float): Offset between steps.

        Returns:
        numpy.ndarray: Coordinates of the step trajectory.
        """
        if phi >= 1:
            phi -= 1.0
        r = np.sqrt(centerToFoot[0] ** 2 + centerToFoot[1] ** 2)
        footAngle = np.arctan2(centerToFoot[1], centerToFoot[0])
        if Wrot >= 0.0:
            circleTrajectory = 90.0 - np.rad2deg(footAngle - self.alpha)
        else:
            circleTrajectory = 270.0 - np.rad2deg(footAngle - self.alpha)
        if phi <= stepOffset:
            phiStance = phi / stepOffset
            stepXLong, stepYLong, stepZLong = self.calculate_stance(phiStance, V, angle)
            stepXRot, stepYRot, stepZRot = self.calculate_stance(phiStance, Wrot, circleTrajectory)
        else:
            phiSwing = (phi - stepOffset) / (1 - stepOffset)
            stepXLong, stepYLong, stepZLong = self.calculate_bezier_swing(phiSwing, V, angle)
            stepXRot, stepYRot, stepZRot = self.calculate_bezier_swing(phiSwing, Wrot, circleTrajectory)
        if centerToFoot[1] > 0:
            if stepXRot < 0:
                self.alpha = -np.arctan2(np.sqrt(stepXRot ** 2 + stepYRot ** 2), r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepXRot ** 2 + stepYRot ** 2), r)
        else:
            if stepXRot < 0:
                self.alpha = np.arctan2(np.sqrt(stepXRot ** 2 + stepYRot ** 2), r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepXRot ** 2 + stepYRot ** 2), r)
        coord = np.empty(3)
        coord[0] = stepXLong + stepXRot
        coord[1] = stepYLong + stepYRot
        coord[2] = stepZLong + stepZRot
        return coord

    def loop(self, V, angle, Wrot, T, offset, bodyToFeet_, stepAsym=0.0, dutyCycle=0.5):
        """
        Computes step trajectory for every foot.

        Parameters:
        V (float): Velocity command.
        angle (float): Angle of the trajectory.
        Wrot (float): Rotation.
        T (float): Time period.
        offset (list): Offset for each step.
        bodyToFeet_ (numpy.ndarray): Initial vector from center of robot to feet.
        stepAsym (float): Asymmetry in step.
        dutyCycle (float): Duty cycle.

        Returns:
        numpy.ndarray: Updated vector from center of robot to feet.
        """
        if V == 0 and angle == 0 and Wrot == 0:
            return bodyToFeet_
        if T <= 0.01:
            T = 0.01
        if self.phi >= 0.99:
            self.lastTime = time.time()
        self.phi = (time.time() - self.lastTime) / T
        stepCoord = self.step_trajectory(
            self.phi + offset[0],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodyToFeet_[0, :])),
            dutyCycle * np.exp(stepAsym),
        )  
        self.bodyToFeet[0, 0] = bodyToFeet_[0, 0] + stepCoord[0]
        self.bodyToFeet[0, 1] = bodyToFeet_[0, 1] + stepCoord[1]
        self.bodyToFeet[0, 2] = bodyToFeet_[0, 2] + stepCoord[2]
        stepCoord = self.step_trajectory(
            self.phi + offset[1],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodyToFeet_[1, :])),
            dutyCycle * np.exp(stepAsym),
        )
        self.bodyToFeet[1, 0] = bodyToFeet_[1, 0] + stepCoord[0]
        self.bodyToFeet[1, 1] = bodyToFeet_[1, 1] + stepCoord[1]
        self.bodyToFeet[1, 2] = bodyToFeet_[1, 2] + stepCoord[2]
        stepCoord = self.step_trajectory(
            self.phi + offset[2],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodyToFeet_[2, :])),
            dutyCycle * np.exp(stepAsym),
        )
        self.bodyToFeet[2, 0] = bodyToFeet_[2, 0] + stepCoord[0]
        self.bodyToFeet[2, 1] = bodyToFeet_[2, 1] + stepCoord[1]
        self.bodyToFeet[2, 2] = bodyToFeet_[2, 2] + stepCoord[2]
        stepCoord = self.step_trajectory(
            self.phi + offset[3],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(bodyToFeet_[3, :])),
            dutyCycle * np.exp(stepAsym),
        )
        self.bodyToFeet[3, 0] = bodyToFeet_[3, 0] + stepCoord[0]
        self.bodyToFeet[3, 1] = bodyToFeet_[3, 1] + stepCoord[1]
        self.bodyToFeet[3, 2] = bodyToFeet_[3, 2] + stepCoord[2]
        return self.bodyToFeet
