from numpy import max
from math import sin, cos, pi


class fuzzyControl(object):
    def __init__(self):
        return

    def control(self, mdl):
        state = mdl.getState()
        position = state[0]
        inclination = state[1]
        linear_vel = state[2]
        angular_vel = state[3]

        # -------------------------------------------------------------
        # Inclination
        # -------------------------------------------------------------
        # Negative membership determination for inclination
        if inclination <= -0.1:
            negative_th = 1
        elif -0.1 < inclination < 0:
            negative_th = -10 * inclination
        else:
            negative_th = 0

        # Zero membership determination for inclination
        if -0.1 < inclination < -0.03:
            zero_th = (100 / 7) * inclination + 10 / 7
        elif -0.03 <= inclination <= 0.03:
            zero_th = 1
        elif 0.03 < inclination < 0.1:
            zero_th = -(100 / 7) * inclination + 10 / 7
        else:
            zero_th = 0

        # Positive membership determination for inclination
        if inclination <= 0:
            positive_th = 0
        elif 0 < inclination < 0.1:
            positive_th = 10 * inclination
        else:
            positive_th = 1

        # -------------------------------------------------------------
        # Angular Velocity
        # -------------------------------------------------------------
        # Negative membership determination for angluar velocity
        if angular_vel <= -0.1:
            negative_thd = 1
        elif -0.1 < angular_vel < 0:
            negative_thd = -10 * angular_vel
        else:
            negative_thd = 0

        # Zero membership determination for angluar velocity
        if -0.15 < angular_vel < -0.03:
            zero_thd = (100 / 12) * angular_vel + 15 / 12
        elif -0.03 <= angular_vel <= 0.03:
            zero_thd = 1
        elif 0.03 < angular_vel < 0.15:
            zero_thd = -(100 / 12) * angular_vel + 15 / 12
        else:
            zero_thd = 0

        # Positive membership determination for angular velocity
        if angular_vel <= 0:
            positive_thd = 0
        elif 0 < angular_vel < 0.1:
            positive_thd = 10 * angular_vel
        else:
            positive_thd = 1

        NL = [0]
        NM = [0]
        NS = [0]
        Z = [0]
        PS = [0]
        PM = [0]
        PL = [0]
        # -------------------------------------------------------------
        # Output membership determination - pendulum rules
        # -------------------------------------------------------------
        # Pendulum rule # 1
        NL.append(min(negative_th, negative_thd))
        # Pendulum rule # 2
        NM.append(min(negative_th, zero_thd))
        # Pendulum rule # 3
        Z.append(min(negative_th, positive_thd))
        # Pendulum rule # 4
        NS.append(min(zero_th, negative_thd))
        # Pendulum rule # 5
        Z.append(min(zero_th, zero_thd))
        # Pendulum rule # 6
        PS.append(min(zero_th, positive_thd))
        # Pendulum rule # 7
        Z.append(min(positive_th, negative_thd))
        # Pendulum rule # 8
        PM.append(min(positive_th, zero_thd))
        # Pendulum rule # 9
        PL.append(min(positive_th, positive_thd))

        # Determination of the force applied to the car
        num_p = max(NL) * -100 + max(NM) * -10 + max(NS) * -5 + max(Z) * 0 + max(PS) * 5 + max(PM) * 10 + max(PL) * 100
        den_p = max(NL) + max(NM) + max(NS) + max(Z) + max(PS) + max(PM) + max(PL)

        return num_p / den_p

        # -------------------------------------------------------------
        # Car
        # -------------------------------------------------------------

        # Position left
        if position <= -2:
            position_left = 1
        elif -2 < position < 0:
            position_left = position * -0.5
        else:
            position_left = 0

        # Position center
        if position <= -1.5:
            position_center = 0
        elif -1.5 < position < -0.5:
            position_center = position + 1.5
        elif 0.5 < position < 1.5:
            position_center = -position + 1.5
        else:
            position_center = 0

        # Position right
        if position >= 2:
            position_right = 1
        elif 2 > position > 0.5:
            position_right = -position + 1.5
        else:
            position_right = 0

        # Aceleration right
        if position >= 2:
            aceleration_right = 1
        elif 2 > position > 0.5:
            aceleration_right = -position + 1.5
        else:
            aceleration_right = 0

        # Aceleration left
        if position <= -2:
            aceleration_left = 1
        elif -2 < position < 0:
            aceleration_left = position * -0.5
        else:
            aceleration_left = 0

        # Stoped
        if position <= -1.5:
            stoped = 0
        elif -1.5 < position < -0.5:
            stoped = position + 1.5
        elif 0.5 < position < 1.5:
            stoped = -position + 1.5
        else:
            stoped = 0

