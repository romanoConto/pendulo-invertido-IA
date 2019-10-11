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
    # Negative membership determination for inclination -- funções de pertinência
        if inclination <= -0.1:
            negative_th = 1
        elif -0.1 < inclination < 0:
            negative_th = -10 * inclination
        else:
            negative_th = 0

    # Zero membership determination for inclination
        if -0.1 < inclination < -0.03:
            zero_th = (100/7) * inclination + 10/7
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
            zero_thd = (100/12) * angular_vel + 15/12
        elif -0.03 <= angular_vel <= 0.03:
            zero_thd = 1
        elif 0.03 < angular_vel < 0.15:
            zero_thd = -(100/12) * angular_vel + 15/12
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
        PM =[0]
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
    ######################## novas regras

    # -------------------------------------------------------------
    # Position
    # -------------------------------------------------------------
    # Negative membership determination for position
        if position <= -2:
            negative_position_car = 1
        elif -2 < position < 0:
            negative_position_car = position * -0.5
        else:
            negative_position_car = 0

    # Zero membership determination for position
        if position <= -1.5:
            zero_position_car = 0
        elif position > 1.5:
            zero_position_car = 0
        elif -1.5 < position < -0.5:
            zero_position_car = (position + 1.5)
        elif 0.5 < position < 1.5:
            zero_position_car = (position - 1.5)
        else:
            zero_position_car = 1

    # Positive membership determination for position
        if position >= 2:
            positive_position_car = 1
        elif 2 > position > 0:
            positive_position_car = position * 0.5
        else:
            positive_position_car = 0

    # -------------------------------------------------------------
    # Linear Velocity
    # -------------------------------------------------------------
    # Negative membership determination for position
        if linear_vel <= -3:
            negative_position_card = 1
        elif -3 < linear_vel < 0:
            negative_position_card = (linear_vel  * (-1/3))
        else:
            negative_position_card = 0

    # Zero membership determination for position
        if linear_vel <= -1.5:
            zero_position_card = 0
        elif -1.5 < linear_vel < -0.5:
            zero_position_card = linear_vel + 1.5
        elif 0.5 < linear_vel < 1.5:
            zero_position_card = linear_vel - 1.5
        else:
            zero_position_card = 1

    # Positive membership determination for position
        if linear_vel >= 3:
            positive_position_card = 1
        elif 3 > linear_vel > 0:
            positive_position_card = linear_vel * (1/3)
        else:
            positive_position_card = 0

    # Car
    #position & lin. veloc.
        NL_car = [0]
        NM_car = [0]
        NS_car = [0]
        Z_car= [0]
        PS_car = [0]
        PM_car = [0]
        PL_car = [0]

    # CAR rule # 1
        NL_car.append(min(negative_position_car, negative_position_card))
    # CAR rule # 2
        NM_car.append(min(negative_position_car, zero_position_card))
    # CAR rule # 3
        Z_car.append(min(negative_position_car, positive_position_card))
    # CAR rule # 4
        NS_car.append(min(zero_position_car, negative_position_card))
    # CAR rule # 5
        Z_car.append(min(zero_position_car, zero_position_card))
    # CAR rule # 6
        PS_car.append(min(zero_position_car, positive_position_card))
    # CAR rule # 7
        Z_car.append(min(positive_position_car, negative_position_card))
    # CAR rule # 8
        PM_car.append(min(positive_position_car, zero_position_card))
    # CAR rule # 9
        PL_car.append(min(positive_position_car, positive_position_card))

    # Determination of the force applied to the car
        num_p = max(NL)*-100 + max(NM)*-10 + max(NS)*-5 + max(Z)*0 + max(PS)*15 + max(PM)*10 + max(PL)*100
        den_p = max(NL)+max(NM)+max(NS)+max(Z)+max(PS)+max(PM)+max(PL)

    # Determination of the force applied to the car (position + linear vel)
        num_p_car = max(NL_car)*-100 + max(NM_car)*-10 + max(NS_car)*-5 + max(Z_car)*0 + max(PS_car)*5 + max(PM_car)*10 + max(PL_car)*100
        den_p_car = max(NL_car)+max(NM_car)+max(NS_car)+max(Z_car)+max(PS_car)+max(PM_car)+max(PL_car)

        print("start")
        print((num_p / den_p) + (num_p_car / den_p_car))

        return ((num_p / den_p) + (num_p_car / den_p_car))