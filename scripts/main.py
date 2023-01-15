import numpy as np
import math
import matplotlib.pyplot as plt
from dof2 import jacobian_2Dof, visualize_2Dof, calc_forward_kinematics_2Dof

def main():
    # Define the robot parameters
    l1 = 1
    l2 = 1
    lam = 0.05

    trajectory = []

    # Define the initial joint angles
    theta = np.array([0.1, 0.1])

    # Define the desired end effector position
    target_pos = np.array([1, 1])

    # Define the initial end effector position
    current_pos = calc_forward_kinematics_2Dof(theta, l1, l2)
    trajectory.append(current_pos[1, :])

    while True:
        # error
        error = target_pos - current_pos[1, :]
        error_norm = np.linalg.norm(error)

        # Jacobian
        J = jacobian_2Dof(theta, l1, l2)

        # inverse kinematics
        theta = theta + lam * np.dot(np.linalg.pinv(J), error)

        # update the current position
        current_pos = calc_forward_kinematics_2Dof(theta, l1, l2)

        trajectory.append(current_pos[1, :])

        # visualize the robot
        visualize_2Dof(theta, l1, l2, trajectory)

        # break if the error is small
        print("error_norm: ", error_norm)
        if error_norm < 0.01:
            #break
            print("press esc to exit")





if __name__ == "__main__":
    main()