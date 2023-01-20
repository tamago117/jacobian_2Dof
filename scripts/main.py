import numpy as np
import math
import matplotlib.pyplot as plt
from dof2 import jacobian_2Dof, visualize_2Dof, calc_forward_kinematics_2Dof

class Dof2_anime:
    def __init__(self):
        fig = plt.figure()
        fig.canvas.mpl_connect('button_press_event', self.click)

        # Define the robot parameters
        self.l1 = 1
        self.l2 = 1
        self.lam = 0.1

        self.trajectory = []

        # Define the initial joint angles
        self.theta = np.array([0.1, 0.1])

        # Define the desired end effector position
        self.target_pos = np.array([0.1, 1])

        # Define the initial end effector position
        self.current_pos = calc_forward_kinematics_2Dof(self.theta, self.l1, self.l2)
        self.trajectory.append(self.current_pos[1, :])

    def process(self):
        while True:
            # error
            error = self.target_pos - self.current_pos[1, :]
            error_norm = np.linalg.norm(error)

            # Jacobian
            J = jacobian_2Dof(self.theta, self.l1, self.l2)

            # inverse kinematics
            self.theta = self.theta + self.lam * np.dot(np.linalg.pinv(J), error)

            # update the current position
            self.current_pos = calc_forward_kinematics_2Dof(self.theta, self.l1, self.l2)

            self.trajectory.append(self.current_pos[1, :])

            # visualize the robot
            visualize_2Dof(self.theta, self.l1, self.l2, self.target_pos, self.trajectory)

            # break if the error is small
            print("error_norm: ", error_norm)
            if error_norm < 0.01:
                #break
                print("press esc to exit")


    def click(self, event):
        self.target_pos = np.array([event.xdata, event.ydata])


if __name__ == "__main__":
    Dof2_anime().process()