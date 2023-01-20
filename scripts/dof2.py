import matplotlib.pyplot as plt
import numpy as np
import math

def visualize_2Dof(joint_angle, l1, l2, target_pos, trajectory=None):

    #calculate the end effector position
    joint_pos = calc_forward_kinematics_2Dof(joint_angle, l1, l2)

    plt.clf()
    # Plot the robot
    plt.plot([0, joint_pos[0, 0]], [0, joint_pos[0, 1]], 'k', linewidth=2)
    plt.scatter(joint_pos[0, 0], joint_pos[0, 1], s=100, c='k')
    plt.plot([joint_pos[0, 0], joint_pos[1, 0]], [joint_pos[0, 1], joint_pos[1, 1]], 'k', linewidth=2)

    plt.plot(target_pos[0], target_pos[1], 'gx')

    if trajectory is not None:
        plt.plot(np.array(trajectory)[:, 0], np.array(trajectory)[:, 1], 'r', linewidth=2)

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    plt.axis("equal")
    plt.pause(0.001)

def calc_forward_kinematics_2Dof(theta, l1, l2):

    #calculate the end effector position
    joint_pos = np.array([[l1*math.cos(theta[0]), l1*math.sin(theta[0])],
                          [l1*math.cos(theta[0]) + l2*math.cos(theta[0]+theta[1]), l1*math.sin(theta[0]) + l2*math.sin(theta[0]+theta[1])]])

    return joint_pos

def jacobian_2Dof(theta, l1, l2):

    J = np.array([[-l1*math.sin(theta[0]) - l2*math.sin(theta[0]+theta[1]), -l2*math.sin(theta[0]+theta[1])],
                  [l1*math.cos(theta[0]) - l2*math.cos(theta[0]+theta[1]), l2*math.cos(theta[0]+theta[1])]])

    return J

if __name__ == '__main__':
    for i in range(100):
        visualize_2Dof([0, 0], 1, 1)