import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def quaternion_to_rotation_matrix(q):
    q_w, q_x, q_y, q_z = q['w'], q['x'], q['y'], q['z']
    R = np.array([[1 - 2*q_y**2 - 2*q_z**2, 2*q_x*q_y - 2*q_z*q_w, 2*q_x*q_z + 2*q_y*q_w],
                  [2*q_x*q_y + 2*q_z*q_w, 1 - 2*q_x**2 - 2*q_z**2, 2*q_y*q_z - 2*q_x*q_w],
                  [2*q_x*q_z - 2*q_y*q_w, 2*q_y*q_z + 2*q_x*q_w, 1 - 2*q_x**2 - 2*q_y**2]])
    return R

class QuaternionListener:
    def __init__(self):
        self.setup_plot()
        print("Listening for quaternion data...")

    def setup_plot(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.draw_axes()
        plt.ion()
        plt.show()

    def draw_axes(self):
        self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
        self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
        self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

    def listener_callback(self, msg):
        R = quaternion_to_rotation_matrix(msg)
        object_axes = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        transformed_axes = R.dot(object_axes)
        self.ax.cla()
        self.draw_axes()
        self.ax.quiver(0, 0, 0, transformed_axes[0, 0], transformed_axes[0, 1], transformed_axes[0, 2], color='r')
        self.ax.quiver(0, 0, 0, transformed_axes[1, 0], transformed_axes[1, 1], transformed_axes[1, 2], color='g')
        self.ax.quiver(0, 0, 0, transformed_axes[2, 0], transformed_axes[2, 1], transformed_axes[2, 2], color='b')
        plt.draw()
        plt.pause(0.01)

def main():
    listener = QuaternionListener()
    try:
        while True:
            # Simulate incoming quaternion data
            fake_quaternion = {'w': 1, 'x': 0, 'y': 0, 'z': 0}
            listener.listener_callback(fake_quaternion)
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    main()
