import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import socket
import struct

def quaternion_to_rotation_matrix(q):
    q_w, q_x, q_y, q_z = q[3], q[0], q[1], q[2]
    R = np.array([[1 - 2*q_y**2 - 2*q_z**2, 2*q_x*q_y - 2*q_z*q_w, 2*q_x*q_z + 2*q_y*q_w],
                  [2*q_x*q_y + 2*q_z*q_w, 1 - 2*q_x**2 - 2*q_z**2, 2*q_y*q_z - 2*q_x*q_w],
                  [2*q_x*q_z - 2*q_y*q_w, 2*q_y*q_z + 2*q_x*q_w, 1 - 2*q_x**2 - 2*q_y**2]])
    return R

class QuaternionListener:
    def __init__(self):
        self.setup_plot()
        self.setup_socket()
        print("Listening for quaternion data...")

    def setup_plot(self):
        # Interactive mode on
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_zlim([-1, 1])
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')

        # Static reference axes (red, green, blue for X, Y, Z respectively)
        self.ref_x = self.ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='X')
        self.ref_y = self.ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y')
        self.ref_z = self.ax.quiver(0, 0, 0, 0, 0, 1, color='b', label='Z')

        # Initialize object axes (that will be rotated)
        self.quiv_x = self.ax.quiver(0, 0, 0, 1, 0, 0, color='r')
        self.quiv_y = self.ax.quiver(0, 0, 0, 0, 1, 0, color='g')
        self.quiv_z = self.ax.quiver(0, 0, 0, 0, 0, 1, color='b')

    def setup_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 13000))  # Bind to the same port as the UDP server

    def listener_callback(self, quat_msg):
        R = quaternion_to_rotation_matrix(quat_msg)
        object_axes = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # Unit axes
        transformed_axes = R.dot(object_axes)  # Apply rotation matrix

        # Update the quiver objects instead of redrawing them from scratch
        self.quiv_x.remove()
        self.quiv_y.remove()
        self.quiv_z.remove()

        self.quiv_x = self.ax.quiver(0, 0, 0, transformed_axes[0, 0], transformed_axes[0, 1], transformed_axes[0, 2], color='r')
        self.quiv_y = self.ax.quiver(0, 0, 0, transformed_axes[1, 0], transformed_axes[1, 1], transformed_axes[1, 2], color='g')
        self.quiv_z = self.ax.quiver(0, 0, 0, transformed_axes[2, 0], transformed_axes[2, 1], transformed_axes[2, 2], color='b')

        plt.draw()
        plt.pause(0.001)  # Short pause to ensure the plot updates

    def start_listening(self):
        while True:
            data, _ = self.sock.recvfrom(1024)  # Receive data from UDP server
            quaternion = struct.unpack('f' * 4, data)  # Unpack received quaternion
            self.listener_callback(quaternion)  # Update plot with new quaternion

def main():
    listener = QuaternionListener()
    try:
        listener.start_listening()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == '__main__':
    main()
