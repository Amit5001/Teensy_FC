import socket
import threading
import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import math

# Function to convert quaternion to a rotation matrix
def quaternion_to_rotation_matrix(q):
    q_w, q_x, q_y, q_z = q[0], q[1], q[2], q[3]
    R = np.array([[1 - 2*q_y**2 - 2*q_z**2, 2*q_x*q_y - 2*q_z*q_w, 2*q_x*q_z + 2*q_y*q_w],
                  [2*q_x*q_y + 2*q_z*q_w, 1 - 2*q_x**2 - 2*q_z**2, 2*q_y*q_z - 2*q_x*q_w],
                  [2*q_x*q_z - 2*q_y*q_w, 2*q_y*q_z + 2*q_x*q_w, 1 - 2*q_x**2 - 2*q_y**2]])
    return R

# Function to convert Euler angles to a rotation matrix
def euler_to_rotation_matrix(e):
    roll, pitch, yaw = e[0], e[1], e[2]
    # roll = math.radians(roll)
    # pitch = math.radians(pitch)
    # yaw = math.radians(yaw)

    R_x = np.array([[1, 0, 0],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll), math.cos(roll)]])
    
    R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                    [0, 1, 0],
                    [-math.sin(pitch), 0, math.cos(pitch)]])
    
    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

# Class to handle the UDP server
class UDPServer:
    def __init__(self, server_addr, client_addr):
        # Initialize the UDP socket
        self.server_addr = server_addr
        self.client_addr = client_addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(self.server_addr)

        # Messages
        self.SYN = bytes([0x01, 0x01])
        self.SYNACK = bytes([0x01, 0x02])
        self.ACK = bytes([0x01, 0x03])
        self.PING = bytes([0x01, 0x10])
        self.PONG = bytes([0x01, 0x11])
        self.TERMINATE = bytes([0x01, 0xFF])

        # Quaternion and Euler messages
        self.quaternion_msg = [0.0] * 4
        self.euler_msg = [0.0] * 3

    def receiver(self):
        print("UDP Server started, waiting for connection...")
        # Initiate SYN-ACK handshake
        self.sock.sendto(self.SYN, self.client_addr)
        message, address = self.sock.recvfrom(1024)
        while message != self.SYNACK:
            message, address = self.sock.recvfrom(1024)
        self.sock.sendto(self.ACK, self.client_addr)
        print("Created a connection")

        # Receive data (either quaternion or euler angles)
        while True:
            message, _ = self.sock.recvfrom(1024)
            if message[:2] == self.PING:
                pong_response = bytearray(self.PONG)
                pong_response.append(message[2])
                self.sock.sendto(bytes(pong_response), self.client_addr)
            else:
                self.handle_data(message)

    def handle_data(self, message):
        # 'q' for quaternion, 'e' for Euler angles
        if message[0] == ord('q'):
            self.quaternion_msg = struct.unpack('f' * 4, message[1:])
        elif message[0] == ord('e'):
            self.euler_msg = struct.unpack('f' * 3, message[1:])

# Class to handle the 3D graph for quaternion or euler visualization
class OrientationListener:
    def __init__(self, udp_server, use_quaternion=True):
        self.udp_server = udp_server
        self.use_quaternion = use_quaternion
        self.setup_plot()

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

    def update_plot(self):
        # This should be called in the main thread
        while True:
            time.sleep(0.01)  # Sleep to control the update rate

            # Use quaternion or Euler angles to update the graph
            if self.use_quaternion:
                # Retrieve the latest quaternion data from the UDP server
                quat_msg = self.udp_server.quaternion_msg
                R = quaternion_to_rotation_matrix(quat_msg)
            else:
                # Retrieve the latest Euler angles from the UDP server
                euler_msg = self.udp_server.euler_msg
                R = euler_to_rotation_matrix(euler_msg)

            # Unit axes
            object_axes = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  
            transformed_axes = R.dot(object_axes)  # Apply rotation matrix

            # Update the quiver objects instead of redrawing them from scratch
            self.quiv_x.remove()
            self.quiv_y.remove()
            self.quiv_z.remove()

            # Ensure the correct mapping of transformed axes back to original X, Y, Z
            self.quiv_x = self.ax.quiver(0, 0, 0, transformed_axes[0, 0], transformed_axes[1, 0], transformed_axes[2, 0], color='r')
            self.quiv_y = self.ax.quiver(0, 0, 0, transformed_axes[0, 1], transformed_axes[1, 1], transformed_axes[2, 1], color='g')
            self.quiv_z = self.ax.quiver(0, 0, 0, transformed_axes[0, 2], transformed_axes[1, 2], transformed_axes[2, 2], color='b')

            plt.draw()
            plt.pause(0.001)  # Ensure the plot updates in real-time

def start_server_and_graph():
    # Server and Client addresses
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.168.1.199', 8888)

    # Create the UDP server object
    udp_server = UDPServer(server_addr, client_addr)

    # Start UDP server receiver in a new thread
    udp_thread = threading.Thread(target=udp_server.receiver)
    udp_thread.daemon = True
    udp_thread.start()

    # Ask user whether to use quaternion or Euler angles for visualization
    choice = input("Choose 'q' for quaternion or 'e' for Euler angles: ").strip().lower()
    use_quaternion = True if choice == 'q' else False

    # Create the OrientationListener for graph updates
    listener = OrientationListener(udp_server, use_quaternion)

    # Start graph updates (in the main thread)
    listener.update_plot()

if __name__ == "__main__":
    start_server_and_graph()
