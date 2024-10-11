import socket
import threading
import struct

FLOAT_SIZE = struct.calcsize('f')
INT_SIZE = struct.calcsize('i')

class UDPServer:
    def __init__(self, server_addr, client_addr):
        self.server_addr = server_addr
        self.client_addr = client_addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Messages
        self.SYN = bytes([0x01, 0x01])
        self.SYNACK = bytes([0x01, 0x02])
        self.ACK = bytes([0x01, 0x03])
        self.PING = bytes([0x01, 0x10])
        self.PONG = bytes([0x01, 0x11])
        self.TERMINATE = bytes([0x01, 0xFF])

        # Data storage for IMUs and other data
        self.polulo_msg = [0.0] * 6
        self.euler_msg = [0.0] * 3
        self.rc_ch_msg = [0] * 16
        self.quaternion_msg = [0.0] * 4

        # Start receiver thread
        threading.Thread(target=self.receiver).start()

    def receiver(self):
        self.sock.bind(self.server_addr)
        print("Server Started")

        # Initiate SYN-ACK handshake
        self.sock.sendto(self.SYN, self.client_addr)
        message, address = self.sock.recvfrom(1024)
        while message != self.SYNACK:
            message, address = self.sock.recvfrom(1024)

        if message != self.SYNACK:
            print("FAILED SYN ACK")
            return
        self.sock.sendto(self.ACK, self.client_addr)
        print("Created a connection")

        while True:
            message, address = self.sock.recvfrom(1024)
            if message[:2] == self.PING:
                pong_response = bytearray(self.PONG)
                pong_response.append(message[2])
                self.sock.sendto(bytes(pong_response), address)
            else:
                self.casting(message)

    def casting(self, message):
        # Casting the messages
        msg_type = chr(message[0])
        message = message[1:]

        # Unpacking float or integer data depending on the message type
        if msg_type in ['e', 'p', 'q']:
            unpacked_data = struct.unpack("f" * (len(message) // FLOAT_SIZE), message)
        elif msg_type == 'r':
            unpacked_data = struct.unpack("i" * (len(message) // INT_SIZE), message)
        
        if msg_type == 'e':  # Euler angles
            self.euler_msg = unpacked_data[:3]
            self.display_euler(self.euler_msg)
            
        elif msg_type == 'p':  # Pololu IMU data
            self.polulo_msg = unpacked_data[:6]
            self.display_imu(self.polulo_msg)
            
        elif msg_type == 'q':  # Quaternion data
            self.quaternion_msg = unpacked_data[:4]
            self.display_quaternion(self.quaternion_msg)
            
        elif msg_type == 'r':  # RC channel data
            self.rc_ch_msg = unpacked_data[:16]
            self.display_rc(self.rc_ch_msg)

    # Functions to display or process the data
    def display_imu(self, imu_data):
        print(f"IMU Data: Acceleration - {imu_data[:3]}, Angular Velocity - {imu_data[3:]}")

    def display_quaternion(self, quaternion_data):
        print(f"Quaternion Data: {quaternion_data}")

    def display_euler(self, euler_data):
        print(f"Euler Angles: {euler_data}")

    def display_rc(self, rc_data):
        print(f"RC Channels: {rc_data}")

class JoyListener:
    def __init__(self, udp_server):
        self.udp_server = udp_server
        print('Listening for joystick data...')

    def process_joy_data(self, axes, buttons):
        axis_byte_array = b''.join([struct.pack('f', f) for f in axes])
        self.udp_server.sock.sendto(axis_byte_array, self.udp_server.client_addr)
        print(f'Sent Joystick Data: Axes - {axes}, Buttons - {buttons}')


def start_udp_server():
    # Create and start the UDP server
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.168.1.199', 8888)
    server = UDPServer(server_addr, client_addr)

    # Create the JoyListener
    joy_listener = JoyListener(server)

    # Simulate joystick data processing (replace this with actual joystick input handling)
    example_axes = [0.5, -0.2, 0.0]
    example_buttons = [1, 0, 0, 1]
    joy_listener.process_joy_data(example_axes, example_buttons)


if __name__ == "__main__":
    start_udp_server()
