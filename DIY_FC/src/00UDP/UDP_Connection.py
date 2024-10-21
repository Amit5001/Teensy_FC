import socket
import threading
import struct

FLOAT_SIZE = struct.calcsize('f')
INT_SIZE = struct.calcsize('i')

class UDPServer:
    def __init__(self, server_addr, client_addr):
        # Initialize server addresses
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
        message_type = chr(message[0])
        message = message[1:]
        messages_struct_float = struct.unpack("f" * (len(message) // FLOAT_SIZE), message)
        messages_struct_int = struct.unpack("i" * (len(message) // INT_SIZE), message)

        if message_type == 'e':
            self.euler_msg = messages_struct_float[:3]
            self.publish_euler(self.euler_msg)
                
        elif message_type == 'p':
            self.polulo_msg = messages_struct_float[:6]
            self.publish_imu(self.polulo_msg)

        elif message_type == 'q':
            self.quaternion_msg = messages_struct_float[:4]
            self.publish_quaternion(self.quaternion_msg)                

        elif message_type == 'r':
            self.rc_ch_msg = messages_struct_int[:16]
            self.publish_rc(self.rc_ch_msg)

    def publish_imu(self, imu_data):
        imu_msg = {
            'linear_acceleration': {'x': imu_data[0], 'y': imu_data[1], 'z': imu_data[2]},
            'angular_velocity': {'x': imu_data[3], 'y': imu_data[4], 'z': imu_data[5]},
        }
        print(f"IMU Data: {imu_msg}")

    def publish_quaternion(self, quaternion_data):
        quat_msg = {
            'x': quaternion_data[0],
            'y': quaternion_data[1],
            'z': quaternion_data[2],
            'w': quaternion_data[3]
        }
        print(f"Quaternion Data: {quat_msg}")

    def publish_euler(self, euler_data):
        euler_msg = {
            'x': euler_data[0],
            'y': euler_data[1],
            'z': euler_data[2]
        }
        print(f"Euler Angles: {euler_msg}")

    def publish_rc(self, rc_data):
        rc_msg = {'rc_channels': rc_data}
        print(f"RC Channels: {rc_msg}")

class JoyListener:
    def __init__(self, udp_server):
        self.udp_server = udp_server

    def joy_callback(self, axes, buttons):
        # Convert joystick axes to a byte array and send it over UDP
        axis_byte_array = b''.join([struct.pack('f', f) for f in axes])
        self.udp_server.sock.sendto(axis_byte_array, self.udp_server.client_addr)
        print(f"Sent joystick axes: {axes}")

def start_udp_server():
    # Create and start the UDP server
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.168.1.199', 8888)
    server = UDPServer(server_addr, client_addr)

    # Create the JoyListener
    joy_listener = JoyListener(server)

    # Simulate joystick data and send it
    server.receiver()

if __name__ == "__main__":
    start_udp_server()
