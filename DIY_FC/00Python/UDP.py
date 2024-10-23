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
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.SYN = bytes([0x01, 0x01])
        self.SYNACK = bytes([0x01, 0x02])
        self.ACK = bytes([0x01, 0x03])
        self.PING = bytes([0x01, 0x10])
        self.PONG = bytes([0x01, 0x11])
        self.TERMINATE = bytes([0x01, 0xFF])
        self.polulo_msg = [0.0] * 6
        self.euler_msg = [0.0] * 3
        self.mag_msg = [0.0] * 3 
        self.quaternion_msg = [0.0] * 4
        self.rc_ch_msg = [0] * 16
        threading.Thread(target=self.receiver).start()

    def receiver(self):
        self.sock.bind(self.server_addr)
        print("Server Started")
        self.sock.settimeout(10)  # Increased timeout duration
        try:
            print("Sending SYN to client")
            self.sock.sendto(self.SYN, self.client_addr)
            retries = 0
            message, address = self.sock.recvfrom(1024)
            while message != self.SYNACK and retries < 5:
                print(f"Retry {retries}: Waiting for SYNACK")
                message, address = self.sock.recvfrom(1024)
                retries += 1
            if message != self.SYNACK:
                print("FAILED SYN ACK after retries")
                return
            self.sock.sendto(self.ACK, self.client_addr)
            print("Created a connection")
        except socket.timeout:
            print("Connection timed out during handshake.")
            return
        self.sock.settimeout(None)
        while True:
            message, address = self.sock.recvfrom(1024)
            if message[:2] == self.PING:
                pong_response = bytearray(self.PONG)
                pong_response.append(message[2])
                self.sock.sendto(bytes(pong_response), address)
            else:
                self.casting(message)

    def casting(self, message):
        type = chr(message[0])
        message = message[1:]
        messages_struct_float = struct.unpack("f" * (len(message) // FLOAT_SIZE), message)
        message_struct_int = struct.unpack("i" * (len(message) // INT_SIZE), message)
        if type == 'm':
            self.mag_msg = messages_struct_float[:3]
            self.publish_mag(self.mag_msg)
        elif type == 'q':
            self.quaternion_msg = messages_struct_float[:4]
            self.publish_quaternion(self.quaternion_msg)
        elif type == 'e':
            self.euler_msg = messages_struct_float[:3]
            self.publish_euler(self.euler_msg)
        elif type == 'p':
            self.polulo_msg = messages_struct_float[:6]
            self.publish_imu(self.polulo_msg)
        elif type == 'r':
            self.rc_ch_msg = message_struct_int[:16]
            self.publish_rc(self.rc_ch_msg)

    def publish_imu(self, imu_data):
        print(f"IMU data: {imu_data}")

    def publish_quaternion(self, quaternion_data):
        print(f"Quaternion data: {quaternion_data}")

    def publish_euler(self, euler_data):
        print(f"Euler angles data: {euler_data}")

    def publish_mag(self, mag_data):
        print(f"Magnetometer data: {mag_data}")

    def publish_rc(self, rc_data):
        print(f"RC channels data: {rc_data}")

class JoyListener:
    def __init__(self, udp_server):
        self.udp_server = udp_server
        print('Listening to joystick input...')

    def joy_callback(self, axes, buttons):
        axis_byte_array = b''.join([struct.pack('f', f) for f in axes])
        self.udp_server.sock.sendto(axis_byte_array, self.udp_server.client_addr)

def start_udp_server():
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.168.1.199', 8888)  # Use localhost for testing
    server = UDPServer(server_addr, client_addr)
    joy_listener = JoyListener(server)
    try:
        while True:
            # Simulate joystick data callback
            axes = [0.0] * 6  # Example axes data
            buttons = [0] * 12  # Example button data
            joy_listener.joy_callback(axes, buttons)
    except KeyboardInterrupt:
        print("Server shutting down...")

if __name__ == "__main__":
    start_udp_server()
