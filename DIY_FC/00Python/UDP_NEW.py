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

        # Socket to send quaternion data to visualizer
        self.vis_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vis_addr = ('127.0.0.1', 13000)  # Address for visualizer

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

        if message_type == 'q':  # quaternion message
            self.quaternion_msg = messages_struct_float[:4]
            self.publish_quaternion(self.quaternion_msg)                

    def publish_quaternion(self, quaternion_data):
        # Pack quaternion data and send to visualizer
        packed_data = struct.pack('f'*4, *quaternion_data)
        self.vis_sock.sendto(packed_data, self.vis_addr)
        print(f"Sent Quaternion: {quaternion_data}")

def start_udp_server():
    # Create and start the UDP server
    server_addr = ('0.0.0.0', 12000)
    client_addr = ('192.169.1.199', 8888)
    server = UDPServer(server_addr, client_addr)

if __name__ == "__main__":
    start_udp_server()
