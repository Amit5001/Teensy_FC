import socket
import time

def send_udp_message():
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Set a timeout for receiving responses
    sock.settimeout(1)
    
    # Teensy address
    teensy_address = ('192.169.1.199', 8888)
    
    try:
        # Send a message
        message = b'Hello, Teensy!'
        print(f'Sending to {teensy_address}: {message}')
        sock.sendto(message, teensy_address)
        
        # Try to receive response (optional, depends on if Teensy is set up to respond)
        try:
            data, server = sock.recvfrom(4096)
            print(f'Received response: {data}')
            return True
        except socket.timeout:
            print("No response received (timeout)")
    except Exception as e:
        print(f"Error sending: {e}")
    finally:
        sock.close()
    
    return False

# Try sending a few times
for i in range(5):
    print(f"\nAttempt {i+1}:")
    if send_udp_message():
        break
    time.sleep(1)