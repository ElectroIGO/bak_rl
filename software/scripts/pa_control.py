import socket
import threading
import time

# Configuration
SERVER_IP = "192.168.0.177"  # UDP server IP
SERVER_PORT = 2075          # UDP server port
MON_INTERVAL = 1             # Seconds between mon commands

class mcuC:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1)
        self.running = False
        self.mon_thread = None
        self.input_thread = None

    def send_command(self, command):
        """Send a command to the microcontroller"""
        try:
            self.sock.sendto(command.encode(), (SERVER_IP, SERVER_PORT))
        except Exception as e:
            print(f"Error sending command: {e}")

    def parse_data(self, data):
        """Parse received data (customize this for your protocol)"""
        # Add your custom parsing logic here
        return data.decode().strip()

    def mon_loop(self):
        """Periodically send mon commands"""
        while self.running:
            self.send_command("mon")
            time.sleep(MON_INTERVAL)

    def input_loop(self):
        """Handle user input"""
        while self.running:
            try:
                user_input = input("Enter command (or 'exit' to quit): ")
                if user_input.lower() == 'exit':
                    self.running = False
                    break
                self.send_command(user_input)
            except Exception as e:
                print(f"Input error: {e}")

    def receive_loop(self):
        """Handle incoming data"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                parsed = self.parse_data(data)
                print(f"Received: {parsed}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receive error: {e}")
                self.running = False

    def start(self):
        """Start the client"""
        self.running = True
        print("Starting client...")
        
        # Send initial command
        self.send_command("pon=1")
        
        # Start monitoring thread
        self.mon_thread = threading.Thread(target=self.mon_loop)
        self.mon_thread.start()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.start()
        
        # Start receive loop in main thread
        self.receive_loop()

    def stop(self):
        """Stop the client"""
        print("\nStopping client...")
        self.running = False
        self.send_command("pon=0")
        self.sock.close()
        self.mon_thread.join()
        self.input_thread.join()
        print("Client stopped")

if __name__ == "__main__":
    client = mcuC()
    try:
        client.start()
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()