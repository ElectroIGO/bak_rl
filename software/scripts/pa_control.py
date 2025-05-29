import socket
import threading
import time

# Configuration
SERVER_IP = "192.168.0.177"
SERVER_PORT = 2075
DEFAULT_MON_INTERVAL = 30  # Changed to DEFAULT to indicate it can be changed
RECV_TIMEOUT = 1.0
CONNECT_TIMEOUT = 3.0
# ASCII Art and Help Message
VIRAC_ASCII = r"""
__     _____ ____      _    ____ 
\ \   / /_ _|  _ \    / \  / ___|
 \ \ / / | || |_) |  / _ \| |    
  \ V /  | ||  _ <  / ___ \ |___ 
   \_/  |___|_| \_\/_/   \_\____|
"""

HELP_TEXT = """
Available Commands:
  start        - Initialize connection and activate monitoring
  help         - Show this help message
  exit         - Quit the application
  pon=1        - Power on device
  pon=0        - Power off device
  mon          - Request status update
  setmon=X     - Set monitoring interval to X seconds (e.g. setmon=5)
  [custom]     - Send custom command to device
"""

class mcuC:
    def __init__(self):
        self.print_banner()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(RECV_TIMEOUT)
        self.running = threading.Event()
        self.connected = False
        self.mon_thread = None
        self.input_thread = None
        self.stop_event = threading.Event()
        self.mon_interval = DEFAULT_MON_INTERVAL
        self.exit = 0

    def print_banner(self):
        """Display ASCII art and help on startup"""
        print(VIRAC_ASCII)
        print("VENTSPILS INTERNATIONAL RADIO ASTRONOMY CENTER")
        print("X-band transmitter bias and monitoring")
        print(HELP_TEXT)

    def show_help(self):
        """Display help information"""
        print("\nCommand Help:")
        print(HELP_TEXT)

    def send_command(self, command):
        """Send a command to the microcontroller"""
        try:
            if not self.connected:
                print("Not connected to server!")
                return
            full_cmd = command + "\n"
            self.sock.sendall(full_cmd.encode())
            print(f"Sent: {command}")
        except Exception as e:
            print(f"Error sending command: {e}")
            self.connected = False

    def parse_data(self, data):
        """Parse received data, handling both monitoring and command messages"""
        raw = data.decode().strip()
        
        # Check if this is a monitoring data message (starts with "mon=")
        if raw.startswith("mon="):
            # Extract the CSV data part after "mon="
            csv_data = raw[4:]  # Remove "mon=" prefix
            return self._parse_mon_data(csv_data)
        else:
            # Handle other command responses (like "pon=1", "olm=0", etc.)
            return self._parse_command_response(raw)

    def _parse_mon_data(self, csv_data):
        """Parse monitoring data CSV string into labeled dictionary"""
        try:
            values = csv_data.split(',')
            if len(values) != 24:
                raise ValueError(f"Expected 24 values, got {len(values)}")
                
            return {
                'pa_on_state': f"{int(values[0])}",
                'psu_pg_state': f"{int(values[1])}",
                'open_loop_mode': f"{int(values[2])}",
                'rs0': f"{float(values[3]):.3f} V",
                'rs0_alert_high': f"{int(values[4])}",
                'rs0_alert_low': f"{int(values[5])}",
                'alert0_state': f"{int(values[6])}",
                'isense0': f"{float(values[7]):.3f} A",
                'isense1': f"{float(values[8]):.3f} A",
                'Ug0': f"{float(values[9]):.3f} V",
                'Ug0_margin': f"{float(values[10]):.3f} V",
                'Ug1': f"{float(values[11]):.3f} V",
                'Ug1_margin': f"{float(values[12]):.3f} V",
                'temperature': f"{float(values[13]):.3f} °C",
                'temperature1': f"{float(values[14]):.3f} °C",
                'temperature2': f"{float(values[15]):.3f} °C",
                'temp_Fpwr': f"{float(values[16]):.3f} °C",
                'temp_Rpwr': f"{float(values[17]):.3f} °C",
                'temperature3': f"{float(values[18]):.3f} °C",
                'refpwrl': f"{float(values[19]):.3f} dBm",
                'refpwr': f"{float(values[20]):.3f} W",
                'fwdpwrl': f"{float(values[21]):.3f} dBm",
                'fwdpwr': f"{float(values[22]):.3f} W",
                's11_param': f"{float(values[23]):.3f}"
            }
        except (ValueError, IndexError) as e:
            return {'error': f"Monitoring data parse failed: {str(e)}", 'raw': csv_data}

    def _parse_command_response(self, response):
        """Parse simple command responses (key=value pairs)"""
        if '=' in response:
            cmd, value = response.split('=', 1)
            return {'command': cmd, 'value': value.strip()}
        return {'raw': response}  # Fallback for non-monitoring data

    def mon_loop(self):
        """Periodically send mon commands with current interval"""
        print(f"Monitoring activated ({self.mon_interval} second interval)")
        while not self.stop_event.is_set() and self.running.is_set():
            if self.connected:
                self.send_command("mon")
            time.sleep(self.mon_interval)
        print("Monitoring stopped")

    def input_loop(self):
        """Handle user input with help command support"""
        while self.running.is_set():
            try:
                user_input = input("Enter command ('help' for options): ")
                user_lower = user_input.lower()
                
                if user_lower == 'exit':
                    self.running = False
                    break
                elif user_lower == 'help':
                    self.show_help()
                elif user_lower == 'start':
                    if self.mon_thread and self.mon_thread.is_alive():
                        print("System already active")
                    else:
                        self.stop_event.clear()
                        self.send_command("pon=1")
                        self.mon_thread = threading.Thread(target=self.mon_loop)
                        self.mon_thread.start()
                elif user_lower == 'stop':
                    if self.mon_thread and self.mon_thread.is_alive():
                        self.send_command("pon=0")
                        self.stop_event.set()
                        self.mon_thread.join(timeout=1)
                        self.mon_thread = None
                        print("System stopped")
                    else:
                        print("System already inactive")
                elif user_lower.startswith('setmon='):
                    try:
                        new_interval = int(user_lower.split('=')[1])
                        if new_interval > 0:
                            self.mon_interval = new_interval
                            print(f"Monitoring interval set to {new_interval} seconds")
                            if self.mon_thread and self.mon_thread.is_alive():
                                print("New interval will take effect after next monitor cycle")
                        else:
                            print("Interval must be greater than 0")
                    except (ValueError, IndexError):
                        print("Invalid interval format. Use 'setmon=X' where X is a number")
                else:
                    self.send_command(user_input)
            except Exception as e:
                print(f"Input error: {e}")

    def receive_loop(self):
        """Handle incoming data"""
        while self.running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    print("Connection closed by server.")
                    self.running = False
                    break
                parsed = self.parse_data(data)
                print(f"Received: {parsed}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Receive error: {e}")
                self.running = False

    def start(self):
        """Start the client but wait for user to send 'start'"""
        try:
            self.sock.settimeout(CONNECT_TIMEOUT)
            print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
            self.sock.connect((SERVER_IP, SERVER_PORT))
            self.connected = True
            print("Connection established!")
            print("Type 'start' to activate monitoring")
        except socket.timeout:
            print(f"Connection timed out after {CONNECT_TIMEOUT} seconds")
            return
        except ConnectionRefusedError:
            print("Connection refused - check server status and port")
            return
        except Exception as e:
            print(f"Connection failed: {type(e).__name__} - {e}")
            return
        
        self.running.set()
    
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.start()
        
        # Start receive loop in main thread
        self.receive_loop()

    def stop(self):
        """Stop the client"""
        print("\nStopping client...")
        if self.running == True:
            self.running.clear()
        self.stop_event.set()
        
        if self.connected:
            self.send_command("pon=0")
            self.sock.close()
        
        # Clean up threads
        if self.mon_thread and self.mon_thread.is_alive():
            self.mon_thread.join(timeout=1)
        if self.input_thread and self.input_thread.is_alive():
            self.input_thread.join(timeout=1)
        
        print("Client stopped")

if __name__ == "__main__":
    client = mcuC()
    try:
        client.start()
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()