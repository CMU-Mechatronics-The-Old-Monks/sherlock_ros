import time
from data_manager import DataManager

# Replace with your actual serial port (check using serial.tools.list_ports)
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
NUM_FLOATS = 4  # sending 4 floats: 4 wheel velocities
timeout = 2.0

# Initialize the data manager
manager = DataManager(num_floats=NUM_FLOATS, port=SERIAL_PORT, baudrate=BAUD_RATE)

print("Sending 1.0 rad/s to all wheels...")
manager.pack_and_transmit([1.0, 1.0, 1.0, 1.0])
# Optionally try to receive a response back from Teensy
print("Listening for response from Teensy...")
data = None
start = time.time()
while time.time() - start < timeout:
    if manager.receive_data():
        data = manager.parse_data()
        print(f"Received from Teensy: {data}")
    time.sleep(0.01)

if data is None:
    print("Nothing heard")


time.sleep(2.0)  # Let the Teensy process it for 1 second

print("Sending 0.0 rad/s to all wheels...")
manager.pack_and_transmit([0.0, 0.0, 0.0, 0.0])
print("Listening for response from Teensy...")
start = time.time()
data = None
while time.time() - start < timeout:
    if manager.receive_data():
        data = manager.parse_data()
        print(f"Received from Teensy: {data}")
    time.sleep(0.01)
if data is None:
    print("Nothing heard")