import serial
from mecanum_nodes.data_packet import DataPacket

class DataManager:
    def __init__(self, num_floats: int, port: str, baudrate: int = 115200, timeout: float = 0.01):
        self.packet = DataPacket(num_floats)
        self.expected_length = self.packet.byte_length()
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def pack_data(self, values: list[float]):
        for i, val in enumerate(values):
            self.packet.set(i, val)

    def parse_data(self) -> list[float]:
        return self.packet.get_data()

    def transmit_data(self):
        self.ser.write(self.packet.serialize())

    def receive_data(self) -> bool:
        if self.ser.in_waiting >= self.expected_length:
            data = self.ser.read(self.expected_length)
            return self.packet.deserialize(data)
        return False

    def pack_and_transmit(self, values: list[float]):
        self.pack_data(values)
        self.transmit_data()
