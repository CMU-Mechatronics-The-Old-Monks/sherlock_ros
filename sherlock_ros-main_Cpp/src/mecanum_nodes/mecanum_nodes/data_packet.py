import struct

class DataPacket:
    START_BYTE = 0xAA
    END_BYTE = 0xBB

    def __init__(self, num_floats: int):
        self.num_floats = num_floats
        self.data = [0.0] * num_floats

    def set(self, index: int, value: float):
        if 0 <= index < self.num_floats:
            self.data[index] = value

    def get(self, index: int) -> float:
        if 0 <= index < self.num_floats:
            return self.data[index]
        return 0.0

    def serialize(self) -> bytes:
        # Pack: start byte + floats + end byte
        payload = struct.pack(f'<{self.num_floats}f', *self.data)
        return bytes([self.START_BYTE]) + payload + bytes([self.END_BYTE])

    def deserialize(self, byte_array: bytes) -> bool:
        if byte_array[0] != self.START_BYTE or byte_array[-1] != self.END_BYTE:
            return False

        payload = byte_array[1:-1]
        if len(payload) != self.num_floats * 4:
            return False

        self.data = list(struct.unpack(f'<{self.num_floats}f', payload))
        return True

    def byte_length(self) -> int:
        return 1 + self.num_floats * 4 + 1

    def get_data(self) -> list:
        return self.data.copy()

    def __repr__(self):
        return f"<DataPacket {self.data}>"
