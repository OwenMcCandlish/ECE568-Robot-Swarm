import socket
import struct
from itertools import batched, chain

__all__ = ["Network"]

Cords = tuple[int, int]

class Packet:
    # Constants
    ACK_MASK = 0b1000_0000
    ACK_POS  = 7
    ID_MASK  = 0b0111_0000
    ID_POS   = 4
    LEN_MASK = 0b0000_0111
    LEN_POS  = 0

    def __init__(
        self, *,
        ack: bool = False,
        id: int = -1,
        length: int = 0,
        data: list[Cords] = []
    ):
        self.ack = ack
        self.id = id
        self.length = length
        self.data = data

    def encode(self) -> bytes:
        header_byte = bytes([
            ((self.id << self.ID_POS) & self.ID_MASK) |
            ((self.ack << self.ACK_POS) & self.ACK_MASK) |
            ((self.length << self.LEN_POS) & self.LEN_MASK)
        ])
        format_str = ">" + "HH" * self.length
        data_bytes = struct.pack(format_str, *list(chain.from_iterable(self.data)))
        return header_byte + data_bytes

    def decode(self, raw_packet: bytes):
        header = raw_packet[0]
        self.ack = bool(header & self.ACK_MASK)
        self.id = (header & self.ID_MASK) >> self.ID_POS
        self.length = (header & self.LEN_MASK) >> self.LEN_POS

        format_str = ">x" + "HH" * self.length
        raw_data = struct.unpack(format_str, raw_packet)
        self.data = list(batched(raw_data, 2))
        return self

class Network:
    # Jetson network parameters
    SSID: str = "jetson"
    PWD: str = "12345"
    # Server parameters
    IP: str = "10.42.0.1" # TODO: change this to ip on subnet the Jetson assigns
    PORT: int = 60007
    NUM_DEVICES: int = 3

    def __init__(self):
        self.devices: dict[int, socket.socket] = {}
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def start(self, num_devices: int = NUM_DEVICES, ip: str = IP, port: int = PORT):
        self.soc.bind((ip, port))
        self.soc.listen(num_devices)

        while (len(self.devices) < num_devices):
            conn, _ = self.soc.accept()
            raw_resv = conn.recv(128)
            packet = Packet().decode(raw_resv)
            conn.sendall(Packet(ack=True).encode()) # ack
            self.devices[packet.id] = conn

    def send(self, id: int, data: list[Cords]):
        if (id not in self.devices):
            raise ConnectionError(f"Device Id={id} not connected.")
        packet = Packet(length=len(data), data=data)
        self.devices[id].sendall(packet.encode())

    def __del__(self):
        self.soc.close()
        for soc in self.devices.values():
            soc.close()


if __name__ == "__main__":
    # Example Usage
    pass

