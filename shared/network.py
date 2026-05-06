import socket
import struct
import errno

__all__ = ["JetsonNetwork", "Esp32Network"]

class Packet:
    """Abstraction for a packet sent over the network"""
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
        data = None
    ):
        self.ack = ack
        self.id = id
        self.length = length
        self.data = data if data is not None else [] # Initialize fresh list

    def encode(self) -> bytes:
        """Encodes a packet into a raw string of bytes"""
        header_byte = bytes([
            ((self.id << self.ID_POS) & self.ID_MASK) |
            ((self.ack << self.ACK_POS) & self.ACK_MASK) |
            ((self.length << self.LEN_POS) & self.LEN_MASK)
        ])

        # > = big endian, H = short
        format_str = ">" + "HH" * self.length

        # Unravel list of Cords into a stream of bytes
        data_bytes = struct.pack(format_str, *(point for cord in self.data for point in cord))
        return header_byte + data_bytes

    def decode(self, raw_packet: bytes, just_header: bool = False):
        """Decodes a raw string of bytes into the packet"""
        header = raw_packet[0]
        self.ack = bool(header & self.ACK_MASK)
        self.id = (header & self.ID_MASK) >> self.ID_POS
        self.length = (header & self.LEN_MASK) >> self.LEN_POS

        if (not just_header and self.length > 0):
            # > = big endian, H = short
            format_str = ">" + "HH" * self.length
            raw_data = struct.unpack(format_str, raw_packet[1:])

            # construct a list of cord pairs from a byte stream
            self.data = [(raw_data[i], raw_data[i+1]) for i in range(0, len(raw_data), 2)]
        return self

class JetsonNetwork:
    """Network class for the Jetson"""
    # Jetson network parameters
    SSID: str = "jetson"
    PWD: str = "12345"
    # Server parameters
    IP: str = "0.0.0.0"
    PORT: int = 60007
    NUM_DEVICES: int = 3

    def __init__(self):
        self.devices: dict[int, socket.socket] = {}
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def start(self, num_devices: int = NUM_DEVICES, ip: str = IP, port: int = PORT):
        """Starts the network interface. Call this prior to calling any other method."""
        self.soc.bind((ip, port))
        self.soc.listen(num_devices)

        while (len(self.devices) < num_devices):
            print("Waiting on connection...")
            conn, _ = self.soc.accept()
            print("Accepted Connection")
            raw_resv = conn.recv(128)
            packet = Packet().decode(raw_resv)
            conn.sendall(Packet(ack=True).encode()) # ack
            self.devices[packet.id] = conn

    def send(self, id: int, data):
        """Sends a packet to the Esp32 with 'id' with the data specified by 'data'"""
        if (id not in self.devices):
            raise ConnectionError(f"Device Id={id} not connected.")
        packet = Packet(length=len(data), data=data)
        self.devices[id].sendall(packet.encode())

    def close(self):
        self.soc.close()
        for soc in self.devices.values():
            soc.close()

    def __del__(self):
        self.close()

class Esp32Network:
    """Represents the netork interface for the Esp32"""
    # Jetson network parameters
    SSID: str = "jetson"
    PWD: str = "12345"
    # Controller parameters
    IP: str = "10.42.0.1" # TODO: change this to ip on subnet the Jetson assigns
    PORT: int = 60007

    def __init__(self, id: int):
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.id: int = id
        self.queue = bytearray()

    def start(self, ip: str = IP, port: int = PORT):
        """Starts the interface. Call this before any other method."""
        print("Connecting...")
        self.soc.connect((ip, port))
        print("Sending init packet..")
        self.soc.sendall(Packet(id=self.id).encode())

        got_ack = False
        print("Waiting on ack..")
        while (not got_ack):
            raw_resv = self.soc.recv(1024)
            packet = Packet().decode(raw_resv)
            if (packet.ack):
                got_ack = True
        print("Registered connection")
        self.soc.setblocking(False) # make non-blocking to allow polling later

    def send(self, data) -> None:
        """Sends a packet with 'data' to the central Jetson Controller"""
        packet = Packet(length=len(data), data=data)
        self.soc.sendall(packet.encode())

    def poll(self) -> bool:
        """Returns true if there is a packet in the receiving queue."""
        try:
            chunk = self.soc.recv(256)
            if chunk:
                self.queue.extend(chunk)
        except OSError as e:
            # non-blocking socket is empty
            if e.args[0] != errno.EAGAIN:
                raise

        if (not self.queue):
            return False

        header = bytes(self.queue[0:1])
        temp_packet = Packet().decode(header, just_header=True)
        packet_length = 1 + 4*temp_packet.length # 1 byte header + 4 bytes for each cord
        return len(self.queue) >= packet_length


    def recv(self) -> Packet | None:
        """Returns a packet of data from the Jetson or None if no packet is in the receive queue"""
        if (not self.poll()):
            return None

        # pop off queue and decode packet
        header = bytes(self.queue[0:1])
        temp_packet = Packet().decode(header, just_header=True)
        packet_byte_len = 1 + 4*temp_packet.length # 1 byte header + 4 bytes for each cord

        # slice off a packet
        packet_bytes = bytes(self.queue[:packet_byte_len])

        # remove packet
        self.queue = self.queue[:packet_byte_len]

        return Packet().decode(packet_bytes)

    def __del__(self):
        self.soc.close()

if __name__ == "__main__":
    # TODO: Example Usage

    pass

