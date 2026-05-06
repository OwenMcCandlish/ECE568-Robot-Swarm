import socket
import json
import time

class Message:
    def __init__(self, data):
        self.data = data

class Esp32Network:
    def __init__(self, r_id):
        self.r_id = r_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))
        self.sock.setblocking(False)
        import config
        self.pi_ip = getattr(config, 'PI_IP', '192.168.1.100')
        self.pi_port = 5005
        self.msg_queue = []
        self.last_hello = 0

    def start(self):
        self.send_hello()

    def send_hello(self):
        try:
            msg = json.dumps({"r_id": self.r_id}).encode('utf-8')
            self.sock.sendto(msg, (self.pi_ip, self.pi_port))
        except Exception as e:
            print("Failed to send HELLO:", e)

    def poll(self):
        has_new = False
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                if "data" in msg:
                    self.msg_queue.append(Message(msg["data"]))
                    has_new = True
        except OSError:
            pass # No more data available
        
        # Periodically send HELLO every 2 seconds to ensure Pi knows our IP
        if time.time() - self.last_hello > 2:
            self.send_hello()
            self.last_hello = time.time()

        return has_new

    def recv(self):
        if self.msg_queue:
            return self.msg_queue.pop(-1) # Get the latest message, skip old ones
        return Message([])

class JetsonNetwork:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))
        self.sock.setblocking(False)
        self.clients = {} # Maps bot_id to (ip, port)

    def start(self, num_devices):
        print(f"[Network] Server started on UDP 5005. Waiting for devices to connect...")
        
    def poll_for_clients(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                msg = json.loads(data.decode('utf-8'))
                if "r_id" in msg:
                    r_id = msg["r_id"]
                    if r_id not in self.clients:
                        print(f"[Network] Robot {r_id} connected from {addr}")
                    self.clients[r_id] = addr
        except Exception:
            pass # No data

    def send(self, bot_id, data):
        self.poll_for_clients()
        if bot_id in self.clients:
            msg = json.dumps({"data": data}).encode('utf-8')
            try:
                self.sock.sendto(msg, self.clients[bot_id])
                return True
            except Exception as e:
                print(f"[Network] Error sending to Robot {bot_id}: {e}")
                return False
        else:
            print(f"[Network] Robot {bot_id} not connected yet. Cannot send.")
            return False
