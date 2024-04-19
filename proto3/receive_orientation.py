import socket
import threading
import time


class Receiver:
    def __init__(self, server_address, server_port):
        self.address = server_address
        self.port = server_port
        self.sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        self.sock.connect((self.address, self.port))
        self.quat = [1.0, 0.0, 0.0, 0.0]
        self.t = None
        self.is_running = True

    def start(self):
        self.t = threading.Thread(target=self.loop)
        self.t.start()
        
    def loop(self):
        with open("quat_log.txt", mode="w") as f:
            f.write("")
        self.quat_vec = []
        self.start_time = time.time()
        self.cycle_interval = 0.05
        while self.is_running:
            cycle_start_time = time.time()
            self.sock.sendall(b"nyan")
            data = self.sock.recv(1024)
            data_str = data.decode()
            if data_str == "this is data":
                continue
            data_float_list = [float(x) for x in data_str.split()]

            if len(data_float_list) != 7:
                continue
            self.quat = data_float_list[:4]
            with open("quat_log.txt", mode="a") as f:
                f.write(f"{time.time() - self.start_time} {self.quat[0]} {self.quat[1]} {self.quat[2]} {self.quat[3]}\n")
            cycle_time = time.time() - cycle_start_time
            if (self.cycle_interval - cycle_time < 0):
                print(f"cycle_time exceeds {self.cycle_interval}, dt={cycle_time}")
            time.sleep(self.cycle_interval-cycle_time)
    
    def stop(self):
        print("stop")
        self.is_running = False
        self.t.join()

def main():
    receiver = Receiver("192.168.10.50", 20021)
    receiver.start()
    time.sleep(5)
    receiver.stop()

if __name__ == "__main__":
    main()
