import socket
import threading
import time

# ひたすらデータ要求が来たら返すサーバー
class DataServer:
    def __init__(self, host_address, host_port):
        self.host_address = host_address
        self.host_port = host_port
        self.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        self.socket.bind((self.host_address, self.host_port))
        self.connected_history=[]
        self.max_connect_num = 1
        self.send_data_str = "this is data"

    def startAndDetach(self):
        self.server_thread = threading.Thread(target=self.start)
        self.server_thread.start()

    def start(self):
        self.socket.listen(self.max_connect_num)
        print(f"waiting connection address={self.host_address}, port={self.host_port}")
        connection, address = self.socket.accept()
        with connection:
            print(f"connected from {address}")
            self.connected_history.append(address)
            while True:
                try:
                    data = connection.recv(1024)
                    if not data:
                        # 接続が切れたり、中身のあるデータが送られてこなかった場合は再接続
                        break
                except ConnectionResetError as e:
                    pass
                lock = threading.Lock()
                lock.acquire()
                send_data_str = self.send_data_str
                lock.release() 
                try:
                    connection.sendall(send_data_str.encode())
                except BrokenPipeError as e:
                    pass
        # 接続が切れたら再びlisten
        self.start()

    def setSendData(self, data_str):
        self.send_data_str = data_str

def main():
    sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
    host = "192.168.12.50"
    port = 20021
    data_server = DataServer(host, port)
    data_server.startAndDetach()
    print("start")
    while True:
        print("hoge")
        time.sleep(1)
 
    
if __name__ == "__main__":
    main()
