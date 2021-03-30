#!/usr/bin/env python

import socket
import rospy

class Server:
    def __init__(self):
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5005
        self.BUFFER_SIZE = 20  # Normally 1024, but we want fast response

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.TCP_IP, self.TCP_PORT))
        self.sock.listen(1)
        self.receive()

    def receive(self):
        while True:
            conn, addr = self.sock.accept()
            try:
                data: str = conn.recv(self.BUFFER_SIZE).decode()
                if not data: break
                print("Server received: ", data)
                answer = "Received LaserScan data!"
                conn.send(answer.encode())
            finally:
                conn.close()

if __name__=='__main__':
    rospy.init_node('rupp_tcp_server', anonymous=True)
    server = Server()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")