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

    def receive(self):
        conn, addr = self.sock.accept()
        print('Connection address: ', addr)
        while 1:
            data: str = conn.recv(self.BUFFER_SIZE).decode()
            if not data: break
            print("received data: ", data)
            conn.send(data.encode())  # echo
        conn.close()

if __name__=='__main__':
    rospy.init_node('rupp_tcp_server', anonymous=True)
    server = Server()
    server.receive()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")