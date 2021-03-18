#!/usr/bin/env python

import socket
import rospy

class Client:
    def __init__(self):
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5005
        self.BUFFER_SIZE = 1024
        self.MESSAGE = "Hello, World!"
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def send(self, message):
        self.sock.connect((self.TCP_IP, self.TCP_PORT))
        self.sock.send(message.encode())
        data = self.sock.recv(self.BUFFER_SIZE)
        self.sock.close()
        print("received data: ", data)

if __name__=='__main__':
    rospy.init_node('rupp_tcp_client', anonymous=True)
    client = Client()
    client.send("SENDING DATA")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

