#!/usr/bin/env python

import socket
import rospy
import sys, getopt
from sensor_msgs.msg import LaserScan

class Client:
    def __init__(self, name):
        self.TCP_IP = '127.0.0.1'
        self.TCP_PORT = 5005
        self.BUFFER_SIZE = 1024
        full_sub = name + "/scan"
        self.sub = rospy.Subscriber(full_sub, LaserScan, self.scan_callback)

    def send(self, message):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self.TCP_IP, self.TCP_PORT))
        sock.send(message.encode())
        data = sock.recv(self.BUFFER_SIZE)
        sock.close()
        print("Client received: ", data)
    
    def scan_callback(self, scan:LaserScan):
        self.send("Here is the LaserScan data!!!")
    
def main(argv):
    name = ''
    try:
        opts, args = getopt.getopt(argv,"hn:",["name="])
    except getopt.GetoptError:
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            sys.exit()
        elif opt in ("-n", "--name"):
            name = arg
    rospy.init_node('rupp_tcp_client', anonymous=True)
    client = Client(name)
    # client.send("SENDING DATA")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__=='__main__':
    main(sys.argv[1:])
    
    

