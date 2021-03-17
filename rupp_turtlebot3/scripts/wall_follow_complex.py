#!/usr/bin/env python
import rospy
import math
from operator import itemgetter
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

# code from turtlebot3_examples -> nodes -> turtlebot3_obstacle
class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()

    def get_scan_try(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
        rf = scan.ranges[9]
        rdf = scan.ranges[44]
        r = scan.ranges[89]
        rdb = scan.ranges[134]
        rb = scan.ranges[169]
        lb = scan.ranges[189]
        ldb = scan.ranges[224]
        l = scan.ranges[269]
        ldf = scan.ranges[314]
        lf = scan.ranges[349]
        scan_filter.append(rf)
        scan_filter.append(rdf)
        scan_filter.append(r)
        scan_filter.append(rdb)
        scan_filter.append(rb)
        scan_filter.append(lb)
        scan_filter.append(ldb)
        scan_filter.append(l)
        scan_filter.append(ldf)
        scan_filter.append(lf)
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            # order: rf, rdf, r, rdb, rb, lb, ldb, l, ldf, lf
            lidar_distances = self.get_scan_try()
            min_distance = min(lidar_distances)
            min_index = min(enumerate(lidar_distances), key=itemgetter(1))[0]
            lf = lidar_distances[0]
            ldf = lidar_distances[1]
            l = lidar_distances[2]
            ldb = lidar_distances[3]
            lb = lidar_distances[4]
            rb = lidar_distances[5]
            rdb = lidar_distances[6]
            r = lidar_distances[7]
            rdf = lidar_distances[8]
            rf = lidar_distances[9]
            left_turn = math.pi/2
            small_left_turn = math.pi/4
            if lf < SAFE_STOP_DISTANCE:
                print("LF")
                twist.linear.x = 0.0
                twist.angular.z = left_turn
                self._cmd_pub.publish(twist)
            elif rf < SAFE_STOP_DISTANCE:
                print("RF")
                twist.linear.x = 0.0
                twist.angular.z = -left_turn
                self._cmd_pub.publish(twist)
            elif r < SAFE_STOP_DISTANCE:
                print("R")
                twist.linear.x = LINEAR_VEL
                twist.angular.z = -math.pi/16
                self._cmd_pub.publish(twist)
            elif l < SAFE_STOP_DISTANCE:
                print("L")
                twist.linear.x = LINEAR_VEL
                twist.angular.z = math.pi/16
                self._cmd_pub.publish(twist)
            elif rdf < SAFE_STOP_DISTANCE:
                print("RDF")
                twist.linear.x = LINEAR_VEL//2
                twist.angular.z = -small_left_turn
                self._cmd_pub.publish(twist)
            elif ldf < SAFE_STOP_DISTANCE:
                print("LDF")
                twist.linear.x = LINEAR_VEL//2
                twist.angular.z = small_left_turn
                self._cmd_pub.publish(twist)
            elif lb < SAFE_STOP_DISTANCE:
                print("LB")
                twist.linear.x = 0.0
                twist.angular.z = math.pi
                self._cmd_pub.publish(twist)
            elif rb < SAFE_STOP_DISTANCE:
                print("RB")
                twist.linear.x = 0.0
                twist.angular.z = math.pi
                self._cmd_pub.publish(twist)
            elif ldb < SAFE_STOP_DISTANCE:
                print("LDB")
                twist.linear.x = 0.0
                twist.angular.z = math.pi*(5//4)
                self._cmd_pub.publish(twist)
            elif rdb < SAFE_STOP_DISTANCE:
                print("RDB")
                twist.linear.x = 0.0
                twist.angular.z = -math.pi*(5//4)
                self._cmd_pub.publish(twist)
            else:
                print("Going straight...")
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)

def main():
    rospy.init_node('turtlebot3_obstacle_test')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()