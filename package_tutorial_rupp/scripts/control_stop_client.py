#!/usr/bin/env python
import sys
import rospy 
from package_tutorial_rupp.srv import Stop_control

class stop_control:
    def __init__(self):
        self.srv_wait = rospy.wait_for_service('rupp/stop_control')
        

    def set_stop(self, stop):
        try:
            self.srv_set = rospy.ServiceProxy('rupp/stop_control', Stop_control)
            self.response = self.srv_set(stop);
            return self.response.stopped
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def usage():
    return "%s 0 for stop, any other integer continues%sys.argv[0]

if __name__ == "__main__":
    control = stop_control
    if len(sys.argv) == 2:
        stop = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting to stop/start control %s"%stop)
    print(control.set_stop(control, stop))
