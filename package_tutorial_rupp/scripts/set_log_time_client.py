#!/usr/bin/env python
import sys
import rospy 
from package_tutorial_rupp.srv import Set_Log_Time

class log_time_setter:
    def __init__(self):
        self.srv_wait = rospy.wait_for_service('rupp/set_log_time')
        

    def set_time(self, new_time):
        try:
            self.srv_set = rospy.ServiceProxy('rupp/set_log_time', Set_Log_Time)
            self.response = self.srv_set(new_time);
            return self.response.was_set
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def usage():
    return "%s New_time"%sys.argv[0]

if __name__ == "__main__":
    log_set = log_time_setter
    if len(sys.argv) == 2:
        entered_time = float(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting to set log time to %s"%entered_time)
    print(log_set.set_time(log_set, entered_time))
