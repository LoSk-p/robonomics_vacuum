#!/usr/bin/env python3

from include.robonomics_vacuum.utils import write_datalog
import rospy
from miio import RoborockVacuum, VacuumStatus
from robonomics_vacuum.msg import RoborockStatus
from robonomics_vacuum.srv import Command

class Roborock:
    def __init__(self) -> None:
        rospy.init_node("roborock_vacuum", anonymous=True)
        self.pub_state = rospy.Publisher("roborock_status", RoborockStatus, queue_size=10)
        rospy.Service("start_cleaning", Command, self.start_cleaning)
        rospy.Service("pause_cleaning", Command, self.pause_cleaning)
        rospy.Service("return_to_base", Command, self.return_to_base)
        address = rospy.get_param("~address")
        token = rospy.get_param("~token")
        self.vacuum = RoborockVacuum(address, token)
        self.last_state = "Charging"

    def start_cleaning(self, req) -> bool:
        try:
            response = self.vacuum.start()
            if response == ['OK']:
                rospy.loginfo(f"Cleaning started")
                return True
            else:
                rospy.logerr(f"Start failed with code {response}")
                return False
        except Exception as e:
            rospy.logerr(f"Start failed: {e}")
            return False

    def pause_cleaning(self, req) -> bool:
        try:
            response = self.vacuum.pause()
            if response == ['OK']:
                rospy.loginfo(f"Cleaning paused")
                return True
            else:
                rospy.logerr(f"Pause failed with code {response}")
                return False
        except Exception as e:
            rospy.logerr(f"Pause failed: {e}")
            return False

    def return_to_base(self, req) -> bool:
        try:
            response = self.vacuum.home()
            if response == ['OK']:
                rospy.loginfo(f"Returning to base started")
                return True
            else:
                rospy.logerr(f"Returning to base failed with code {response}")
                return False
        except Exception as e:
            rospy.logerr(f"Returning to base failed: {e}")
            return False

    def get_msg_status(self, data: VacuumStatus) -> RoborockStatus:
        status_msg = RoborockStatus()
        status_msg.battery = data.battery
        status_msg.clean_area = data.clean_area
        status_msg.clean_time = str(data.clean_time)
        status_msg.error = data.error
        status_msg.fanspeed = data.fanspeed
        status_msg.got_error = data.got_error
        status_msg.in_segment_cleaning = data.in_segment_cleaning
        status_msg.in_zone_cleaning = data.in_zone_cleaning
        status_msg.is_on = data.is_on
        status_msg.is_paused = data.is_paused
        status_msg.map = data.map
        status_msg.state = data.state
        status_msg.state_code = data.state_code
        return status_msg

    def spin(self) -> None:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            status = self.vacuum.status()
            status_msg = self.get_msg_status(status)
            self.pub_state.publish(status_msg)
            if self.last_state != status.state:
                write_datalog(f"State changed from {self.last_state} to {status.state}")
                self.last_state = status.state
            rate.sleep()

if __name__ == '__main__':
    Roborock().spin()