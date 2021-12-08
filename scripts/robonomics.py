#!/usr/bin/env python3

import rospy
from robonomics_vacuum.srv import Command
from robonomics_vacuum.utils import get_keypair, robonomics_connect

class RobonomicsControl:
    def __init__(self) -> None:
        self.substrate = robonomics_connect()
        rospy.init_node("robonomics_control")
        self.keypair = get_keypair()
        rospy.wait_for_service("start_cleaning")
        self.start_cleaning = rospy.ServiceProxy("start_cleaning", Command)
        rospy.wait_for_service("pause_cleaning")
        self.pause_cleaning = rospy.ServiceProxy("pause_cleaning", Command)
        rospy.wait_for_service("return_to_base")
        self.return_to_base = rospy.ServiceProxy("return_to_base", Command)


    def subscription_handler(self, obj, update_nr, subscription_id) -> None:
        ch = self.substrate.get_chain_head()
        chain_events = self.substrate.get_events(ch)
        for ce in chain_events:
            # if ce.value["event_id"] == "NewLaunch":
            #     print(ce)
            #     print(self.keypair.ss58_address)
            if ce.value["event_id"] == "NewLaunch" and ce.params[1]["value"] == self.keypair.ss58_address:
                if ce.params[2]["value"] is True:
                    print('"ON" launch command from employer')
                    self.start_cleaning()
                elif ce.params[2]["value"] is False:
                    print('"OFF" launch command from employer')
                    self.pause_cleaning()
                    self.return_to_base()

    def spin(self) -> None:
        self.substrate.subscribe_block_headers(self.subscription_handler)

if __name__ == '__main__':
    RobonomicsControl().spin()