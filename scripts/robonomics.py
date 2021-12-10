#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from robonomics_vacuum.srv import Command
from robonomics_vacuum.srv import Element
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
        rospy.Service("write_datalog", Element, self.write_datalog)
        rospy.Subscriber("datalog", String, self.write_datalog)

    def write_datalog(self, data) -> str:
        rospy.loginfo(f"Got message to write datalog: {data}")
        substrate = robonomics_connect()
        keypair = get_keypair()
        try:
            call = substrate.compose_call(
                call_module="Datalog",
                call_function="record",
                call_params={
                    'record': data
                }
            )
            extrinsic = substrate.create_signed_extrinsic(call=call, keypair=keypair)
            receipt = substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
            rospy.loginfo(f"Datalog created with extrinsic hash: {receipt.extrinsic_hash}")
            return receipt.extrinsic_hash
        except Exception as e:
            rospy.loginfo(f"Can't send datalog with error {e}")
            return "Failed sending datalog"

    def subscription_handler(self, obj, update_nr, subscription_id) -> None:
        ch = self.substrate.get_chain_head()
        chain_events = self.substrate.get_events(ch)
        for ce in chain_events:
            if ce.value["event_id"] == "NewLaunch":
                rospy.loginfo(ce.params[1]["value"])
                rospy.loginfo(self.keypair.ss58_address)
            if ce.value["event_id"] == "NewLaunch" and ce.params[1]["value"] == self.keypair.ss58_address:
                if ce.params[2]["value"] is True:
                    rospy.loginfo('"ON" launch command from employer')
                    self.start_cleaning()
                elif ce.params[2]["value"] is False:
                    rospy.loginfo('"OFF" launch command from employer')
                    self.pause_cleaning()
                    self.return_to_base()

    def spin(self) -> None:
        self.substrate.subscribe_block_headers(self.subscription_handler)

if __name__ == '__main__':
    RobonomicsControl().spin()