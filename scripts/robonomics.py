#!/usr/bin/env python3

from substrateinterface import SubstrateInterface, Keypair
import rospy
import rospkg
from robonomics_vacuum.utils import read_config
from robonomics_vacuum.srv import Command

class RobonomicsControl:
    def __init__(self) -> None:
        self.substrate = self.connect()
        rospy.init_node("robonomics_control")
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('robonomics_vacuum')
        config = read_config(f"{self.path}/config/config.yaml")
        mnemonic = config['robonomics_seed']
        self.keypair = Keypair.create_from_mnemonic(mnemonic)
        rospy.wait_for_service("start_cleaning")
        self.start_cleaning = rospy.ServiceProxy("start_cleaning", Command)
        rospy.wait_for_service("pause_cleaning")
        self.pause_cleaning = rospy.ServiceProxy("pause_cleaning", Command)
        rospy.wait_for_service("return_to_base")
        self.return_to_base = rospy.ServiceProxy("return_to_base", Command)

    def connect(self) -> SubstrateInterface:
        substrate = SubstrateInterface(
            url="wss://main.frontier.rpc.robonomics.network",
            ss58_format=32,
            type_registry_preset="substrate-node-template",
            type_registry= {
                "types": {
                    "Record": "Vec<u8>",
                    "Parameter": "Bool",
                    "LaunchParameter": "Bool",
                    "<T as frame_system::Config>::AccountId": "AccountId",
                    "RingBufferItem": {
                        "type": "struct",
                        "type_mapping": [["timestamp", "Compact<u64>"], ["payload", "Vec<u8>"]],
                    },
                    "RingBufferIndex": {
                        "type": "struct",
                        "type_mapping": [["start", "Compact<u64>"], ["end", "Compact<u64>"]],
                    },
                }
            }
            )
        return substrate

    def write_datalog(self, data: str) -> str:
        call = self.substrate.compose_call(
            call_module="Datalog",
            call_function="record",
            call_params={
                'record': data
            }
        )
        extrinsic = self.substrate.create_signed_extrinsic(call=call, keypair=self.keypair)
        receipt = self.substrate.submit_extrinsic(extrinsic, wait_for_inclusion=True)
        rospy.loginfo(f"Datalog created with extrinsic hash: {receipt.extrinsic_hash}")
        return receipt

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