#!/usr/bin/env python3

import yaml
import typing as tp
from substrateinterface import SubstrateInterface, Keypair
import rospkg
import rospy

def read_config(path: str) -> tp.Dict:
        with open(path) as f:
            config = yaml.safe_load(f)
        return config

def robonomics_connect() -> SubstrateInterface:
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

def get_keypair() -> Keypair:
    rospack = rospkg.RosPack()
    path = rospack.get_path('robonomics_vacuum')
    config = read_config(f"{path}/config/config.yaml")
    mnemonic = config['robonomics_seed']
    keypair = Keypair.create_from_mnemonic(mnemonic, ss58_format=32)
    return keypair

def write_datalog(substrate: SubstrateInterface, keypair: Keypair, data: str) -> str:
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
