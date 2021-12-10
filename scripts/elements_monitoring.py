#!/usr/bin/env python3

import rospy
import rospkg
import typing as tp
import yaml
from robonomics_vacuum.srv import Element
from miio import RoborockVacuum
import os
import datetime
from robonomics_vacuum.utils import read_config

class ElementsMonitoring:
    def __init__(self) -> None:
        rospy.init_node("elements_monitoring")
        address = rospy.get_param("~address")
        token = rospy.get_param("~token")
        self.vacuum = RoborockVacuum(address, token)
        history = self.vacuum.clean_history()
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('robonomics_vacuum')
        self.default_elements = read_config(f"{self.path}/config/config.yaml")
        if not os.path.exists(f"{self.path}/data/cleaning_info.yaml"):
            elements = {'number_cleaning': history.count, 'elements': []}
            for element in self.default_elements['elements']:
                elements['elements'].append({'name': element['name'], 'time_from_last_replace': 0})
            self.rewrite_yaml(path=f"{self.path}/data/cleaning_info.yaml", data=elements)
        rospy.Service("replace_element", Element, self.replace_element)
        rospy.wait_for_service("write_datalog")
        self.write_datalog = rospy.ServiceProxy("write_datalog", Element)
        # rospy.Subscriber("roborock_status", RoborockStatus, self.listener)
    
    def rewrite_yaml(self, path: str, data: tp.List) -> None:
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

    def replace_element(self, req) -> None:
        rospy.loginfo(f"type req: {type(req)}")
        elements = read_config(f"{self.path}/data/cleaning_info.yaml")
        for element in elements['elements']:
            if element['name'] == req.element:
                element['time_from_last_replace'] = 0
        self.rewrite_yaml(path=f"{self.path}/data/cleaning_info.yaml", data=elements)
        return "OK"

    def send_message(self, element: str) -> None:
        rospy.loginfo(f"Creating datalog with message: \"You should replace element {element}\"")
        self.write_datalog(data=f"You should replace element {element}")
    
    def spin(self) -> None:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            history = self.vacuum.clean_history()
            config = read_config(f"{self.path}/data/cleaning_info.yaml")
            clean_time = datetime.timedelta(hours=0, minutes=0, seconds=0)
            if history.count > config['number_cleaning']: 
                for i in range(history.count - config['number_cleaning']):
                    clean = self.vacuum.clean_details(history.ids[-1-i])
                    clean_time += clean.duration
                elements = config
                for element in elements['elements']:
                    last_delta = int(element['time_from_last_replace'])
                    last_delta += clean_time.seconds
                    element['time_from_last_replace'] = last_delta
                    for default_element in self.default_elements['elements']:
                        if element['name'] == default_element['name']:
                            if element['time_from_last_replace']/3600 > default_element['working_time']:
                                self.send_message(element['name'])
                elements['number_cleaning'] = history.count
                self.rewrite_yaml(path=f"{self.path}/data/cleaning_info.yaml", data=elements)


if __name__ == '__main__':
    ElementsMonitoring().spin()
        
