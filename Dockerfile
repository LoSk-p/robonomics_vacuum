FROM ros:melodic-ros-base-bionic
WORKDIR /root/vacuum_ws/src/robonomics_vacuum
COPY config/config.yaml config/config.yaml
ADD include include
ADD launch launch
ADD msg msg
ADD srv srv
COPY scripts/elements_monitoring.py scripts/elements_monitoring.py
COPY scripts/robonomics.py scripts/robonomics.py
COPY scripts/roborock.py scripts/roborock.py 
COPY CMakeLists.txt CMakeLists.txt
COPY package.xml package.xml
COPY setup.py setup.py
COPY roslaunch_run.sh roslaunch_run.sh
COPY requirements.txt requirements.txt
RUN /bin/bash -c 'apt-get update; sudo apt-get install -y python3-pip; python3 -m pip install --upgrade pip'
RUN /bin/bash -c 'python3 -m pip install -r requirements.txt'
WORKDIR /root/vacuum_ws
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash; catkin_make' 
WORKDIR /root/vacuum_ws/src/robonomics_vacuum
RUN /bin/bash -c 'chmod +x roslaunch_run.sh'
ENTRYPOINT ["/root/vacuum_ws/src/robonomics_vacuum/roslaunch_run.sh"]