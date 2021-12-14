# Robonomics Vacuum

## Docker installation
 Run package with docker (you need [installed docker](https://docs.docker.com/engine/install/ubuntu/))

### Installation
Clone the package:
```bash
git clone https://github.com/LoSk-p/robonomics_vacuum.git
```
### Configuration for Roborock

Firstly you need to get an access token to your vacuum. The instructions are [here](https://python-miio.readthedocs.io/en/latest/discovery.html#tokens-from-mi-home-logs).
Also you need your vacuum IP address in your local network. You can find in the `Xiaomi Home` app. Open your vacuum and go to `settings` (three dots in the upper right corner). Address will be in `Additional settings/Network info`.

Then create config file in the config directory:
```bash
cd ~/robonomics_vacuum/config
nano config.yaml
```
And add there information about your the elements you need to monitore (filter, brushes etc.). Information about it you also can find in the `Xiaomi Home` app (in your device settings go to the `Maintenance`). Add to config the name of the element and its working time, also add your mnemonic seed from Robonomics account. The example of the config you can find in [config_template.yaml](config/config_template.yaml) file.

### Build
Build the docker container:
```bash
cd ~/robonomics_vacuum
docker build -t robonomics_vacuum .
```
### Run
Now you can run it with thw vacuum address and access token:
```bash
docker run -v data:/root/vacuum_ws/src/robonomics_vacuum/data --name robonomics_vacuum robonomics_vacuum <vacuum_address> <vacuum_access_token>
```
You can access this container with:
```bash
docker exec -ti robonomics_vacuum /bin/bash
```

## Manual installation

### Requirements
- ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

### installation

Create workspace and install ROS package:
```bash
mkdir -p vacuum_ws/src
cd ~/vacuum_ws/src
git clone https://github.com/LoSk-p/robonomics_vacuum.git
```
Install python packages:
```bash
cd robonomics_vacuum
pip3 install -r requirements.txt
```
Build the workspace:
```bash
cd ~/vacuum_ws
catkin_make
```
Create and full the `config.yaml` file like in [Configuration for Roborock](#configuration-for-roborock).
### Run
Now you can run the launch file:
```bash
source ~/vacuum_ws/devel/setup.bash
roslaunch robonomics_vacuum vacuum_run.launch address:=<vacuum_address> token:=<vacuum_access_token>
```

## Usage

You can see the information about vacuum status in /roborock/roborock_status topic:
```bash
rostopic echo /roborock/roborock_status
```
And control vacuum with services: `start_cleaning`, `pause_cleaning`, `return_to_base`. For example:
```bash
rosservice call /roborock/start_cleaning
```
Or with Robonomics ON/OFF launch transactions to the vacuum address.
Also use `replace_element` service when you replace or clean the element of your vacuum:
```bash
rosservice call /roborock/replace_element "element: '<element_name>'"
```