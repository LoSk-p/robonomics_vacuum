# Robonomics Vacuum

## Requirements

- ROS Melodic (installation instructions [here](http://wiki.ros.org/melodic/Installation/Ubuntu)) 

## installation

Install python packages:
```bash
pip3 install substrate-interface python-miio pyyaml
```
Create workspace and install ROS package:
```bash
mkdir -p vacuum_ws/src
cd vacuum_ws/src
git clone https://github.com/LoSk-p/robonomics_vacuum.git
```
Build the workspace:
```bash
cd vacuum_ws
catkin_make
```

## Roborock

Firstly you need to get an access token to your vacuum. The instructions are [here](https://python-miio.readthedocs.io/en/latest/discovery.html#tokens-from-mi-home-logs).
Also you need your vacuum IP address in your local network. You can find in the `Xiaomi Home` app. Open your vacuum and go to `settings` (three dots in the upper right corner). Address will be in `Additional settings/Network info`.

Then create config file in the config directory:
```bash
cd vacuum_ws/src/config
touch config.yaml
```
And add there information about your the elements you need to monitore (filter, brushes etc.). Information about it you also can find in the `Xiaomi Home` app (in your device settings go to the `Maintenance`). Add to config the name of the element and its working time, also add your mnemonic seed from Robonomics account. The example of the config you can find in [config_template.yaml](config/config_template.yaml) file.

Now you can run the launch file:
```bash
source ~/vacuum_ws/devel/setup.bash
roslaunch robonomics_vacuum vacuum_run.launch address:=<vacuum_address> token:=<vacuum_access_token>
```
