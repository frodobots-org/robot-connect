# plugin for Earth Rover 


## Build Instructions
```
export EARTH_ROVER_OPEN_SOURCE_PATH=<path_to_your_earth_rover_open_source_repo>
git clone https://github.com/frodobots-org/robot-connect.git
cd robot-connect/plugins/earth-rover
mkdir build
cd build
cmake ..
make -j4
```

## Run Instructions

Create a `teleop.ini` file with the following content and modify the token:
```
[teleop]
token = <your token>
video = 1
audio = 0

[plugin]
media = /userdata/libteleop_media_earthrover.so
ctrl = /userdata/libteleop_ctrl_earthrover.so
```

Connect to your Earth Rover via WiFi and run:
```
adb connect <ip_address_of_your_earth_rover>
# install teleop agent
curl -sL https://teleop-sdk.s3.ap-southeast-1.amazonaws.com/install.sh | bash -s -- arm adb
# install plugins
adb push libteleop_media_earthrover.so /data/
adb push libteleop_ctrl_earthrover.so /data/
adb push teleop.ini /data/
# make sure Earth Rover has internet connection
adb shell teleop_agent /data/teleop.ini
```

## Control Instructions
Go to dashboard and choose your device. Connect your gamepad and start driving!
