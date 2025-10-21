## Xlerobot

## 1. Install XLeRobot on Pi

Follow the installation and calibration steps from the official docs:  
👉 [https://xlerobot.readthedocs.io/zh-cn/latest/software/index.html](https://xlerobot.readthedocs.io/zh-cn/latest/software/index.html)

### Start the end effector based control
(Activate your env before run the command)
```bash
git clone https://github.com/frodobots-org/teleop-device-sdk.git
cd teleop-device-sdk/plugins/xlerobot
cp 99_xlerobot_teleop_keyboard.py /home/user/lerobot/example/ #one's own local path
python3.10 99_xlerobot_teleop_keyboard.py
```
---

## 2. Running the Teleop Agent on Pi
### Install Dependencies
```bash
apt install -y
apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly libzmq3-dev \
libcurl4-openssl-dev libssl-dev libgstreamer1.0-dev nlohmann-json3-dev cmake
```

### Install and configure the shaking operation
```bash
cd teleop-device-sdk/plugins/xlerobot
cp -r frodobots /opt
```

### Build plugin for xlerobot
```
cd teleop-device-sdk/plugins/xlerobot

#Modify a teleop_ctrl_xlerobot.cc file
```diff
--- a/teleop-device-sdk/plugins/xlerobot/teleop_ctrl_xlerobot.cc
+++ b/teleop-device-sdk/plugins/xlerobot/teleop_ctrl_xlerobot.cc
@@ -279,7 +279,7 @@ void TeleopCtrlXLerobot::Invoke() {
  socket_action_.connect("tcp://<your ip>:5558"); #Change to the local Raspberry Pi's IP address
  socket_observation_.connect("tcp://<your ip>:5556");
  is_running_ = true;
  printf("Connected\n");
  pull_thread_ = std::thread(&TeleopCtrlXLerobot::PullThread, this);
```

mkdir build
cd build
cmake ..
make -j4
cp libteleop_ctrl_xlerobot.so libteleop_media_xlerobot.so /opt/frodobots/lib/
```

### Download Teleop Agent
```bash
curl -sL https://teleop-sdk.s3.ap-southeast-1.amazonaws.com/install.sh | sudo bash
```

### Configure Teleop Agent
Modify a teleop ini file
```bash
[teleop]
token =<your token> 
video = 3
audio = 0
project = lekiwi
record = false

[signal]
cert = /opt/frodobots/cert/cert.pem
key = /opt/frodobots/cert/priv.pem
ca = /opt/frodobots/cert/AmazonRootCA1.pem
device = <your device>

[plugin]
media = /opt/frodobots/lib/libteleop_media_xlerobot.so
ctrl = /opt/frodobots/lib/libteleop_ctrl_xlerobot.so
# camera3: front camera
# camera2: right arm camera
# camera1: left arm camera
camera1 = v4l2src device=/dev/video2 ! image/jpeg, width=640, height=480, framerate=15/1
camera2 = v4l2src device=/dev/video4 ! image/jpeg, width=640, height=480, framerate=15/1
camera3 = v4l2src device=/dev/video0 ! image/jpeg, width=640, height=480, framerate=30/1
```
### Run the Agent

Start the agent using the config file:

```bash
/opt/frodobots/teleop.sh
```
