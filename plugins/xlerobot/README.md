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

### Install Dependencies
```bash
apt install -y
apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly libzmq3-dev \
libcurl4-openssl-dev libssl-dev libgstreamer1.0-dev nlohmann-json3-dev cmake
```

### Build plugin for xlerobot
```
cd teleop-device-sdk/plugins/xlerobot
mkdir build
cd build
cmake ..
make -j4
```

### Download Teleop Agent
```bash
curl -sL https://teleop-sdk.s3.ap-southeast-1.amazonaws.com/install.sh | sudo bash
```
