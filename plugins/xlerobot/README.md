## Xlerobot

## Running the Teleop Agent on Pi

### Install Dependencies
```bash
apt install -y
apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly libzmq3-dev \
libcurl4-openssl-dev libssl-dev
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
