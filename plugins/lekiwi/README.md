# Lekiwi

## 1. Install Lerobot on Pi

Follow the installation and calibration steps from the official docs:  
👉 [https://huggingface.co/docs/lerobot/lekiwi](https://huggingface.co/docs/lerobot/lekiwi)

### Disable camera in Lerobot  
The camera will be managed by `teleop_agent`, so disable it in the source code:

```diff
--- a/lerobot/common/robots/lekiwi/lekiwi.py
+++ b/lerobot/common/robots/lekiwi/lekiwi.py
@@ -71,7 +71,7 @@ class LeKiwi(Robot):
         )
         self.arm_motors = [motor for motor in self.bus.motors if motor.startswith("arm")]
         self.base_motors = [motor for motor in self.bus.motors if motor.startswith("base")]
-        self.cameras = make_cameras_from_configs(config.cameras)
+        self.cameras = {} #make_cameras_from_configs(config.cameras)

--- a/lerobot/common/robots/lekiwi/lekiwi_host.py
+++ b/lerobot/common/robots/lekiwi/lekiwi_host.py
@@ -110,7 +110,7 @@ def main():
             elapsed = time.time() - loop_start_time
 
             time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))
-            duration = time.perf_counter() - start
+            #duration = time.perf_counter() - start
         print("Cycle time reached.")
 
     except KeyboardInterrupt:
```

### Start the Lekiwi host
(Activate your env before run the command)
```bash
python -m lerobot.common.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi
```

### Start the end effector based control
(Activate your env before run the command)
```bash
git clone https://github.com/frodobots-org/teleop-device-sdk.git
cd teleop-device-sdk/plugins/lekiwi
python teleoperator.py
```
---

## 2. Running the Teleop Agent on Pi

### Install Dependencies
```bash
apt install -y
apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly libzmq3-dev \
libcurl4-openssl-dev libssl-dev
```

### Build plugin for lekiwi
```
cd teleop-device-sdk/plugins/lekiwi
mkdir build
cd build
cmake ..
make -j4
```

### Download Teleop Agent
```bash
curl -sL https://teleop-sdk.s3.ap-southeast-1.amazonaws.com/install.sh | sudo bash
```

### Configure Teleop Agent
Create a teleop ini file
```bash
[teleop]
token = <your token>
video = 2
audio = 0

[plugin]
media = <your plugins path>/libteleop_media_gst.so
ctrl = <your plugins path>/libteleop_ctrl_zmq.so
pipeline1 = v4l2src device=/dev/video0 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=60 bitrate=2048 bframes=0 ! appsink name=sink sync=false
pipeline2 = v4l2src device=/dev/video2 ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=60 bitrate=1024 bframes=0 ! appsink name=sink sync=false
```

### Run the Agent

Start the agent using the config file:

```
teleop_agent <your teleop.ini file>
```

---

## 3. Start Controlling Lekiwi
- Go to the 👉 [Robots Connect Dashboard](https://robots-connect.netlify.app/dashboard)
- Click **Drive** to enter the control interface

## 4. Join [TeleArms](https://telearms.com/) subnet (Optional)
- Contact us to get your token for TeleArms
