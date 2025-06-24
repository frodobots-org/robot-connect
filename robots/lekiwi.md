## Lekiwi Setup Guide

### 1. Install Lerobot on Pi

Follow the installation and calibration steps from the official docs:  
👉 [https://huggingface.co/docs/lerobot/lekiwi](https://huggingface.co/docs/lerobot/lekiwi)

#### Disable camera in Lerobot  
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

#### Start the Lerobot Host

Once setup and calibration are complete, start the teleoperation host:

```bash
python -m lerobot.common.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi
```

---

### 2. Running the Teleop Agent on Pi

#### Install Dependencies

Install necessary libraries:

```
apt install -y
apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
gstreamer1.0-plugins-ugly libzmq3-dev \
libcurl4-openssl-dev libssl-dev
```

#### Download Teleop Agent

Download [teleop agent](https://github.com/frodobots-org/robots-connect/releases/download/v0.0.1-alpha/teleop-agent.lekiwi.run.tgz) to your Pi.

#### Configure Teleop Agent

- Copy the Lekiwi-specific Teleop config:  
  👉 [lekiwi.teleop.ini](https://github.com/frodobots-org/robots-connect/blob/main/configs/lekiwi.teleop.ini)

- Open the file and update the `token` field with the one from your  
  👉 [Dashboard](https://robots-connect.netlify.app/dashboard)

#### Run the Agent

Start the agent using the config file:

```
./teleop-agent.run -- <absolute path>/lekiwi.teleop.ini
```

---

### 3. Start Controlling Lekiwi

- Go to the 👉 [Robots Connect Dashboard](https://robots-connect.netlify.app/dashboard)
- Click **Drive** to enter the control interface
- You should see:
  - Wrist camera
  - Front camera
- Click **Connect**, and select your **SO-ARM100** device
- (Optional) Calibrate the Robot

  - Click **Calibrate**
  - Move all joints through their full range of motion  
    👉 [Calibration Video](https://huggingface.co/docs/lerobot/en/so101#calibration-video)
  - Click **Calibrating** again to save calibration data
- Click **Play** to begin controlling your Lekiwi 😄

---

