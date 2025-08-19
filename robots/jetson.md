## Jetson Setup Guide

### Compile your plugins
```
git clone https://github.com/frodobots-org/robot-connect.git
cd plugins/jetson
mkdir build
cd build
cmake ..
make
```

### Install teleop agent
```
curl -sL https://teleop-sdk.s3.ap-southeast-1.amazonaws.com/install.sh | sudo bash
```

#### Configure Teleop Agent

- Copy the Teleop config:  
  👉 [jetson.teleop.ini](https://github.com/frodobots-org/robots-connect/blob/main/configs/jetson.teleop.ini)

- Open the file and update the `token` field with the one from your  
  👉 [Dashboard](https://robots-connect.netlify.app/dashboard)

#### Run the Agent

Start the agent using the config file:

```
./teleop_agent <your path>/teleop.ini
```

---

### 3. Start Controlling
- Go to the 👉 [Robot Connect Dashboard](https://robots-connect.netlify.app/dashboard)
- Click **Drive** to enter the control interface
