#include <zmq.hpp>
#include <string>
#include <iostream>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <map>
#include <nlohmann/json.hpp>
#include <teleop_ctrl_plugin.h>

using json = nlohmann::json;

class TeleopCtrlZmq : public TeleopCtrlPlugin {
 public:
  TeleopCtrlZmq();
  void Invoke() override;
  void Terminate() override;
  void OnMessageReceived(const char* message) override;
  void HandleKeyUpdate(const json& key_action,
                       const std::string& key_name,
	               const std::string& map_key,
	               double delta);
  void HandleKeyUpdate(const json& key_action,
                       const std::string& key_name,
		       int index,
	               const std::string& map_key,
                       double delta,
		       bool without_delta = false);
  void ProcessKeyboardAction(const json& action, std::string& action_string);
  void ProcessGamepadAction(const json& action, std::string& action_string);

 private:
  void PullThread();
  zmq::context_t context_;
  zmq::socket_t socket_action_;
  zmq::socket_t socket_observation_;
  std::atomic<bool> is_running_ = false;
  std::thread pull_thread_;
  json action_;
  json observation_;
  std::mutex lock_;
};

TeleopCtrlZmq::TeleopCtrlZmq() : context_(1),
                                 is_running_(false),
                                 socket_action_(context_, zmq::socket_type::push),
                                 socket_observation_(context_, zmq::socket_type::pull) {
}

void TeleopCtrlZmq::ProcessKeyboardAction(const json& action,
    std::string& action_string) {
  json keyboard_action = json::object();
  if (action.contains("data")) {
    auto data = action["data"];
    for (auto const& [key, value] : data.items()) {
      if (!(value.is_boolean() && value == false)) {
        keyboard_action[key] = value;
      }
    }
  }
  action_string = keyboard_action.dump();
}

void TeleopCtrlZmq::ProcessGamepadAction(const json& action,
    std::string& action_string) {
  json keyboard_action = json::object();
  if (action.contains("data")) {
    auto data = action["data"];

    int mode = 0;
    // 0: base control
    // 1: arm translation
    // 2: arm rotation
    if (data["buttons"][7].get<double>() > 0 && data["buttons"][2].get<double>() > 0) {
      keyboard_action["q"] = true;
    }

    if (data["buttons"][5].get<double>() > 0 && data["buttons"][7].get<double>() > 0) {
      mode = 2;
    } else if (data["buttons"][5].get<double>() > 0) {
      mode = 1;
    }

    if (mode == 0) {

      if (data["axes"][1].get<double>() > 0.75) {
        keyboard_action["s"] = true;
      }

      if (data["axes"][1].get<double>() < -0.75) {
        keyboard_action["w"] = true;
      }

      if (data["axes"][0].get<double>() > 0.75) {
        keyboard_action["d"] = true;
      }

      if (data["axes"][0].get<double>() < -0.75) {
        keyboard_action["a"] = true;
      }

      if (data["axes"][2].get<double>() > 0.75) {
        keyboard_action["x"] = true;
      }

      if (data["axes"][2].get<double>() < -0.75) {
        keyboard_action["z"] = true;
      }

    } else if (mode == 1) {

      if (data["axes"][3].get<double>() > 0.8) {
        keyboard_action["j"] = true;
      } else if (data["axes"][3].get<double>() < -0.8) {
        keyboard_action["u"] = true;
      } else if (data["axes"][2].get<double>() > 0.8) {
        keyboard_action["k"] = true;
      } else if (data["axes"][2].get<double>() < -0.8) {
        keyboard_action["h"] = true;
      }

      // axis z
      if (data["buttons"][3].get<double>() > 0) {
        keyboard_action["y"] = true;
      }
 
      if (data["buttons"][0].get<double>() > 0) {
        keyboard_action["i"] = true;
      }

    } else if (mode == 2) {

       if (data["axes"][3].get<double>() > 0.8) {
        keyboard_action["g"] = true;
      } else if (data["axes"][3].get<double>() < -0.8) {
        keyboard_action["t"] = true;
      } else if (data["axes"][2].get<double>() > 0.8) {
        keyboard_action["r"] = true;
      } else if (data["axes"][2].get<double>() < -0.8) {
        keyboard_action["f"] = true;
      }
    }

    // gripper
    if (data["buttons"][1].get<double>() > 0) {
      keyboard_action["n"] = true;
    }

    if (data["buttons"][2].get<double>() > 0) {
      keyboard_action["b"] = true;
    }
  }
  action_string = keyboard_action.dump();
}

void TeleopCtrlZmq::OnMessageReceived(const char* data) {

  size_t len = std::strlen(data);
  if (is_running_) {

    {
      std::lock_guard<std::mutex> lock(lock_); 
      if (!action_.contains("arm_gripper.pos")) {
        action_ = observation_;
      }
    }
    std::string action_string;
    json action = json::parse(data);
    if (action.contains("type") && action["type"] == "keyboard") {
      ProcessKeyboardAction(action, action_string);
    } else if (action.contains("type") && action["type"] == "gamepad") {
      ProcessGamepadAction(action, action_string);
    } else {
      action_string = std::string(data);
    }

    if (action_string.length() > 0) {
      zmq::message_t message(action_string.c_str(), action_string.length());
      socket_action_.send(message, zmq::send_flags::none);
    }
  }
}

void TeleopCtrlZmq::Terminate() {
  is_running_ = false;
  if (pull_thread_.joinable()) {
    pull_thread_.join();
  }
  socket_action_.close();
  socket_observation_.close();
  context_.close();
}

void TeleopCtrlZmq::Invoke() {
  socket_action_.connect("tcp://localhost:5558");
  socket_observation_.connect("tcp://localhost:5556");
  is_running_ = true;
  printf("Connected\n");
  pull_thread_ = std::thread(&TeleopCtrlZmq::PullThread, this);
  while (is_running_) {
    if (is_recording_) {
      //std::cout << observation_ << std::endl;

      std::vector<float> observation = {
        observation_["arm_shoulder_pan.pos"],
        observation_["arm_shoulder_lift.pos"],
        observation_["arm_wrist_flex.pos"],
        observation_["arm_elbow_flex.pos"],
        observation_["arm_wrist_roll.pos"],
        observation_["arm_gripper.pos"],
        observation_["x.vel"],
        observation_["y.vel"],
        observation_["theta.vel"]
      };

      std::vector<float> action = {
        action_["arm_shoulder_pan.pos"],
        action_["arm_shoulder_lift.pos"],
        action_["arm_wrist_flex.pos"],
        action_["arm_elbow_flex.pos"],
        action_["arm_wrist_roll.pos"],
        action_["arm_gripper.pos"],
        action_["x.vel"],
        action_["y.vel"],
        action_["theta.vel"]
      };

      IngestTelemetry(observation.data(), observation.size(), action.data(), action.size());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void TeleopCtrlZmq::PullThread() {
  while (is_running_) {
    zmq::message_t msg;
    zmq::recv_result_t result = socket_observation_.recv(msg, zmq::recv_flags::dontwait);
    if (result) {
      std::string received(static_cast<char*>(msg.data()), msg.size());

      {
        std::lock_guard<std::mutex> lock(lock_); 
        observation_ = json::parse(received);
      }

      //std::cout << "observation: " << received << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

extern "C" TeleopCtrlPlugin* create_plugin() {
  return new TeleopCtrlZmq();
}
