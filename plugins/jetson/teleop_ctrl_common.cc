#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <teleop_ctrl_plugin.h>

class TeleopCtrlCommon : public TeleopCtrlPlugin {
 public:
  TeleopCtrlCommon();
  void Invoke() override;
  void Terminate() override;
  void OnMessageReceived(const char* message) override;
 private:
  bool is_running_ = false;
};

TeleopCtrlCommon::TeleopCtrlCommon() {
}

void TeleopCtrlCommon::OnMessageReceived(const char* data) {

  std::cout << "Received message: " << data << std::endl;
}

void TeleopCtrlCommon::Terminate() {
  is_running_ = false;
}

void TeleopCtrlCommon::Invoke() {
  while (is_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

extern "C" TeleopCtrlPlugin* create_plugin() {
  return new TeleopCtrlCommon();
}
