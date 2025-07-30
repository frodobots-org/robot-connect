#include <zmq.hpp>
#include <string>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <teleop_ctrl_plugin.h>

class TeleopCtrlZmq : public TeleopCtrlPlugin {
 public:
  TeleopCtrlZmq();
  void Invoke() override;
  void Terminate() override;
  void OnMessageReceived(const char* message) override;
 private:
  void PullThread();
  zmq::context_t context_;
  zmq::socket_t socket_action_;
  zmq::socket_t socket_observation_;
  std::atomic<bool> is_running_ = false;
  std::thread pull_thread_;
};

TeleopCtrlZmq::TeleopCtrlZmq() : context_(1),
                                 is_running_(false),
                                 socket_action_(context_, zmq::socket_type::push),
                                 socket_observation_(context_, zmq::socket_type::pull) {
}

void TeleopCtrlZmq::OnMessageReceived(const char* data) {

  size_t len = std::strlen(data);
  if (is_running_) {
    zmq::message_t message(data, len);
   //std::cout << "action: " << message << std::endl;
    socket_action_.send(message, zmq::send_flags::none);
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
  socket_action_.connect("tcp://localhost:5555");
  socket_observation_.connect("tcp://localhost:5556");
  is_running_ = true;
  printf("Connected\n");
  pull_thread_ = std::thread(&TeleopCtrlZmq::PullThread, this);
  while (is_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void TeleopCtrlZmq::PullThread() {
  while (is_running_) {
    zmq::message_t msg;
    zmq::recv_result_t result = socket_observation_.recv(msg, zmq::recv_flags::dontwait);
    if (result) {
      std::string received(static_cast<char*>(msg.data()), msg.size());
      if (on_observation_data_ && is_recording_) {
        on_observation_data_(received);
      }
      //std::cout << "observation: " << received << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

extern "C" TeleopCtrlPlugin* create_plugin() {
  return new TeleopCtrlZmq();
}
