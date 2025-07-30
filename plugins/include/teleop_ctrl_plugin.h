#ifndef TELEOP_CTRL_PLUGIN_H_
#define TELEOP_CTRL_PLUGIN_H_

#include <stdint.h>

typedef void (*SendMessageImpl)(const char*, size_t);

class TeleopCtrlPlugin {
 public:
  virtual void Invoke() = 0;
  virtual void Terminate() = 0;
  virtual void OnMessageReceived(const char* message) = 0;
  virtual ~TeleopCtrlPlugin() {}
  void send_message_impl(SendMessageImpl send_message_impl) { send_message_impl_ = send_message_impl; }
  void SendMessage(const char* message, size_t size) {
    if (send_message_impl_) {
      send_message_impl_(message, size);
    }
  }
  const char* plugin_type() const { return "ctrl"; }
  void on_observation_data(void (*on_observation_data)(std::string observation)) {
    on_observation_data_ = on_observation_data;
  }
  void is_recording(bool is_recording) { is_recording_ = is_recording; }
 private:
  SendMessageImpl send_message_impl_;
 protected:
  void (*on_observation_data_)(std::string observation);
  bool is_recording_;
};

#endif // TELEOP_CTRL_PLUGIN_H_
