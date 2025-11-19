#ifndef TELEOP_CTRL_PLUGIN_H_
#define TELEOP_CTRL_PLUGIN_H_

#include <stdint.h>
#include <vector>

typedef void (*SendMessageImpl)(const char*, size_t, const char*);
typedef void (*IngestTelemetryImpl)(const float*, size_t, const float*, size_t);
class TeleopCtrlPlugin {
 public:
  virtual void Invoke() = 0;
  virtual void Terminate() = 0;
  virtual void OnMessageReceived(const char* message) = 0;
  virtual ~TeleopCtrlPlugin() {}
  void send_message_impl(SendMessageImpl send_message_impl) {
    send_message_impl_ = send_message_impl;
  }
  void ingest_telemetry_impl(IngestTelemetryImpl IngestTelemetryImpl) {
    ingest_telemetry_impl_ = IngestTelemetryImpl;
  }
  void SendMessage(const char* message, size_t size, const char* user_id) {
    if (send_message_impl_) {
      send_message_impl_(message, size, user_id);
    }
  }
  void IngestTelemetry(const float* obs, size_t obs_size, const float* act, size_t act_size) {
    if (ingest_telemetry_impl_) {
      ingest_telemetry_impl_(obs, obs_size, act, act_size);
    }
  }
  const char* plugin_type() const { return "ctrl"; }
  void is_recording(bool is_recording) { is_recording_ = is_recording; }
 private:
  SendMessageImpl send_message_impl_;
  IngestTelemetryImpl ingest_telemetry_impl_;
 protected:
  bool is_recording_;
};

#endif // TELEOP_CTRL_PLUGIN_H_
