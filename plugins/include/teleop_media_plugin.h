#ifndef TELEOP_MEDIA_PLUGIN_H_
#define TELEOP_MEDIA_PLUGIN_H_

#include <stdint.h>
#include <map>

class TeleopMediaPlugin {
 public:
  virtual void Invoke() = 0;
  virtual void Terminate() = 0;
  virtual void OnEvent(const char* event_name, const void* event_data) = 0;

  virtual void OnAudioDataReceived(const uint8_t*, size_t, const uint32_t) { }
  void send_video_data(int (*send_video_data)(const uint8_t*, size_t, int channel)) {
    send_video_data_ = send_video_data;
  }

  void send_audio_data(int (*send_audio_data)(const uint8_t*, size_t)) {
    send_audio_data_ = send_audio_data;
  }

  int SendVideoData(const uint8_t* data, size_t size, int channel = 0) {
    return send_video_data_ ? send_video_data_(data, size, channel) : -1;
  }

  int SendAudioData(const uint8_t* data, size_t size) {
    return send_audio_data_ ? send_audio_data_(data, size) : -1;
  }

  std::map<std::string, std::string>& config() {
    return config_;
  }
  virtual ~TeleopMediaPlugin() {}
  uint32_t user_id() const { return user_id_; }
  void user_id(uint32_t user_id) { user_id_ = user_id; }
  const char* plugin_type() const { return "media"; }
 private:
  int (*send_video_data_)(const uint8_t*, size_t, int channel) = nullptr;
  int (*send_audio_data_)(const uint8_t*, size_t) = nullptr;
 protected:
  uint32_t user_id_;
  std::map<std::string, std::string> config_;
};

#endif // TELEOP_MEDIA_PLUGIN_H_

