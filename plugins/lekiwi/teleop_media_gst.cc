#include <iostream>
#include <thread>
#include <gst/gst.h>
#include "teleop_media_plugin.h"

uint64_t get_timestamp_ms() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

void set_bit_1x2_pixels(uint8_t *y_plane, int start_x, int start_y, bool bit) {
  uint8_t val_on = 200;
  uint8_t val_off = 16;

  if (bit) {
    y_plane[start_y * 640 + start_x] = val_off;
    y_plane[start_y * 640 + start_x + 1] = val_on;
  } else {
    y_plane[start_y * 640 + start_x] = val_on;
    y_plane[start_y * 640 + start_x + 1] = val_off;
  }
}

void write_byte_1x2_pixels(uint8_t *y_plane, int start_x, int start_y, uint8_t byte) {
  for (int i = 0; i < 8; ++i) {
    bool bit = (byte >> (7 - i)) & 1;
    set_bit_1x2_pixels(y_plane, start_x + i * 2, start_y, bit);
  }
}

uint8_t calc_checksum(uint8_t *data, int len) {
  uint8_t cs = 0;
  for (int i = 0; i < len; i++) {
    cs ^= data[i];
  }
  return cs;
}

void write_timestamp_and_checksum(uint8_t *y_plane, int start_y, uint64_t timestamp) {
  uint8_t buf[9];
  memcpy(buf, &timestamp, 8);

  buf[8] = calc_checksum(buf, 8);

  for (int i = 0; i < 9; i++) {
    write_byte_1x2_pixels(y_plane, i * 16, start_y, buf[i]);
  }
}

static void on_buffer_handoff(GstElement* identity, GstBuffer* buffer, GstPad* pad, gpointer user_data) {
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) return;

  guint8* y_plane = map.data;
  guint64 data = get_timestamp_ms();
  write_timestamp_and_checksum(y_plane, 0, data); 
  gst_buffer_unmap(buffer, &map);
}

class TeleopMediaGst : public TeleopMediaPlugin {
 public:
  TeleopMediaGst();
  ~TeleopMediaGst();
  void Invoke() override;
  void Terminate() override;
  void OnEvent(const char* event_name, const void* data) override;
 private:
  GstElement* pipeline1_;
  GstElement* appsink1_;
  GstElement* pipeline2_;
  GstElement* appsink2_;
  GstElement* identity1_;
  GstElement* identity2_;
  bool is_running_;
  static GstFlowReturn OnVideoSampleMajor(GstElement* sink, gpointer user_data);
  static GstFlowReturn OnVideoSampleMinor(GstElement* sink, gpointer user_data);
};

TeleopMediaGst::TeleopMediaGst() {
  config_["camera1"] = "";
  config_["camera2"] = "";
}

TeleopMediaGst::~TeleopMediaGst() {
}

GstFlowReturn TeleopMediaGst::OnVideoSampleMajor(GstElement* sink, gpointer user_data) {
  TeleopMediaGst* self = static_cast<TeleopMediaGst*>(user_data);
  GstSample* sample = NULL;
  GstBuffer* buffer;
  GstMapInfo info;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (sample) {
    buffer = gst_sample_get_buffer(sample);
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    //printf("send major video data, size: %d\n", info.size - 6);
    self->SendVideoData(info.data + 6, info.size - 6);
    gst_buffer_unmap(buffer, &info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }
  return GST_FLOW_ERROR;
}

GstFlowReturn TeleopMediaGst::OnVideoSampleMinor(GstElement* sink, gpointer user_data) {
  TeleopMediaGst* self = static_cast<TeleopMediaGst*>(user_data);
  GstSample* sample = NULL;
  GstBuffer* buffer;
  GstMapInfo info;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (sample) {
    buffer = gst_sample_get_buffer(sample);
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    //printf("send minor video data, size: %d\n", info.size - 6);
    self->SendVideoData(info.data + 6, info.size - 6, 1);
    gst_buffer_unmap(buffer, &info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }
  return GST_FLOW_ERROR;
}


void TeleopMediaGst::OnEvent(const char* event_name, const void* data) {
}


void TeleopMediaGst::Invoke() {
  gst_init(NULL, NULL);
  is_running_ = true;
  std::string encode_pipeine = " ! videoconvert ! video/x-raw, format=I420, width=640, height=480 ! identity name=id ! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=60 bitrate=1024 bframes=0 ! appsink name=sink sync=false";
  
  if (config_.count("camera1") && !config_["camera1"].empty()) {
    std::string pipeline1 = config_["camera1"] + encode_pipeine;
    pipeline1_ = gst_parse_launch(pipeline1.c_str(), NULL);
    appsink1_ = gst_bin_get_by_name(GST_BIN(pipeline1_), "sink");
    identity1_ = gst_bin_get_by_name(GST_BIN(pipeline1_), "id");
    g_signal_connect(identity1_, "handoff", G_CALLBACK(on_buffer_handoff), NULL);
    g_signal_connect(appsink1_, "new-sample", G_CALLBACK(OnVideoSampleMinor), this);
    g_object_set(appsink1_, "emit-signals", TRUE, NULL);
    gst_element_set_state(pipeline1_, GST_STATE_PLAYING);
  }

  if (config_.count("camera2") && !config_["camera2"].empty()) {
    std::string pipeline2 = config_["camera2"] + encode_pipeine;
    pipeline2_ = gst_parse_launch(pipeline2.c_str(), NULL);
    appsink2_ = gst_bin_get_by_name(GST_BIN(pipeline2_), "sink");
    identity2_ = gst_bin_get_by_name(GST_BIN(pipeline2_), "id");
    g_signal_connect(identity2_, "handoff", G_CALLBACK(on_buffer_handoff), NULL);
    g_signal_connect(appsink2_, "new-sample", G_CALLBACK(OnVideoSampleMajor), this);
    g_object_set(appsink2_, "emit-signals", TRUE, NULL);
    gst_element_set_state(pipeline2_, GST_STATE_PLAYING);
  }

  while (is_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  gst_element_set_state(pipeline1_, GST_STATE_NULL);
  gst_element_set_state(pipeline2_, GST_STATE_NULL);
  gst_object_unref(pipeline1_);
  gst_object_unref(pipeline2_);
  gst_object_unref(appsink1_);
  gst_object_unref(appsink2_);
}

void TeleopMediaGst::Terminate() {
  is_running_ = false;
}

extern "C" TeleopMediaPlugin* create_plugin() {
  return new TeleopMediaGst();
}
