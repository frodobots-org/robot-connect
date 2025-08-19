#include <iostream>
#include <thread>
#include <gst/gst.h>
#include "teleop_media_plugin.h"

#define PIPELINE "v4l2src device=/dev/video2 ! video/x-raw, width=640, height=480, framerate=30/1 ! nvvidconv ! nvv4l2h264enc bitrate=1000000 insert-sps-pps=true disable-cabac=true qp-range=30,45:30,45:30,45 idrinterval=30 maxperf-enable=1 ! appsink name=sink sync=false"

class TeleopMediaGst : public TeleopMediaPlugin {
 public:
  TeleopMediaGst();
  ~TeleopMediaGst();
  void Invoke() override;
  void Terminate() override;
  void OnEvent(const char* event_name, const void* data) override;
 private:
  GstElement* pipeline_;
  GstElement* appsink_;
  bool is_running_;
  static GstFlowReturn OnVideoSample(GstElement* sink, gpointer user_data);
};

TeleopMediaGst::TeleopMediaGst() {
}

TeleopMediaGst::~TeleopMediaGst() {
}

GstFlowReturn TeleopMediaGst::OnVideoSample(GstElement* sink, gpointer user_data) {
  TeleopMediaGst* self = static_cast<TeleopMediaGst*>(user_data);
  GstSample* sample = NULL;
  GstBuffer* buffer;
  GstMapInfo info;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  if (sample) {
    buffer = gst_sample_get_buffer(sample);
    gst_buffer_map(buffer, &info, GST_MAP_READ);
    self->SendVideoData(info.data, info.size);
    gst_buffer_unmap(buffer, &info);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }
  return GST_FLOW_ERROR;
}

void TeleopMediaGst::OnEvent(const char* event_name, const void* data) {
  printf("Received event: %s\n", event_name);
}


void TeleopMediaGst::Invoke() {
  gst_init(NULL, NULL);
  is_running_ = true;
  
  pipeline_ = gst_parse_launch(PIPELINE, NULL);
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(OnVideoSample), this);
  g_object_set(appsink_, "emit-signals", TRUE, NULL);
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);

  while (is_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(pipeline_);
  gst_object_unref(appsink_);
}

void TeleopMediaGst::Terminate() {
  is_running_ = false;
}

extern "C" TeleopMediaPlugin* create_plugin() {
  return new TeleopMediaGst();
}
