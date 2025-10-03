#include <rk_debug.h>
#include <rk_mpi_cal.h>
#include <rk_mpi_ivs.h>
#include <rk_mpi_mb.h>
#include <rk_mpi_mmz.h>
#include <rk_mpi_rgn.h>
#include <rk_mpi_sys.h>
#include <rk_mpi_tde.h>
#include <rk_mpi_venc.h>
#include <rk_mpi_vi.h>
#include <rk_mpi_vpss.h>
#include <sample_comm.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include "teleop_media_plugin.h"

const int kWidth = 1024;
const int kHeight = 576;
typedef struct _rkMpiCtx {
  SAMPLE_VI_CTX_S vi[2];
  SAMPLE_VENC_CTX_S venc[2];
} SAMPLE_MPI_CTX_S;

std::queue<std::vector<uint8_t>> g_frame_queue;
std::mutex g_frame_queue_mutex;
bool g_is_running;

class TeleopMediaEarthRover : public TeleopMediaPlugin {
 public:
  TeleopMediaEarthRover();
  ~TeleopMediaEarthRover();
  void Invoke() override;
  void Terminate() override;
  void OnEvent(const char* event_name, const void* data) override;

 private:
  MPP_CHN_S vi_chn_[2];
  MPP_CHN_S venc_chn_[2];
  SAMPLE_MPI_CTX_S mpi_ctx_;
};

TeleopMediaEarthRover::TeleopMediaEarthRover() {
  RK_MPI_SYS_Init();
  memset(&mpi_ctx_, 0, sizeof(mpi_ctx_));
}

TeleopMediaEarthRover::~TeleopMediaEarthRover() {
}

void TeleopMediaEarthRover::OnEvent(const char* event_name, const void* data) {
}

static void* venc0_get_stream(void* pArgs) {
  SAMPLE_VENC_CTX_S* ctx = (SAMPLE_VENC_CTX_S*)(pArgs);
  RK_S32 s32Ret = RK_FAILURE;
  void* pData = RK_NULL;
  RK_S32 loopCount = 0;

  while (true) {
    s32Ret = SAMPLE_COMM_VENC_GetStream(ctx, &pData);
    if (s32Ret == RK_SUCCESS) {
      std::vector<uint8_t> frame_data((uint8_t*)pData, (uint8_t*)pData + ctx->stFrame.pstPack->u32Len);
      {
        std::lock_guard<std::mutex> lock(g_frame_queue_mutex);
        g_frame_queue.push(frame_data);
      }

      SAMPLE_COMM_VENC_ReleaseStream(ctx);
    }
    usleep(1000);
  }

  return RK_NULL;
}

void TeleopMediaEarthRover::Invoke() {
  g_is_running = true;
  int ret = 0;
  SAMPLE_MPI_CTX_S* ctx = &mpi_ctx_;

  int cam_0_fps = 30;
  int cam_0_enable_hdr = 0;
  int cam_0_video_0_width = 1024;
  int cam_0_video_0_height = 576;

  CODEC_TYPE_E enCodecType = RK_CODEC_TYPE_H264;
  int enable_buf_share = 1;
  VENC_RC_MODE_E enRcMode = VENC_RC_MODE_H264CBR;
  const RK_CHAR* pCodecName = "H264";
  RK_S32 s32CamId = -1;
  RK_S32 s32CamNum = 1;
  RK_S32 s32loopCnt = -1;
  RK_S32 s32BitRate = 4 * 1024;
  RK_S32 i;
  const char* iq_file_dir = "/etc/iqfiles";

  ctx = (SAMPLE_MPI_CTX_S*)(malloc(sizeof(SAMPLE_MPI_CTX_S)));
  memset(ctx, 0, sizeof(SAMPLE_MPI_CTX_S));

  SAMPLE_COMM_ISP_Init(0, RK_AIQ_WORKING_MODE_NORMAL, RK_TRUE, iq_file_dir);
  SAMPLE_COMM_ISP_Run(0);
  SAMPLE_COMM_ISP_SetFrameRate(0, cam_0_fps);

  if (RK_MPI_SYS_Init() != RK_SUCCESS) {
    goto __FAILED;
  }

  for (i = 0; i < s32CamNum; i++) {
    if (i == 0) {
      ctx->vi[i].u32Width = cam_0_video_0_width;
      ctx->vi[i].u32Height = cam_0_video_0_height;
    }
    ctx->vi[i].s32DevId = 0;
    ctx->vi[i].u32PipeId = 0;
    ctx->vi[i].s32ChnId = 0;
    ctx->vi[i].stChnAttr.stIspOpt.u32BufCount = 2;
    ctx->vi[i].stChnAttr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
    ctx->vi[i].stChnAttr.u32Depth = 0;
    ctx->vi[i].stChnAttr.enPixelFormat = RK_FMT_YUV420SP;
    ctx->vi[i].stChnAttr.enCompressMode = COMPRESS_MODE_NONE;
    ctx->vi[i].stChnAttr.stFrameRate.s32SrcFrameRate = -1;
    ctx->vi[i].stChnAttr.stFrameRate.s32DstFrameRate = -1;
    SAMPLE_COMM_VI_CreateChn(&ctx->vi[i]);

    vi_chn_[i].enModId = RK_ID_VI;
    vi_chn_[i].s32DevId = ctx->vi[i].s32DevId;
    vi_chn_[i].s32ChnId = ctx->vi[i].s32ChnId;

    ctx->venc[i].s32ChnId = i;
    if (i == 0) {
      ctx->venc[i].u32Width = cam_0_video_0_width;
      ctx->venc[i].u32Height = cam_0_video_0_height;
      ctx->venc[i].stChnAttr.stVencAttr.u32BufSize =
          cam_0_video_0_width * cam_0_video_0_height / 4;
      ctx->venc[i].u32Fps = cam_0_fps;
    }
    ctx->venc[i].u32Gop = 50;
    ctx->venc[i].u32BitRate = s32BitRate;
    ctx->venc[i].enCodecType = enCodecType;
    ctx->venc[i].enRcMode = enRcMode;
    if (i == 0)
      ctx->venc[i].getStreamCbFunc = venc0_get_stream;
    ctx->venc[i].s32loopCount = s32loopCnt;
    ctx->venc[i].dstFilePath = NULL;
    ctx->venc[i].stChnAttr.stVencAttr.u32Profile = 0;
    ctx->venc[i].stChnAttr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
    ctx->venc[i].enable_buf_share = enable_buf_share;
    SAMPLE_COMM_VENC_CreateChn(&ctx->venc[i]);

    venc_chn_[i].enModId = RK_ID_VENC;
    venc_chn_[i].s32DevId = 0;
    venc_chn_[i].s32ChnId = ctx->venc[i].s32ChnId;
  }
  SAMPLE_COMM_Bind(&vi_chn_[0], &venc_chn_[0]);

  printf("%s initial finish\n", __func__);

  while (g_is_running) {
    g_frame_queue_mutex.lock();
    if (!g_frame_queue.empty()) {
      std::vector<uint8_t> frame = g_frame_queue.front();
      g_frame_queue.pop();
      g_frame_queue_mutex.unlock();
      // process frame
      // printf("send frame size: %zu\n", frame.size());
      SendVideoData(frame.data(), frame.size());
    } else {
      g_frame_queue_mutex.unlock();
      usleep(1000);
    }
  }

  printf("%s exit!\n", __func__);
  for (i = 0; i < s32CamNum; i++) {
    if (ctx->venc[i].getStreamCbFunc) {
      pthread_join(ctx->venc[i].getStreamThread, NULL);
    }
  }

  SAMPLE_COMM_UnBind(&vi_chn_[0], &venc_chn_[0]);

  for (i = 0; i < s32CamNum; i++) {
    SAMPLE_COMM_VENC_DestroyChn(&ctx->venc[i]);
    SAMPLE_COMM_VI_DestroyChn(&ctx->vi[i]);
  }

__FAILED:
  RK_MPI_SYS_Exit();
  for (int i = 0; i < s32CamNum; i++) {
    SAMPLE_COMM_ISP_Stop(i);
  }
}

void TeleopMediaEarthRover::Terminate() {
  g_is_running = false;
}

extern "C" TeleopMediaPlugin* create_plugin() {
  return new TeleopMediaEarthRover();
}
