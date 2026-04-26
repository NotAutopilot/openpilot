#include "frogpilot/ui/screenrecorder/omx_encoder.h"

#ifdef STUB_OMX_ENCODER
// OMX is comma-device-only (Snapdragon BSP). Stub on macOS dev hosts and
// Linux x86_64 CI; ScreenRecorder still compiles+links, the record button
// becomes a no-op. Real implementation runs only on larch64.
OmxEncoder::OmxEncoder(const char*, int, int, int, int) {}
OmxEncoder::~OmxEncoder() {}
int OmxEncoder::encode_frame_rgba(const uint8_t*, int, int, uint64_t) { return 0; }
void OmxEncoder::encoder_open(const char*) {}
void OmxEncoder::encoder_close() {}
#else

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <sys/stat.h>
#include <unistd.h>

#include <OMX_Component.h>
#include <OMX_IndexExt.h>
#include <OMX_QCOMExtns.h>
#include <OMX_VideoExt.h>

extern "C" {
#include <libavutil/opt.h>
}

#include "libyuv.h"
#include "msm_media_info.h"

#include "common/swaglog.h"
#include "common/util.h"

#define OMX_CHECK(_expr) assert(OMX_ErrorNone == (_expr))
#define PORT_INDEX_IN 0
#define PORT_INDEX_OUT 1

using namespace libyuv;

LIBYUV_API
int ABGRToNV12(const uint8_t* src_abgr, int src_stride_abgr,
               uint8_t* dst_y, int dst_stride_y,
               uint8_t* dst_uv, int dst_stride_uv,
               int width, int height) {
  int y;
  int halfwidth = (width + 1) >> 1;

  void (*ABGRToUVRow)(const uint8_t* src_abgr0, int src_stride_abgr,
                      uint8_t* dst_u, uint8_t* dst_v, int width) = ABGRToUVRow_NEON;
  void (*ABGRToYRow)(const uint8_t* src_abgr, uint8_t* dst_y, int width) = ABGRToYRow_NEON;
  void (*MergeUVRow_)(const uint8_t* src_u, const uint8_t* src_v,
                      uint8_t* dst_uv, int width) = MergeUVRow_NEON;

  if (!src_abgr || !dst_y || !dst_uv || width <= 0 || height == 0) {
    return -1;
  }

  if (height < 0) {
    height = -height;
    src_abgr = src_abgr + (height - 1) * src_stride_abgr;
    src_stride_abgr = -src_stride_abgr;
  }

  align_buffer_64(row_u, ((halfwidth + 31) & ~31) * 2);
  uint8_t* row_v = row_u + ((halfwidth + 31) & ~31);

  for (y = 0; y < height - 1; y += 2) {
    ABGRToUVRow(src_abgr, src_stride_abgr, row_u, row_v, width);
    MergeUVRow_(row_u, row_v, dst_uv, halfwidth);
    ABGRToYRow(src_abgr, dst_y, width);
    ABGRToYRow(src_abgr + src_stride_abgr, dst_y + dst_stride_y, width);
    src_abgr += src_stride_abgr * 2;
    dst_y += dst_stride_y * 2;
    dst_uv += dst_stride_uv;
  }

  if (height & 1) {
    ABGRToUVRow(src_abgr, 0, row_u, row_v, width);
    MergeUVRow_(row_u, row_v, dst_uv, halfwidth);
    ABGRToYRow(src_abgr, dst_y, width);
  }

  free_aligned_buffer_64(row_u);
  return 0;
}

void OmxEncoder::wait_for_state(OMX_STATETYPE state_) {
  std::unique_lock<std::mutex> lk(state_lock);
  state_cv.wait(lk, [&]{ return state == state_; });
}

static OMX_CALLBACKTYPE omx_callbacks = {
  .EventHandler = OmxEncoder::event_handler,
  .EmptyBufferDone = OmxEncoder::empty_buffer_done,
  .FillBufferDone = OmxEncoder::fill_buffer_done,
};

OMX_ERRORTYPE OmxEncoder::event_handler(OMX_HANDLETYPE component, OMX_PTR app_data, OMX_EVENTTYPE event,
                                   OMX_U32 data1, OMX_U32 data2, OMX_PTR event_data) {
  OmxEncoder *e = (OmxEncoder*)app_data;
  if (event == OMX_EventCmdComplete) {
    assert(data1 == OMX_CommandStateSet);
    {
      std::lock_guard<std::mutex> lk(e->state_lock);
      e->state = (OMX_STATETYPE)data2;
    }
    e->state_cv.notify_all();
  } else if (event == OMX_EventError) {
    LOGE("OMX error 0x%08x", data1);
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE OmxEncoder::empty_buffer_done(OMX_HANDLETYPE component, OMX_PTR app_data,
                                            OMX_BUFFERHEADERTYPE *buffer) {
  OmxEncoder *e = (OmxEncoder*)app_data;
  e->free_in->push(std::move(buffer));
  return OMX_ErrorNone;
}

OMX_ERRORTYPE OmxEncoder::fill_buffer_done(OMX_HANDLETYPE component, OMX_PTR app_data,
                                           OMX_BUFFERHEADERTYPE *buffer) {
  OmxEncoder *e = (OmxEncoder*)app_data;
  e->done_out->push(std::move(buffer));
  return OMX_ErrorNone;
}

OmxEncoder::OmxEncoder(const char* path, int width, int height, int fps, int bitrate)
    : path(path), width(width), height(height), fps(fps) {

  OMX_ERRORTYPE err = OMX_Init();
  if (err != OMX_ErrorNone) {
    LOGE("OMX_Init failed: %x", err);
    return;
  }

  OMX_STRING component = (OMX_STRING)("OMX.qcom.video.encoder.avc");
  err = OMX_GetHandle(&handle, component, this, &omx_callbacks);
  if (err != OMX_ErrorNone) {
    LOGE("Error getting codec: %x", err);
    OMX_Deinit();
    return;
  }

  // Input Port Configuration
  OMX_PARAM_PORTDEFINITIONTYPE in_port = {0};
  in_port.nSize = sizeof(in_port);
  in_port.nPortIndex = (OMX_U32) PORT_INDEX_IN;
  OMX_CHECK(OMX_GetParameter(handle, OMX_IndexParamPortDefinition, (OMX_PTR) &in_port));

  in_port.format.video.nFrameWidth = width;
  in_port.format.video.nFrameHeight = height;
  in_port.format.video.nStride = VENUS_Y_STRIDE(COLOR_FMT_NV12, width);
  in_port.format.video.nSliceHeight = height;
  in_port.nBufferSize = VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height);
  in_port.format.video.xFramerate = (fps * 65536);
  in_port.format.video.eCompressionFormat = OMX_VIDEO_CodingUnused;
  in_port.format.video.eColorFormat = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;

  OMX_CHECK(OMX_SetParameter(handle, OMX_IndexParamPortDefinition, (OMX_PTR) &in_port));
  OMX_CHECK(OMX_GetParameter(handle, OMX_IndexParamPortDefinition, (OMX_PTR) &in_port));

  in_buf_headers.resize(in_port.nBufferCountActual);
  free_in = std::make_unique<BlockingQueue<OMX_BUFFERHEADERTYPE *>>(in_port.nBufferCountActual);

  // Output Port Configuration
  OMX_PARAM_PORTDEFINITIONTYPE out_port = {0};
  out_port.nSize = sizeof(out_port);
  out_port.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
  OMX_CHECK(OMX_GetParameter(handle, OMX_IndexParamPortDefinition, (OMX_PTR)&out_port));

  out_port.format.video.nFrameWidth = width;
  out_port.format.video.nFrameHeight = height;
  out_port.format.video.nBitrate = bitrate;
  out_port.format.video.eCompressionFormat = OMX_VIDEO_CodingAVC;
  out_port.format.video.eColorFormat = OMX_COLOR_FormatUnused;

  OMX_CHECK(OMX_SetParameter(handle, OMX_IndexParamPortDefinition, (OMX_PTR) &out_port));
  OMX_CHECK(OMX_GetParameter(handle, OMX_IndexParamPortDefinition, (OMX_PTR) &out_port));

  out_buf_headers.resize(out_port.nBufferCountActual);
  done_out = std::make_unique<BlockingQueue<OMX_BUFFERHEADERTYPE *>>(out_port.nBufferCountActual);

  // Bitrate Control
  OMX_VIDEO_PARAM_BITRATETYPE bitrate_type = {0};
  bitrate_type.nSize = sizeof(bitrate_type);
  bitrate_type.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
  OMX_CHECK(OMX_GetParameter(handle, OMX_IndexParamVideoBitrate, (OMX_PTR) &bitrate_type));
  bitrate_type.eControlRate = OMX_Video_ControlRateVariable;
  bitrate_type.nTargetBitrate = bitrate;
  OMX_CHECK(OMX_SetParameter(handle, OMX_IndexParamVideoBitrate, (OMX_PTR) &bitrate_type));

  // AVC Parameters
  OMX_VIDEO_PARAM_AVCTYPE avc = {0};
  avc.nSize = sizeof(avc);
  avc.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
  OMX_CHECK(OMX_GetParameter(handle, OMX_IndexParamVideoAvc, &avc));
  avc.nBFrames = 0;
  avc.nPFrames = 15;
  avc.eProfile = OMX_VIDEO_AVCProfileHigh;
  avc.eLevel = OMX_VIDEO_AVCLevel31;
  avc.nAllowedPictureTypes |= OMX_VIDEO_PictureTypeB;
  avc.eLoopFilterMode = OMX_VIDEO_AVCLoopFilterEnable;
  avc.nRefFrames = 1;
  avc.bUseHadamard = OMX_TRUE;
  avc.bEntropyCodingCABAC = OMX_TRUE;
  avc.bWeightedPPrediction = OMX_TRUE;
  avc.bconstIpred = OMX_TRUE;
  OMX_CHECK(OMX_SetParameter(handle, OMX_IndexParamVideoAvc, &avc));

  OMX_CHECK(OMX_SendCommand(handle, OMX_CommandStateSet, OMX_StateIdle, NULL));

  for (OMX_BUFFERHEADERTYPE* &buf : in_buf_headers) {
    OMX_CHECK(OMX_AllocateBuffer(handle, &buf, PORT_INDEX_IN, this, in_port.nBufferSize));
  }
  for (OMX_BUFFERHEADERTYPE* &buf : out_buf_headers) {
    OMX_CHECK(OMX_AllocateBuffer(handle, &buf, PORT_INDEX_OUT, this, out_port.nBufferSize));
  }

  wait_for_state(OMX_StateIdle);
  OMX_CHECK(OMX_SendCommand(handle, OMX_CommandStateSet, OMX_StateExecuting, NULL));
  wait_for_state(OMX_StateExecuting);

  for (OMX_BUFFERHEADERTYPE* &buf : out_buf_headers) {
    OMX_CHECK(OMX_FillThisBuffer(handle, buf));
  }
  for (OMX_BUFFERHEADERTYPE* &buf : in_buf_headers) {
    free_in->push(std::move(buf));
  }
}

void OmxEncoder::handle_out_buf(OmxEncoder *encoder, OMX_BUFFERHEADERTYPE *out_buf) {
  uint8_t *buf_data = out_buf->pBuffer + out_buf->nOffset;

  if (out_buf->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
    if (encoder->codec_config.size() < out_buf->nFilledLen) {
        encoder->codec_config.resize(out_buf->nFilledLen);
    }
    memcpy(encoder->codec_config.data(), buf_data, out_buf->nFilledLen);
#ifdef QCOM2
    out_buf->nTimeStamp = 0;
#endif
  }

  if (encoder->of) {
    fwrite(buf_data, out_buf->nFilledLen, 1, encoder->of);
  }

  if (!encoder->wrote_codec_config && !encoder->codec_config.empty()) {
    encoder->out_stream->codecpar->extradata = (uint8_t*)av_mallocz(encoder->codec_config.size() + AV_INPUT_BUFFER_PADDING_SIZE);
    encoder->out_stream->codecpar->extradata_size = encoder->codec_config.size();
    memcpy(encoder->out_stream->codecpar->extradata, encoder->codec_config.data(), encoder->codec_config.size());

    int err = avformat_write_header(encoder->ofmt_ctx, NULL);
    assert(err >= 0);
    encoder->wrote_codec_config = true;
  }

  if (out_buf->nTimeStamp > 0) {
    AVRational in_timebase = {1, 1000000};
    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = buf_data;
    pkt.size = out_buf->nFilledLen;

    enum AVRounding rnd = static_cast<enum AVRounding>(AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX);
    pkt.pts = pkt.dts = av_rescale_q_rnd(out_buf->nTimeStamp, in_timebase, encoder->out_stream->time_base, rnd);
    pkt.duration = av_rescale_q(1, AVRational{1, encoder->fps}, encoder->out_stream->time_base);

    if (out_buf->nFlags & OMX_BUFFERFLAG_SYNCFRAME) {
      pkt.flags |= AV_PKT_FLAG_KEY;
    }

    if (av_write_frame(encoder->ofmt_ctx, &pkt) < 0) {
      LOGW("ts encoder write issue");
    }
    av_packet_unref(&pkt);
  }

#ifdef QCOM2
  if (out_buf->nFlags & OMX_BUFFERFLAG_EOS) {
    out_buf->nTimeStamp = 0;
  }
#endif
  OMX_CHECK(OMX_FillThisBuffer(encoder->handle, out_buf));
}

int OmxEncoder::encode_frame_rgba(const uint8_t *ptr, int in_width, int in_height, uint64_t ts) {
  if (!is_open) return -1;

  OMX_BUFFERHEADERTYPE* in_buf = nullptr;
  if (!free_in->pop_wait_for(in_buf, std::chrono::milliseconds(50))) {
      LOGW("OmxEncoder: dropped frame, input queue full");
      return -1;
  }

  int ret = counter;
  uint8_t *in_buf_ptr = in_buf->pBuffer;

  int in_y_stride = VENUS_Y_STRIDE(COLOR_FMT_NV12, width);
  int in_uv_stride = VENUS_UV_STRIDE(COLOR_FMT_NV12, width);
  uint8_t *in_uv_ptr = in_buf_ptr + (in_y_stride * VENUS_Y_SCANLINES(COLOR_FMT_NV12, height));

  int err = ABGRToNV12(ptr, width * 4, in_buf_ptr, in_y_stride, in_uv_ptr, in_uv_stride, width, height);
  assert(err == 0);

  in_buf->nFilledLen = VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height);
  in_buf->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
  in_buf->nOffset = 0;
  in_buf->nTimeStamp = ts / 1000LL;
  last_t = in_buf->nTimeStamp;

  OMX_CHECK(OMX_EmptyThisBuffer(handle, in_buf));

  OMX_BUFFERHEADERTYPE *out_buf;
  while (done_out->try_pop(out_buf)) {
    handle_out_buf(this, out_buf);
  }

  dirty = true;
  counter++;
  return ret;
}

void OmxEncoder::encoder_open(const char* filename) {
  if (!filename || strlen(filename) == 0) return;

  std::filesystem::path p(path);
  std::error_code ec;
  std::filesystem::create_directories(p, ec);
  if (ec) {
    LOGE("Failed to create directories: %s", ec.message().c_str());
    return;
  }

  vid_path = (p / filename).string();

  if (avformat_alloc_output_context2(&ofmt_ctx, NULL, NULL, vid_path.c_str()) < 0 || !ofmt_ctx) {
    LOGE("Failed to allocate output context");
    return;
  }

  out_stream = avformat_new_stream(ofmt_ctx, NULL);
  if (!out_stream) {
    avformat_free_context(ofmt_ctx);
    ofmt_ctx = nullptr;
    return;
  }

  out_stream->time_base = AVRational{1, fps};
  out_stream->codecpar->codec_id = AV_CODEC_ID_H264;
  out_stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
  out_stream->codecpar->width = width;
  out_stream->codecpar->height = height;

  if (ofmt_ctx->oformat && ofmt_ctx->oformat->priv_class) {
    av_opt_set(ofmt_ctx->priv_data, "movflags", "faststart", 0);
  }

  if (avio_open(&ofmt_ctx->pb, vid_path.c_str(), AVIO_FLAG_WRITE) < 0) {
    avformat_free_context(ofmt_ctx);
    ofmt_ctx = nullptr;
    LOGE("Failed to open output file");
    return;
  }

  wrote_codec_config = false;

  lock_path = (p / (std::string(filename) + ".lock")).string();
  int lock_fd = HANDLE_EINTR(open(lock_path.c_str(), O_RDWR | O_CREAT, 0664));
  if (lock_fd >= 0) close(lock_fd);

  is_open = true;
  counter = 0;
}

void OmxEncoder::encoder_close() {
  if (!is_open) return;

  if (dirty) {
    OMX_BUFFERHEADERTYPE* in_buf = nullptr;
    if(free_in->pop_wait_for(in_buf, std::chrono::milliseconds(100))) {
      in_buf->nFilledLen = 0;
      in_buf->nOffset = 0;
      in_buf->nFlags = OMX_BUFFERFLAG_EOS;
      in_buf->nTimeStamp = last_t + 1000000LL / fps;
      OMX_CHECK(OMX_EmptyThisBuffer(handle, in_buf));

      int retries = 0;
      while (retries++ < 100) {
         OMX_BUFFERHEADERTYPE *out_buf;
         if(done_out->pop_wait_for(out_buf, std::chrono::milliseconds(10))) {
            handle_out_buf(this, out_buf);
            if (out_buf->nFlags & OMX_BUFFERFLAG_EOS) break;
         }
      }
    }
    dirty = false;
  }

  if (out_stream) {
    out_stream->nb_frames = counter;
    out_stream->duration = av_rescale_q(counter, AVRational{1, fps}, out_stream->time_base);
  }

  if (ofmt_ctx) {
    av_write_trailer(ofmt_ctx);
    avio_closep(&ofmt_ctx->pb);
    avformat_free_context(ofmt_ctx);
    ofmt_ctx = nullptr;
  }

  if (!lock_path.empty()) {
    unlink(lock_path.c_str());
  }

  is_open = false;
}

OmxEncoder::~OmxEncoder() {
  if (is_open) encoder_close();

  if (!handle) return;

  OMX_SendCommand(handle, OMX_CommandStateSet, OMX_StateIdle, NULL);
  wait_for_state(OMX_StateIdle);
  OMX_SendCommand(handle, OMX_CommandStateSet, OMX_StateLoaded, NULL);

  for (auto buf : in_buf_headers) OMX_FreeBuffer(handle, PORT_INDEX_IN, buf);
  for (auto buf : out_buf_headers) OMX_FreeBuffer(handle, PORT_INDEX_OUT, buf);

  wait_for_state(OMX_StateLoaded);
  OMX_FreeHandle(handle);
  OMX_Deinit();
}

#endif  // __APPLE__
