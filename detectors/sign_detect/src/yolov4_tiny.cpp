#include "yolov4/yolov4_tiny.hpp"
#include "yolov4/logger.hpp"

#include <NvInferPlugin.h>
#include <NvOnnxParser.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>

YoloV4Tiny::YoloV4Tiny(YoloParams const &params)
    : mBatchSize(params.batchSize),
      mChannel(params.channel),
      mHeight(params.height),
      mWidth(params.width),
      mKeepTopK(params.keepTopK) {
  initLibNvInferPlugins(&trt_logger::gLogger.getTRTLogger(), "");
  assert(mBatchSize == 1);
  assert(mChannel == 3);
  if (params.enginePath.empty() || !fs::exists(params.enginePath)) {
    throw std::invalid_argument("Cannot find engine");
  }
  loadEngine(params.enginePath);

  mBuffers = std::make_unique<trt_buf::BufferManager>(mEngine);
  if (!mBuffers) {
    trt_logger::gLogFatal << "Could not create BufferManager" << std::endl;
  }
  trt_logger::gLogInfo << "Allocated BufferManager" << std::endl;

  mContext = TrtUniquePtr<nvinfer1::IExecutionContext>(
      mEngine->createExecutionContext());
  if (!mContext) {
    trt_logger::gLogFatal << "Could not create IExecutionContext" << std::endl;
  }
  trt_logger::gLogInfo << "Ready for fun with sign" << std::endl;
}

void YoloV4Tiny::loadEngine(fs::path const &enginePath) {
  std::vector<char> trtModelStream;
  std::ifstream fi(enginePath, std::ios::binary);
  if (!fi.is_open()) {
    trt_logger::gLogError << "Failed open engine file" << std::endl;
    return;
  }
  fi.seekg(0, fi.end);
  size_t size = fi.tellg();
  fi.seekg(0, fi.beg);
  trtModelStream.resize(size);
  fi.read(trtModelStream.data(), size);

  TrtUniquePtr<nvinfer1::IRuntime> infer(
      nvinfer1::createInferRuntime(trt_logger::gLogger));
  //   if (mParams.dlaCore >= 0) {
  //     infer->setDLACore(mParams.dlaCore);
  //   }
  mEngine = std::shared_ptr<nvinfer1::ICudaEngine>(
      infer->deserializeCudaEngine(trtModelStream.data(), size, nullptr),
      InferDeleter());
  if (!mEngine) {
    trt_logger::gLogFatal << "Failed deserialize engine" << std::endl;
  }
  trt_logger::gLogInfo << "TRT Engine loaded from " << enginePath.filename()
                       << std::endl;
}

std::vector<BBox> YoloV4Tiny::detectImg(cv::Mat const &inImgs) {
  if (!processInput(inImgs)) {
    return {};
  }
  mBuffers->copyInputToDevice();
  if (!mContext->executeV2(mBuffers->getDeviceBindings().data())) {
    return {};
  }
  mBuffers->copyOutputToHost();
  return getBboxes();
}

bool YoloV4Tiny::processInput(cv::Mat const &inImg) {
  if (!mEngine || !mBuffers) {
    trt_logger::gLogFatal << "TRT engine or buffers not allocated" << std::endl;
    return false;
  }

  float *hostInputBuffer =
      static_cast<float *>(mBuffers->getHostBuffer(mInTensorName));

  cv::Mat inChannels[mChannel];
  cv::Mat resized;
  cv::resize(inImg, resized, {mWidth, mHeight}, 0, 0, cv::INTER_LINEAR);
  cv::split(resized, inChannels);

  int volBatch = mChannel * mHeight * mWidth;
  int volChannel = mHeight * mWidth;
  int volW = mWidth;

  int d_c_pos = 0;
  for (int c = 0; c < mChannel; ++c) {
    int s_h_pos = 0;
    int d_h_pos = d_c_pos;
    for (int h = 0; h < mHeight; ++h) {
      int s_pos = s_h_pos;
      int d_pos = d_h_pos;
      for (int w = 0; w < mWidth; ++w) {
        hostInputBuffer[d_pos] = float(inChannels[c].data[s_pos]) / 255;
        ++s_pos;
        ++d_pos;
      }
      s_h_pos += volW;
      d_h_pos += volW;
    }
    d_c_pos += volChannel;
  }

  return true;
}

std::vector<BBox> YoloV4Tiny::getBboxes() {
  int32_t *num_detections =
      static_cast<int32_t *>(mBuffers->getHostBuffer(mOutTensorNames[0]));
  float *nmsed_boxes =
      static_cast<float *>(mBuffers->getHostBuffer(mOutTensorNames[1]));
  float *nmsed_scores =
      static_cast<float *>(mBuffers->getHostBuffer(mOutTensorNames[2]));
  float *nmsed_classes =
      static_cast<float *>(mBuffers->getHostBuffer(mOutTensorNames[3]));

  if (!num_detections || !nmsed_boxes || !nmsed_scores || !nmsed_classes) {
    trt_logger::gLogWarning << "NULL value output detected!" << std::endl;
  }

  std::vector<BBox> bboxes;
  for (int t = 0; t < mKeepTopK; ++t) {
    if (static_cast<int>(nmsed_classes[t]) < 0) {
      break;
    }

    int box_coord_pos = 4 * t;
    float x1 = nmsed_boxes[box_coord_pos];
    float y1 = nmsed_boxes[box_coord_pos + 1];
    float x2 = nmsed_boxes[box_coord_pos + 2];
    float y2 = nmsed_boxes[box_coord_pos + 3];

    bboxes.emplace_back(BBox{std::min(x1, x2), std::min(y1, y2),
                             std::max(x1, x2), std::max(y1, y2),
                             nmsed_scores[t], int(nmsed_classes[t])});
  }

  return bboxes;
}