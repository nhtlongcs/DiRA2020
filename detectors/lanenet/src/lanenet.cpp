#include "lanenet/lanenet.hpp"
#include "trt/logger.hpp"

#include <NvInferPlugin.h>
#include <NvOnnxParser.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>

LaneNet::LaneNet(LaneNetParams const &params)
    : mBatchSize(params.batchSize),
      mChannel(params.channel),
      mHeight(params.height),
      mWidth(params.width) {
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
  trt_logger::gLogInfo << "Ready for fun with lane" << std::endl;
}

void LaneNet::loadEngine(fs::path const &enginePath) {
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

std::pair<cv::Mat, cv::Mat> LaneNet::inferLaneImg(cv::Mat const &inImg) {
  if (!processInput(inImg)) {
    return {};
  }
  mBuffers->copyInputToDevice();
  if (!mContext->executeV2(mBuffers->getDeviceBindings().data())) {
    return {};
  }
  mBuffers->copyOutputToHost();
  return extractLane();
}

bool LaneNet::processInput(cv::Mat const &inImg) {
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

  // TODO change to inImg image

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

std::pair<cv::Mat, cv::Mat> LaneNet::extractLane() {
  float *img1 =
      static_cast<float *>(mBuffers->getHostBuffer(mOutTensorNames[0]));
  float *img2 =
      static_cast<float *>(mBuffers->getHostBuffer(mOutTensorNames[1]));

  if (!img1 || !img2) {
    trt_logger::gLogWarning << "NULL value output detected!" << std::endl;
  }

  return std::make_pair(cv::Mat(mHeight, mWidth, CV_32FC1, img1),
                        cv::Mat(mHeight, mWidth, CV_32FC1, img2));
}