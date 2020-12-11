#pragma once

#include "trt/buffers.hpp"

#include <NvInfer.h>
#include <opencv2/core.hpp>

#include <array>
#include <experimental/filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::experimental::filesystem;

struct InferDeleter {
  template <typename T>
  void operator()(T *obj) const {
    if (obj) {
      obj->destroy();
    }
  }
};

struct LaneNetParams {
  fs::path enginePath;
  int batchSize, channel, width, height;
};

class LaneNet {
  template <typename T>
  using TrtUniquePtr = std::unique_ptr<T, InferDeleter>;

 public:
  LaneNet(LaneNetParams const &params);
  std::pair<cv::Mat, cv::Mat> inferLaneImg(cv::Mat const &inImg);

 private:
  void loadEngine(fs::path const &enginePath);
  bool processInput(cv::Mat const &inImg);
  std::pair<cv::Mat, cv::Mat> extractLane();

  std::shared_ptr<nvinfer1::ICudaEngine> mEngine;
  std::unique_ptr<trt_buf::BufferManager> mBuffers;
  TrtUniquePtr<nvinfer1::IExecutionContext> mContext;
  int mBatchSize, mChannel, mWidth, mHeight;

  inline static std::string const mInTensorName = "input";
  inline static std::array<std::string, 2> const mOutTensorNames = {"output_0",
                                                                    "output_1"};
};