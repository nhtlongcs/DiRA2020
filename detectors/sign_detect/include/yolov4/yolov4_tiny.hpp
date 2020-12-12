#pragma once

#include "buffers.hpp"

#include <NvInfer.h>
#include <opencv2/core.hpp>

#include <array>
#include <experimental/filesystem>
#include <memory>
#include <string>
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

struct BBox {
  float x1;
  float y1;
  float x2;
  float y2;
  float score;
  int cls;
};

struct YoloParams {
  fs::path enginePath;
  int nbCls;
  int batchSize, channel, width, height;
  int keepTopK;
};

class YoloV4Tiny {
  template <typename T>
  using TrtUniquePtr = std::unique_ptr<T, InferDeleter>;

 public:
  YoloV4Tiny(YoloParams const &params);
  std::vector<BBox> detectImg(cv::Mat const &inImgs);

 private:
  void loadEngine(fs::path const &enginePath);
  bool processInput(cv::Mat const &inImg);
  std::vector<BBox> getBboxes();

  std::shared_ptr<nvinfer1::ICudaEngine> mEngine;
  std::unique_ptr<trt_buf::BufferManager> mBuffers;
  TrtUniquePtr<nvinfer1::IExecutionContext> mContext;
  int mBatchSize, mChannel, mWidth, mHeight;
  int mKeepTopK;

  inline static std::string const mInTensorName = "input";
  inline static std::array<std::string, 4> const mOutTensorNames = {
      "num_detections", "nmsed_boxes", "nmsed_scores", "nmsed_classes"};
};