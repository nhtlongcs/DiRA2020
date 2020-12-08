#include "yolov4/yolov4_tiny.hpp"
#include "yolov4/logger.hpp"

#include <NvInferPlugin.h>
#include <NvOnnxParser.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>

namespace {
static long long constexpr int operator"" _GiB(long long unsigned int val) {
  return val * (1 << 30);
}
static long long constexpr int operator"" _MiB(long long unsigned int val) {
  return val * (1 << 20);
}
static long long constexpr int operator"" _KiB(long long unsigned int val) {
  return val * (1 << 10);
}
}  // namespace

YoloV4Tiny::YoloV4Tiny(YoloParams const &params)
    : mBatchSize(params.batchSize),
      mChannel(params.channel),
      mHeight(params.height),
      mWidth(params.width),
      mKeepTopK(params.keepTopK) {
  initLibNvInferPlugins(&trt_logger::gLogger.getTRTLogger(), "");
  assert(!params.enginePath.empty());
  if (fs::exists(params.enginePath)) {
    loadEngine(params.enginePath);
  } else {
    parseOnnxModel(params.onnxPath, params.enginePath);
  }

  assert(mChannel == 3);
  mBuffers = std::make_unique<trt_buf::BufferManager>(mEngine);
  if (!mBuffers) {
    trt_logger::gLogError << "Could not create BufferManager" << std::endl;
  }
  trt_logger::gLogInfo << "Allocated BufferManager" << std::endl;

  mContext = TrtUniquePtr<nvinfer1::IExecutionContext>(
      mEngine->createExecutionContext());
  if (!mContext) {
    trt_logger::gLogError << "Could not create IExecutionContext" << std::endl;
  }
  trt_logger::gLogInfo << "Ready for fun" << std::endl;
}

void YoloV4Tiny::parseOnnxModel(fs::path const &onnxPath,
                                fs::path const &enginePath) {
  auto builder = TrtUniquePtr<nvinfer1::IBuilder>(
      nvinfer1::createInferBuilder(trt_logger::gLogger.getTRTLogger()));
  if (!builder) {
    trt_logger::gLogError << "Failed create IBuilder" << std::endl;
  }

  const auto explicitBatch =
      1U << static_cast<uint32_t>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = TrtUniquePtr<nvinfer1::INetworkDefinition>(
      builder->createNetworkV2(explicitBatch));
  if (!network) {
    trt_logger::gLogError << "Failed create INetworkDefinition" << std::endl;
  }

  auto config =
      TrtUniquePtr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  if (!config) {
    trt_logger::gLogError << "Failed create IBuilderConfig" << std::endl;
  }

  auto parser = TrtUniquePtr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, trt_logger::gLogger.getTRTLogger()));
  if (!parser) {
    trt_logger::gLogError << "Failed create IParser" << std::endl;
  }

  trt_logger::gLogInfo << "Parsing ONNX file: " << std::endl;

  if (!parser->parseFromFile(onnxPath.c_str(),
                             int(nvinfer1::ILogger::Severity::kERROR))) {
    trt_logger::gLogError << "Unable to parse ONNX model file: "
                          << onnxPath.filename() << std::endl;
  }

  builder->setMaxBatchSize(0);
  config->setMaxWorkspaceSize(2_GiB);
  if (builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  };
  trt_logger::gLogInfo << "Building TensorRT engine " << std::endl;

  mEngine = std::shared_ptr<nvinfer1::ICudaEngine>(
      builder->buildEngineWithConfig(*network, *config), InferDeleter());
  if (!mEngine) {
    trt_logger::gLogError << "Failed build engine" << std::endl;
  }

  std::ofstream fo(enginePath, std::ios::binary);
  if (!fo.is_open()) {
    trt_logger::gLogError << "Failed write engine file" << std::endl;
  }
  TrtUniquePtr<nvinfer1::IHostMemory> hostMem(mEngine->serialize());
  assert(hostMem);
  fo.write(reinterpret_cast<const char *>(hostMem->data()), hostMem->size());
  trt_logger::gLogInfo << "TRT Engine file saved to " << enginePath.filename()
                       << std::endl;

  // assert(network->getNbInputs() == 1);
  auto inDims = network->getInput(0)->getDimensions();
  assert(inDims.nbDims == 4);
  mBatchSize = inDims.d[0];
  mChannel = inDims.d[1];
  mHeight = inDims.d[2];
  mWidth = inDims.d[3];

  //   // Calibrator life time needs to last until after the engine is built.
  //   std::unique_ptr<IInt8Calibrator> calibrator;

  //   // issue for int8 mode
  //   if (mParams.int8) {
  //     BatchStream calibrationStream(mParams.explicitBatchSize,
  //                                   mParams.nbCalBatches,
  //                                   mParams.calibrationBatches,
  //                                   mParams.dataDirs);
  //     calibrator.reset(new Int8EntropyCalibrator2<BatchStream>(
  //         calibrationStream, 0, "Yolo",
  //         mParams.inputTensorNames[0].c_str()));
  //     config->setFlag(BuilderFlag::kINT8);
  //     config->setInt8Calibrator(calibrator.get());
  //   }

  // Enable DLA if mParams.dlaCore is true
  //   samplesCommon::enableDLA(builder.get(), config.get(), mParams.dlaCore);
}

void YoloV4Tiny::loadEngine(fs::path const &enginePath) {
  std::vector<char> trtModelStream;
  size_t size = 0;
  std::ifstream fi(enginePath, std::ios::binary);
  if (!fi.is_open()) {
    trt_logger::gLogError << "Failed open engine file" << std::endl;
    return;
  }
  fi.seekg(0, fi.end);
  size = fi.tellg();
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
    trt_logger::gLogError << "Failed deserialize engine" << std::endl;
  }
  trt_logger::gLogInfo << "TRT Engine loaded from " << enginePath.filename()
                       << std::endl;
}

std::vector<std::vector<BBox>> YoloV4Tiny::detectBatch(
    std::vector<cv::Mat> const &inImgs) {
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

bool YoloV4Tiny::processInput(std::vector<cv::Mat> const &inImgs) {
  if (!mEngine || !mBuffers) {
    trt_logger::gLogFatal << "TRT engine or buffers not allocated" << std::endl;
    return false;
  }
  int nbInImgs = inImgs.size();
  if (mBatchSize < nbInImgs || inImgs.empty()) {
    trt_logger::gLogError << "Wrong input size" << std::endl;
    return false;
  }

  float *hostInputBuffer =
      static_cast<float *>(mBuffers->getHostBuffer(mInTensorNames));

  std::vector<std::vector<cv::Mat>> inChannels;
  for (auto b = 0; b < mBatchSize; ++b) {
    inChannels.emplace_back(mChannel);  // TODO confirm this shiet
    if (b < nbInImgs) {
      // Input is cv::Mat RGB
      // cv::cvtColor(this->mSampleImage, rgb_img, cv::COLOR_BGR2RGB);

      cv::Mat resized;
      cv::resize(inImgs[b], resized, {mWidth, mHeight}, 0, 0, cv::INTER_LINEAR);
      cv::split(resized, inChannels[b]);
    } else {
      cv::Mat zeros = cv::Mat::zeros({mWidth, mHeight}, CV_8UC3);
      cv::split(zeros, inChannels[b]);
    }
  }

  int volBatch = mChannel * mHeight * mWidth;
  int volChannel = mHeight * mWidth;
  int volW = mWidth;
  int d_batch_pos = 0;

  for (int b = 0; b < mBatchSize; ++b) {
    int d_c_pos = d_batch_pos;
    for (int c = 0; c < mChannel; ++c) {
      int s_h_pos = 0;
      int d_h_pos = d_c_pos;
      for (int h = 0; h < mHeight; ++h) {
        int s_pos = s_h_pos;
        int d_pos = d_h_pos;
        for (int w = 0; w < mWidth; ++w) {
          hostInputBuffer[d_pos] = float(inChannels[b][c].data[s_pos]) / 255.0f;
          ++s_pos;
          ++d_pos;
        }
        s_h_pos += volW;
        d_h_pos += volW;
      }
      d_c_pos += volChannel;
    }
    d_batch_pos += volBatch;
  }

  return true;
}

std::vector<std::vector<BBox>> YoloV4Tiny::getBboxes() {
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

  int n_detect_pos = 0;
  int box_pos = 0;
  int score_pos = 0;
  int cls_pos = 0;

  std::vector<std::vector<BBox>> bboxes(mBatchSize);
  for (auto &box : bboxes) {
    for (int t = 0; t < mKeepTopK; ++t) {
      if (static_cast<int>(nmsed_classes[cls_pos + t]) < 0) {
        break;
      }

      int box_coord_pos = box_pos + 4 * t;
      float x1 = nmsed_boxes[box_coord_pos];
      float y1 = nmsed_boxes[box_coord_pos + 1];
      float x2 = nmsed_boxes[box_coord_pos + 2];
      float y2 = nmsed_boxes[box_coord_pos + 3];

      box.emplace_back(BBox{std::min(x1, x2), std::min(y1, y2),
                            std::max(x1, x2), std::max(y1, y2),
                            nmsed_scores[score_pos + t],
                            int(nmsed_classes[cls_pos + t])});
    }

    ++n_detect_pos;
    box_pos += 4 * mKeepTopK;
    score_pos += mKeepTopK;
    cls_pos += mKeepTopK;
  }
  return bboxes;
}