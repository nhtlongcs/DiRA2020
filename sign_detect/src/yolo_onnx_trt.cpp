#include "sign_detect/yolo_onnx_trt.hpp"
#include "argparse.hpp"

#include <ros/package.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <fstream>

YoloOnnxTrt::YoloOnnxTrt(ros::NodeHandle const& nh, YoloParams const& params,
                         std::string const& subTopic,
                         std::string const& pubTopic)
    : yolo(params),
      mNh(nh),
      mIt(nh),
      mBatchSize(params.batchSize),
      mVisualize(!pubTopic.empty()) {
  mImgSub = mIt.subscribe(subTopic, 4, &YoloOnnxTrt::imgCallback, this);
  if (mVisualize) {
    mImgPub = mIt.advertise(pubTopic, 1);
  }
  readClsName();
  ROS_ASSERT(mClsNames.size() == params.nbCls);
}

void YoloOnnxTrt::readClsName() {
  fs::path clsNamePath =
      ros::package::getPath("sign_detect") / fs::path("data/signs.txt");
  std::ifstream fi(clsNamePath);
  std::string line;
  while (!fi.eof()) {
    std::getline(fi, line);
    if (!line.empty()) {
      mClsNames.emplace_back(line);
    }
  }
}

void YoloOnnxTrt::drawSampleBboxes(std::vector<BBox> const& bboxes) {
  int H = mSampleImg.rows;
  int W = mSampleImg.cols;

  for (auto const& box : bboxes) {
    if (box.cls != -1) {
      int x1 = box.x1 * W;
      int y1 = box.y1 * H;
      int x2 = box.x2 * W;
      int y2 = box.y2 * H;
      cv::rectangle(mSampleImg, {x1, y1}, {x2, y2}, cv::Scalar(255, 0, 0), 2);
      cv::putText(mSampleImg,          // target image
                  mClsNames[box.cls],  // text
                  {x1, y1},            // top-left position
                  cv::FONT_HERSHEY_DUPLEX, 0.8,
                  CV_RGB(118, 185, 0),  // font color
                  2);
    }
  }
}

void YoloOnnxTrt::imgCallback(sensor_msgs::ImageConstPtr const& msg) {
  cv_bridge::CvImageConstPtr inImgPtr;
  try {
    inImgPtr = cv_bridge::toCvShare(msg);  // TODO color
  } catch (cv_bridge::Exception const& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  if (mBatchImgs.size() == mBatchSize) {
    auto start = std::chrono::high_resolution_clock::now();
    auto bboxes = yolo.detectBatch(mBatchImgs);
    auto end = std::chrono::high_resolution_clock::now();

    if (mVisualize) {
      mSampleImg = mBatchImgs.front().clone();
      drawSampleBboxes(bboxes.front());
      cv_bridge::CvImage outImg;
      outImg.encoding = "rgb8";
      outImg.image = mSampleImg;
      mImgPub.publish(outImg.toImageMsg());
    } else {
      runTime += std::chrono::duration<double, std::milli>(end - start).count();
      ROS_INFO_STREAM("Iteration " << ++curBench);
      int nbRun = nbBench / mBatchSize;
      if (curBench >= nbRun) {
        double msPerIteration = runTime / nbRun / mBatchSize;
        ROS_INFO_STREAM("Time each image: " << int(msPerIteration) << " ms");
        ROS_INFO_STREAM("FPS: " << int(1000 / msPerIteration));
        ros::shutdown();
      }
    }

    mBatchImgs.clear();
  }
  mBatchImgs.emplace_back(inImgPtr->image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolo_onnx_trt", ros::init_options::NoRosout);
  ros::NodeHandle nh;

  YoloParams params;
  ros::param::param<int>("batchSize", params.batchSize, 1);
  ros::param::param<int>("channel", params.channel, 3);
  ros::param::param<int>("height", params.height, 288);
  ros::param::param<int>("width", params.width, 384);
  ros::param::param<int>("keepTopK", params.keepTopK, 1);
  ros::param::param<int>("nbCls", params.nbCls, 6);

  bool visualize;
  ros::param::param<bool>("visualize", visualize, false);

  std::string onnxFile;
  ros::param::param<std::string>("onnxFile", onnxFile,
                                 "data/yolov4-tiny_1_3_288_384_static");
  params.onnxPath =
      ros::package::getPath("sign_detect") / fs::path(onnxFile + "_nms.onnx");
  params.enginePath =
      ros::package::getPath("sign_detect") / fs::path(onnxFile + ".trt");

  argparse::ArgumentParser program("yolo_onnx_trt");
  program.add_argument("-i", "--in")
      .help("input camera topic")
      .default_value(std::string("/camera/rgb/image_raw"));
  program.add_argument("-o", "--out")
      .help("output topic, implies visualize")
      .default_value(std::string(""));
  ;
  ROS_INFO_STREAM(program.get<std::string>("-o"));
  for (int i = 0; i < argc; ++i) {
    ROS_INFO_STREAM(argv[i]);
  }

  try {
    program.parse_args(argc, argv);
    ROS_INFO("[YoloOnnxTrt] Initializing node");
    YoloOnnxTrt yot(nh, params, program.get<std::string>("-i"),
                    program.get<std::string>("-o"));
    ros::spin();
  } catch (std::runtime_error const& err) {
    ROS_FATAL_STREAM(err.what());
    return EXIT_FAILURE;
  } catch (std::string const& err) {
    ROS_FATAL_STREAM(err);
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}