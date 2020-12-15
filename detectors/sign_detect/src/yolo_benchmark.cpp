#include "sign_detect/yolo_benchmark.hpp"

#include <ros/package.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cds_msgs/sign.h>

#include <fstream>

YoloOnnxTrt::YoloOnnxTrt(ros::NodeHandle const& nh, YoloParams const& params,
                         std::string const& subTopic,
                         std::string const& pubTopic,
                         std::string const& pubIDTopic)
    : yolo(params),
      mNh(nh),
      mIt(nh),
      mBatchSize(params.batchSize),
      mVisualize(!pubTopic.empty()),
      counter{10} {
  std::string transport_hint;
  ROS_ASSERT(ros::param::get("/transport_hint", transport_hint));
  mImgSub = mIt.subscribe(subTopic, 4, &YoloOnnxTrt::imgCallback, this, image_transport::TransportHints{transport_hint});
  if (mVisualize) {
    mImgPub = mIt.advertise(pubTopic, 1);
  }
  signPub = mNh.advertise<cds_msgs::sign>(pubIDTopic, 1);
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

cv::Mat YoloOnnxTrt::drawSampleBboxes(cv::Mat const& img,
                                      std::vector<BBox> const& bboxes) {
  auto sampleImg = img.clone();
  int H = sampleImg.rows;
  int W = sampleImg.cols;

  for (auto const& box : bboxes) {
    if (box.cls != -1) {
      int x1 = box.x1 * W;
      int y1 = box.y1 * H;
      int x2 = box.x2 * W;
      int y2 = box.y2 * H;
      cv::rectangle(sampleImg, {x1, y1}, {x2, y2}, cv::Scalar(255, 0, 0), 2);
      cv::putText(sampleImg,           // target image
                  mClsNames[box.cls],  // text
                  {x1, y1},            // top-left position
                  cv::FONT_HERSHEY_DUPLEX, 1,
                  CV_RGB(118, 185, 0),  // font color
                  2);
    }
  }
  return sampleImg;
}

void YoloOnnxTrt::imgCallback(sensor_msgs::ImageConstPtr const& msg) {
  cds_msgs::sign signID_msg;
  cv_bridge::CvImageConstPtr inImgPtr;
  try {
    inImgPtr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception const& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }
  auto start = std::chrono::high_resolution_clock::now();
  cv::Mat inputImage, outputImage;
  cv::cvtColor(inImgPtr->image, inputImage, cv::COLOR_BGR2RGB);
  auto bboxes = yolo.detectImg(inputImage);

  {
    // debug history
    std::string log;
    const auto& history = counter.getHistory();
    for (const auto& h : history)
    {
      log += std::to_string(h) + " ";
    }
    ROS_DEBUG_STREAM("History: " << log);
  }


  {
    // debug counter
    std::string log1, log2;
    const auto& c = counter.getCounter();
    for (const auto& pair : c)
    {
      log1 += std::to_string(pair.first) + " ";
      log2 += std::to_string(pair.second) + " ";
    }
    ROS_DEBUG_STREAM("Counter: " << log1);
    ROS_DEBUG_STREAM("Counter: " << log2);
  }

  if (bboxes.size() > 0)
  {
    counter.update(bboxes[0].cls + 1);
    int maxCount, signId;
    if (counter.getMaxCount(signId, maxCount))
    {
      signID_msg.sign_id = signId;
    }
    signPub.publish(signID_msg);
  }
  // ROS_INFO_STREAM("BBOXES LENGTH = " << bboxes.size());
  auto end = std::chrono::high_resolution_clock::now();
  if (mVisualize && ++inferCount == 3) {
    cv_bridge::CvImage outImg;
    outImg.encoding = "bgr8";
    outputImage = drawSampleBboxes(inImgPtr->image, bboxes);
    cv::cvtColor(outputImage, outImg.image, cv::COLOR_RGB2BGR);
    mImgPub.publish(outImg.toImageMsg());
    inferCount = 0;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "yolo_benchmark", ros::init_options::NoRosout);
  ros::NodeHandle nh;

  YoloParams params;
  ros::param::param<int>("~batchSize", params.batchSize, 1);
  ros::param::param<int>("~channel", params.channel, 3);
  ros::param::param<int>("~height", params.height, 288);
  ros::param::param<int>("~width", params.width, 384);
  ros::param::param<int>("~keepTopK", params.keepTopK, 1);
  ros::param::param<int>("~nbCls", params.nbCls, 6);

  std::string rgbTopic, signTopic, signIdTopic;
  ROS_ASSERT(ros::param::get("/rgb_topic", rgbTopic));
  ROS_ASSERT(ros::param::get("/sign_topic", signTopic));
  ROS_ASSERT(ros::param::get("/signid_topic", signIdTopic));

  std::string engineFile;
  ros::param::get("~engineFile", engineFile);
  params.enginePath = ros::package::getPath("sign_detect") / fs::path(engineFile);

  try {
    ROS_INFO("[YoloOnnxTrt] Initializing node");
    YoloOnnxTrt yot(nh, params, rgbTopic, signTopic, signIdTopic);
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