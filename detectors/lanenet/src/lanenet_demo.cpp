#include "lanenet/lanenet_demo.hpp"
// #include "argparse.hpp"

#include <ros/package.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <fstream>


LaneNetDemo::LaneNetDemo(ros::NodeHandle& nh, LaneNetParams const& params,
                         std::string const& rgbTopic,
                         std::string const& outLaneTopic,
                         std::string const& outRoadTopic,
                         image_transport::TransportHints const& transportHint)
    : lanenet(params),
      mIt(nh),
      mBatchSize(params.batchSize),
      mVisualize(true),
      isRoadUseDeep{true},
      isLaneUseDeep{true} {
  mImgSub = mIt.subscribe(rgbTopic, 4, &LaneNetDemo::imgCallback, this, transportHint);
  if (mVisualize) {
    mImgPub1 = mIt.advertise(outLaneTopic, 1);
    mImgPub2 = mIt.advertise(outRoadTopic, 1);
  }
  _srvLane = nh.advertiseService("setLaneUseDeep", &LaneNetDemo::setLaneUseDeepHandler, this);
  _srvRoad = nh.advertiseService("setRoadUseDeep", &LaneNetDemo::setRoadUseDeepHandler, this);
}

void LaneNetDemo::imgCallback(sensor_msgs::ImageConstPtr const& msg) {
  cv_bridge::CvImageConstPtr inImgPtr;
  try {
    inImgPtr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception const& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  cv::Mat laneProcessed, roadProcessed;
  auto start = std::chrono::high_resolution_clock::now();
  if (!isRoadUseDeep && !isLaneUseDeep)
  {
    laneProcessed = laneImageProc(inImgPtr->image);
    roadProcessed = roadImageProc(inImgPtr->image);
  } else
  {
    auto laneImgs = lanenet.inferLaneImg(inImgPtr->image);
    if (isRoadUseDeep)
    {
      cv::convertScaleAbs(laneImgs.second, roadProcessed, 255, 0);
      // cv::resize(roadProcessed, roadProcessed, inImgPtr->image.size());
      cv::resize(roadProcessed, roadProcessed, cv::Size{320, 240});

      cv::Mat road2 = roadProcessed.clone();
      auto point = cv::Point{roadProcessed.cols / 2, roadProcessed.rows - 10};
      if (road2.at<uint8_t>(point) > 250)
      {
        cv::floodFill(road2, cv::Point{roadProcessed.cols / 2, roadProcessed.rows - 10}, cv::Scalar{0});
        cv::bitwise_xor(roadProcessed, road2, roadProcessed);
      }
    } else
    {
      roadProcessed = roadImageProc(inImgPtr->image);
    }

    if (isLaneUseDeep)
    {
      cv::convertScaleAbs(laneImgs.first, laneProcessed, 255, 0);
      // cv::resize(laneProcessed, laneProcessed, inImgPtr->image.size());
      cv::resize(laneProcessed, laneProcessed, cv::Size{320, 240});
    } else
    {
      laneProcessed = laneImageProc(inImgPtr->image);
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  if (mVisualize && ++inferCount == 3) {
    cv_bridge::CvImage pubImg;
    pubImg.encoding = "mono8";
    pubImg.image = laneProcessed;
    mImgPub1.publish(pubImg.toImageMsg());
    pubImg.image = roadProcessed;
    mImgPub2.publish(pubImg.toImageMsg());
    inferCount = 0;
  }
}

cv::Mat LaneNetDemo::roadImageProc(const cv::Mat& image)
{
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, gray, 150, 255, cv::THRESH_BINARY_INV);
  return gray;
}

cv::Mat LaneNetDemo::laneImageProc(const cv::Mat& image)
{
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, gray, 150, 255, cv::THRESH_BINARY);
  return gray;
}

bool LaneNetDemo::setRoadUseDeepHandler(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  isRoadUseDeep = req.data;
  resp.success = true;
  return true;
}

bool LaneNetDemo::setLaneUseDeepHandler(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
{
  isLaneUseDeep = req.data;
  resp.success = true;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lanenet_benchmark", ros::init_options::NoRosout);
  ros::NodeHandle nh;

  LaneNetParams params;
  ros::param::param<int>("~batchSize", params.batchSize, 1);
  ros::param::param<int>("~channel", params.channel, 3);
  ros::param::param<int>("~height", params.height, 224);
  ros::param::param<int>("~width", params.width, 224);

  std::string engineFile;
  ros::param::get("~engineFile", engineFile);
  params.enginePath = ros::package::getPath("lanenet") / fs::path(engineFile);
  ROS_INFO("[LaneNet Benchmark] Load engine from %s", engineFile.c_str());

  std::string rgbTopic, laneSegTopic, roadSegTopic, transportHint;
  ROS_ASSERT(ros::param::get("/rgb_topic", rgbTopic));
  ROS_ASSERT(ros::param::get("/lane_segmentation_topic", laneSegTopic));
  ROS_ASSERT(ros::param::get("/road_segmentation_topic", roadSegTopic));
  ROS_ASSERT(ros::param::get("/transport_hint", transportHint));

  try {
    ROS_INFO("[LaneNet Benchmark] Initializing node");
    LaneNetDemo lnb(nh, params, rgbTopic, laneSegTopic, roadSegTopic, image_transport::TransportHints{transportHint});
    ros::spin();
  } catch (std::runtime_error const& ex) {
    ROS_FATAL_STREAM(ex.what());
    return EXIT_FAILURE;
  } catch (std::string const& ex) {
    ROS_FATAL_STREAM(ex);
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}