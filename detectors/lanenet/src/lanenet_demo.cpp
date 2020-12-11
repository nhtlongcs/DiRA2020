#include "lanenet/lanenet_demo.hpp"
// #include "argparse.hpp"

#include <ros/package.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <fstream>


LaneNetDemo::LaneNetDemo(ros::NodeHandle const& nh, LaneNetParams const& params,
                         std::string const& rgbTopic,
                         std::string const& outLaneTopic,
                         std::string const& outRoadTopic,
                         image_transport::TransportHints const& transportHint)
    : lanenet(params),
      mIt(nh),
      mBatchSize(params.batchSize),
      mVisualize(true) {
  mImgSub = mIt.subscribe(rgbTopic, 4, &LaneNetDemo::imgCallback, this, transportHint);
  if (mVisualize) {
    mImgPub1 = mIt.advertise(outLaneTopic, 1);
    mImgPub2 = mIt.advertise(outRoadTopic, 1);
  }
}

void LaneNetDemo::imgCallback(sensor_msgs::ImageConstPtr const& msg) {
  cv_bridge::CvImageConstPtr inImgPtr;
  try {
    inImgPtr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception const& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }
  auto start = std::chrono::high_resolution_clock::now();
  auto laneImgs = lanenet.inferLaneImg(inImgPtr->image);
  auto end = std::chrono::high_resolution_clock::now();
  if (mVisualize && ++inferCount == 3) {
    cv_bridge::CvImage pubImg;
    pubImg.encoding = "mono8";
    cv::convertScaleAbs(laneImgs.first, pubImg.image, 255, 0);
    cv::resize(pubImg.image, pubImg.image, inImgPtr->image.size());
    mImgPub1.publish(pubImg.toImageMsg());
    cv::convertScaleAbs(laneImgs.second, pubImg.image, 255, 0);
    cv::resize(pubImg.image, pubImg.image, inImgPtr->image.size());
    mImgPub2.publish(pubImg.toImageMsg());
    inferCount = 0;
  } else if (!mVisualize) {
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