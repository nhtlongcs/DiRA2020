#include <ros/ros.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

const std::string camurl = "file:///home/ken/catkin_ws/src/virtualcamera/cfg/sony_calibration.yaml";

image_transport::Publisher colorPub, depthPub;
ros::Publisher caminfoPub;
camera_info_manager::CameraInfoManager *caminfo;

void imageCallback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::ImageConstPtr& depth)
{
  const auto& colorRepublish = color;

  sensor_msgs::CameraInfo ci;
  ci = caminfo->getCameraInfo();
  ci.header = colorRepublish->header;

  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImage out_ptr;

  cv::Mat out;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::BGR8);
    if (cv_ptr->image.empty())
    {
      return;
    }

    cv::cvtColor(cv_ptr->image, out, cv::COLOR_BGR2GRAY);
    out.convertTo(out, CV_16UC1, USHRT_MAX * 1.0f / 255);

    cv::resize(out, out, {1280, 960});
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", depth->encoding.c_str());
    return;
  }
  
  out_ptr.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  out_ptr.image = out;
  const auto depthRepublish = out_ptr.toImageMsg();

  // NOTE: this will change frame_id of depth to color frame_id
  depthRepublish->header = colorRepublish->header;

  ROS_INFO("depth frame_id = %s, color frame_id = %s, camera frame_id = %s", depthRepublish->header.frame_id.c_str(), colorRepublish->header.frame_id.c_str(), ci.header.frame_id.c_str());

  depthPub.publish(depthRepublish);
  colorPub.publish(colorRepublish);
  caminfoPub.publish(ci);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "virtual_camera");
  ros::NodeHandle nh;

  std::string camera_name = "/hcmus_avengers/camera/rgb";
  image_transport::ImageTransport it(nh);
  colorPub = it.advertise(camera_name + "/image_raw", 1);
  depthPub = it.advertise("depth_output", 1);
  caminfoPub = nh.advertise<sensor_msgs::CameraInfo>(camera_name + "/camera_info", 1);

  caminfo = new camera_info_manager::CameraInfoManager(nh, camera_name, camurl);

  image_transport::SubscriberFilter color_sub(it, "/hcmus_avengers/camera/rgb", 1);
  image_transport::SubscriberFilter depth_sub(it, "/hcmus_avengers/camera/depth", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync{MySyncPolicy( 5 ), color_sub, depth_sub};
  sync.registerCallback(imageCallback);

  ros::spin();

  delete caminfo;

  return 0;
}