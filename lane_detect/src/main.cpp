#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>

#include "detectlane.h"
#include "detectsign.h"
#include "carcontrol.h"
#include "laneline.h"

#define MAX_SPEED 30
// #define MIN_SPEED MAX_SPEED
#define MIN_SPEED 5

bool STREAM = true;

// VideoCapture capture("video.avi");
DetectLane * laneDetector;
DetectSign * signDetector;
CarControl *car;
int skipFrame = 1;

cv::Point driveCloseToLeft(std::shared_ptr<LaneLine> leftLane, int laneWidth)
{
    ROS_INFO("DRIVE CLOSE TO THE LEFT SIDE");
    cv::Point leftDrive;
    leftLane->getDrivePoint(leftDrive);
    return {leftDrive.x + 30, leftDrive.y};
}

cv::Point driveCloseToRight(std::shared_ptr<LaneLine> rightLane, int laneWidth)
{
    ROS_INFO("DRIVE CLOSE TO THE RIGHT SIDE");
    cv::Point rightDrive;
    rightLane->getDrivePoint(rightDrive);
    return {rightDrive.x - 30, rightDrive.y};
}

cv::Point driveStraight(std::shared_ptr<LaneLine> leftLane, std::shared_ptr<LaneLine> rightLane)
{
    ROS_INFO("DRIVE STRAIGHT");
    cv::Point leftDrive, rightDrive;
    leftLane->getDrivePoint(leftDrive);
    rightLane->getDrivePoint(rightDrive);
    return (leftDrive + rightDrive) / 2;
}

cv::Point turnLeft(std::shared_ptr<LaneLine> leftLane, std::shared_ptr<LaneLine> rightLane)
{
    ROS_INFO("TURN LEFT");
    if (rightLane->recover(leftLane, laneDetector->getLaneWidth()))
    {
        return driveStraight(leftLane, rightLane);// - cv::Point{120, 0}; // turn factor
    }
    return driveCloseToLeft(leftLane, laneDetector->getLaneWidth());
}

cv::Point turnRight(std::shared_ptr<LaneLine> leftLane, std::shared_ptr<LaneLine> rightLane)
{
    ROS_INFO("TURN RIGHT");
    if (leftLane->recover(rightLane, laneDetector->getLaneWidth()))
    {
        return driveStraight(leftLane, rightLane);// + cv::Point{120, 0}; // turn factor
    }
    return driveCloseToRight(rightLane, laneDetector->getLaneWidth());
}

static const int RATE = 15;
static int delay = RATE; // delay 1s
static int countTurning = RATE * 3; // turn in 3s
static int prevSign = 0, sign = 0;

void planning(cv::Point& drivePoint, int& driveSpeed)
{
    drivePoint = car->getCarPos();

    auto laneWidth = laneDetector->getLaneWidth();
    auto leftLane = laneDetector->getLeftLane();
    auto rightLane = laneDetector->getRightLane();

    prevSign = sign;
    sign = signDetector->detect();
    if (sign != 0)
    {
        // NOTE: test only
        sign = -1;
    }

    if (sign != 0)
    {
        ROS_INFO("Turn %d", sign);
        if (sign != prevSign)
        {
            countTurning = RATE * 3;
            delay = RATE;
        }
        driveSpeed = MIN_SPEED;
        leftLane->reset();
        rightLane->reset();
        laneDetector->detect();
    }
    else if (countTurning > 0)
    {
        sign = prevSign;
        if (delay > 0)
        {
            ROS_INFO("DELAY...");
            delay--;
        } else
        {
            ROS_INFO("TURNING...");
            countTurning--;
        }
    }
    

    if (leftLane->isFound() && rightLane->isFound())
    {
        if (sign == 0)
        {
            driveSpeed = MAX_SPEED;
            drivePoint = driveStraight(leftLane, rightLane);
        } else if (sign < 0)
        {
            drivePoint = turnLeft(leftLane, rightLane);
        } else
        {
            drivePoint = turnRight(leftLane, rightLane);
        }
    } else if (leftLane->isFound())
    {
        if (rightLane->recover(leftLane, laneWidth))
        {
            if (sign <= 0) // go straight or turn left
            {
                if (sign == 0)
                {
                    driveSpeed = MAX_SPEED;
                }
                drivePoint = driveStraight(leftLane, rightLane);
            } else
            {
                driveSpeed = MIN_SPEED;
                drivePoint = turnRight(leftLane, rightLane);
            }
        } else
        {
            if (sign > 0)
            {
                ROS_INFO("TURN RIGHT BUT RIGHT LANE NOT FOUND!!");
            }
            drivePoint = driveCloseToLeft(leftLane, laneWidth);
        }
    } else if (rightLane->isFound())
    {
        if (leftLane->recover(rightLane, laneWidth))
        {
            if (sign >= 0) // go straight or turn right
            {
                if (sign == 0)
                {
                    driveSpeed = MAX_SPEED;
                }
                drivePoint = driveStraight(leftLane, rightLane);
            } else
            {
                drivePoint = turnLeft(leftLane, rightLane);
            }
        } else
        {
            if (sign < 0)
            {
                ROS_INFO("TURN LEFT BUT LEFT LANE NOT FOUND!!");
            }
            drivePoint = driveCloseToRight(rightLane, laneWidth);
        }
    } else
    {
        // ROS_INFO("BOTH LANES NOT FOUND!");
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(cv_ptr->image.rows > 0)
        {
            laneDetector->updateRGB(cv_ptr->image);
            signDetector->updateRGB(cv_ptr->image);
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        if(cv_ptr->image.rows > 0)
        {
            laneDetector->updateDepth(cv_ptr->image);
        } 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{    
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    std::string path = ros::package::getPath("lane_detect");
    std::string left_path = path + "/images/left.png";
    std::string right_path = path + "/images/right.png";


    cv::namedWindow("Threshold");
    cv::namedWindow("RGB");
    cv::namedWindow("depth");
    laneDetector = new DetectLane();
    signDetector = new DetectSign(left_path, right_path);
    car = new CarControl();

    ros::Rate rate(RATE);

    int driveSpeed = MAX_SPEED;
    cv::Point drivePoint = car->getCarPos();

    image_transport::Subscriber sub2 = it.subscribe("team1/camera/depth", 1, imageCallback2);
    image_transport::Subscriber sub = it.subscribe("team1/camera/rgb", 1, imageCallback);

    while (ros::ok()) {
        ros::spinOnce();

        laneDetector->detect();
        planning(drivePoint, driveSpeed);
        car->driverCar(drivePoint, driveSpeed);
        laneDetector->show(&drivePoint);
        cv::waitKey(1);

        rate.sleep();
    } 
    cv::destroyAllWindows();

    delete signDetector;
    delete car;
    delete laneDetector;
}
