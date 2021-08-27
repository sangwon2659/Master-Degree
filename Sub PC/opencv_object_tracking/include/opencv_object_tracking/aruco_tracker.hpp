#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <opencv_object_tracking/Data.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using sensor_msgs::ImageConstPtr;
using sensor_msgs::PointCloud2;
using std::vector;
using opencv_object_tracking::Data;
using Eigen::Vector3d;
using Eigen::Matrix3d;

static const std::string windowName1 = "Gray Image";

class ArucoTracker{
    
    public:
    void initialization();
    void imageCallback(const ImageConstPtr& img_msg);
    void calcPose();
    void subscriberSetting();
    void publisherSetting();
    Matrix3d getRot(double yaw);

    private:
    ros::NodeHandle nh;
    ros::Publisher publisher_pose;
    image_transport::Subscriber image_subscriber;

    double markerSize;

    cv::Matx33f K;
    cv::Vec<float,5> k;
    
    cv::Mat rvec;
    cv::Mat tvec;
    Vector3d position;
    Matrix3d tf;
    cv::Mat translation;
    cv::Mat rotation;
    cv::Mat R;

    double theta;
    double s[3];
    double trace;
    float squareSize;
    float Rotation[3];

    Vector3d position_;
    Matrix3d R_;
};

void ArucoTracker::initialization()
{

    K << 610.84985, 0, 360.66794, 0, 609.7962, 240.23146, 0, 0, 1;
    k << 0.0923153, -0.163137, -0.00135823, 0.0230248, 0.103919;

    markerSize = 0.02; // 115 mm

    tf << 0, 0, 1, 
        -1, 0, 0,
        0, -1, 0;

    std::cout<<"camera matrix info\n";
    std::cout<<K<<std::endl;

    std::cout<<"distortion coefficients\n";
    std::cout<<k<<std::endl;

}

void ArucoTracker::imageCallback(const ImageConstPtr& img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv bridge exception: %s",e.what());
        return;
    }

    cv::Mat img;
    cv::cvtColor(cv_ptr->image,img,cv::IMREAD_COLOR);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
    cv::aruco::detectMarkers(img,dictionary,markerCorners,markerIds,parameters,rejectedCandidates);

    cv::aruco::drawDetectedMarkers(img,markerCorners,markerIds);

    if(markerIds.size()>0)
    {
        cv::aruco::estimatePoseSingleMarkers(markerCorners,markerSize,K,k,rvec,tvec);
        //cv::aruco::drawAxis(img,K,k,rvec,tvec,0.1);
        double* p = (double*) tvec.data;
        Vector3d p_; 
        p_ << p[0], p[1], p[2];
        position_ = tf*p_;

        cv::Rodrigues(rvec,R);
        rotation = R;
        calcPose();
    }


    if(markerIds.size() == 0){
        std::cout<<"Not detected\n";
        return;
    }

    //cv::imshow("image",img);
    cv::waitKey(1);

}

void ArucoTracker::calcPose()
{
    Data data;

    double* Rotation = (double*) rotation.data;

    trace = Rotation[0] + Rotation[4] + Rotation[8];
    theta = acos((trace-1.0)/2.0);

    double magnitude;

    s[0] = (Rotation[7] - Rotation[5])/sin(theta)/2.0;
    s[1] = (Rotation[2] - Rotation[6])/sin(theta)/2.0;
    s[2] = (Rotation[3] - Rotation[1])/sin(theta)/2.0;
    
    double qx, qy, qz, qw;
    double pitch;

    qw = cos(theta/2.0);
    qx = sin(theta/2.0)*s[0];
    qy = sin(theta/2.0)*s[1];
    qz = sin(theta/2.0)*s[2];

    if(theta == 0)      
    {
        qw = 1.0;
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
    }

    
    double roll, yaw;

    data.roll = asin(2*(qw*qy-qz*qx))*180.0/3.141459;
    data.pitch = atan2(2*(qw*qx+qy*qz),1.0-(qx*qx+qy*qy))*180.0/3.141459;
    data.yaw = atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))*180.0/3.141459;

    publisher_pose.publish(data);
}

void ArucoTracker::subscriberSetting()
{
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("/camera/color/image_raw",1,&ArucoTracker::imageCallback,this);
}

void ArucoTracker::publisherSetting()
{
    publisher_pose = nh.advertise<Data>("aruco_pose",1);
}

Matrix3d ArucoTracker::getRot(double yaw)
{
    Matrix3d rot;
    rot<<1, 0, 0,
        0, cos(yaw), -sin(yaw),
        0, sin(yaw), cos(yaw);

    return rot;
}
