/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "../../../include/Converter.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include"../../../include/System.h"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

ros::Publisher pub_pose;
ros::Publisher pub_poseStamped;
bool isFirst = true;
cv::Mat cv_zero;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

	pub_pose = nodeHandler.advertise<geometry_msgs::Pose>("pose", 1000);
	pub_poseStamped = nodeHandler.advertise<geometry_msgs::PoseStamped>("poseStamped", 1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

// 以下を参考にした
// https://github.com/appliedAI-Initiative/orb_slam_2_ros/blob/master/ros/src/Node.cc
tf2::Transform TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);


  tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
									rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
									rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
								   );

  tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
									-1, 0, 0,
									 0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

geometry_msgs::Transform ConvertPositionToTransform (std_msgs::Header header, cv::Mat position) {
	tf2::Transform tf_position = TransformFromMat(position);
	//geometry_msgs::PoseStamped pose_msg;
	geometry_msgs::Transform trans_msg;
	//trans_msg.header = header;
	trans_msg = tf2::toMsg(tf_position); 
	//std::cout << "trans_msg=" << trans_msg << std::endl << std::endl;
	return trans_msg;
}

geometry_msgs::TransformStamped ConvertPositionToTransformStamped (std_msgs::Header header, cv::Mat position, string child_frame_id) {
	tf2::Transform tf_position = TransformFromMat(position);
	//geometry_msgs::PoseStamped pose_msg;
	geometry_msgs::TransformStamped trans_msg;
	trans_msg.header = header;
	trans_msg.transform = tf2::toMsg(tf_position); 
	trans_msg.child_frame_id = child_frame_id;
	//std::cout << "trans_msg=" << trans_msg << std::endl << std::endl;
	return trans_msg;
}

/*
bool areEqual(const cv::Mat& a, const cv::Mat& b) {
    cv::Mat temp;
    cv::bitwise_xor(a,b,temp); //It vectorizes well with SSE/NEON
    return !(cv::countNonZero(temp) );
}
*/

bool matIsEqual(const cv::Mat mat1, const cv::Mat mat2){
    // treat two empty mat as identical as well
    if (mat1.empty() && mat2.empty()) {
        return true;
    }
    // if dimensionality of two mat is not identical, these two mat is not identical
    if (mat1.cols != mat2.cols || mat1.rows != mat2.rows || mat1.dims != mat2.dims) {
        return false;
    }
    cv::Mat diff;
    cv::compare(mat1, mat2, diff, cv::CMP_NE);
    int nz = cv::countNonZero(diff);
    return nz==0;
}


void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f SE3f = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
	// https://github.com/strasdat/Sophus/issues/80
	std::cout << "SE3f.matrix()=" << SE3f.matrix() << std::endl << std::endl;
	std::cout << "SE3f.matrix()=" << typeid(SE3f.matrix()).name() << std::endl << std::endl;
	//std::cout << "SE3f.unit_quaternion()=" << SE3f.unit_quaternion() << std::endl << std::endl;
	//std::cout << "SE3f.log()=" << SE3f.log() << std::endl << std::endl;
	Eigen::Matrix4f eigen_mat = SE3f.matrix();
	std::cout << "eigen_mat" << eigen_mat<< std::endl << std::endl;

	// Eigen::Matrixからcv::Matへの変換
	// http://dronevisionml.blogspot.com/2015/07/opencv-mat-eigen.html
	cv::Mat cv_mat;
	cv::eigen2cv(eigen_mat, cv_mat);
	std::cout << "cv_mat=" << cv_mat << std::endl << std::endl;

	if (isFirst) {
		isFirst = false;
		cv_zero = cv_mat;
	}
	std::cout << "cv_zero=" << cv_zero << std::endl << std::endl;
	bool flag = matIsEqual(cv_mat, cv_zero);
	std::cout << "flag=" << flag << std::endl << std::endl;

/*
	cv::Mat diff = cv_zero != cv_mat;
	bool eq = cv::countNonZero(diff) == 0;
	std::cout << "eq=" << eq << std::endl << std::endl;
*/

	//if (!cv_mat.empty()) {
	if (!matIsEqual(cv_mat, cv_zero)) {
		// Convert to geometry_msgs::Transform
		geometry_msgs::Transform transform;
		transform = ConvertPositionToTransform(cv_ptr->header, cv_mat);
		//std::cout << "transform=" << transform << std::endl << std::endl;

		// Convert to geometry_msgs::TransformStamped
		geometry_msgs::TransformStamped transformStamped;
		transformStamped = ConvertPositionToTransformStamped(cv_ptr->header, cv_mat, "/child");
		//std::cout << "transformStamped=" << transformStamped << std::endl << std::endl;

		// Build to geometry_msgs::Pose
		geometry_msgs::Pose pose;
		// Vector3.float64 --> Point.float64
		pose.position.x = transform.translation.x;
		pose.position.y = transform.translation.y; 
		pose.position.z = transform.translation.z;
		pose.orientation = transform.rotation;
		std::cout << "pose=" << pose << std::endl << std::endl;

		// Build to geometry_msgs::poseStamped
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.header = transformStamped.header;
		poseStamped.pose = pose;
		std::cout << "poseStamped=" << poseStamped << std::endl << std::endl;

		//pub_pose.publish(pose);
		pub_poseStamped.publish(poseStamped);
	}
}


