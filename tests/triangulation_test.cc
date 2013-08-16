
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boxes.h>
#include<iostream>



std::vector<cv::Point3d> createPointCloud()
{
	std::vector<cv::Point3d> point_cloud;
	double x = 0.4, y=0.4, z=0.2; 
 	for (y; y > 0; y-=0.01)
	{
		point_cloud.push_back(cv::Point3d(x,y,z));
	}

	x = 0.1;
	z = 0.4;
	for (y=0.4; y > 0; y-=0.01)
	{
		point_cloud.push_back(cv::Point3d(x,y,z));
	}

	x = -0.1;
	z = 0.3;
	for (y=0.4; y > 0; y-=0.01)
	{
		point_cloud.push_back(cv::Point3d(x,y,z));
	}
	return point_cloud;	
}


int main() {
	std::vector<cv::Point3d> point_cloud = createPointCloud();
	//create Camera Matrix
	cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
	camera_matrix.at<double>(0, 0) = 3000;
	camera_matrix.at<double>(1, 1) = 3000;
	camera_matrix.at<double>(2, 2) = 1.0;
	camera_matrix.at<double>(0, 2) = 3000 / 2;
	camera_matrix.at<double>(1, 2) = 3000 / 2;

	//create projection matriices
	
	cv::Mat p1 = cv::Mat::eye(3, 4, CV_64F);
	cv::Mat p2 = cv::Mat::eye(3, 4, CV_64F);
	p2.at<double>(0,3) = 0.1;
	p2.at<double>(1,3) = 0.1;

	std::vector<cv::Point2f> pts1;
	std::vector<cv::Point2f> pts2;

	cv::Mat img1 = cv::Mat::zeros(3000,3000,CV_8UC3);
	cv::Mat img2 = cv::Mat::zeros(3000,3000,CV_8UC3);
	
	
	int colorOffset = (0xffffff)/point_cloud.size();
	int color = 0x000000;

	for(unsigned int i=0; i < point_cloud.size(); i++)
	{	
		//project points 2 images
		cv::Point3d pt = point_cloud[i];
		cv::Mat pt_(4, 1 , CV_64F);
		pt_.at<double>(0,0) = pt.x;
		pt_.at<double>(1,0) = pt.y;
		pt_.at<double>(2,0) = pt.z;
		pt_.at<double>(3,0) = 1;

		cv::Mat pt1_ = camera_matrix * p1 * pt_;
		cv::Mat pt2_ = camera_matrix * p2 * pt_;

		cv::Point2f pt1(pt1_.at<double>(0,0), pt1_.at<double>(1,0));
		cv::Point2f pt2(pt2_.at<double>(0,0), pt2_.at<double>(1,0));

		std::cout << pt1.x << " " << pt1.y << "/" << pt2.x << " " << pt2.y <<std::endl;

		color+= colorOffset;

		img1.at<cv::Vec3b>(pt1.y,pt1.x)[0] = (color)%0xff;
		img1.at<cv::Vec3b>(pt1.y,pt1.x)[1] = (color >> 8)%0xff;
		img1.at<cv::Vec3b>(pt1.y,pt1.x)[2] = (color >> 16)%0xff;

		img2.at<cv::Vec3b>(pt2.y,pt2.x)[0] = (color)%0xff;
		img2.at<cv::Vec3b>(pt2.y,pt2.x)[1] = (color >> 8)%0xff;
		img2.at<cv::Vec3b>(pt2.y,pt2.x)[2] = (color >> 16)%0xff;
		
		cv::imwrite( "test1.jpg", img1 );
		cv::imwrite( "test2.jpg", img2 );

		pts1.push_back(pt1);
		pts2.push_back(pt1);
		
	}

	exit(0);
}

