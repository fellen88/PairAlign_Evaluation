
#include"stdafx.h"
#include "preprocessing.h"


PreProcessing::PreProcessing()
{
	ImageWidth = 848;
	ImageHeight = 480;
	
	pcolorBuffer = nullptr;                                   // 共享内存指针
	pdepthBuffer = nullptr;                                   // 共享内存指针

	hcolorMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)"color");
	hdepthMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)"depth");
}

/**将uchar类型的数据转换为Mat类型*/
int PreProcessing::UcharToMat(uchar *p2, cv::Mat& src, int flag)
{
	int img_width = src.cols;
	int img_height = src.rows;
	//Mat img(Size(img_width, img_height), CV_8UC3);
	for (int i = 0; i < img_width * img_height * 3; i++)
	{
		src.at<cv::Vec3b>(i / (img_width * 3), (i % (img_width * 3)) / 3)[i % 3] = p2[i];//BGR格式
		//src.at<Vec3b>(i / (img_width * 3), (i % (img_width * 3)) / 3)[i % 3] = p2[i];//换为RGB使用
	}
	flag = 1;
	return flag;
}

bool PreProcessing::SetParameters()
{

	return false;
}

bool PreProcessing::RecieveImage()
{
	uchar *p1 = (uchar*)malloc(sizeof(uchar)*ImageHeight*ImageWidth * 3);
	memcpy(p1, pcolorBuffer, sizeof(uchar)*ImageHeight*ImageWidth * 3);
	cv::Mat color(cv::Size(ImageWidth, ImageHeight), CV_8UC3);
	UcharToMat(p1, color, 0);
	cv::imshow("color", color);

	uchar *p2 = (uchar*)malloc(sizeof(uchar)*ImageHeight*ImageWidth * 3);
	memcpy(p2, pdepthBuffer, sizeof(uchar)*ImageHeight*ImageWidth * 3);
	cv::Mat depth(cv::Size(ImageWidth, ImageHeight), CV_8UC3);
	UcharToMat(p2, depth, 0);
	cv::imshow("depth", depth);
	return false;
}

bool PreProcessing::ImagetoPointCloud()
{
	return false;
}
