#include "stdafx.h"
#include "segmentation.h"

void Segmentation::TestImg()
{
	cv::Mat img = cv::imread("../x64/Release/test.jpg"); 
	cv::imshow("test", img);//ÏÔÊ¾Í¼Æ¬6Ãë
  cv::waitKey(6000);
}