
#pragma once
#include "stdafx.h"
#include<Windows.h>
#include<iostream>
#include <string>
#include <cstring>
#include <opencv2/opencv.hpp>

class PreProcessing
{
public:
	int ImageWidth;
	int ImageHeight;

	LPVOID pcolorBuffer;                                   // 共享内存指针
	LPVOID pdepthBuffer;                                   // 共享内存指针

	HANDLE hcolorMap;
	HANDLE hdepthMap;

	PreProcessing();
	int UcharToMat(uchar *p2, cv::Mat& src, int flag);
	bool DebugVisualization;
	bool RecieveImage();
	bool SetParameters();
	bool ImagetoPointCloud();

};