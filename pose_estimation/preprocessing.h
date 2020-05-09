
#pragma once
#include "stdafx.h"
#include<Windows.h>
#include<iostream>
#include <string>
#include <cstring>
#include <opencv2/opencv.hpp>

using namespace cv;

class PreProcessing
{
public:
	int WIDTH;
	int HEIGHT; 
	
	// π≤œÌƒ⁄¥Ê÷∏’Î
	LPVOID pcolorBuffer;                                  
	LPVOID pdepthBuffer;                                  
	LPVOID cameraStateBuffer;
	LPVOID pictureStateBuffer;

	HANDLE hcolorMap;
	HANDLE hdepthMap;
	HANDLE hcameraMap;
	HANDLE hpictureMap;

	bool isOpenFileMapping;

	enum CameraState
	{
		DISCONNECTED = 0,
		CONNECTED

	};

	enum PictureState
	{
		FIRST,
		WRITING,
		WRITED,
		READING,
		READED
	};

	PreProcessing();
	~PreProcessing();
	int ucharToMat(uchar *p2, cv::Mat& src, int flag);
	bool DebugVisualization;
	bool RecieveImage();
	bool SetParameters();
	bool ImagetoPointCloud();

};