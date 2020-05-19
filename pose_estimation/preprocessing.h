
#pragma once
#include "stdafx.h"

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
	bool GetImages();
	bool SetParameters();
	bool DepthtoPointCloud();
	bool Load3DModel(const PointCloud::Ptr object_model, std::string name);
	void Show(const PointCloud::Ptr pointcloud, std::string name);

};