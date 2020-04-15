#pragma once
#include <opencv2/opencv.hpp>
#include "features.h"
#include "preprocessing.h"

class Segmentation:public Features, public PreProcessing
{
  public:

  bool DEBUG_VISUALIZER;
	void TestImg();
};
