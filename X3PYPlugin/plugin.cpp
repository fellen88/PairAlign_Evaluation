
#include <module/plugininc.h>
#include "plugin.h"
#include "../pose_estimation/pose_estimation.h"

void Plugin::test()
{
	PoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  p_pose_estimation_->p_segmentation_->TestImg();
}
