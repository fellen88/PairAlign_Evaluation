
#include <module/plugininc.h>
#include "plugin.h"
#include "../pose_estimation/pose_estimation.h"

void Plugin::test()
{
	PoseEstimation *pose_estimation_ =GetInstance();
	pose_estimation_->Compute();

}
