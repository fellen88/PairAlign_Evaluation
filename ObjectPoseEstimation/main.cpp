#include <stdlib.h>
#include "..\pose_estimation\pose_estimation.h"
#pragma comment (lib, "..\\X3PYPlugin\\bin\\x64\\release\\pose_estimation.lib")

int main(int argc, char **argv)
{
	PoseEstimation *pose_estimation_ =GetInstance();
	int n_count = 0;
	while (n_count < 1)
	{
		pose_estimation_->Compute();
		n_count++;
	}
	system("pause");
	return 0;
}

