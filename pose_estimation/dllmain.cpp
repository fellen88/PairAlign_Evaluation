// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "stdafx.h"

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
      case DLL_PROCESS_ATTACH:
	  {
		google::InitGoogleLogging("pose_estimation.dll");
		google::SetLogDestination(google::GLOG_INFO, "F:\\Algorithm Demos\\ObjectPoseEstimation\\Logs\\PoseEstimation_");
		google::FlushLogFilesUnsafe(google::GLOG_INFO);
		google::SetStderrLogging(google::GLOG_INFO);
		google::SetLogFilenameExtension("log_");
		LOG(INFO) << "pose_estimation dll attached ";
		break;
      }
      case DLL_THREAD_ATTACH:
		break;
      case DLL_THREAD_DETACH:
		break;
      case DLL_PROCESS_DETACH:
	  {
	    google::ShutdownGoogleLogging();
		break;
	  }		
    }
    return TRUE;
}
