#pragma once
#include <module/classmacro.h>
#include <x3py_interface.h>
const char* const g_clsidSimple = "94071767-ba6b-4769-9eb4-PoseEstimation";

class Plugin : public IX3py
{
		X3BEGIN_CLASS_DECLARE(Plugin, g_clsidSimple)
        X3DEFINE_INTERFACE_ENTRY(IX3py)
    X3END_CLASS_DECLARE()

public:
	void test();
};
