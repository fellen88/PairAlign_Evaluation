#ifndef X3_EXAMPLE_ISIMPLE_H
#define X3_EXAMPLE_ISIMPLE_H

#include <objptr.h>

class ISimple : public x3::IObject
{
    X3DEFINE_IID(ISimple);
    virtual void test() = 0;

 //   virtual int subtract(int a, int b) const = 0;

	//virtual int disPatch(int iCnt) const = 0;
	////virtual int disPatchChar(int iCnt) const = 0;
	//virtual int disPatchChar(int iCnt, const char* pcData) const = 0;
	//virtual int disPatchInt(int iCnt, const int* piData) const = 0;
	//virtual int disPatchFloat(int iCnt, const float* pfData) const = 0;
};

#endif