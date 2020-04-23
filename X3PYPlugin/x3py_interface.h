#ifndef X3_IX3PY_H
#define X3_IX3PY_H

#include <objptr.h>

class IX3py : public x3::IObject
{
    X3DEFINE_IID(IX3py);
    void test();

  //virtual int subtract(int a, int b) const = 0;
	//virtual int disPatch(int iCnt) const = 0;
	////virtual int disPatchChar(int iCnt) const = 0;
	//virtual int disPatchChar(int iCnt, const char* pcData) const = 0;
	//virtual int disPatchInt(int iCnt, const int* piData) const = 0;
	//virtual int disPatchFloat(int iCnt, const float* pfData) const = 0;
};

#endif