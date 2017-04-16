#ifndef _KINECT_H_
#define _KINECT_H_

#define _AFXDLL

#include <afxwin.h> 
#include <WinNT.h>
#include <NuiApi.h>

#define WIDTH 640
#define HEIGHT 480


void ProcessDepth(INuiSensor* m_pNuiSensor, HANDLE m_pDepthStreamHandle, USHORT *m_depth);
void ProcessColor(INuiSensor* m_pNuiSensor, HANDLE m_pColorStreamHandle, UCHAR *m_colorRGB);

#endif // _KINECT_H_