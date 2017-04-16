#pragma once

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/video/video.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/videostab/videostab.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/ts/ts.hpp"
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/legacy/legacy.hpp"

#ifdef _DEBUG
#pragma comment(lib,"opencv_core2411d.lib")
#pragma comment(lib,"opencv_highgui2411d.lib")
#pragma comment(lib,"opencv_imgproc2411d.lib")
#pragma comment(lib,"opencv_video2411d.lib")
#pragma comment(lib,"opencv_nonfree2411d.lib")
#pragma comment(lib,"opencv_videostab2411d.lib")
#pragma comment(lib,"opencv_features2d2411d.lib")
#pragma comment(lib,"opencv_objdetect2411d.lib")
#pragma comment(lib,"opencv_flann2411d.lib")
#pragma comment(lib,"opencv_photo2411d.lib")
#pragma comment(lib,"opencv_calib3d2411d.lib")
#pragma comment(lib,"opencv_ml2411d.lib")
#pragma comment(lib,"opencv_contrib2411d.lib")
#pragma comment(lib,"opencv_ts2411d.lib")
#pragma comment(lib,"opencv_stitching2411d.lib")
#pragma comment(lib,"opencv_legacy2411d.lib")
#else
#pragma comment(lib,"opencv_core2411.lib")
#pragma comment(lib,"opencv_highgui2411.lib")
#pragma comment(lib,"opencv_imgproc2411.lib")
#pragma comment(lib,"opencv_video2411.lib")
#pragma comment(lib,"opencv_nonfree2411.lib")
#pragma comment(lib,"opencv_videostab2411.lib")
#pragma comment(lib,"opencv_features2d2411.lib")
#pragma comment(lib,"opencv_objdetect2411.lib")
#pragma comment(lib,"opencv_flann2411.lib")
#pragma comment(lib,"opencv_photo2411.lib")
#pragma comment(lib,"opencv_calib3d2411.lib")
#pragma comment(lib,"opencv_ml2411.lib")
#pragma comment(lib,"opencv_contrib2411.lib")
#pragma comment(lib,"opencv_ts2411.lib")
#pragma comment(lib,"opencv_stitching2411.lib")
#pragma comment(lib,"opencv_legacy2411.lib")
#endif