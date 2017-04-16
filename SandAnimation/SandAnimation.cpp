#include <stdio.h>

#include "kinect.h"
#include "use_opencv.h"
#include "BlobLabeling.h"

using namespace cv;
using namespace std;

// 키넥트
INuiSensor* m_pNuiSensor;
HANDLE m_pDepthStreamHandle;
HANDLE m_pColorStreamHandle;
HANDLE m_hNextDepthFrameEvent;
HANDLE m_hNextColorFrameEvent;

HRESULT CreateFirstConnected(void);
void LutUpTableCustom(Mat &src, Mat &dst, Mat &gradation, const int MODE);

#define X_COLOR_DIFF 8

int main(void)
{
	CreateFirstConnected();
	srand((unsigned)time(NULL));

	USHORT *m_depth = new USHORT[WIDTH * HEIGHT];
	UCHAR *m_color = new UCHAR[WIDTH * HEIGHT * 3];

	Mat depth = Mat::zeros(HEIGHT, WIDTH, CV_32FC1);
	Mat color = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
	Mat water_map = Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
	Mat water_depth = Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
	Mat is_black = Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
	Mat output = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
	Mat water_img = imread("water.bmp");

	int min_depth = 1300;
	int max_depth = 1650;

	int dir[7][7] = {
		{0, 0, 1, 2, 1, 0, 0},
		{0, 1, 2, 3, 2, 0, 0},
		{1, 2, 3, 4, 3, 2, 1},
		{2, 3, 4, 0, 4, 3, 2},
		{1, 2, 3, 4, 3, 2, 1},
		{0, 1, 2, 3, 2, 1, 0},
		{0, 0, 1, 2, 1, 0, 0}};

		// 이건 변경하면 안됨
		const float MIN_VALUE = 0.0f, MAX_VALUE = 1800.0f;
		const int GROUND_DRAW = 0, CONTOUR_DRAW = 1, WATER_DRAW = 2;

		cvNamedWindow("depth", CV_WINDOW_NORMAL);

		bool ground_draw = true, contour_draw = true;

		int skew_idx = 0;

		FILE *fp = fopen("setting.txt", "rt");
		int resolution_x, resolution_y;
		int CUSTOM_MODE;
		cv::Point2f src_point[4];
		cv::Point2f dst_point[4]; // 1024 768

		for(int i = 0; i < 4; i++) {
			fscanf(fp, "%f %f\n", &src_point[i].x, &src_point[i].y);
		}
		fclose(fp);

		printf("가로 해상도 (기본 1024): ");
		scanf("%d", &resolution_x);
		printf("세로 해상도 (기본 768): ");
		scanf("%d", &resolution_y);
		printf("1. 땅-물(물 흐름 X)					 2. 땅(물 흐름 O): ");
		scanf("%d", &CUSTOM_MODE);
		printf("화면 최대 키:F,  등고선 제거:C,  물제거 : R");

		Mat ground_img;
		if(CUSTOM_MODE == 1)
			ground_img = imread("ground1.bmp");
		else
			ground_img = imread("ground2.bmp");

		dst_point[0].x = 0; 	dst_point[0].y = 0;
		dst_point[1].x = resolution_x; 	dst_point[1].y = 0;
		dst_point[2].x = resolution_x; 	dst_point[2].y = resolution_y;
		dst_point[3].x = 0; 	dst_point[3].y = resolution_y;

		for(int frame_cnt = 0; frame_cnt < 50000; frame_cnt++){
			int x = src_point[0].x > src_point[3].x ? src_point[0].x : src_point[3].x;
			int y = src_point[0].y > src_point[1].y ? src_point[0].y : src_point[1].y;
			int depth_width = src_point[1].x < src_point[2].x ? src_point[1].x : src_point[2].x;
			int depth_height = src_point[2].y < src_point[3].y ? src_point[2].y : src_point[3].y;

			if(frame_cnt == 10000)
				frame_cnt = 20;

			ProcessDepth(m_pNuiSensor, m_pDepthStreamHandle, m_depth);
			ProcessColor(m_pNuiSensor, m_pColorStreamHandle, m_color);

			for(int i = 0; i < HEIGHT; i++) {
				for(int j = 0; j < WIDTH; j++) {
					color.at<Vec3b>(i, j)[0] = m_color[i * WIDTH * 3 + j * 3 + 0];
					color.at<Vec3b>(i, j)[1] = m_color[i * WIDTH * 3 + j * 3 + 1];
					color.at<Vec3b>(i, j)[2] = m_color[i * WIDTH * 3 + j * 3 + 2];
					depth.at<float>(i, j) = m_depth[i * WIDTH + j];
					output.at<Vec3b>(i, j)[0]	= 205;
					output.at<Vec3b>(i, j)[1]	= 205;
					output.at<Vec3b>(i, j)[2] = 205; 
				}
			}

			Mat blured_depth, medianed_depth;
			blur(depth, blured_depth, Size(5, 5));
			medianBlur(blured_depth, medianed_depth, 5);
			Mat stretched_depth = Mat::zeros(480, 640, CV_16UC1);
			for(int i = 0; i < HEIGHT; i++) {
				for(int j = 0; j < WIDTH; j++) {
					stretched_depth.at<unsigned short>(i, j) = ((medianed_depth.at<float>(i, j) - min_depth) / (max_depth - min_depth)) * MAX_VALUE;
					stretched_depth.at<unsigned short>(i, j) = stretched_depth.at<unsigned short>(i, j) < 0 ? 0 : stretched_depth.at<unsigned short>(i, j);
					stretched_depth.at<unsigned short>(i, j) = stretched_depth.at<unsigned short>(i, j) >= MAX_VALUE ? (unsigned short)MAX_VALUE - 1 : stretched_depth.at<unsigned short>(i, j);
				}
			}
			/////////////////// 스트레칭 완료 ////////////////////////////////

			// 땅 그리기
			if(ground_draw == true)
				LutUpTableCustom(stretched_depth, output, ground_img, GROUND_DRAW);
			// 등고선 그리기
			if(contour_draw == true)
				LutUpTableCustom(stretched_depth, output, ground_img, CONTOUR_DRAW);

			if(CUSTOM_MODE == 2) {
				const int FRAME_RATE = 3, FRAME_CONTINUE = 15;
				if(frame_cnt > 10 && frame_cnt % FRAME_RATE == 0){
					// 물뿌려주기 찾기
					Mat hsv_img;
					cvtColor(color, hsv_img, CV_BGR2HSV);
					//printf("%f\n", depth.at<float>(100, 100));
					const int MASK_SIZE = 5; // MASK_SIZE * 2 + 1 필터
					const int VAL_THRESHOLD = 160;
					const int SAT_THRESHOLD = 80;
					for(int i = (y > MASK_SIZE ? y : MASK_SIZE); i < ((depth_height < HEIGHT - 50 - MASK_SIZE) ? depth_height : (HEIGHT - 50 - MASK_SIZE)); i++) {
						for(int j = (x > 50 + MASK_SIZE ? x : 50 + MASK_SIZE); j < ((depth_width < WIDTH - MASK_SIZE) ? depth_width : WIDTH - MASK_SIZE); j++) {
							int bit_adr = (frame_cnt / FRAME_RATE) % FRAME_CONTINUE;
							if(/*depth.at<float>(i, j) < (min_depth - 100.0f) &&*/ hsv_img.at<Vec3b>(i, j)[1] < SAT_THRESHOLD && hsv_img.at<Vec3b>(i, j)[2] < VAL_THRESHOLD) {
								int count = 0;
								for(int mr = -MASK_SIZE; mr <= MASK_SIZE; mr++) {
									for(int mc = -MASK_SIZE; mc <= MASK_SIZE; mc++) {
										if(/*depth.at<float>(i + mr, j + mc) < (min_depth - 100.0f) && */hsv_img.at<Vec3b>(i + mr, j + mc)[1] < SAT_THRESHOLD && hsv_img.at<Vec3b>(i + mr, j + mc)[2] < VAL_THRESHOLD)
											count++;
									}
								}
								//printf("%d\n", count);
								if(count > 70) {
									//printf("%d\n", count);
									is_black.at<unsigned char>(i, j)++;
								} else {
									is_black.at<unsigned char>(i, j) = 0;
								}
							} else {
								is_black.at<unsigned char>(i, j) = 0;
							}
						}
					}
				}

				for(int i = 0; i < HEIGHT; i++) {
					for(int j = (x < X_COLOR_DIFF ? X_COLOR_DIFF : x); j < WIDTH; j++) {
						if(is_black.at<unsigned char>(i, j) >= FRAME_CONTINUE && depth.at<float>(i, j) < 1400 && depth.at<float>(i, j) > 1100) {
							if(water_depth.at<unsigned short>(i, j - X_COLOR_DIFF) < 800)
								water_depth.at<unsigned short>(i, j - X_COLOR_DIFF) += 1;
							//else
							//	water_depth.at<unsigned short>(i, j - X_COLOR_DIFF) = 799;
							water_map.at<unsigned char>(i, j - X_COLOR_DIFF) = 255;
						}
					}
				}

				CBlobLabeling blob;
				IplImage *trans = &IplImage(water_map);
				blob.SetParam(trans, 5);
				blob.DoLabeling();
				Mat water_label = blob.m_image;

				for(int n = 0; n < blob.m_nBlobs; n++) {
					cv::Rect rect(blob.m_recBlobs[n].x, blob.m_recBlobs[n].y, blob.m_recBlobs[n].width, blob.m_recBlobs[n].height);

					float depth_sum = 0;
					int water_sum = 0, area_size = 0;
					for(int i = rect.y; i < rect.y + rect.height; i++) {
						for(int j = rect.x; j < rect.x + rect.width; j++) {
							if(water_label.at<unsigned char>(i, j) == 0)
								water_map.at<unsigned char>(i, j) = water_label.at<unsigned char>(i, j);
							if(water_map.at<unsigned char>(i, j) == 255) {
								area_size++;
								depth_sum += depth.at<float>(i, j);
								water_sum += (int)water_depth.at<unsigned short>(i, j);
							}
						}
					}

					// 물높이 맞추기
					float average_depth = (depth_sum - water_sum) / area_size;
					//printf("%d %f %d %d %f\n", n, depth_sum, water_sum, area_size, average_depth);
					for(int i = rect.y; i < rect.y + rect.height; i++) {
						for(int j = rect.x; j < rect.x + rect.width; j++) {
							if(water_map.at<unsigned char>(i, j) == 255) {
								int val = depth.at<float>(i, j) - average_depth;
								if(val < 0) {
									water_map.at<unsigned char>(i, j) = 255;
									water_depth.at<unsigned short>(i, j) = 1;
								} else {
									water_depth.at<unsigned short>(i, j) = val;
								}
							}
						}
					}
				}

				Mat water_advance = water_map.clone();
				for(int i = y; i < depth_height; i++) {
					for(int j = x; j < depth_width; j++) {
						if(water_map.at<unsigned char>(i, j) == 0) {
							int count = 0;
							int cur_depth = depth.at<float>(i, j);
							for(int mr = -3; mr <= 3; mr++) {
								for(int mc = -3; mc <= 3; mc++) {
									if(dir[mr + 3][mc + 3]) {
										if(water_map.at<unsigned char>(i + mr, j + mc) != 0) {
											count += (cur_depth - (depth.at<float>(i + mr, j + mc) - water_depth.at<unsigned short>(i + mr, j + mc))) * dir[mr + 3][mc + 3];
										}
									}
								}
							}	
							/*if(count > 10)
							printf("%d\n",count);*/
							if(count > 800 && depth.at<float>(i, j) > 1400) {
								water_advance.at<unsigned char>(i, j) = 255;
							}
						}
					}
				}

				for(int i = y; i < depth_height ; i++) {
					for(int j = x; j < depth_width; j++) {
						water_map.at<unsigned char>(i, j) = water_advance.at<unsigned char>(i, j);
					}
				}
				for(int i = y; i < depth_height ; i++) {
					for(int j = x; j < depth_width; j++) {
						if(depth.at<float>(i, j) < 1500) {
							water_depth.at<unsigned short>(i, j) = 0;
							water_map.at<unsigned char>(i, j) = 0;
						}
					}
				}
				if(frame_cnt % 200 == 0) {
					for(int i = y; i < depth_height ; i++) {
						for(int j = x; j < depth_width; j++) {					
							if(water_depth.at<unsigned short>(i, j) > 0) {
								water_depth.at<unsigned short>(i, j)--;
								if(water_depth.at<unsigned short>(i, j) == 0)
									water_map.at<unsigned char>(i, j) = 0;
							}
						}
					}
				}

				LutUpTableCustom(water_depth, output, water_img, WATER_DRAW);
			}

			cv::Mat transform_matrix, skew_image;
			transform_matrix = cv::getPerspectiveTransform(src_point, dst_point);
			cv::warpPerspective(output, skew_image, transform_matrix, Size(resolution_x, resolution_y), cv::INTER_LANCZOS4, 0, cv::Scalar(255));

			// 완성본 그리기
			Mat crop_depth = output(cv::Rect(x - 8, y, depth_width - x, depth_height - y)).clone();
			Mat flip_depth;
			flip(skew_image, flip_depth, 1);
			/*Mat resize_depth;
			resize(flip_depth, resize_depth, Size(1200, 800), 0, 0, CV_INTER_CUBIC);*/
			//resize(flip_depth, resize_depth, Size(1200, 800), 0, 0, CV_INTER_CUBIC);

			imshow("depth", flip_depth);

			for(int i = 0; i < 4; i++) {
				circle(color, src_point[i], 3, Scalar(0, 0, 255), 3);
			}

			imshow("color", color);
			char key = waitKey(30);
			if(key == 'q' || key == 'Q') {
				break;
			} else if(key == 'r' || key == 'R') {
				water_map.release();
				water_depth.release();
				water_map = Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
				water_depth = Mat::zeros(HEIGHT, WIDTH, CV_16UC1);
			} else if(key == 'f' || key == 'F') {
				cvSetWindowProperty("depth", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			} else if(key == 'c' || key == 'C') {
				contour_draw = contour_draw ? false : true;
			} else if(key == 'g' || key == 'G') {
				ground_draw = ground_draw ? false : true;
			} else if(key == 'w' || key == 'W') {	// 상
				src_point[skew_idx].y--;
			} else if(key == 'a' || key == 'A') {	// 좌
				src_point[skew_idx].x--;
			} else if(key == 's' || key == 'S') {	// 하
				src_point[skew_idx].y++;
			} else if(key == 'd' || key == 'D') {	// 우
				src_point[skew_idx].x++;
			} else if(key == 'u' || key == 'U') {  // 좌상
				skew_idx = 0;
			} else if(key == 'i' || key == 'I') {  // 우상
				skew_idx = 1;
			} else if(key == 'j' || key == 'J') {  // 좌하
				skew_idx = 3;
			} else if(key == 'k' || key == 'K') {  // 우하
				skew_idx = 2;
			}
		}

		fp = fopen("setting.txt", "wt");
		for(int i = 0; i < 4; i++) {
			fprintf(fp, "%f %f\n", src_point[i].x, src_point[i].y);
		}
		fclose(fp);

		delete []m_depth;
		delete []m_color;

		depth.release();
		color.release();
		output.release();
		water_map.release();
		ground_img.release();
		water_img.release();

		return 0;
}

HRESULT CreateFirstConnected(void)
{
	INuiSensor*	pNuiSensor;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if(FAILED(hr)) {
		return hr;
	}

	// Look at each Kinect sensor
	for(int i = 0; i < iSensorCount; ++i) {
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if(FAILED(hr)) {
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if(S_OK == hr) {
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if(NULL != m_pNuiSensor) {
		// Initialize the Kinect and specify that we'll be using depth
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON); 
		if(SUCCEEDED(hr)) {
			// Create an event that will be signaled when depth data is available
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			// Open a depth image stream to receive depth frames

			hr = m_pNuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH,	NUI_IMAGE_RESOLUTION_640x480, 0, 2,	m_hNextDepthFrameEvent,	&m_pDepthStreamHandle);
			if(SUCCEEDED(hr))
				hr = m_pNuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,	NUI_IMAGE_RESOLUTION_640x480, 0, 2,	m_hNextColorFrameEvent, &m_pColorStreamHandle);
		}
	}

	if(NULL == m_pNuiSensor || FAILED(hr)) {
		AfxMessageBox(_T("No ready Kinect found!"));
		return E_FAIL;
	}

	return hr;
}

void LutUpTableCustom(Mat &src, Mat &dst, Mat &gradation, const int MODE)
{
	if(MODE == 0) {
		for(int i = 0; i < HEIGHT; i++) {
			for(int j = 0; j < WIDTH; j++) {
				unsigned short a = src.at<unsigned short>(i, j);
				dst.at<Vec3b>(i, j)[0] = gradation.at<Vec3b>(0, src.at<unsigned short>(i, j))[0];
				dst.at<Vec3b>(i, j)[1] = gradation.at<Vec3b>(0, src.at<unsigned short>(i, j))[1];
				dst.at<Vec3b>(i, j)[2] = gradation.at<Vec3b>(0, src.at<unsigned short>(i, j))[2];
			}
		}
	} else if(MODE == 1) {
		for(int i = 0; i < HEIGHT; i++) {
			for(int j = 0; j < WIDTH; j++) {
				if(src.at<unsigned short>(i, j) % 150 < 10) {
					dst.at<Vec3b>(i, j)[0] = 0;
					dst.at<Vec3b>(i, j)[1] = 0;
					dst.at<Vec3b>(i, j)[2] = 0;
				}
			}
		}
	} else if(MODE == 2) {
		for(int i = 0; i < HEIGHT; i++) {
			for(int j = 0; j < WIDTH; j++) {			
				if(src.at<unsigned short>(i, j) != 0) {
					int val = src.at<unsigned short>(i, j);
					val = val >= 600 ? 599 : val;
					dst.at<Vec3b>(i, j)[0] = gradation.at<Vec3b>(0, val)[0];
					dst.at<Vec3b>(i, j)[1] = gradation.at<Vec3b>(0, val)[1];
					dst.at<Vec3b>(i, j)[2] = gradation.at<Vec3b>(0, val)[2];
				}
			}
		}
	}
}