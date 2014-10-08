#include <kinectdriver.h>

using namespace std;
using namespace cv;

KinectDriver::KinectDriver()
{
	is_stopping = false;
	users_color[0] = cv::Vec3b(255, 0, 0);
	users_color[1] = cv::Vec3b(0, 255, 0);
	users_color[2] = cv::Vec3b(0, 0, 255);
	users_color[3] = cv::Vec3b(255, 255, 0);
	users_color[4] = cv::Vec3b(255, 0, 255);
	users_color[5] = cv::Vec3b(0, 255, 255);
}

KinectDriver::~KinectDriver()
{
	NuiShutdown();
}

bool KinectDriver::initialize(const int width, const int height)
{
	HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR);

	if (FAILED(hr))
		return false;

	h1 = CreateEvent(NULL, TRUE, FALSE, NULL);
	h2 = NULL;

	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, h1, &h2);

	if (FAILED(hr))
		return false;

	h3 = CreateEvent(NULL, TRUE, FALSE, NULL);
	h4 = NULL;

	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, h3, &h4);

	if (FAILED(hr))
		return false;

	if (NuiImageStreamSetImageFrameFlags(h4, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE) != S_OK)
		fprintf(stdout, "NO NEAR MODE\n");

	def_width_color = 640;
	def_height_color = 480;
	def_width_depth = 320;
	def_height_depth = 240;

	this->width = width;
	this->height = height;

	return true;
}

void KinectDriver::record(const string &destFolder)
{
	is_stopping = false;
	rgb_folder = destFolder + "rgb/";
	depth_folder = destFolder + "depth/";
	skel_folder = destFolder + "skeleton/";
	users_folder = destFolder + "users/";

	CreateDirectory(rgb_folder.c_str(), NULL);
	CreateDirectory(depth_folder.c_str(), NULL);
	CreateDirectory(skel_folder.c_str(), NULL);
	CreateDirectory(users_folder.c_str(), NULL);

	rec_t = std::thread(&KinectDriver::start, this);
}

void KinectDriver::stop()
{
	mtx.lock();
	is_stopping = true;
	mtx.unlock();
	if (rec_t.joinable())
		rec_t.join();
}

void KinectDriver::start()
{
	int n_frame = 0;
	string frame_name;

	IplImage* color_big = cvCreateImageHeader(cvSize(def_width_color, def_height_color), IPL_DEPTH_8U, 4);
	Mat depth_big(def_height_depth, def_width_depth, CV_16SC1);
	Mat users_big(def_height_depth, def_width_depth, CV_8UC3);
	Mat color_small(height, width, CV_8UC4);
	Mat depth_to_show(def_height_depth, def_width_depth, CV_8UC1);
	Mat skeleton(def_height_depth, def_width_depth, CV_8UC3);

	int fr = 0;
	while (!is_stopping)
	{
		try
		{
			const NUI_IMAGE_FRAME *col_image;
			const NUI_IMAGE_FRAME *depth_image;
			NUI_SKELETON_FRAME skeleton_frame;

			HANDLE hEvents[4];
			hEvents[0] = h2;
			hEvents[1] = h4;

			WaitForMultipleObjects(2, hEvents, TRUE, INFINITE);
			HRESULT hr = NuiImageStreamGetNextFrame(h2, 0, &col_image);
			hr &= NuiImageStreamGetNextFrame(h4, 0, &depth_image);
			//hr &= NuiSkeletonGetNextFrame(0, &skeleton_frame);
			if (FAILED(hr))
				continue;

			if (col_image != NULL)
			{
				INuiFrameTexture * pTexture = col_image->pFrameTexture;
				NUI_LOCKED_RECT LockedRect;
				pTexture->LockRect(0, &LockedRect, NULL, 0);
				if (LockedRect.Pitch != 0)
				{
					BYTE * pBuffer = (BYTE*)LockedRect.pBits;
					cvSetData(color_big, pBuffer, color_big->widthStep);
				}

				cv::Mat color_small(cv::cvarrToMat(color_big));

				NuiImageStreamReleaseFrame(h2, col_image);
			}

			if (depth_image != NULL && depth_image->dwFrameFlags == 1)
			{
				INuiFrameTexture * pTextureDepth = depth_image->pFrameTexture;
				NUI_LOCKED_RECT LockedRectDepth;
				printf("fr %i\n", fr);
				fr++;
				pTextureDepth->LockRect(0, &LockedRectDepth, NULL, 0);
				if (LockedRectDepth.Pitch != 0)
				{
					int m = 0;
					int n = 0;
					USHORT * pBuff = (USHORT*)LockedRectDepth.pBits;
					for (int i = 0; i < def_width_depth*def_height_depth; i++)
					{
						unsigned short real_depth = (pBuff[i] & 0xFFF8) >> 3;
						int p = (pBuff[i] & 0x0007);
						if (n == def_width_depth)
						{
							n = 0;
							m++;
						}
						if (p != 0)
							users_big.at<cv::Vec3b>(m, n) = users_color[p];
						else
							users_big.at<cv::Vec3b>(m, n) = cv::Vec3b(0, 0, 0);

						depth_big.at<ushort>(m, n) = real_depth;

						int depth_val = (int)((real_depth / 4096.0) * 255.0);
						depth_to_show.at<uchar>(m, n) = depth_val;
						n++;
					}
					NuiImageStreamReleaseFrame(h4, depth_image);
				}
			}

			//getSkeletons(body_frame, skeleton, n_frame);*/	

			imshow("Color", color_small);
			imshow("Depth", depth_to_show);
			imshow("Users", users_big);
			//imshow("Skeleton", skeleton);*/
			waitKey(30);

			/*stringstream ss; ss << n_frame;
			string id = ss.str();
			frame_name = rgb_folder + "img" + id + ".png";
			imwrite(frame_name, color_small);
			frame_name = depth_folder + "img" + id + ".png";
			imwrite(frame_name, depth_big);	
			frame_name = users_folder + "img" + id + ".png";
			imwrite(frame_name, users_big);*/
			n_frame++;
		}
		catch (std::exception e)
		{
			printf("Program was stopped while waiting\n");
		}
	}
	cvReleaseImageHeader(&color_big);
}