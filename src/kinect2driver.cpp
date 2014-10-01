#include <kinect2driver.h>

using namespace std;
using namespace cv;

Kinect2Driver::Kinect2Driver()
{
	is_stopping = false;
	users_color[0] = cv::Vec3b(255, 0, 0);
	users_color[1] = cv::Vec3b(0, 255, 0);
	users_color[2] = cv::Vec3b(0, 0, 255);
	users_color[3] = cv::Vec3b(255, 255, 0);
	users_color[4] = cv::Vec3b(255, 0, 255);
	users_color[5] = cv::Vec3b(0, 255, 255);
}

Kinect2Driver::~Kinect2Driver()
{
	safeRelease(color_reader);
	safeRelease(color_description);
	safeRelease(depth_reader);
	safeRelease(depth_description);
	safeRelease(body_reader);
	safeRelease(coordinate_mapper);
	safeRelease(users_reader);

	sensor->Close();
	safeRelease(sensor);
}

bool Kinect2Driver::initialize(const int width, const int height)
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&sensor);
	if (FAILED(hr))
		return false;

	IColorFrameSource* color_source = NULL;
	IDepthFrameSource* depth_source = NULL;
	IBodyFrameSource* body_source = NULL;
	IBodyIndexFrameSource* users_source = NULL;

	hr = sensor->Open();

	if (SUCCEEDED(hr))
	{
		hr = sensor->get_ColorFrameSource(&color_source);
		hr &= sensor->get_DepthFrameSource(&depth_source);
		hr &= sensor->get_BodyFrameSource(&body_source);
		hr &= sensor->get_CoordinateMapper(&coordinate_mapper);
		hr &= sensor->get_BodyIndexFrameSource(&users_source);
	}

	if (SUCCEEDED(hr))
	{
		hr = color_source->OpenReader(&color_reader);
		hr &= color_source->get_FrameDescription(&color_description);
		hr &= depth_source->OpenReader(&depth_reader);
		hr &= depth_source->get_FrameDescription(&depth_description);
		hr &= body_source->OpenReader(&body_reader);
		hr &= users_source->OpenReader(&users_reader);
	}
	
	safeRelease(color_source);
	safeRelease(depth_source);
	safeRelease(body_source);

	if (!sensor || FAILED(hr))
	{
		printf("No Kinect found\n");
		return false;
	}

	color_description->get_Width(&def_width[0]);
	color_description->get_Height(&def_height[0]);

	depth_description->get_Width(&def_width[1]);
	depth_description->get_Height(&def_height[1]);

	this->width = width;
	this->height = height;

	return true;
}

void Kinect2Driver::record(const string &destFolder)
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

	rec_t = std::thread(&Kinect2Driver::start, this);
}

void Kinect2Driver::stop()
{
	mtx.lock();
	is_stopping = true;
	mtx.unlock();
	if (rec_t.joinable())
		rec_t.join();
}

void Kinect2Driver::getSkeletons(IBodyFrame* body_frame, Mat &depth_to_show, const int n_frame)
{
	stringstream ss; ss << n_frame;
	string id = ss.str();
	ofstream skeleton_file;
	string frame_name = skel_folder + "joints" + id + ".txt";
	skeleton_file.open(frame_name);
	
	IBody* body[BODY_COUNT] = { 0 };
	body_frame->GetAndRefreshBodyData(BODY_COUNT, body);

	for (int count = 0; count < BODY_COUNT; count++)
	{
		BOOLEAN tracked = false;
		body[count]->get_IsTracked(&tracked);

		if (tracked)
		{
			skeleton_file << "user " << count << "\n";
			Joint joint[JointType::JointType_Count];
			body[count]->GetJoints(JointType::JointType_Count, joint);

			for (int type = 0; type < JointType::JointType_Count; type++)
			{
				skeleton_file << "joint " << type << " ";
				DepthSpacePoint depth_space_point = { 0 };
				coordinate_mapper->MapCameraPointToDepthSpace(joint[type].Position, &depth_space_point);
				int x = static_cast<int>(depth_space_point.X);
				int y = static_cast<int>(depth_space_point.Y);
				skeleton_file << joint[type].Position.X << joint[type].Position.Y << joint[type].Position.Z << depth_space_point.X << depth_space_point.Y << "\n";
				if ((x >= 0) && (x < def_width[0]) && (y >= 0) && (y < def_height[0]))
				{
					cv::circle(depth_to_show, cv::Point(x, y), 5, static_cast< cv::Scalar >(users_color[count]), -1);
				}
			}
		}
	}

	skeleton_file.close();
}

void Kinect2Driver::start()
{
	int n_frame = 0;
	string frame_name;

	Mat color_big(def_height[0], def_width[0], CV_8UC4);
	Mat depth_big(def_height[1], def_width[1], CV_16UC1);
	Mat users_big(def_height[1], def_width[1], CV_8UC3);
	Mat depth_to_show(def_height[1], def_width[1], CV_8UC3);
	Mat color_small(height, width, CV_8UC4);
	Mat users_small(height, width, CV_8UC3);
	Mat depth_tmp(def_height[1], def_width[1], CV_8UC1);
	unsigned int buffer_color = def_width[0] * def_height[0] * 4 * sizeof(unsigned char);
	unsigned int buffer_depth = def_width[1] * def_height[1] * sizeof(unsigned short);
	namedWindow("Depth");
	namedWindow("Users");

	while (!is_stopping)
	{
		try
		{
			IColorFrame* color_frame = nullptr;
			IDepthFrame* depth_frame = nullptr;
			IBodyFrame* body_frame = nullptr;
			IBodyIndexFrame* users_frame = nullptr;
			
			HRESULT hr = color_reader->AcquireLatestFrame(&color_frame);
			hr &= depth_reader->AcquireLatestFrame(&depth_frame);
			hr &= body_reader->AcquireLatestFrame(&body_frame);
			hr &= users_reader->AcquireLatestFrame(&users_frame);

			if (FAILED(hr))
				continue;

			color_frame->CopyConvertedFrameDataToArray(buffer_color, reinterpret_cast<BYTE*>(color_big.data), ColorImageFormat::ColorImageFormat_Bgra);
			cv::resize(color_big, color_small, cv::Size(), 0.5, 0.5);
			safeRelease(color_frame);

			depth_frame->AccessUnderlyingBuffer(&buffer_depth, reinterpret_cast<UINT16**>(&depth_big.data));
			depth_big.convertTo(depth_tmp, CV_8U, -255.0f / 4500.0f, 255.0f);
			cvtColor(depth_tmp, depth_to_show, COLOR_GRAY2RGB);
			safeRelease(depth_frame);

			unsigned int buffer_users = 0;
			unsigned char* tmp = nullptr;
			users_frame->AccessUnderlyingBuffer(&buffer_users, &tmp);
			for (int y = 0; y < def_height[1]; y++)
			{
				for (int x = 0; x < def_width[1]; x++)
				{
					unsigned int index = y * def_width[1] + x;
					if (tmp[index] != 0xff)
					{
						users_big.at<cv::Vec3b>(y, x) = users_color[tmp[index]];
					}
					else
					{
						users_big.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
					}
				}
			}
			cv::resize(users_big, users_small, cv::Size(), 0.5, 0.5);
			safeRelease(users_frame);

			getSkeletons(body_frame, depth_to_show, n_frame);
			
			imshow("Color", color_small);
			imshow("Depth", depth_to_show);
			imshow("Users", users_small);
			waitKey(10);

			stringstream ss; ss << n_frame;
			string id = ss.str();
			frame_name = rgb_folder + "img" + id + ".png";
			imwrite(frame_name, color_small);
			frame_name = depth_folder + "img" + id + ".png";
			imwrite(frame_name, depth_big);
			frame_name = users_folder + "img" + id + ".png";
			imwrite(frame_name, users_small);
			n_frame++;
		}
		catch (std::exception e)
		{
			printf("Program was stopped while waiting\n");
		}
	}
}