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
	safeRelease(multi_source_frame_reader);
	safeRelease(coordinate_mapper);

	if (sensor)
	{
		sensor->Close();
	}

	safeRelease(sensor);
}

bool Kinect2Driver::initialize(const int width, const int height)
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&sensor);
	if (FAILED(hr))
	{
		return false;
	}

	if (sensor)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		if (SUCCEEDED(hr))
		{
			hr = sensor->get_CoordinateMapper(&coordinate_mapper);
		}

		hr = sensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = sensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex | FrameSourceTypes::FrameSourceTypes_Body,
				&multi_source_frame_reader);
		}
	}

	this->width = width;
	this->height = height;

	if (!sensor || FAILED(hr))
	{
		return false;
	}

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

void Kinect2Driver::getSkeletons(IBody** bodies, Mat &skeleton, const int n_frame)
{
	stringstream ss; ss << n_frame;
	string id = ss.str();
	ofstream skeleton_file;
	string frame_name = skel_folder + "joints" + id + ".txt";
	skeleton_file.open(frame_name);

	if (coordinate_mapper)
	{
		for (int count = 0; count < BODY_COUNT; count++)
		{
			IBody* body = bodies[count];
			if (body)
			{
				BOOLEAN tracked = false;
				HRESULT hr = body->get_IsTracked(&tracked);

				if (SUCCEEDED(hr) && tracked)
				{
					skeleton_file << "user " << count << "\n";
					Joint joints[JointType::JointType_Count];
					hr = body->GetJoints(_countof(joints), joints);

					if (SUCCEEDED(hr))
					{
						for (int type = 0; type < _countof(joints); type++)
						{
							skeleton_file << "joint " << type << " ";
							DepthSpacePoint depth_space_point = { 0 };
							coordinate_mapper->MapCameraPointToDepthSpace(joints[type].Position, &depth_space_point);
							int x = static_cast<int>(depth_space_point.X);
							int y = static_cast<int>(depth_space_point.Y);
							skeleton_file << joints[type].Position.X << " " << joints[type].Position.Y << " " << joints[type].Position.Z << " " << depth_space_point.X << " " << depth_space_point.Y << "\n";
							if ((x >= 0) && (x < depth_width) && (y >= 0) && (y < depth_height))
							{
								cv::circle(skeleton, cv::Point(x, y), 5, static_cast<cv::Scalar>(users_color[count]), -1);
							}
						}
					}
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

	Mat color_big;
	Mat color_small(height, width, CV_8UC4);
	Mat depth_big;
	Mat users_big;
	Mat depth_to_show;
	Mat skeleton;
	unsigned int buffer_color;
	unsigned int buffer_depth;
	
	while (!is_stopping)
	{
		if (!multi_source_frame_reader)
		{
			return;
		}

		IMultiSourceFrame* multi_source_frame = NULL;
		IDepthFrame* depth_frame = NULL;
		IColorFrame* color_frame = NULL;
		IBodyIndexFrame* body_index_frame = NULL;
		IBodyFrame* body_frame = NULL;

		HRESULT hr = multi_source_frame_reader->AcquireLatestFrame(&multi_source_frame);

		if (SUCCEEDED(hr))
		{
			IDepthFrameReference* depth_reference_frame = NULL;

			hr = multi_source_frame->get_DepthFrameReference(&depth_reference_frame);
			if (SUCCEEDED(hr))
			{
				hr = depth_reference_frame->AcquireFrame(&depth_frame);
			}

			safeRelease(depth_reference_frame);
		}

		if (SUCCEEDED(hr))
		{
			IColorFrameReference* color_frame_reference = NULL;

			hr = multi_source_frame->get_ColorFrameReference(&color_frame_reference);
			if (SUCCEEDED(hr))
			{
				hr = color_frame_reference->AcquireFrame(&color_frame);
			}

			safeRelease(color_frame_reference);
		}

		if (SUCCEEDED(hr))
		{
			IBodyIndexFrameReference* body_ind_frame_reference = NULL;

			hr = multi_source_frame->get_BodyIndexFrameReference(&body_ind_frame_reference);
			if (SUCCEEDED(hr))
			{
				hr = body_ind_frame_reference->AcquireFrame(&body_index_frame);
			}

			safeRelease(body_ind_frame_reference);
		}

		if (SUCCEEDED(hr))
		{
			IBodyFrameReference* body_frame_reference = NULL;

			hr = multi_source_frame->get_BodyFrameReference(&body_frame_reference);
			if (SUCCEEDED(hr))
			{
				hr = body_frame_reference->AcquireFrame(&body_frame);
			}

			safeRelease(body_frame_reference);
		}

		if (SUCCEEDED(hr))
		{
			INT64 depth_time = 0;
			IFrameDescription* depth_description = NULL;

			IFrameDescription* color_description = NULL;
			int color_width = 0;
			int color_height = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;

			IFrameDescription* body_index_description = NULL;
			int body_index_width = 0;
			int body_index_height = 0;
			UINT nBodyIndexBufferSize = 0;
			BYTE *pBodyIndexBuffer = NULL;

			IBody* bodies[BODY_COUNT] = { 0 };

			// get depth frame data

			hr = depth_frame->get_RelativeTime(&depth_time);

			if (SUCCEEDED(hr))
			{
				hr = depth_frame->get_FrameDescription(&depth_description);
			}

			if (SUCCEEDED(hr))
			{
				hr = depth_description->get_Width(&depth_width);
			}

			if (SUCCEEDED(hr))
			{
				hr = depth_description->get_Height(&depth_height);
			}

			// get color frame data

			if (SUCCEEDED(hr))
			{
				hr = color_frame->get_FrameDescription(&color_description);
			}

			if (SUCCEEDED(hr))
			{
				hr = color_description->get_Width(&color_width);
			}

			if (SUCCEEDED(hr))
			{
				hr = color_description->get_Height(&color_height);
			}

			if (SUCCEEDED(hr))
			{
				hr = color_frame->get_RawColorImageFormat(&imageFormat);
			}

			// get body index frame data

			if (SUCCEEDED(hr))
			{
				hr = body_index_frame->get_FrameDescription(&body_index_description);
			}

			if (SUCCEEDED(hr))
			{
				hr = body_index_description->get_Width(&body_index_width);
			}

			if (SUCCEEDED(hr))
			{
				hr = body_index_description->get_Height(&body_index_height);
			}

			if (SUCCEEDED(hr))
			{
				hr = body_frame->GetAndRefreshBodyData(_countof(bodies), bodies);
			}

			if (SUCCEEDED(hr))
			{
				color_big.create(color_height, color_width, CV_8UC4);
				depth_big.create(depth_height, depth_width, CV_16SC1);
				users_big.create(body_index_height, body_index_width, CV_8UC3);
				skeleton.create(body_index_height, body_index_width, CV_8UC3);
				skeleton = Mat::zeros(body_index_height, body_index_width, CV_8UC3);
				depth_to_show.create(depth_height, depth_width, CV_8UC1);
				buffer_color = color_width * color_height * 4 * sizeof(unsigned char);
				buffer_depth = depth_width * depth_height * sizeof(unsigned short);

				unsigned int buffer_users = 0;
				unsigned char* tmp = nullptr;
				hr = body_index_frame->AccessUnderlyingBuffer(&buffer_users, &tmp);
				if (SUCCEEDED(hr))
				{
					for (int y = 0; y < depth_height; y++)
					{
						for (int x = 0; x < depth_width; x++)
						{
							unsigned int index = y * depth_width + x;
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
				}

				hr = depth_frame->AccessUnderlyingBuffer(&buffer_depth, reinterpret_cast<UINT16**>(&depth_big.data));
				if (SUCCEEDED(hr))
					depth_big.convertTo(depth_to_show, CV_8U, -255.0f / 4500.0f, 255.0f);

				hr = color_frame->CopyConvertedFrameDataToArray(buffer_color, reinterpret_cast<BYTE*>(color_big.data), ColorImageFormat::ColorImageFormat_Bgra);
				if (SUCCEEDED(hr))
					cv::resize(color_big, color_small, color_small.size());

				getSkeletons(bodies, skeleton, n_frame);

				imshow("Color", color_small);
				imshow("Depth", depth_to_show);
				imshow("Users", users_big);
				imshow("Skeleton", skeleton);
				waitKey(10);

				stringstream ss; ss << n_frame;
				string id = ss.str();
				frame_name = rgb_folder + "img" + id + ".png";
				imwrite(frame_name, color_small);
				frame_name = depth_folder + "img" + id + ".png";
				imwrite(frame_name, depth_big);
				frame_name = users_folder + "img" + id + ".png";
				imwrite(frame_name, users_big);
				n_frame++;
			}

			for (int i = 0; i < _countof(bodies); ++i)
			{
				safeRelease(bodies[i]);
			}
			safeRelease(depth_description);
			safeRelease(color_description);
			safeRelease(body_index_description);
		}

		safeRelease(depth_frame);
		safeRelease(color_frame);
		safeRelease(body_index_frame);
		safeRelease(body_frame);
		safeRelease(multi_source_frame);
	}
}