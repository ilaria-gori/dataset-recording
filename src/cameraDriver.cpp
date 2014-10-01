#include <cameraDriver.h>

using namespace std;
using namespace cv;

CameraDriver::CameraDriver()
{
	is_stopping = false;
}

CameraDriver::~CameraDriver()
{
	//destroy objects
}

bool CameraDriver::initialize(const int width, const int height)
{
	if (!camera.isOpened())
		return false;
	this->width = width;
	this->height = height;
	return true;
}

void CameraDriver::record(const string &destFolder)
{
	is_stopping = false;
	rgb_folder = destFolder + "rgb/";

	CreateDirectory(rgb_folder.c_str(), NULL);

	rec_t = std::thread(&CameraDriver::start, this);
}

void CameraDriver::stop()
{
	mtx.lock();
	is_stopping = true;
	mtx.unlock();
	if (rec_t.joinable())
		rec_t.join();
}

void CameraDriver::start()
{
	int n_frame=0;
	string frame_name;
	while (!is_stopping)
	{
		try
		{
			Mat frame;
			camera.read(frame);
			imshow("Color",frame);
			waitKey(10);
			stringstream ss; ss << n_frame;
			string id = ss.str();
			frame_name = rgb_folder + "img" + id+ ".png";
			imwrite(frame_name, frame);
			n_frame++;
		}
		catch (std::exception e)
		{
			printf("Program was stopped while waiting\n");
		}
	}
}