#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <string>

class Driver
{
	public:

		virtual bool initialize(const int width, const int height) = 0;

		virtual void record(const std::string &destFolder) = 0;

		virtual void stop() = 0;

};
#endif