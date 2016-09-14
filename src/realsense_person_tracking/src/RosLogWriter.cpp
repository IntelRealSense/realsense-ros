/*#include "PtLogWriter.h"
#include "ros/ros.h"

void PtLogWriter::Write(Level level, const std::string& message)
{
	const char* c_message = message.c_str();
	switch (level)
	{
		case DEBUG:
            ROS_DEBUG_STREAM(c_message);
			break;
		case INFO:
			ROS_INFO_STREAM(c_message);
			break;
		case WARN:
			ROS_WARN_STREAM(c_message);
			break;
		case ERROR:
			ROS_ERROR_STREAM(c_message);
			break;
	}
}

void PtLogWriter::Write(Level level, const std::wstring& message)
{
}*/
