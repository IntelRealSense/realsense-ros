// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "../include/realsense_node_factory.h"
#include "../include/base_realsense_node.h"
#include "../include/t265_realsense_node.h"
#include <iostream>
#include <map>
#include <mutex>
#include <condition_variable>
#include <signal.h>
#include <thread>
#include <sys/time.h>

using namespace realsense2_camera;

#define REALSENSE_ROS_EMBEDDED_VERSION_STR (VAR_ARG_STRING(VERSION: REALSENSE_ROS_MAJOR_VERSION.REALSENSE_ROS_MINOR_VERSION.REALSENSE_ROS_PATCH_VERSION))
constexpr auto realsense_ros_camera_version = REALSENSE_ROS_EMBEDDED_VERSION_STR;

PLUGINLIB_EXPORT_CLASS(realsense2_camera::RealSenseNodeFactory, nodelet::Nodelet)

rs2::device _device;

RealSenseNodeFactory::RealSenseNodeFactory()
{
	ROS_INFO("RealSense ROS v%s", REALSENSE_ROS_VERSION_STR);
	ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

	signal(SIGINT, signalHandler);
	auto severity = rs2_log_severity::RS2_LOG_SEVERITY_WARN;
	tryGetLogSeverity(severity);
	if (rs2_log_severity::RS2_LOG_SEVERITY_DEBUG == severity)
		ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	rs2::log_to_console(severity);
}

void RealSenseNodeFactory::closeDevice()
{
    for(rs2::sensor sensor : _device.query_sensors())
	{
		sensor.stop();
		sensor.close();
	}
}

void RealSenseNodeFactory::signalHandler(int signum)
{
	ROS_INFO_STREAM(strsignal(signum) << " Signal is received! Terminating RealSense Node...");
	closeDevice();
	ros::shutdown();
	exit(signum);
}

rs2::device RealSenseNodeFactory::getDevice()
{
	rs2::device retDev;
	auto list = _ctx.query_devices();
	if (0 == list.size())
	{
		ROS_WARN("No RealSense devices were found!");
	}
	else
	{
		bool found = false;
		for (auto&& dev : list)
		{
			auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			ROS_DEBUG_STREAM("Device with serial number " << sn << " was found.");
			if (_serial_no.empty() || sn == _serial_no)
			{
				retDev = dev;
				_serial_no = sn;
				found = true;
				break;
			}
		}
		if (!found)
		{
			ROS_ERROR_STREAM("The requested device with serial number " << _serial_no << " is NOT found!");
		}
	}

	bool remove_tm2_handle(retDev && RS_T265_PID != std::stoi(retDev.get_info(RS2_CAMERA_INFO_PRODUCT_ID), 0, 16));
	if (remove_tm2_handle)
	{
		_ctx.unload_tracking_module();
	}

	if (retDev && _initial_reset)
	{
		_initial_reset = false;
		try
		{
			ROS_INFO("Resetting device...");
			retDev.hardware_reset();
			retDev = rs2::device();
			
		}
		catch(const std::exception& ex)
		{
			ROS_WARN_STREAM("An exception has been thrown: " << ex.what());
		}
	}
	return retDev;
}

void RealSenseNodeFactory::change_device_callback(rs2::event_information& info)
{
	if (info.was_removed(_device))
	{
		ROS_ERROR("The device has been disconnected!");
		_realSenseNode.reset(nullptr);
		_device = rs2::device();
	}
	if (!_device)
	{
		rs2::device_list new_devices = info.get_new_devices();
		if (new_devices.size() > 0)
		{
			ROS_INFO("Checking new devices...");
			_device = getDevice();
			if (_device)
			{
				StartDevice();
			}
		}
	}
}

void RealSenseNodeFactory::onInit()
{
	try
	{
#ifdef BPDEBUG
		std::cout << "Attach to Process: " << getpid() << std::endl;
		std::cout << "Press <ENTER> key to continue." << std::endl;
		std::cin.get();
#endif
		ros::NodeHandle nh = getNodeHandle();
		auto privateNh = getPrivateNodeHandle();
		privateNh.param("serial_no", _serial_no, std::string(""));

		std::string rosbag_filename("");
		privateNh.param("rosbag_filename", rosbag_filename, std::string(""));
		if (!rosbag_filename.empty())
		{
			ROS_INFO_STREAM("publish topics from rosbag file: " << rosbag_filename.c_str());
			auto pipe = std::make_shared<rs2::pipeline>();
			rs2::config cfg;
			cfg.enable_device_from_file(rosbag_filename.c_str(), false);
			cfg.enable_all_streams();
			pipe->start(cfg); //File will be opened in read mode at this point
			_device = pipe->get_active_profile().get_device();
			_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
			_realSenseNode->publishTopics();
			_realSenseNode->registerDynamicReconfigCb(nh);
		}
		else
		{
			std::function<void(rs2::event_information&)> change_device_callback_function = [this](rs2::event_information& info){change_device_callback(info);};
			_ctx.set_devices_changed_callback(change_device_callback_function);
			privateNh.param("initial_reset", _initial_reset, false);
			_device = getDevice();
		}

		if (_device)
		{
			StartDevice();
		}
	}
	catch(const std::exception& ex)
	{
		ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
		throw;
	}
	catch(...)
	{
		ROS_ERROR_STREAM("Unknown exception has occured!");
		throw;
	}
}

void RealSenseNodeFactory::StartDevice()
{
	ros::NodeHandle nh = getNodeHandle();
	ros::NodeHandle privateNh = getPrivateNodeHandle();
	// TODO
	std::string pid_str(_device.get_info(RS2_CAMERA_INFO_PRODUCT_ID));
	uint16_t pid = std::stoi(pid_str, 0, 16);
	switch(pid)
	{
	case SR300_PID:
	case RS400_PID:
	case RS405_PID:
	case RS410_PID:
	case RS460_PID:
	case RS415_PID:
	case RS420_PID:
	case RS420_MM_PID:
	case RS430_PID:
	case RS430_MM_PID:
	case RS430_MM_RGB_PID:
	case RS435_RGB_PID:
	case RS435i_RGB_PID:
	case RS_USB2_PID:
		_realSenseNode = std::unique_ptr<BaseRealSenseNode>(new BaseRealSenseNode(nh, privateNh, _device, _serial_no));
		break;
	case RS_T265_PID:
		_realSenseNode = std::unique_ptr<T265RealsenseNode>(new T265RealsenseNode(nh, privateNh, _device, _serial_no));
		break;
	default:
		ROS_FATAL_STREAM("Unsupported device!" << " Product ID: 0x" << pid_str);
		ros::shutdown();
		exit(1);
	}
	assert(_realSenseNode);
	_realSenseNode->publishTopics();
	_realSenseNode->registerDynamicReconfigCb(nh);
}

void RealSenseNodeFactory::tryGetLogSeverity(rs2_log_severity& severity) const
{
	static const char* severity_var_name = "LRS_LOG_LEVEL";
	auto content = getenv(severity_var_name);

	if (content)
	{
		std::string content_str(content);
		std::transform(content_str.begin(), content_str.end(), content_str.begin(), ::toupper);

		for (uint32_t i = 0; i < RS2_LOG_SEVERITY_COUNT; i++)
		{
			auto current = std::string(rs2_log_severity_to_string((rs2_log_severity)i));
			std::transform(current.begin(), current.end(), current.begin(), ::toupper);
			if (content_str == current)
			{
				severity = (rs2_log_severity)i;
				break;
			}
		}
	}
}
