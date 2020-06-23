// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#pragma once

#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API

namespace realsense2_camera
{

    namespace self_calibration
    {

        struct self_calibration_result
        {
            rs2::calibration_table new_calibration_table;
            float health_score;
            bool success;
        };

        struct self_calibration_health_thresholds
        {
            constexpr static float OPTIMAL = 0.25;
            constexpr static float USABLE = 0.75;
            constexpr static float UNUSABLE = 1.0;
            constexpr static std::size_t MAX_NUM_UNSUCCESSFUL_ITERATIONS = 5;
        };

        rs2::calibration_table get_current_calibration_table(rs2::auto_calibrated_device& dev);

        self_calibration_result self_calibration_step(const std::string& json_config, rs2::auto_calibrated_device& dev);

        bool validate_self_calibration(const self_calibration_result& result);

        void restore_factory_calibration(rs2::auto_calibrated_device& dev);

        self_calibration_result self_calibrate(rs2::auto_calibrated_device& dev);

        void write_calibration_to_device(const rs2::calibration_table& table, rs2::auto_calibrated_device& dev);

    }

    void run_self_calibration(rs2::device& dev);

}