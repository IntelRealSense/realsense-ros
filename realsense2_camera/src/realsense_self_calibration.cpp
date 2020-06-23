// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

#include "realsense_self_calibration.h"

#include <iostream>

namespace realsense2_camera
{
    namespace self_calibration
    {
        rs2::calibration_table get_current_calibration_table(rs2::auto_calibrated_device& dev)
        {
            // Get current calibration table.
            return dev.get_calibration_table();
        }

        self_calibration_result self_calibration_step(const std::string& json_config, rs2::auto_calibrated_device& dev)
        {
            self_calibration_result result;

            // Run actual self-calibration.
            try {
                result.new_calibration_table = dev.run_on_chip_calibration(json_config, &result.health_score);
            } catch (const rs2::error& e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl; 
                throw;
            }

            return result;
        }

        bool validate_self_calibration(const self_calibration_result& result)
        {
            std::cout << "Self Calibration finished. Health = " << result.health_score << std::endl;
            const auto abs_score = std::abs(result.health_score);
            if (abs_score < self_calibration_health_thresholds::USABLE)
            {
                // Usable results.
                if (abs_score < self_calibration_health_thresholds::OPTIMAL)
                {
                    std::cout << "Optimal calibration results. You can proceed to write them to the device ROM." << std::endl;
                } 
                else if (abs_score < self_calibration_health_thresholds::USABLE)
                {
                    std::cout << "Calibration results are usable but not ideal. It is recommended to repeat the procedure." << std::endl;
                }
                return true;
            }

            // Unusable results
            std::cout << "Calibration results are unusable, please repeat the procedure." << std::endl;
            return false;
        }

        self_calibration_result self_calibrate(rs2::auto_calibrated_device& dev)
        {
            self_calibration_result calib_results;

            // Self calibration parameters;
            std::stringstream ss;
                    ss << "{\n \"speed\":" << 3 
                    << ",\n \"scan parameter\":" << 1 << "}";
            const std::string json = ss.str();

            // Loop until we get a good calibration.
            bool valid_calibration = false;
            std::size_t num_calib_attempts = 0;
            while(!valid_calibration && 
                    num_calib_attempts < self_calibration_health_thresholds::MAX_NUM_UNSUCCESSFUL_ITERATIONS)
            {
                std::cout << "Self calibration, iteration " << num_calib_attempts << "..." << std::endl;

                try {
                    calib_results = self_calibration_step(json, dev);
                    valid_calibration = validate_self_calibration(calib_results);
                } catch (const rs2::error& e) {
                    std::cout << "Self Calibration procedure was unsuccessful. Please point your sensor to an area with visible texture." << std::endl;
                }

                num_calib_attempts++;
            }

            calib_results.success = valid_calibration;

            return calib_results;
        }

        void write_calibration_to_device(const rs2::calibration_table& table, rs2::auto_calibrated_device& dev)
        {
            try {
                // Apply self calibration table.
                dev.set_calibration_table(table);

                // Write results to device's ROM.
                dev.write_calibration();
            } catch (const rs2::error& e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl; 
            }
        }

        void restore_factory_calibration(rs2::auto_calibrated_device& dev)
        {
            try {
                // Restore factory calibration.
                dev.reset_to_factory_calibration();
            } catch (const rs2::error& e) {
                std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl; 
            }
        }
    }

void run_self_calibration(rs2::device& dev)
{
    // Get auto calibration handler.
    rs2::auto_calibrated_device calib_dev = dev.as<rs2::auto_calibrated_device>();

    // Get current calibration table.
    std::cout << "Fetching current calibration table from device..." << std::endl;
    const rs2::calibration_table prev_calibration_table = self_calibration::get_current_calibration_table(calib_dev);

    // Self calibration.
    const self_calibration::self_calibration_result calib_results = self_calibration::self_calibrate(calib_dev);

    if(calib_results.success)
    {
        // Apply the new calibration values.
        self_calibration::write_calibration_to_device(calib_results.new_calibration_table, calib_dev);
    } else {
        // Restore factory calibration.
        self_calibration::restore_factory_calibration(calib_dev);
        std::cout << "Factory calibration restored." << std::endl;
    }
}

}