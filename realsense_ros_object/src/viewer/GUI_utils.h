// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#pragma once

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include <string>
#include <geometry_msgs/Point32.h>
#include "realsense_ros_object/ObjectsInBoxes.h"

/**
 * @brief Class designed to serve the localization demo app, and to display localization rectangles
 * together with their recognized objects.
 */
class GUI_utils
{
public:
  bool init_GUI(cv::String window_name);

  /**
   * @brief Sets new thickness for the drawn bounding rectangles.
   *
   * @remark The thickness values are defined by OpenCV. negative values will draw a filled rectangle.
   *
   * @param[in] new_thickness The new thickness to be set
   */
  void set_rect_thickness(int new_thickness);

  /**
   * @brief Returns the current value of the thickness of the drawn bounding rectangles.
   *
   * @return bounding rectangle thickness value.
   */
  int get_rect_thickness();

  /**
   * @brief Sets new line type for the drawn bounding rectangles.
   *
   * @remark The line types values are defined by OpenCV. Please view OpenCV documentation.
   *
   * @param[in] new_line_type The new line type to be set
   */
  void set_line_type(int new_line_type);

  /**
   * @brief Returns the current value of the line type of the drawn bounding rectangles.
   *
   * @return bounding rectangle line type value.
   */
  int get_line_type();

  /**
   * @brief Sets new shift for the drawn bounding rectangles.
   *
   * @remark The shift values are defined by OpenCV as number of fractional bits in the point coordinates
   *
   * @param[in] new_shift The new shift value to be set
   */
  void set_rect_shift(int new_shift);

  /**
   * @brief Returns the current value of the shift of the drawn bounding rectangles.
   *
   * @return bounding rectangle shift value.
   */
  int get_rect_shift();

  /**
   * @brief Sets new text thickness for the bounding rectangle title.
   *
   * @remark The thickness values are defined by OpenCV.
   *
   * @param[in] new_text_thickness The new shift value to be set
   */
  void set_text_thickness(int new_text_thickness);

  /**
   * @brief Returns the current value of the textThickness for the bounding rectangle title.
   *
   * @return Text thickness value.
   */
  int get_text_thickness();

  /**
   * @brief Sets new text font value for the bounding rectangle title.
   *
   * @remark The text font values are defined by OpenCV.
   *
   * @param[in] new_text_font The new text font to be set
   */
  void set_text_font(int new_text_font);

  /**
   * @brief Returns the text font value for the bounding rectangle title.
   *
   * @return Text font value.
   */
  int get_text_font();

  /**
   * @brief Sets new text font scale value for the bounding rectangle title.
   *
   * @remark The text font scale values are defined by OpenCV.
   *
   * @param[in] new_font_scale The new text font scale to be set
   */
  void set_font_scale(int new_font_scale);

  /**
   * @brief Returns the text font scale value for the bounding rectangle title.
   *
   * @return Text font scale value.
   */
  int get_font_scale();

  /**
   * @brief Sets new text line type value for the bounding rectangle title.
   *
   * @remark The text line type values are defined by OpenCV.
   *
   * @param[in] new_text_line_type The new text font scale to be set
   */
  void set_text_line_type(int new_text_line_type);

  /**
   * @brief Returns the text line type value for the bounding rectangle title.
   *
   * @return Text line type value.
   */
  int get_text_line_type();

  /**
   * @brief Returns pointer to processed image (color image)
   *
   * @return Pointer to color image
   */
  cv::Mat& get_color_cvmat();

  /**
   * @brief Returns the window name showing the streaming video.
   *
   * @return Window name.
   */
  cv::String get_win_name();

  /**
   * @brief Receives rectangle parameters as represented by object recognition module and an
   * object name, then draws the rectangle with the objects name as a title.
   *
   * @param[in] name Name of the bounded object.
   * @param[in] class_ID The class ID that represents the object class. Should suit the objects name.
   * @param[in] x Relative coordinate x of top left point.
   * @param[in] y Relative coordinate y of top left point.
   * @param[in] height Relative height of the rectangle.
   * @param[in] width Relative width of the rectangle.
   *
   * @return True if rectangle was drawn successfully , false otherwise.
   */
  bool draw_rect(cv::String name, int class_ID, int x, int y, int width, int height);

  bool draw_results(const sensor_msgs::ImageConstPtr& color, const realsense_ros_object::ObjectsInBoxes& msg);


  /**
   * @brief Draws the previously calculated results
   *
   * @return Pressed key, if exists.
   */
  char show_results();

  /**
   * @brief Write the previously calculated results to text file
   *
   * @return True if results were writen successfully , false otherwise
   */
  char save_results(const realsense_ros_object::ObjectsInBoxes& msg, std::string file_path = "");

  /**
   * @brief Read the calculated results from text file.
   *
   * @return vector of results
   */
  std::vector<realsense_ros_object::ObjectsInBoxes> read_results(std::string file_path);

protected:
  /**
   * @brief Gets an object ID and returns unique color.
   *
   * @warning Addresses only objects recognized by the localization or recognition mechanism.
   * @remark In case of unrecognized object - the color black will be returned.
   *
   * @param[in] class_ID: The class ID that represents the object class.
   *
   * @return Unique RGBA color represented as a cv::Scalar.
   */
  cv::Scalar get_color(int class_ID);

  /**
   * @brief Gets an object ID and returns a text color that suits its unique color.
   *
   * @warning Addresses only objects recognized by the localization or recognition mechanism.
   * @remark In case of unrecognized object - the color white will be returned.
   *
   * @param[in] class_ID: The class ID that represents the object class.
   *
   * @return RGBA text color that suits the objects unique color.
   */
  cv::Scalar get_text_color(int class_ID);

  /**
   * @brief Shows "No Results" message, meaning the OR localization algorithm returned no recognised results.
   */
  void draw_no_results();

  /**
   * @brief Draws rectangle in a color suitable to classID
   *
   * @param[in] top_left_point Top left point of the rectangle
   * @param[in] width Width of the rectangle
   * @param[in] height of the rectangle
   * @param[in] class_ID the id of the object class, used to determine the color
   */
  void draw_rect_no_text(cv::Point top_left_point, int width, int height, int class_ID);

  /**
   * @brief Draws text on image
   *
   * @param[in] text The text to draw
   * @param[in] origin_point The bottom left corner of the text
   * @param[in] class_ID the id of the object class, used to determine the color
   */
  void draw_text(cv::String text, cv::Point origin_point, int class_ID);

  std::string get_3D_location_string(geometry_msgs::Point32 location);

  int m_rect_thickness;
  int m_rect_line_type;
  int m_rect_shift;
  int m_text_thickness;
  int m_text_font;
  double m_font_scale;
  int m_text_line_type;
  cv::Mat m_image;
  cv::String m_window_name;
  std::vector<int> m_objects_IDs_for_tracking_localization;

  static const int m_max_classes = 44;

  const unsigned char m_color_arr[m_max_classes][4] =
  {
    {102, 0, 0, 0},
    {153, 0, 0, 0},
    {255, 102, 102, 0},
    {255, 128, 0, 0},
    {255, 178, 102, 0},
    {255, 255, 0, 0},
    {76, 153, 0, 0},
    {128, 255, 0, 0},
    {0, 255, 128, 0},
    {0, 153, 153, 0},
    {0, 204, 204, 0},
    {0, 255, 255, 0},
    {0, 76, 153, 0},
    {0, 128, 255, 0},
    {153, 204, 255, 0},
    {0, 0, 153, 0},
    {0, 0, 255, 0},
    {102, 102, 255, 0},
    {76, 0, 153, 0},
    {127, 0, 255, 0},
    {178, 102, 255, 0},
    {153, 0, 153, 0},
    {204, 0, 204, 0},
    {255, 102, 255, 0},
    {102, 0, 51, 0},
    {204, 0, 102, 0},
    {255, 102, 178, 0},
    {96, 96, 96, 0},
    {192, 192, 192, 0},
    {204, 204, 255, 0},
    {255, 204, 229, 0},
    {0, 102, 51, 0},
    {102, 51, 0, 0},
    {102, 102, 0, 0},
    {153, 153, 0, 0},
    {204, 255, 204, 0},
    {255, 255, 255, 0},
    {255, 204, 204, 0},
    {153, 153, 255, 0},
    {255, 153, 204, 0},
    {0, 153, 76, 0},
    {204, 102, 0, 0},
    {0, 102, 102, 0},
    {0, 0, 0, 0}
  };

  const unsigned char m_text_arr[m_max_classes][4] =
  {
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {0, 0, 0, 0}, //3
    {255, 255, 255, 0},
    {0, 0, 0, 0}, //5
    {0, 0, 0, 0},
    {255, 255, 255, 0}, //7
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {255, 255, 255, 0}, //10
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {255, 255, 255, 0}, //13
    {255, 255, 255, 0}, //14
    {0, 0, 0, 0},
    {255, 255, 255, 0}, //16
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0}, //28
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {255, 255, 255, 0}, //32
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0}, //35
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0},
    {255, 255, 255, 0}
  };
  const cv::Mat m_color_mat = cv::Mat(m_max_classes, 4, CV_8UC1, const_cast<void*>((const void*)(&m_color_arr)));
  const cv::Mat m_text_mat = cv::Mat(m_max_classes, 4, CV_8UC1, const_cast<void*>((const void*)(&m_text_arr)));
};
