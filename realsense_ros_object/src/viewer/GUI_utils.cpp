// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include "GUI_utils.h"
#include <iomanip>
#include <pwd.h>
#include <fstream>

bool GUI_utils::init_GUI(cv::String window_name)
{
  m_rect_thickness = 3;
  m_rect_line_type = 8;
  m_rect_shift = 0;
  m_text_thickness = 1;
  m_text_font = cv::FONT_HERSHEY_SIMPLEX;
  m_font_scale = 0.75;
  m_text_line_type = 5;
  m_window_name = window_name;
  m_image.create(468, 628, CV_8UC3);
  if (!m_image.data)
    return false;

  return true;
}

void GUI_utils::set_rect_thickness(int newThickness)
{
  m_rect_thickness = newThickness;
}

int GUI_utils::get_rect_thickness()
{
  return m_rect_thickness;
}

void GUI_utils::set_line_type(int newLineType)
{
  m_rect_line_type = newLineType;
}

int GUI_utils::get_line_type()
{
  return m_rect_line_type;
}

void GUI_utils::set_rect_shift(int newShift)
{
  m_rect_shift = newShift;
}

int GUI_utils::get_rect_shift()
{
  return m_rect_shift;
}

void GUI_utils::set_text_thickness(int newTextThickness)
{
  m_text_thickness = newTextThickness;
}

int GUI_utils::get_text_thickness()
{
  return m_text_thickness;
}

void GUI_utils::set_text_font(int newTextFont)
{
  m_text_font = newTextFont;
}

int GUI_utils::get_text_font()
{
  return m_text_font;
}

void GUI_utils::set_font_scale(int newFontScale)
{
  m_font_scale = newFontScale;
}

int GUI_utils::get_font_scale()
{
  return m_font_scale;
}

void GUI_utils::set_text_line_type(int newTextLineType)
{
  m_text_line_type = newTextLineType;
}

int GUI_utils::get_text_line_type()
{
  return m_text_line_type;
}

cv::Mat& GUI_utils::get_color_cvmat()
{
  return m_image;
}

cv::String GUI_utils::get_win_name()
{
  return m_window_name;
}

void GUI_utils::draw_text(cv::String text, cv::Point originPoint, int classID)
{
  // Getting the rectangle color
  cv::Scalar color = get_color(classID);

  //Getting title text size
  int baseline = 0;
  cv::Size nameTextSize = cv::getTextSize(text, m_text_font, m_font_scale, m_text_thickness, &baseline);
  baseline += m_text_thickness;

  // Getting beginning point of text
  cv::Point textOrigin(originPoint.x, originPoint.y - nameTextSize.height / 2);

  // Draw the box
  int filledRectangle = -1;
  cv::Point originShifted = textOrigin + cv::Point(0, baseline);
  cv::Point opositeToOrigin = textOrigin + cv::Point(nameTextSize.width, -nameTextSize.height);
  cv::rectangle(m_image, originShifted, opositeToOrigin, color, filledRectangle, m_rect_line_type, m_rect_shift);

  // Then put the text itself
  cv::putText(m_image, text, textOrigin, m_text_font, m_font_scale, get_text_color(classID), m_text_thickness, m_text_line_type);
}

void GUI_utils::draw_rect_no_text(cv::Point topLeftPoint, int width, int height, int classID)
{
  // Translating the given data to two opposite corners of the rectangle
  cv::Point* bottomRightPoint = new cv::Point(topLeftPoint.x + width, topLeftPoint.y + height);

  // Getting the rectangle color
  cv::Scalar color = get_color(classID);

  // Drawing the rectangle
  cv::rectangle(m_image, topLeftPoint, *bottomRightPoint, color, m_rect_thickness, m_rect_line_type, m_rect_shift);
}

bool GUI_utils::draw_rect(cv::String name, int classID, int x, int y, int width, int height)
{
  if (x == 0 || y == 0)
  {
    return false;
  }
  //Getting title text size
  int baseline = 0;
  cv::Size nameTextSize = cv::getTextSize(name, m_text_font, m_font_scale, m_text_thickness, &baseline);

  int minCoordinateValue = 10;
  cv::Point* topLeftPoint = new cv::Point(std::max(minCoordinateValue, x),
                                          std::max(minCoordinateValue  + nameTextSize.height, y));

  draw_rect_no_text(*topLeftPoint, width + std::min(x, 0), height  + std::min(y, 0), classID);

  draw_text(name, *topLeftPoint, classID);

  return true;
}


void GUI_utils::draw_no_results()
{
  cv::String text = "No object was found";
  int baseline = 0;

  // Getting the rectangle color
  int classID = 0; // This text doesn't belong to any object class
  cv::Scalar color = get_color(classID); // Meaning no color - black

  //Getting title text size
  cv::Size textSize = cv::getTextSize(text, m_text_font, m_font_scale, m_text_thickness, &baseline);
  baseline += m_text_thickness;

  // Getting beginning point of text
  cv::Point textOrigin(0, 0);

  // Draw the box
  int filledRectangle = -1;
  int rectangleHeightFactor = 2; // I can't say why, but this number seems to put the rectangle in it's right place
  cv::Point originShifted = textOrigin + cv::Point(0, baseline);
  cv::Point opositeToOrigin = textOrigin + cv::Point(textSize.width, rectangleHeightFactor * textSize.height);
  cv::rectangle(m_image, originShifted, opositeToOrigin, color, filledRectangle, m_rect_line_type, m_rect_shift);

  int finalTextShift = 3; // I can't say why, but this number seems to put the text in it's right place

  // Then put the text itself
  cv::putText(m_image, text, finalTextShift * originShifted, m_text_font, m_font_scale,
              get_text_color(classID), m_text_thickness, m_text_line_type);

}

bool GUI_utils::draw_results(const sensor_msgs::ImageConstPtr& color, const realsense_ros_object::ObjectsInBoxes& msg)
{

  try
  {

    m_image =  cv_bridge::toCvShare(color, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color->encoding.c_str());
    return false;
  }

  if (m_image.data == NULL)
  {
    std::cout << "No image found! Check path.\n";
    return false;//ERROR
  }

  for (int i = 0; i < (int)msg.objects_vector.size(); i++)
  {

    cv::String text = msg.objects_vector[i].object.object_name + " " + get_3D_location_string(msg.objects_vector[i].location.coordinates); //todo: drow vertical and horisontical margin


    draw_rect(text, 3 * i, (int)msg.objects_vector[i].object_bbox.x, (int)msg.objects_vector[i].object_bbox.y, (int)msg.objects_vector[i].object_bbox.width, (int)msg.objects_vector[i].object_bbox.height);
  }

  return true;
}

char GUI_utils::save_results(const realsense_ros_object::ObjectsInBoxes &msg, std::string file_path)
{
  if(file_path == "")
  {
    const char *home_dir = getenv("HOME");

    if ( home_dir== NULL) {
        home_dir = getpwuid(getuid())->pw_dir;
    }
    file_path = std::string(home_dir) + "/object_bag_results.txt";
  }

  std::fstream myfile;
  myfile.open (file_path, std::ofstream::out | std::ofstream::app);
  static int frame_num = -1;
  frame_num++;
  myfile << "frame " + std::to_string(frame_num) << std::endl;
  for (int i = 0; i < (int)msg.objects_vector.size(); i++)
  {
    myfile << "object id: " << i << std::endl;
    myfile << msg.objects_vector[i].object.object_name << std::endl;
    myfile << msg.objects_vector[i].object_bbox.x << std::endl;
    myfile << msg.objects_vector[i].object_bbox.y << std::endl;
    myfile << msg.objects_vector[i].object_bbox.width << std::endl;
    myfile << msg.objects_vector[i].object_bbox.height << std::endl;

    myfile << msg.objects_vector[i].location.coordinates.x << std::endl;
    myfile << msg.objects_vector[i].location.coordinates.y << std::endl;
    myfile << msg.objects_vector[i].location.coordinates.z << std::endl;
    myfile << msg.objects_vector[i].location.horiz_margin << std::endl;
    myfile << msg.objects_vector[i].location.vert_margin << std::endl;
  }
  myfile.close();
}

std::vector<realsense_ros_object::ObjectsInBoxes> GUI_utils::read_results(std::string file_path)
{

  std::string frame("frame");
  std::string object_id("object id:");
  std::vector<realsense_ros_object::ObjectsInBoxes> res;
  std::ifstream resFile;
  resFile.open(file_path);
  std::string str;
  if (resFile.is_open())
  {
    bool new_frame = true;
    realsense_ros_object::ObjectsInBoxes objs;
    while(new_frame)
    {
      new_frame = false;
      std::getline(resFile, str);
      while(std::getline(resFile, str))
      {
        if(!str.compare(0, object_id.size(), object_id))
        {
          realsense_ros_object::ObjectInBox objectInBox;
          std::getline(resFile, objectInBox.object.object_name);
          std::getline(resFile, str);
          objectInBox.object_bbox.x = std::stoi(str);
          std::getline(resFile, str);
          objectInBox.object_bbox.y = std::stoi(str);
          std::getline(resFile, str);
          objectInBox.object_bbox.width = std::stoi(str);
          std::getline(resFile, str);
          objectInBox.object_bbox.height = std::stoi(str);
          std::getline(resFile, str);
          objectInBox.location.coordinates.x = std::stoi(str);
          std::getline(resFile, str);
          objectInBox.location.coordinates.y = std::stoi(str);
          std::getline(resFile, str);
          objectInBox.location.coordinates.z = std::stoi(str);
          std::getline(resFile, str);
          objectInBox.location.horiz_margin = std::stof(str);
          std::getline(resFile, str);
          objectInBox.location.vert_margin = std::stof(str);
          objs.objects_vector.push_back(objectInBox);
        }
        else if (!str.compare(0, frame.size(), frame))
        {
            new_frame = true;
            res.push_back(objs);
        }
      }

    }
  }

  for(auto & frame : res)
  {
      save_results(frame, std::string("/home/sshoshan/object_bag_results2.txt"));
  }


  return res;
}

char GUI_utils::show_results()
{
  cv::namedWindow(m_window_name, cv::WINDOW_AUTOSIZE);
  cv::imshow(m_window_name, m_image);
  char c = cv::waitKey(5);
  return c;
}

cv::Scalar GUI_utils::get_color(int classID)
{
  if (classID > m_max_classes || classID < 1)
  {
    classID = m_max_classes;
  }

  return cv::Scalar(m_color_mat.at<unsigned char>(classID - 1, 0),
                    m_color_mat.at<unsigned char>(classID - 1, 1),
                    m_color_mat.at<unsigned char>(classID - 1, 2),
                    m_color_mat.at<unsigned char>(classID - 1, 3));
}

cv::Scalar GUI_utils::get_text_color(int classID)
{
  if (classID > m_max_classes || classID < 1)
  {
    classID = m_max_classes;
  }

  return cv::Scalar(m_text_mat.at<unsigned char>(classID - 1, 0),
                    m_text_mat.at<unsigned char>(classID - 1, 1),
                    m_text_mat.at<unsigned char>(classID - 1, 2),
                    m_text_mat.at<unsigned char>(classID - 1, 3));
}

std::string GUI_utils::get_3D_location_string(geometry_msgs::Point32 location)
{
  std::stringstream str_stream;
  str_stream << std::fixed << std::setprecision(1) << (location.z / 1000);
  std::string str = "@";
  str += str_stream.str() + "m";
  return str;
}
