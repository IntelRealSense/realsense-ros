#include <named_filter.h>
#include <fstream>
#include <sensor_msgs/point_cloud2_iterator.hpp>


using namespace realsense2_camera;

void NamedFilter::setParameters()
{
    if (_name == "disparity_end")
        return; // parameters for disparity are set in the disparity_start filter.
    std::stringstream module_name_str;
    std::string module_name = create_graph_resource_name(rs2_to_ros(_filter->get_info(RS2_CAMERA_INFO_NAME)));
    module_name_str << module_name;
    _params.registerDynamicOptions(*(_filter.get()), module_name_str.str());
    module_name_str << ".enable";

    _params.getParameters()->setParamT(module_name_str.str(), rclcpp::ParameterValue(_is_enabled), _is_enabled, [this](const rclcpp::Parameter& parameter)
            {
                set(parameter.get_value<bool>());
            });
}

PointcloudFilter::PointcloudFilter(std::string name, std::shared_ptr<rs2::filter> filter, rclcpp::Node& node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool is_enabled):
    NamedFilter(name, filter, parameters, logger, is_enabled),
    _node(node)
    {
        _params.getParameters()->setParamT(std::string("allow_no_texture_points"), rclcpp::ParameterValue(ALLOW_NO_TEXTURE_POINTS), _allow_no_texture_points);
        _params.getParameters()->setParamT(std::string("ordered_pc"), rclcpp::ParameterValue(ORDERED_PC), _ordered_pc);
        set(_is_enabled);
    }

void PointcloudFilter::set(const bool is_enabled)
{
    std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
    if ((is_enabled) && (!_pointcloud_publisher))
    {
        ROS_INFO_STREAM("create pointcloud publisher");
        _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("depth/color/points", 1);
        ROS_INFO_STREAM("Done." << __LINE__);
    }
    else if ((!is_enabled) && (_pointcloud_publisher))
    {
        ROS_INFO_STREAM("delete pointcloud publisher");
        _pointcloud_publisher.reset();
        ROS_INFO_STREAM("Done." << __LINE__);
    }
    NamedFilter::set(is_enabled);
}

void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

void PointcloudFilter::Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id)
{
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if ((!_pointcloud_publisher) || (!(_pointcloud_publisher->get_subscription_count())))
            return;
    }
    rs2_stream texture_source_id = static_cast<rs2_stream>(_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(5);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };
        
        texture_frame_itr = find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f) 
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = _filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
    }

    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();

    _valid_pc_indices.clear();
    rs2_intrinsics depth_intrin = pc.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");    
    modifier.resize(pc.size());
    if (_ordered_pc)
    {
        _msg_pointcloud.width = depth_intrin.width;
        _msg_pointcloud.height = depth_intrin.height;
        _msg_pointcloud.is_dense = false;
    }

    vertex = pc.get_vertices();
    size_t valid_count(0);
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }
        _msg_pointcloud.point_step = addPointField(_msg_pointcloud, format_str.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32, _msg_pointcloud.point_step);
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(_msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
        {
            float i(color_point->u);
            float j(color_point->v);
            bool valid_color_pixel(i >= 0.f && i <=1.f && j >= 0.f && j <=1.f);
            bool valid_pixel(vertex->z > 0 && (valid_color_pixel || _allow_no_texture_points));
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                if (valid_color_pixel)
                {
                    color_pixel[0] = i * texture_width;
                    color_pixel[1] = j * texture_height;
                    int pixx = static_cast<int>(color_pixel[0]);
                    int pixy = static_cast<int>(color_pixel[1]);
                    int offset = (pixy * texture_width + pixx) * num_colors;
                    reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
                }
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_color;
                ++valid_count;
            }
        }
    }
    else
    {
        std::string format_str = "intensity";
        _msg_pointcloud.point_step = addPointField(_msg_pointcloud, format_str.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32, _msg_pointcloud.point_step);
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");

        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++)
        {
            bool valid_pixel(vertex->z > 0);
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;
    
                ++iter_x; ++iter_y; ++iter_z;
                ++valid_count;
            }
        }
    }
    _msg_pointcloud.header.stamp = t;
    _msg_pointcloud.header.frame_id = frame_id;
    if (!_ordered_pc)
    {
        _msg_pointcloud.width = valid_count;
        _msg_pointcloud.height = 1;
        _msg_pointcloud.is_dense = true;
        modifier.resize(valid_count);
    }
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (_pointcloud_publisher)
            _pointcloud_publisher->publish(_msg_pointcloud);
    }
}

// void PointcloudFilter::Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id)
// {
//     {
//         std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
//         if ((!_pointcloud_publisher) || (!(_pointcloud_publisher->get_subscription_count())))
//             return;
//     }
//     ROS_DEBUG("Publish pointscloud");
//     rs2_stream texture_source_id = static_cast<rs2_stream>(_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
//     bool use_texture = texture_source_id != RS2_STREAM_ANY;
//     static int warn_count(0);
//     static const int DISPLAY_WARN_NUMBER(5);
//     rs2::frameset::iterator texture_frame_itr = frameset.end();
//     if (use_texture)
//     {
//         std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };
        
//         texture_frame_itr = find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f) 
//                                 {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
//                                             (available_formats.find(f.get_profile().format()) != available_formats.end()); });
//         if (texture_frame_itr == frameset.end())
//         {
//             warn_count++;
//             std::string texture_source_name = _filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
//             ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
//             return;
//         }
//         warn_count = 0;
//     }

//     int texture_width(0), texture_height(0);
//     int num_colors(0);

//     const rs2::vertex* vertex = pc.get_vertices();
//     const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();

//     _valid_pc_indices.clear();
//     for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
//     {
//         if (static_cast<float>(vertex->z) > 0)
//         {
//             float i = static_cast<float>(color_point->u);
//             float j = static_cast<float>(color_point->v);
//             if (_allow_no_texture_points || (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f))
//             {
//                 _valid_pc_indices.push_back(point_idx);
//             }
//         }
//     }

//     _msg_pointcloud.header.stamp = t;
//     _msg_pointcloud.header.frame_id = frame_id;
//     _msg_pointcloud.width = _valid_pc_indices.size();
//     _msg_pointcloud.height = 1;
//     _msg_pointcloud.is_dense = true;

//     sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
//     modifier.setPointCloud2FieldsByString(1, "xyz");    

//     vertex = pc.get_vertices();
//     if (use_texture)
//     {
//         rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
//         texture_width = texture_frame.get_width();
//         texture_height = texture_frame.get_height();
//         num_colors = texture_frame.get_bytes_per_pixel();
//         uint8_t* color_data = (uint8_t*)texture_frame.get_data();
//         std::string format_str;
//         switch(texture_frame.get_profile().format())
//         {
//             case RS2_FORMAT_RGB8:
//                 format_str = "rgb";
//                 break;
//             case RS2_FORMAT_Y8:
//                 format_str = "intensity";
//                 break;
//             default:
//                 throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
//         }
//         _msg_pointcloud.point_step = addPointField(_msg_pointcloud, format_str.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32, _msg_pointcloud.point_step);
//         _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
//         _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

//         sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
//         sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
//         sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
//         sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(_msg_pointcloud, format_str);
//         color_point = pc.get_texture_coordinates();

//         float color_pixel[2];
//         unsigned int prev_idx(0);
//         for (auto idx=_valid_pc_indices.begin(); idx != _valid_pc_indices.end(); idx++)
//         {
//             unsigned int idx_jump(*idx-prev_idx);
//             prev_idx = *idx;
//             vertex+=idx_jump;
//             color_point+=idx_jump;

//             *iter_x = vertex->x;
//             *iter_y = vertex->y;
//             *iter_z = vertex->z;

//             float i(color_point->u);
//             float j(color_point->v);
//             if (i >= 0.f && i <= 1.f && j >= 0.f && j <= 1.f)
//             {
//                 color_pixel[0] = i * texture_width;
//                 color_pixel[1] = j * texture_height;
//                 int pixx = static_cast<int>(color_pixel[0]);
//                 int pixy = static_cast<int>(color_pixel[1]);
//                 int offset = (pixy * texture_width + pixx) * num_colors;
//                 reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
//             }

//             ++iter_x; ++iter_y; ++iter_z;
//             ++iter_color;
//         }
//     }
//     else
//     {
//         sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
//         sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
//         sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
//         unsigned int prev_idx(0);
//         for (auto idx=_valid_pc_indices.begin(); idx != _valid_pc_indices.end(); idx++)
//         {
//             unsigned int idx_jump(*idx-prev_idx);
//             prev_idx = *idx;
//             vertex+=idx_jump;

//             *iter_x = vertex->x;
//             *iter_y = vertex->y;
//             *iter_z = vertex->z;

//             ++iter_x; ++iter_y; ++iter_z;
//         }
//     }
//     {
//         std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
//         if (_pointcloud_publisher)
//             _pointcloud_publisher->publish(_msg_pointcloud);
//     }
// }

