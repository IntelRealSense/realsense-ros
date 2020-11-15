#include <dynamic_params.h>

namespace realsense2_camera
{
    Parameters::Parameters(rclcpp::Node& node) :
    _node(node),
    _logger(rclcpp::get_logger("RealSenseCameraNode"))
    {
        _ros_callback = _node.add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) 
                { 
                    for (const auto & parameter : parameters) 
                    {
                        try
                        {
                            (_param_functions.at(parameter.get_name()))(parameter);
                        }
                        catch(const std::out_of_range& e)
                        {}
                        catch(const std::exception& e)
                        {
                            std::cerr << e.what() << '\n';
                        }                            
                    }
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    return result;
                });
    }

    rclcpp::ParameterValue Parameters::setParam(std::string param_name, rclcpp::ParameterValue initial_value, 
                              std::function<void(const rclcpp::Parameter&)> func, 
                              rcl_interfaces::msg::ParameterDescriptor descriptor)
    {
        rclcpp::ParameterValue result_value(initial_value);
        if (!_node.has_parameter(param_name))
            result_value = _node.declare_parameter(param_name, initial_value, descriptor);
        else
        {
            result_value = _node.get_parameter(param_name).get_parameter_value();
        }        
        _param_functions[param_name] = func;
        return result_value;
    }

    void Parameters::removeParam(std::string param_name)
    {
        _param_functions.erase(param_name);
    }
}