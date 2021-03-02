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

    Parameters::~Parameters()
    {
        for (auto const& param : _param_functions)
        {
            _node.undeclare_parameter(param.first);
        }
        if (_ros_callback)
        {
            _node.remove_on_set_parameters_callback(_ros_callback.get());
            _ros_callback.reset();
        }
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
        if (func)
            _param_functions[param_name] = func;
        else
            _param_functions[param_name] = [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("Parameter can not be changed in runtime.");
            };
        return result_value;
    }


    template <class T>
    void Parameters::setParamT(std::string param_name, rclcpp::ParameterValue initial_value, 
                              T& param, 
                              rcl_interfaces::msg::ParameterDescriptor descriptor)
    {
        if (!_node.has_parameter(param_name))
            param = _node.declare_parameter(param_name, initial_value, descriptor).get<T>();
        else
        {
            param = _node.get_parameter(param_name).get_parameter_value().get<T>();
        }        
        _param_functions[param_name] = [&param](const rclcpp::Parameter& parameter)
            {
                std::cout << "parameter: " << parameter.get_name() << " = " << parameter.get_value<T>() << std::endl;
                param = parameter.get_value<T>();
            };
    }

    void Parameters::removeParam(std::string param_name)
    {
        _node.undeclare_parameter(param_name);
        _param_functions.erase(param_name);
    }

    template void Parameters::setParamT<bool>(std::string param_name, rclcpp::ParameterValue initial_value, bool& param, rcl_interfaces::msg::ParameterDescriptor descriptor);    
}