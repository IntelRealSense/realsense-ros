#include <dynamic_params.h>

namespace realsense2_camera
{
    Parameters::Parameters(rclcpp::Node& node) :
    _node(node),
    _logger(node.get_logger()),
    _params_backend(node)
    {
        _params_backend.add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) 
                { 
                    for (const auto & parameter : parameters) 
                    {
                        if (_param_functions.find(parameter.get_name()) != _param_functions.end())
                        {
                            auto functions = _param_functions[parameter.get_name()];
                            if (functions.empty())
                            {
                                ROS_WARN_STREAM("Parameter " << parameter.get_name() << " can not be changed in runtime.");
                            }
                            else
                            {
                                for (auto func : _param_functions[parameter.get_name()])
                                {
                                    func(parameter);
                                }
                            }
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
    }

    rclcpp::ParameterValue Parameters::setParam(std::string param_name, rclcpp::ParameterValue initial_value, 
                              std::function<void(const rclcpp::Parameter&)> func, 
                              rcl_interfaces::msg::ParameterDescriptor descriptor)
    {
        rclcpp::ParameterValue result_value(initial_value);
        try
        {
            if (!_node.has_parameter(param_name))
            {
                result_value = _node.declare_parameter(param_name, initial_value, descriptor);
            }
            else
            {
                result_value = _node.get_parameter(param_name).get_parameter_value();
            }
        }
        catch(const std::exception& e)
        {
            std::stringstream range;
            for (auto val : descriptor.floating_point_range)
            {
                range << val.from_value << ", " << val.to_value;
            }
            for (auto val : descriptor.integer_range)
            {
                range << val.from_value << ", " << val.to_value;
            }
            ROS_WARN_STREAM("Could not set param: " << param_name << " with " << 
                             rclcpp::Parameter(param_name, initial_value).value_to_string() << 
                            "Range: [" << range.str() << "]" <<
                             ": " << e.what());
            return initial_value;
        }
        
        if (func)
        {
            _param_functions[param_name].push_back(func);
        }
        else
        {
            _param_functions[param_name] = std::vector<std::function<void(const rclcpp::Parameter&)> >();
        }
        if (result_value != initial_value && func)
        {
            func(rclcpp::Parameter(param_name, result_value));
        }
        return result_value;
    }


    template <class T>
    void Parameters::setParamT(std::string param_name, rclcpp::ParameterValue initial_value, 
                              T& param, 
                              std::function<void(const rclcpp::Parameter&)> func,
                              rcl_interfaces::msg::ParameterDescriptor descriptor)
    {
        // NOTICE: callback function is set AFTER the parameter is declared!!!
        if (!_node.has_parameter(param_name))
            param = _node.declare_parameter(param_name, initial_value, descriptor).get<T>();
        else
        {
            param = _node.get_parameter(param_name).get_parameter_value().get<T>();
        }        
        _param_functions[param_name].push_back([&param, func](const rclcpp::Parameter& parameter)
            {
                param = parameter.get_value<T>();
            });
        if (func) 
        {
            _param_functions[param_name].push_back(func);
        }
        _param_names[&param] = param_name;
    }

    template <class T>
    void Parameters::setParamValue(T& param, const T& value)
    {
        param = value;
        try
        {
            std::string param_name = _param_names.at(&param);
            
            rcl_interfaces::msg::SetParametersResult results = _node.set_parameter(rclcpp::Parameter(param_name, value));
            if (!results.successful)
            {
                ROS_WARN_STREAM("Parameter: " << param_name << " was not set:" << results.reason);
            }
        }
        catch(const std::out_of_range& e)
        {
            ROS_WARN_STREAM("Parameter was not internally declared.");
        }
        catch(const rclcpp::exceptions::ParameterNotDeclaredException& e)
        {
            std::string param_name = _param_names.at(&param);
            ROS_WARN_STREAM("Parameter: " << param_name << " was not declared:" << e.what());
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
        }                            
    }

    void Parameters::removeParam(std::string param_name)
    {
        _node.undeclare_parameter(param_name);
        _param_functions.erase(param_name);
    }

    template void Parameters::setParamT<bool>(std::string param_name, rclcpp::ParameterValue initial_value, bool& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template void Parameters::setParamT<int>(std::string param_name, rclcpp::ParameterValue initial_value, int& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template void Parameters::setParamT<double>(std::string param_name, rclcpp::ParameterValue initial_value, double& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);

    template void Parameters::setParamValue<int>(int& param, const int& value);
    template void Parameters::setParamValue<bool>(bool& param, const bool& value);
    template void Parameters::setParamValue<double>(double& param, const double& value);
}