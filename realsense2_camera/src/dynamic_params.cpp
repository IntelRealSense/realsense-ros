#include <dynamic_params.h>

namespace realsense2_camera
{
    Parameters::Parameters(rclcpp::Node& node) :
    _node(node),
    _logger(rclcpp::get_logger("RealSenseCameraNode")),
    _params_backend(node),
    _is_running(true)
    {
        _params_backend.add_on_set_parameters_callback(
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
                            std::cerr << e.what() << ":" << parameter.get_name() << '\n';
                        }                            
                    }
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    return result;
                });
        monitor_update_functions(); // Start parameters update thread
    }

    // pushUpdateFunctions:
    // Cannot update ros parameters from within ros parameters callback.
    // This function is used by the parameter callback function to update other ros parameters.
    void Parameters::pushUpdateFunctions(std::vector<std::function<void()> > funcs)
    {
        _update_functions_v.insert(_update_functions_v.end(), funcs.begin(), funcs.end());
        _update_functions_cv.notify_one();
    }

    void Parameters::monitor_update_functions()
    {
        int time_interval(1000);
        std::function<void()> func = [this, time_interval](){
            std::mutex mu;
            std::unique_lock<std::mutex> lock(mu);
            while(_is_running) {
                _update_functions_cv.wait_for(lock, std::chrono::milliseconds(time_interval), [&]{return !_is_running || !_update_functions_v.empty();});
                while (!_update_functions_v.empty())
                {
                    _update_functions_v.front()();
                    _update_functions_v.pop_front();
                }
            }
        };
        _update_functions_t = std::make_shared<std::thread>(func);
    }

    Parameters::~Parameters()
    {
        _is_running = false;
        if (_update_functions_t && _update_functions_t->joinable())
            _update_functions_t->join();
        for (auto const& param : _param_functions)
        {
            _node.undeclare_parameter(param.first);
        }
        // remove_on_set_parameters_callback(_params_backend);
    }


    rclcpp::ParameterValue Parameters::readAndDeleteParam(std::string param_name, const rclcpp::ParameterValue& initial_value)
    {
        // Function is meant for reading parameters needed in initialization but should not be declared by the app.
        rclcpp::ParameterValue result_value(initial_value);
        if (!_node.has_parameter(param_name))
        {
            result_value = _node.declare_parameter(param_name, rclcpp::ParameterValue(initial_value));
            _node.undeclare_parameter(param_name);
        }
        else
        {
            result_value = _node.get_parameter(param_name).get_parameter_value();
        }
        return result_value;
    }

    rclcpp::ParameterValue Parameters::setParam(std::string param_name, const rclcpp::ParameterValue& initial_value, 
                              std::function<void(const rclcpp::Parameter&)> func, 
                              rcl_interfaces::msg::ParameterDescriptor descriptor)
    {
        rclcpp::ParameterValue result_value(initial_value);
        try
        {
            ROS_DEBUG_STREAM("setParam::Setting parameter: " << param_name);
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
                            " Range: [" << range.str() << "]" <<
                             ": " << e.what());
            return initial_value;
        }
        
        if (_param_functions.find(param_name) != _param_functions.end())
            ROS_DEBUG_STREAM("setParam::Replace function for : " << param_name);
        
        if (func)
            _param_functions[param_name] = func;
        else
            _param_functions[param_name] = [this](const rclcpp::Parameter& )
            {
                ROS_WARN_STREAM("Parameter can not be changed in runtime.");
            };
        if (result_value != initial_value && func)
        {
            func(rclcpp::Parameter(param_name, result_value));
        }
        return result_value;
    }


    template <class T>
    void Parameters::setParamT(std::string param_name, const rclcpp::ParameterValue& initial_value, 
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
        _param_functions[param_name] = [&param, func](const rclcpp::Parameter& parameter)
            {
                param = parameter.get_value<T>();
                if (func) func(parameter);
            };
        _param_names[&param] = param_name;
    }

    template <class T>
    void Parameters::setParamValue(T& param, const T& value)
    {
        // setParamValue updates a variable and its parallel in the parameters server.
        // NOTICE: <param> must have the same address it was declared with.
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
            ROS_ERROR_STREAM(__FILE__ << ":" << __LINE__ << ":" << e.what());
        }                            
    }

    void Parameters::setRosParamValue(const std::string param_name, void const* const value)
    {
        // setRosParamValue sets a value to a parameter in the parameters server.
        rclcpp::ParameterType param_type = _node.get_parameter(param_name).get_type();
        rcl_interfaces::msg::SetParametersResult results;
        switch(param_type)
        {
            case rclcpp::PARAMETER_BOOL:
                ROS_INFO_STREAM("Set " << param_name << " to " << *(bool*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(bool*)value));
                break;
            case rclcpp::PARAMETER_INTEGER:
                ROS_INFO_STREAM("Set " << param_name << " to " << *(int*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(int*)value));
                break;
            case rclcpp::PARAMETER_DOUBLE:
                ROS_INFO_STREAM("Set " << param_name << " to " << *(double*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(double*)value));
                break;
            case rclcpp::PARAMETER_STRING:
                ROS_INFO_STREAM("Set " << param_name << " to " << *(std::string*)value);
                results = _node.set_parameter(rclcpp::Parameter(param_name, *(std::string*)value));
                break;
            default:
                ROS_ERROR_STREAM("Setting parameter of type " <<  _node.get_parameter(param_name).get_type_name() << " is not implemented.");
        }
        if (!results.successful)
        {
            ROS_WARN_STREAM("Parameter: " << param_name << " was not set:" << results.reason);
        }
    }

    template <class T>
    void Parameters::queueSetRosValue(const std::string& param_name, const T value)
    {
        std::vector<std::function<void()> > funcs;
        funcs.push_back([this, param_name, value](){setRosParamValue(param_name, &value);});
        pushUpdateFunctions(funcs);
    }


    void Parameters::removeParam(std::string param_name)
    {
        if (_node.has_parameter(param_name))
        {
            _node.undeclare_parameter(param_name);
        }
        _param_functions.erase(param_name);
    }

    template void Parameters::setParamT<bool>(std::string param_name, const rclcpp::ParameterValue& initial_value, bool& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template void Parameters::setParamT<int>(std::string param_name, const rclcpp::ParameterValue& initial_value, int& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);
    template void Parameters::setParamT<double>(std::string param_name, const rclcpp::ParameterValue& initial_value, double& param, std::function<void(const rclcpp::Parameter&)> func, rcl_interfaces::msg::ParameterDescriptor descriptor);

    template void Parameters::setParamValue<int>(int& param, const int& value);
    template void Parameters::setParamValue<bool>(bool& param, const bool& value);
    template void Parameters::setParamValue<double>(double& param, const double& value);

    template void Parameters::queueSetRosValue<std::string>(const std::string& param_name, const std::string value);
}