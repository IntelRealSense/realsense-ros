/**
 * Copyright 2019 PAL Robotics S.L.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef REGISTERED_PARAM_H
#define REGISTERED_PARAM_H

#include <string>
#include <map>
#include <sstream>
#include <type_traits>
#include <dynamic_reconfigure/ParamDescription.h>
namespace ddynamic_reconfigure {
   
template <typename T>
class RegisteredParam
{
public:
  RegisteredParam(const std::string &name, const std::string &description, T min_value,
                  T max_value, std::map<std::string, T> enum_dictionary = {},
                  const std::string &enum_description = "")
    : name_(name)
    , description_(description)
    , min_value_(min_value)
    , max_value_(max_value)
    , enum_dictionary_(enum_dictionary)
    , enum_description_(enum_description)
  {
  }
  
  virtual ~RegisteredParam()
  {}

  virtual T getCurrentValue() const = 0;
  virtual void updateValue(T new_value) = 0;
  
  std::string getTypeName() const
  {
   if (std::is_same<T, int>::value)
   {
    return "int";
   }
   else if (std::is_same<T, double>::value)
   {
    return "double";
   }
   else if (std::is_same<T, bool>::value)
   {
    return "bool";
   }
   else if (std::is_same<T, std::string>::value)
   {
    return "str";
   }
   throw std::runtime_error("Unexpected type for param " + name_);
  }
  
  std::string getValueString(T value) const
  {
   std::stringstream ss;
   ss << value;
   
   if (std::is_same<T, std::string>::value)
   {
    return "'" + ss.str() + "'";
   }
   return ss.str();
  }
  
  virtual dynamic_reconfigure::ParamDescription getParamDescription() const 
  {
    dynamic_reconfigure::ParamDescription p;
    p.name = name_;
    p.description = description_;
    p.level = 0;
    p.type = getTypeName();
    if (!enum_dictionary_.empty())
    {
      p.edit_method = getEditMethod();
    }
    return p;
  }

  std::string getEditMethod() const
  {
    // Based on https://github.com/awesomebytes/ddynamic_reconfigure's implementation
    std::stringstream ret;
    ret << "{";
    {
      ret << "'enum_description': '" << enum_description_ << "', ";
      ret << "'enum': [";
      {
        auto it = enum_dictionary_.cbegin();
        ret << makeConst(it->first, it->second, "");
        for (it++; it != enum_dictionary_.cend(); it++)
        {
          ret << ", " << makeConst(it->first, it->second, "");
        };
      }
      ret << "]";
    }
    ret << "}";
    return ret.str();
  }

  std::string makeConst(const std::string &name, T value, const std::string &desc) const
  {
    std::stringstream ret;
    ret << "{";
    {
      ret << "'srcline': 0, ";  // the sole reason this is here is because dynamic placed
                                // it in its enum JSON.
      ret << "'description': '" << desc << "', ";
      ret << "'srcfile': '/does/this/really/matter.cfg', ";  // the answer is no. This is
                                                             // useless.
      ret << "'cconsttype': 'const " << getTypeName() << "', ";
      ret << "'value': " << getValueString(value) << ", ";
      ret << "'ctype': '" << getTypeName() << "', ";
      ret << "'type': '" << getTypeName() << "', ";
      ret << "'name': '" << name << "'";
    }
    ret << "}";
    return ret.str();
  }

  const std::string name_;
  const std::string description_;
  const T min_value_;
  const T max_value_;
  const std::map<std::string, T> enum_dictionary_;
  const std::string enum_description_;
};


template <typename T>
class PointerRegisteredParam : public RegisteredParam<T>
{
public:
  PointerRegisteredParam(const std::string &name, const std::string &description,
                         T min_value, T max_value, T *variable, 
                         std::map<std::string, T> enum_dictionary = {},
                         const std::string &enum_description = "")
    : RegisteredParam<T>(name, description, min_value, max_value, enum_dictionary, enum_description)
    , variable_(variable)
  {
  }

  T getCurrentValue() const override
  {
    return *variable_;
  }
  void updateValue(T new_value) override
  {
    *variable_ = new_value;
  }

protected:
  T *variable_;
};

template <typename T>
class CallbackRegisteredParam : public RegisteredParam<T>
{
public:
  CallbackRegisteredParam(const std::string &name, const std::string &description, T min_value,
                          T max_value, T current_value, boost::function<void(T value)> callback,
                          std::map<std::string, T> enum_dictionary = {}, 
                          const std::string &enum_description = "")
    : RegisteredParam<T>(name, description, min_value, max_value, enum_dictionary, enum_description)
    , current_value_(current_value)
    , callback_(callback)
  {
  }

  T getCurrentValue() const override
  {
    return current_value_;
  }
  
  void updateValue(T new_value) override
  {
    callback_(new_value);
    current_value_ = new_value;
  }

protected:
  T current_value_;
  boost::function<void(T value)> callback_;
};


} // namespace ddynamic_reconfigure

#endif // REGISTERED_PARAM_H
