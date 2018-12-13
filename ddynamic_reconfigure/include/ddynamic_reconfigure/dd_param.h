//
// Created by Noam Dori on 18/06/18.
//

#ifndef DDYNAMIC_RECONFIGURE_DD_PARAM_H
#define DDYNAMIC_RECONFIGURE_DD_PARAM_H

#include <string>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include "dd_value.h"

using namespace dynamic_reconfigure;
using namespace std;
namespace ddynamic_reconfigure {
    class DDParam;// declaration for the sake of order.
    // this is the pointer to any type of Dynamic Dynamic parameter.
    typedef boost::shared_ptr<DDParam> DDPtr;
    /**
     * @brief The DDParam class is the abstraction of all parameter types, and is the template for creating them.
     *        At this point, not much is known about the parameter, but the following:
     *
     *        - the parameter has a name
     *        - the parameter has a severity level
     *        - the parameter has a description
     *        - the parameter contains some value, though its type and contents are unknown.
     *
     *        Other than storing data, the parameter also has specialised methods to interact with DDynamicReconfigure in order to apply changes and send them.
     *        These methods should not be touched by the user.
     *
     *        Since this class is abstract, the class has multiple implementations whicch are not directly exposed but are used,
     *        so its worth checking out their descriptions.
     *
     *        While this class is abstract, it does have one implemented thing, and that is its stream operator (`<<`) which can be freely used.
     *
     *        While DDParam is abstract, all of its concrete implementations should follow this guideline:
     *              DD<Type>(const string &name, unsigned int level, const string &description, <type> def, <extra-args>)
     *        Where:
     *        - <Type> is the type name you are implementing
     *        - name is the reference name
     *        - level is the severity level
     *        - description is the object's description
     *        - def is the default value and the first value stored right after construction.
     *
     *        You may then include extra arguments as you wish, required or optional.
     */
    class DDParam {
    public:

        /**
         * @brief gets the name of the parameter, that is, the ID used in the program when requesting it.
         * @return the unique string name of the parameter.
         */
        virtual string getName() const = 0;

        /**
         * @brief fetches the level of the parameter
         * @return the level of the param.
         */
        virtual int getLevel() const = 0;

        /**
         * @brief gets the value of this parameter.
         * @return the value stored in this param.
         */
        virtual Value getValue() const = 0;
        
        /**
         * @brief checks whether or not the raw value stored in the value is compatible with the given parameter.
         *        Compatible is a very broad word in this scenario.
         *        It means that the value can be placed in the parameter regardless of other limitations.
         * @param val the value to test
         * @return true is this parameter can handle the original value, false otherwise.
         */
        virtual bool sameType(Value val) = 0;
        
        /**
         * @brief checks whether or not the value stored in the value object,
         *        when converted to the type of the internal value, are equal. This acts regardless of type.
         * @param val the value to test
         * @return true is this parameter can is the same as the original value, false otherwise.
         */
        virtual bool sameValue(Value val) = 0;

        /**
         * @brief sets the value of this parameter as this one.
         * @param val the value to use
         */
        virtual void setValue(Value val) = 0;

        /**
         * @brief updates a group message according to this param's info.
         * @param group the group to update.
         * @note this is an internal method. It is recommended not to use it.
         */
        virtual void prepGroup(Group &group) = 0;

        /**
         * @brief updates a config message according to this param's info.
         * @param conf the group to update.
         * @note this is an internal method. It is recommended not to use it.
         */
        virtual void prepConfig(Config &conf) = 0;

        /**
         * @brief updates a config description message according to this param's info.
         * @param conf_desc the config description to update.
         * @note this is an internal method. It is recommended not to use it.
         */
        virtual void prepConfigDescription(ConfigDescription &conf_desc) = 0;

        /**
         * @brief the operator taking care of streaming the param values
         * @param os the stream to place the param into
         * @param param the param you want to place into the stream
         * @return os, but with param added.
         */
        friend ostream& operator<<(ostream& os, const DDParam &param);
    };
}
#endif //DDYNAMIC_RECONFIGURE_DD_PARAM_H
