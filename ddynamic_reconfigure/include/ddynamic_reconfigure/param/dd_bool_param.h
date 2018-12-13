//
// Created by Noam Dori on 19/06/18.
//

#ifndef DDYNAMIC_RECONFIGURE_DD_BOOL_PARAM_H
#define DDYNAMIC_RECONFIGURE_DD_BOOL_PARAM_H

#include "ddynamic_reconfigure/dd_param.h"

namespace ddynamic_reconfigure {
    /**
     * @brief a boolean implementation of the parameter.
     * These are used to handle true/false values, or bit quantities if needed.
     * In ROS, booleans are handled as u-bytes (u-int8), so be careful with these!
     */
    class DDBool : virtual public DDParam {
    public:
        string getName() const;

        void prepGroup(Group &group);

        void prepConfig(Config &conf);

        void prepConfigDescription(ConfigDescription &conf_desc);

        int getLevel() const;

        bool sameType(Value val);

        bool sameValue(Value val);

        void setValue(Value val);

        Value getValue() const;

        /**
         * @brief creates a new bool param
         * @param name the name of the parameter
         * @param level the change level
         * @param description details about the parameter
         * @param def the default value
         */
        DDBool(const string &name, unsigned int level, const string &description, bool def) {
            name_ = name;
            level_ = level;
            desc_ = description;
            def_ = def;
            val_ = def;
        }
    protected:
        /**
         * @brief the level of the parameter:
         * the degree in which things need to be shut down if this param changes
         */
        unsigned int level_;
        /**
         * @brief the default value (def_),
         * and the current value (val_)
         */
        bool def_,val_;
        /**
         * @brief the name of the parameter (name_),
         * and its description (desc_)
         */
        string name_, desc_;
    };
}


#endif //DDYNAMIC_RECONFIGURE_DD_BOOL_PARAM_H
