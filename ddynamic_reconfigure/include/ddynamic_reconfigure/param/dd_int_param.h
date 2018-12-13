//
// Created by Noam Dori on 19/06/18.
//

#ifndef DDYNAMIC_RECONFIGURE_DD_INT_PARAM_H
#define DDYNAMIC_RECONFIGURE_DD_INT_PARAM_H

#include "ddynamic_reconfigure/dd_param.h"

namespace ddynamic_reconfigure {
    /**
     * @brief an integer implementation of the parameter.
     * This is used to 32 bit signed integral numbers.
     * This can also handle shorts, bytes, and other integrals provided they are not too big
     * (by then looping will occur)
     */
    class DDInt : virtual public DDParam {
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
         * creates a new int param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value
         * @param description details about the parameter
         * @param max the maximum allowed value. Defaults to INT32_MAX
         * @param min the minimum allowed value. Defaults to INT32_MIN
         */
        inline DDInt(const string &name, unsigned int level, const string &description,
                int def, int min = INT32_MIN, int max = INT32_MAX) {
            name_ = name;
            level_ = level;
            desc_ = description;
            def_ = def;
            val_ = def;
            max_ = max;
            min_ = min;
        }

    protected:
        /**
         * @brief the level of the parameter:
         * the degree in which things need to be shut down if this param changes
         */
        unsigned int level_;
        /**
         * @brief the default value (def_),
         * the current value (val_),
         * the minimum allowed value (min_),
         * and the maximum allowed value (max_)
         */
        int def_,max_,min_,val_;
        /**
         * @brief the name of the parameter (name_),
         * and its description (desc_)
         */
        string name_, desc_;
    };
}

#endif //DDYNAMIC_RECONFIGURE_DD_INT_PARAM_H
