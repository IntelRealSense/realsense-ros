//
// Created by Noam Dori on 20/06/18.
//

#ifndef DDYNAMIC_RECONFIGURE_DD_ENUM_PARAM_H
#define DDYNAMIC_RECONFIGURE_DD_ENUM_PARAM_H

#include "dd_int_param.h"
#include <boost/foreach.hpp>

namespace ddynamic_reconfigure {
    typedef map<string,pair<int,string> > EnumMap;
    /**
     * @brief an integer enum implementation of the parameter.
     *        This is an extension to the int parameter,
     *        which allows creating string aliases for certain (if not all) numbers available.
     *
     */
    class DDEnum : virtual public DDInt {
    public:

        void prepGroup(Group &group);

        bool sameType(Value val);

        bool sameValue(Value val);

        void setValue(Value val);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value in integer form
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         */
        DDEnum(const string &name, unsigned int level, const string &description,
                int def, const map<string,int> &dictionary);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def an alias of the default value
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         */
        DDEnum(const string &name, unsigned int level, const string &description,
                const string& def, const map<string,int> &dictionary);

        #ifdef __clang__
        #pragma clang diagnostic push
        #pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
        #endif
        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def the default value in integer form
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         * @note since ROS cannot send the enum and const descriptions, this method is useless.
         *       Please use the constructor which takes a map<string,int> instead.
         * @deprecated see note. This is not tested, so it may fail.
         */
        DDEnum(const string &name, unsigned int level, const string &description,
               int def, const pair<map<string,pair<int,string> >,string> &dictionary);

        /**
         * @brief creates a new int-enum param
         * @param name the name of the parameter
         * @param level the change level
         * @param def an alias of the default value
         * @param description details about the parameter
         * @param dictionary the alias dictionary this enum will use.
         * @note since ROS cannot send the enum and const descriptions, this method is useless.
         *       Please use the constructor which takes a map<string,int> instead.
         * @deprecated see note. This is not tested, so it may fail.
         */
        DDEnum(const string &name, unsigned int level, const string &description,
               const string& def, const pair<map<string,pair<int,string> >,string>  &dictionary);
        #ifdef __clang__
        #pragma clang diagnostic pop
        #endif

    protected:
        /** 
         * @brief A dictionary from the string aliases to their integer counterparts.
         * This method of storage allows integers to have multiple aliases.
         */
        const EnumMap dict_;
        /**
         * @brief this holds the physical enum's description. why is this here? because 1D-reconfigure.
         */
        string enum_description_;
    private:
        /**
         * converts the value given to an integer according to the embedded dictionary.
         * @param val the value to look up within the dictionary
         * @return if the value is a string which exists in the dictionary, returns the int definition of the term given.
         *         otherwise, returns the Value object defined conversion of the type to an integer.
         */
        int lookup(Value val);

        /**
         * generates the 'edit_method' sting for prepGroup().
         * @return a string that should not be touched.
         */
        string makeEditMethod();

        /**
         * generates a 'const' sting for prepGroup().
         * @param name the name of the constant
         * @param value the value of the constant
         * @param desc the description given to the constant.
         * @return a string that should not be touched.
         */
        string makeConst(string name, int value, string desc);
    };
}

#endif //DDYNAMIC_RECONFIGURE_DD_ENUM_PARAM_H
