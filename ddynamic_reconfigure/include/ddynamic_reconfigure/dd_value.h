//
// Created by Noam Dori on 19/06/18.
//
#include <string>
#include <sstream>
#include <cstdio>
#include <boost/functional/hash.hpp>

#ifndef DDYNAMIC_RECONFIGURE_DD_VALUE_H
#define DDYNAMIC_RECONFIGURE_DD_VALUE_H
using namespace std;
using namespace boost;
namespace ddynamic_reconfigure {
    /**
     * @brief The Value class is used to wrap all basic data-types (bool,int,double,string) in something generic.
     *        The value object always stores an explicit basic data-type.
     *        This has three main uses:
     *
     *        1. Values can represent all basic data-types.
     *        This means that arguments that need something relatively similar from all basic data-types can now just use the value in its argument.
     *        This also goes for when you need to return something that is of different data-types from different classes
     *        (one can only return integer, other can only return strings).
     *
     *        2. Values can be explicitly converted to all basic data-types they wrap.
     *        This means that converting an int to a string is far easier.
     *
     *        3. Values store the type they were instantiated with. This can be tested against to get the original piece of data the value stored.
     */
    class Value {
    private:
        int int_val_;
        string str_val_, type_;
        double double_val_;
        bool bool_val_;
    public:
        /**
         * @brief creates an integer value wrapper
         * @param val the int to wrap
         */
        inline explicit Value(int val) : double_val_(0.0), bool_val_(false) {
            type_ = "int";
            int_val_ = val;
        }

        /**
         * @brief creates an double value wrapper
         * @param val the double to wrap
         */
        inline explicit Value(double val) : int_val_(0), bool_val_(false) {
            type_ = "double";
            double_val_ = val;
        }

        /**
         * @brief creates a string value wrapper
         * @param val the string to wrap
         */
        inline explicit Value(const string &val) : int_val_(0), double_val_(0.0), bool_val_(false) {
            type_ = "string";
            str_val_ = val;
        }

        /**
         * @brief creates a c-string value wrapper, though it is considered a regular string.
         * @param val the c-string to wrap
         */
        inline explicit Value(const char* val) : int_val_(0), double_val_(0.0), bool_val_(false) {
            *this = Value(string(val));
        }

        /**
         * @brief creates an integer value wrapper
         * @param val the boolean to wrap
         */
        inline explicit Value(bool val) : int_val_(0), double_val_(0.0) {
            type_ = "bool";
            bool_val_ = val;
        }

        /**
         * @brief gets the type this value wrapper stores
         * @return a string containing the type: one of {"int","double","bool","string"}
         */
        inline string getType() const {
            return type_;
        }

        /**
         * @brief converts the stored value into an integer.
         * @return for native integers: returns itself.
         *         for native doubles: returns a casted integer.
         *         for native booleans: returns 1 if true, 0 if false.
         *         for native strings: if the string contains an integer value (for example "1" contains the int '1' in it),
         *                             it will return the integer interpretation of that string.
         *                             Otherwise, returns the hash value of the string.
         */
        inline int toInt() const {
            if (type_ == "string") {
                int i;
                if (sscanf(str_val_.c_str(), "%d", &i) == 0) {
                    return (int) boost::hash<string>()(str_val_);
                }
                return i;
            } else if (type_ == "bool") { return bool_val_ ? 1 : 0; }
            else if (type_ == "double") { return (int) double_val_; }
            else { return int_val_; }
        }

        /**
         * @brief converts the stored value into a string.
         * @return for native integers: returns the number in string form ('1' -> "1")
         *         for native doubles: returns the number in shorthand string form ('1.0' -> "1")
         *         for native booleans: returns "true" if true, "false" if false.
         *         for native strings: returns itself.
         */
        inline string toString() const {
            stringstream ss;
            if(type_ == "string") { return str_val_;}
            else if(type_ == "bool") {return bool_val_ ? "true" : "false";}
            else if(type_ == "double") {ss << double_val_; return ss.str();}
            else {ss << int_val_; return ss.str();}
        }

        /**
         * @brief converts the stored value into a double.
         * @return for native integers: returns itself
         *         for native doubles: returns a itself
         *         for native booleans: returns 1.0 if true, 0.0 if false.
         *         for native strings: if the string contains a floating value (for example "1.1" contains the double '1.1' in it),
         *                             it will return the double interpretation of that string.
         *                             Otherwise, returns the hash value of the string.
         */
        inline double toDouble() const {
            if(type_ == "string") {
                double f;
                if(sscanf(str_val_.c_str(), "%lf", &f) == 0) {
                    return boost::hash<string>()(str_val_);
                }
                return f;
            } else if(type_ == "bool") {return bool_val_ ? 1.0 : 0.0;}
            else if(type_ == "double") {return double_val_;}
            else {return int_val_;}
        }

        /**
         * @brief converts the stored value into a boolean.
         * @return for native integers: returns true if the value is bigger than 0, false otherwise.
         *         for native doubles: returns true if the value is bigger than 0, false otherwise.
         *         for native booleans: returns itself
         *         for native strings: returns true if the string's value is 'true', false otherwise.
         */
        inline bool toBool() const {
            if(type_ == "string") { return str_val_ == "true";}
            else if(type_ == "bool") {return bool_val_;}
            else if(type_ == "double") {return double_val_ > 0.0;}
            else {return int_val_ > 0;}
        }
    };
}

#endif //DDYNAMIC_RECONFIGURE_DD_VALUE_H
