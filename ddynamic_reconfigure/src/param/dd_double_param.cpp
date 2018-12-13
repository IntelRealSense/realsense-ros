//
// Created by Noam Dori on 19/06/18.
//

#include <ddynamic_reconfigure/param/dd_double_param.h>

namespace ddynamic_reconfigure {
    string DDDouble::getName() const {
        return name_;
    }

    void DDDouble::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "double";
        group.parameters.push_back(desc);
    }

    void DDDouble::prepConfig(Config &conf) {
        DoubleParameter param;
        param.name = name_;
        param.value = val_;
        conf.doubles.push_back(param);
    }

    void DDDouble::prepConfigDescription(ConfigDescription &conf_desc) {
        DoubleParameter param;
        param.name = name_;
        param.value = def_;
        conf_desc.dflt.doubles.push_back(param);
        param.value = max_;
        conf_desc.max.doubles.push_back(param);
        param.value = min_;
        conf_desc.min.doubles.push_back(param);
    }

    int DDDouble::getLevel() const {
        return level_;
    }

    bool DDDouble::sameType(Value val) {
        return val.getType() == "double";
    }

    bool DDDouble::sameValue(Value val) {
        return val.toDouble() == val_;
    }

    void DDDouble::setValue(Value val) {
        val_ = val.toDouble();
    }

    Value DDDouble::getValue() const {
        return Value(val_);
    }
}
