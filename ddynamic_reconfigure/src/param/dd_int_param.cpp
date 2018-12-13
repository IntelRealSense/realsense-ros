//
// Created by Noam Dori on 19/06/18.
//

#include <ddynamic_reconfigure/param/dd_int_param.h>

namespace ddynamic_reconfigure {
    string DDInt::getName() const {
        return name_;
    }

    void DDInt::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "int";
        group.parameters.push_back(desc);
    }

    void DDInt::prepConfig(Config &conf) {
        IntParameter param;
        param.name = name_;
        param.value = val_;
        conf.ints.push_back(param);
    }

    void DDInt::prepConfigDescription(ConfigDescription &conf_desc) {
        IntParameter param;
        param.name = name_;
        param.value = def_;
        conf_desc.dflt.ints.push_back(param);
        param.value = max_;
        conf_desc.max.ints.push_back(param);
        param.value = min_;
        conf_desc.min.ints.push_back(param);
    }

    int DDInt::getLevel() const {
        return level_;
    }

    bool DDInt::sameType(Value val) {
        return val.getType() == "int";
    }

    bool DDInt::sameValue(Value val) {
        return val.toInt() == val_;
    }

    void DDInt::setValue(Value val) {
        val_ = val.toInt();
    }

    Value DDInt::getValue() const {
        return Value(val_);
    }
}
