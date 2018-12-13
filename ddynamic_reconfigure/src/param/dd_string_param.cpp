//
// Created by Noam Dori on 19/06/18.
//

#include <ddynamic_reconfigure/param/dd_string_param.h>

namespace ddynamic_reconfigure {
    string DDString::getName() const {
        return name_;
    }

    void DDString::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "string";
        group.parameters.push_back(desc);
    }

    void DDString::prepConfig(Config &conf) {
        StrParameter param;
        param.name = name_;
        param.value = val_;
        conf.strs.push_back(param);
    }

    void DDString::prepConfigDescription(ConfigDescription &conf_desc) {
        StrParameter param;
        param.name = name_;
        param.value = def_;
        conf_desc.dflt.strs.push_back(param);
        param.value = "";
        conf_desc.max.strs.push_back(param);
        param.value = "";
        conf_desc.min.strs.push_back(param);
    }

    int DDString::getLevel() const {
        return level_;
    }

    bool DDString::sameType(Value val) {
        return val.getType() == "string";
    }

    bool DDString::sameValue(Value val) {
        return val.toString() == val_;
    }

    void DDString::setValue(Value val) {
        val_ = val.toString();
    }

    Value DDString::getValue() const {
        return Value(val_);
    }
}
