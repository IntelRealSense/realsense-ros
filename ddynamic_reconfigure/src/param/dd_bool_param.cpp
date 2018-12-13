//
// Created by Noam Dori on 19/06/18.
//

#include <ddynamic_reconfigure/param/dd_bool_param.h>

namespace ddynamic_reconfigure {
    string DDBool::getName() const {
        return name_;
    }

    void DDBool::prepGroup(Group &group) {
        ParamDescription desc;
        desc.name  = name_;
        desc.level = level_;
        desc.description = desc_;
        desc.type = "bool";
        group.parameters.push_back(desc);
    }

    void DDBool::prepConfig(Config &conf) {
        BoolParameter param;
        param.name = name_;
        param.value = (unsigned char)val_;
        conf.bools.push_back(param);
    }

    void DDBool::prepConfigDescription(ConfigDescription &conf_desc) {
        BoolParameter param;
        param.name = name_;
        param.value = (unsigned char)def_;
        conf_desc.dflt.bools.push_back(param);
        param.value = (unsigned char)true;
        conf_desc.max.bools.push_back(param);
        param.value = (unsigned char)false;
        conf_desc.min.bools.push_back(param);
    }

    int DDBool::getLevel() const {
        return level_;
    }

    bool DDBool::sameType(Value val) {
        return val.getType() == "bool";
    }

    bool DDBool::sameValue(Value val) {
        return val.toBool() == val_;
    }

    void DDBool::setValue(Value val) {
        val_ = val.toBool();
    }

    Value DDBool::getValue() const {
        return Value(val_);
    }
}
