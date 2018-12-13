#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <ddynamic_reconfigure/param/dd_all_params.h>
#include <ddynamic_reconfigure/TutorialParams.h>

/**
  Topics:
  * /dd_server/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
  * /dd_server/parameter_updates [dynamic_reconfigure/Config]

  Services:
  * /dd_server/set_parameters:  dynamic_reconfigure/Reconfigure
*/
using namespace ddynamic_reconfigure;
using namespace ros;
using namespace ddynamic_reconfigure;

bool paramService(TutorialParams::Request& req, TutorialParams::Response& res, DDynamicReconfigure& dd) {
    res.int_param = dd.get("int_param").toInt();
    res.double_param = dd.get("double_param").toDouble();
    res.str_param = dd.get("str_param").toString();
    res.enum_param = dd.get("enum_param").toInt();
    return true;
}

void callback(const DDMap& map, int) {
    ROS_INFO("Reconfigure Request: %d %f %s %s %ld",
            get(map, "int_param").toInt(),
            get(map, "double_param").toDouble(),
            get(map, "str_param").toString().c_str(),
            get(map, "bool_param").toBool() ? "True" : "False",
            map.size());
}

int main(int argc, char **argv) {
    // ROS init stage
    init(argc, argv, "dd_server");
    NodeHandle nh;

    // DDynamic setup stage
    DDynamicReconfigure dd(nh);
    dd.add(new DDInt("int_param", 0, "An Integer parameter", 0, 50, 100));
    dd.add(new DDDouble("double_param", 0, "A double parameter", .5, 0, 1));
    dd.add(new DDString("str_param", 0, "A string parameter", "Hello World"));
    dd.add(new DDBool("bool_param", 0, "A Boolean parameter", true));
    map<string, int> dict;
        dict["Small"] = 0;
        dict["Medium"] = 1;
        dict["Large"] = 2;
        dict["ExtraLarge"] = 3;
    dd.add(new DDEnum("enum_param", 0, "A size parameter which is edited via an enum", 1, dict));
    dd.start(callback);

    // Actual Server Node code
    ROS_INFO("Spinning node");
    function<bool(TutorialParams::Request &, TutorialParams::Response &)> f = bind(paramService, _1, _2, dd);
    ServiceServer checkParam = nh.advertiseService("get_params", f);
    MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}

