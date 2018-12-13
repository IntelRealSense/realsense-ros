//
// Created by Noam Dori on 01/07/18.
//
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ddynamic_reconfigure/dd_value.h>
#include <ddynamic_reconfigure/TutorialParams.h>
#include <dynamic_reconfigure/Reconfigure.h>

using namespace ros;
namespace ddynamic_reconfigure {

    /**
     * @brief A ROS client making sure the server sends the new information.
     */
    TEST(DDFullScaleTest, doTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // ROS init stage
        NodeHandle nh;

        ServiceClient dynamics = nh.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters");
        int new_int = 2, new_enum = 2;
        double new_double = 0.2;
        string new_string = "2";
        bool new_bool = false;

        dynamic_reconfigure::Reconfigure srv;
        
        dynamic_reconfigure::IntParameter int_param;
        int_param.name = "int_param";
        int_param.value = new_int;
        srv.request.config.ints.push_back(int_param);

        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = "double_param";
        double_param.value = new_double;
        srv.request.config.doubles.push_back(double_param);

        dynamic_reconfigure::BoolParameter bool_param;
        bool_param.name = "bool_param";
        bool_param.value = (unsigned char)new_bool;
        srv.request.config.bools.push_back(bool_param);

        dynamic_reconfigure::StrParameter str_param;
        str_param.name = "str_param";
        str_param.value = new_string;
        srv.request.config.strs.push_back(str_param);

        dynamic_reconfigure::IntParameter enum_param;
        enum_param.name = "enum_param";
        enum_param.value = new_enum;
        srv.request.config.ints.push_back(enum_param);

        ASSERT_TRUE(dynamics.call(srv));

        ServiceClient client = nh.serviceClient<ddynamic_reconfigure::TutorialParams>("get_params");
        ddynamic_reconfigure::TutorialParams params;
        ASSERT_TRUE(client.call(params));

        ASSERT_EQ(new_int,params.response.int_param);
        ASSERT_EQ(new_double,params.response.double_param);
        ASSERT_EQ(new_string,params.response.str_param);
        ASSERT_EQ(new_bool,params.response.bool_param);
        ASSERT_EQ(new_enum,params.response.enum_param);
    }
}


int main(int argc, char** argv) {
    init(argc, argv, "dd_client");
    testing::InitGoogleTest(&argc, argv);

    srand((unsigned int)random());

    return RUN_ALL_TESTS();
}