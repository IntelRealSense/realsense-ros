#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <gmock/gmock.h>
#include <dynamic_reconfigure/Reconfigure.h>

using ::testing::_;
using ::testing::Mock;
using ::testing::Exactly;

using namespace ddynamic_reconfigure;

namespace  pal
{
class MockClass
{
public:
  MockClass()
    : double_param_(0.0), int_param_(0), bool_param_(false)
  {}
  MOCK_METHOD0(userCallback,
               void());
  
  MOCK_METHOD1(strCallback,
               void(std::string));

  MOCK_METHOD1(doubleCallback,
               void(double));
  
  MOCK_METHOD1(intCallback,
               void(int));
  
  MOCK_METHOD1(boolCallback,
               void(bool));
  std::string str_param_;
  double double_param_;
  int int_param_;
  bool bool_param_;
};


class DDynamicReconfigureTest : public ::testing::Test
{
public:
  void cfgCb(const dynamic_reconfigure::ConfigConstPtr& msg)
  {
    cfg_msg_ = msg;
  }

  void waitForCfg()
  {
    cfg_msg_.reset();
    while (!cfg_msg_.get() && ros::ok())
    {
    }
  }

  dynamic_reconfigure::ConfigConstPtr cfg_msg_;
};

TEST_F(DDynamicReconfigureTest, basicTest)
{
  ros::NodeHandle nh("~");
  DDynamicReconfigure dd(nh);
  MockClass mock;
  mock.int_param_ = 0;
  dd.RegisterVariable(&mock.int_param_, "int_param", -10000, 10000);

  dd.PublishServicesTopics();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  dynamic_reconfigure::Reconfigure srv;
  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = 1234;

  srv.request.config.ints.push_back(int_param);
  EXPECT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
  EXPECT_EQ(mock.int_param_, int_param.value);
}

TEST_F(DDynamicReconfigureTest, globalCallbackTest)
{
  ros::NodeHandle nh("~");
  DDynamicReconfigure dd(nh);
  MockClass mock;
  mock.int_param_ = 0;
  mock.bool_param_ = false;
  dd.RegisterVariable(&mock.int_param_, "int_param", 0, 100);
  dd.RegisterVariable(&mock.bool_param_, "bool_param");
  dd.RegisterVariable(&mock.double_param_, "double_param", -50, 50);
  dd.setUserCallback(boost::bind(&MockClass::userCallback, &mock));
  dd.PublishServicesTopics();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  EXPECT_CALL(mock,
              userCallback())
      .Times(Exactly(1));

  dynamic_reconfigure::Reconfigure srv;
  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = -1234;
  srv.request.config.ints.push_back(int_param);

  dynamic_reconfigure::BoolParameter bool_param;
  bool_param.name = "bool_param";
  bool_param.value = true;
  srv.request.config.bools.push_back(bool_param);

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param";
  double_param.value = 42.4242;
  srv.request.config.doubles.push_back(double_param);

  EXPECT_NE(mock.int_param_, int_param.value);
  EXPECT_FALSE(mock.bool_param_);
  EXPECT_NE(mock.double_param_, double_param.value);
  EXPECT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
  EXPECT_TRUE(mock.bool_param_);
  EXPECT_EQ(mock.int_param_, int_param.value);
  EXPECT_NEAR(mock.double_param_, double_param.value, 0.0001);
}



TEST_F(DDynamicReconfigureTest, callbackTest)
{
  ros::NodeHandle nh("~");
  DDynamicReconfigure dd(nh);
  MockClass mock;
  dd.registerVariable<int>("int_param", mock.int_param_,
                      boost::bind(&MockClass::intCallback, &mock, _1));
  dd.registerVariable<double>("double_param", mock.double_param_,
                      boost::bind(&MockClass::doubleCallback, &mock, _1));
  dd.registerVariable<bool>("bool_param", mock.bool_param_,
                      boost::bind(&MockClass::boolCallback, &mock, _1));
  dd.registerVariable<std::string>("str_param", mock.str_param_,
                      boost::bind(&MockClass::strCallback, &mock, _1));
    dd.PublishServicesTopics();
    ros::AsyncSpinner spinner(1);
    spinner.start();
  
  dynamic_reconfigure::Reconfigure srv;
  dynamic_reconfigure::IntParameter int_param;
  int_param.name = "int_param";
  int_param.value = -1234;

  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "double_param";
  double_param.value = 42.4242;
  
  dynamic_reconfigure::StrParameter str_param;
  str_param.name = "str_param";
  str_param.value = "hello";
  
  EXPECT_CALL(mock,
              intCallback(int_param.value))
      .Times(Exactly(2));
  EXPECT_CALL(mock,
              doubleCallback(double_param.value))
      .Times(Exactly(1));  
  EXPECT_CALL(mock,
              boolCallback(_))
      .Times(Exactly(0));
  EXPECT_CALL(mock,
              strCallback("hello"))
      .Times(Exactly(1));
  
  srv.request.config.ints.push_back(int_param);
  EXPECT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
  
  
  srv.request.config.doubles.push_back(double_param);
  srv.request.config.strs.push_back(str_param);
  EXPECT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));  
}

TEST_F(DDynamicReconfigureTest, threadTest)
{
  ros::NodeHandle nh("foo");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  DDynamicReconfigure dd(nh);
  int int_param = 0;
  double double_param = 1.0;
  std::string str_param = "";
  std::string second_str_param = "";
  bool bool_param = false;
  dd.registerVariable("int_param", &int_param, "");
  dd.registerVariable("double_param", &double_param, "");
  dd.registerVariable("str_param", &str_param, "");
  dd.registerVariable("second_str_param", &second_str_param, "");
  dd.registerVariable("bool_param", &bool_param, "");
  dd.PublishServicesTopics();
  
  ros::Subscriber sub =
      nh.subscribe<dynamic_reconfigure::Config>("/foo/parameter_updates", 1, boost::bind(&DDynamicReconfigureTest::cfgCb, this, _1));

  waitForCfg();

  ASSERT_EQ(1, cfg_msg_->ints.size());
  ASSERT_EQ("int_param", cfg_msg_->ints[0].name);
  ASSERT_EQ(int_param, cfg_msg_->ints[0].value);
  ASSERT_EQ(1, cfg_msg_->doubles.size());
  ASSERT_EQ("double_param", cfg_msg_->doubles[0].name);
  ASSERT_EQ(double_param, cfg_msg_->doubles[0].value);
  ASSERT_EQ(2, cfg_msg_->strs.size());
  ASSERT_EQ("str_param", cfg_msg_->strs[0].name);
  ASSERT_EQ(str_param, cfg_msg_->strs[0].value);;
  ASSERT_EQ("second_str_param", cfg_msg_->strs[1].name);
  ASSERT_EQ(second_str_param, cfg_msg_->strs[1].value);
  ASSERT_EQ(1, cfg_msg_->bools.size());
  ASSERT_EQ("bool_param", cfg_msg_->bools[0].name);
  ASSERT_EQ(bool_param, cfg_msg_->bools[0].value);

  int_param = 5;
  double_param = 1e-3;
  str_param = "changed";
  bool_param = true;
  
  waitForCfg();
  ASSERT_EQ(1, cfg_msg_->ints.size());
  ASSERT_EQ("int_param", cfg_msg_->ints[0].name);
  ASSERT_EQ(int_param, cfg_msg_->ints[0].value);
  ASSERT_EQ(1, cfg_msg_->doubles.size());
  ASSERT_EQ("double_param", cfg_msg_->doubles[0].name);
  ASSERT_EQ(double_param, cfg_msg_->doubles[0].value);
  ASSERT_EQ(2, cfg_msg_->strs.size());
  ASSERT_EQ("str_param", cfg_msg_->strs[0].name);
  ASSERT_EQ(str_param, cfg_msg_->strs[0].value);;
  ASSERT_EQ("second_str_param", cfg_msg_->strs[1].name);
  ASSERT_EQ(second_str_param, cfg_msg_->strs[1].value);
  ASSERT_EQ(1, cfg_msg_->bools.size());
  ASSERT_EQ("bool_param", cfg_msg_->bools[0].name);
  ASSERT_EQ(bool_param, cfg_msg_->bools[0].value);
}
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ddynamic_reconfigure_test");
  ros::NodeHandle nh;

  ::testing::InitGoogleMock(&argc, argv);

  return RUN_ALL_TESTS();
}
