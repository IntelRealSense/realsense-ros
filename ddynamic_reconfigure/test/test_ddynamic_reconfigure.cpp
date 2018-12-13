#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "OCDFAInspection"
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <ddynamic_reconfigure/param/dd_all_params.h>
#include <exception>
using namespace std;
namespace ddynamic_reconfigure {

    TEST(DDynamicReconfigureTest, mapTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDynamicReconfigure dd(nh); // gets our main class running

        DDParam* ptr = new DDBool("exists",0,"",true);
        DDPtr dd_ptr = DDPtr(ptr);

        dd.add(dd_ptr);
        ASSERT_NE(DDPtr(),dd.at("exists"));

        dd.remove(ptr);
        ASSERT_EQ(DDPtr(),dd.at("exists"));

        dd.remove(dd_ptr);
        ASSERT_EQ(DDPtr(),dd.at("exists"));
    }

    void basicCallback(const DDMap& map, int, bool *flag) {
        *flag = true;
    }

    /**
     * @brief preliminary test which makes sure we can use callbacks
     */
    TEST(DDynamicReconfigureTest, basicCallbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDynamicReconfigure dd(nh); // gets our main class running

        bool flag = false;
        DDFunc callback = bind(&basicCallback,_1,_2,&flag);
        dd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_TRUE(flag);
    }

    void intCallback(const DDMap& map, int, int *flag) {
        ASSERT_EQ("int",at(map,"int_param")->getValue().getType());
        *flag = at(map,"int_param")->getValue().toInt();
    }

    /**
     * @brief tests that int parameters are registered properly
     */
    TEST(DDynamicReconfigureTest, intTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        int flag = 0;
        DDFunc callback = bind(&intCallback,_1,_2,&flag);

        DDynamicReconfigure dd(nh);
        dd.add(new DDInt("int_param", 0,"int_param", 0));
        dd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::IntParameter int_param;
        int_param.name = "int_param";

        int_param.value = (int)random();
        srv.request.config.ints.push_back(int_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(int_param.value, flag);

        int_param.value = INT32_MAX;
        srv.request.config.ints.push_back(int_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(int_param.value, flag);

        int_param.value = INT32_MIN;
        srv.request.config.ints.push_back(int_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(int_param.value, flag);
    }

    void doubleCallback(const DDMap& map, int, double *flag) {
        ASSERT_EQ("double",at(map,"double_param")->getValue().getType());
        *flag = at(map,"double_param")->getValue().toDouble();
    }

    /**
     * @brief tests that double parameters are registered properly
     */
    TEST(DDynamicReconfigureTest, doubleTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        double flag = 0;
        DDFunc callback = bind(&doubleCallback,_1,_2,&flag);

        DDynamicReconfigure dd(nh);
        dd.add(new DDDouble("double_param", 0,"double_param", 0));
        dd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = "double_param";

        double_param.value = (double)random();
        srv.request.config.doubles.push_back(double_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(double_param.value, flag);

        double_param.value = DBL_MAX;
        srv.request.config.doubles.push_back(double_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(double_param.value, flag);

        double_param.value = DBL_MIN;
        srv.request.config.doubles.push_back(double_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(double_param.value, flag);

        double_param.value = -DBL_MAX;
        srv.request.config.doubles.push_back(double_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(double_param.value, flag);

        double_param.value = -DBL_MIN;
        srv.request.config.doubles.push_back(double_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(double_param.value, flag);
    }

    void boolCallback(const DDMap& map, int, bool *flag) {
        ASSERT_EQ("bool",at(map,"bool_param")->getValue().getType());
        *flag = at(map,"bool_param")->getValue().toBool();
    }

    /**
     * @brief tests that boolean parameters are registered properly
     */
    TEST(DDynamicReconfigureTest, boolTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        bool flag = true;
        DDFunc callback = bind(&boolCallback,_1,_2,&flag);

        DDynamicReconfigure dd(nh);
        dd.add(new DDBool("bool_param", 0,"bool_param", false));
        dd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::BoolParameter bool_param;
        bool_param.name = "bool_param";

        bool_param.value = (unsigned char)false;
        srv.request.config.bools.push_back(bool_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ((bool)bool_param.value, flag);

        flag = false;

        bool_param.value = (unsigned char)true;
        srv.request.config.bools.push_back(bool_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ((bool)bool_param.value, flag);
    }

    void strCallback(const DDMap& map, int, string *flag) {
        ASSERT_EQ("string",at(map,"string_param")->getValue().getType());
        *flag = at(map,"string_param")->getValue().toString();
    }

    /**
     * @brief tests that string parameters are registered properly
     */
    TEST(DDynamicReconfigureTest, stringTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        string flag = "YOU SHOULDN'T RECEIVE THIS";
        DDFunc callback = bind(&strCallback,_1,_2,&flag);

        DDynamicReconfigure dd(nh);
        dd.add(new DDString("string_param", 0,"string_param", ""));
        dd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::StrParameter string_param;
        string_param.name = "string_param";

        string_param.value = string("\000"); // NOLINT(bugprone-string-literal-with-embedded-nul)
        srv.request.config.strs.push_back(string_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(string_param.value, flag);

        string_param.value = "";
        srv.request.config.strs.push_back(string_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(string_param.value, flag);

        string_param.value = "Hello World";
        srv.request.config.strs.push_back(string_param);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(string_param.value, flag);
    }

    void enumCallback(const DDMap& map, int, int *flag) {
        ASSERT_EQ("int",at(map,"enum_param")->getValue().getType());
        *flag = at(map,"enum_param")->getValue().toInt();
    }

    /**
     * @brief tests that int-enum parameters are registered properly
     */
    TEST(DDynamicReconfigureTest, enumTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        int flag = 0;
        DDFunc callback = bind(&enumCallback,_1,_2,&flag);

        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;

        DDynamicReconfigure dd(nh);
        dd.add(new DDEnum("enum_param", 0,"enum_param", "ONE", dict));
        dd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        dynamic_reconfigure::IntParameter int_enum;
        int_enum.name = "enum_param";

        int_enum.value = 1;
        srv.request.config.ints.push_back(int_enum);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(int_enum.value, flag);

        int_enum.value = 10;
        srv.request.config.ints.push_back(int_enum);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(int_enum.value, flag);

        int_enum.value = -1;
        srv.request.config.ints.push_back(int_enum);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(int_enum.value, flag);

        srv.request.config.ints.clear();
        dynamic_reconfigure::StrParameter str_enum;
        str_enum.name = "enum_param";

        str_enum.value = "ONE";
        srv.request.config.strs.push_back(str_enum);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(dict[str_enum.value], flag);

        str_enum.value = "TEN";
        srv.request.config.strs.push_back(str_enum);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(dict[str_enum.value], flag);

        str_enum.value = "NEG-ONE";
        srv.request.config.strs.push_back(str_enum);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(dict[str_enum.value], flag);
    }

    void complexCallback(const DDMap& map, int level) {
        ASSERT_EQ(0, level);
        ASSERT_EQ(1, at(map,"int_param")->getValue().toInt());
        ASSERT_EQ(0.6, at(map,"double_param")->getValue().toDouble());
        ASSERT_EQ("Goodbye Home", at(map,"str_param")->getValue().toString());
        ASSERT_EQ(false, at(map,"bool_param")->getValue().toBool());
        ASSERT_EQ(3, at(map,"enum_param")->getValue().toInt());
    }

    /**
     * @brief tests that ddynamic can handle complex callbacks and param lists.
     */
    TEST(DDynamicReconfigureTest, callbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDynamicReconfigure dd(nh); // gets our main class running
        dd.add(new DDInt("int_param", 0, "An Integer parameter", 0, 50, 100));
        dd.add(new DDDouble("double_param", 0, "A double parameter", .5, 0, 1));
        dd.add(new DDString("str_param", 0, "A string parameter", "Hello World"));
        dd.add(new DDBool("bool_param", 0, "A Boolean parameter", true));
        map<string, int> dict; {
            dict["Small"] = 0;
            dict["Medium"] = 1;
            dict["Large"] = 2;
            dict["ExtraLarge"] = 3;
        }
        dd.add(new DDEnum("enum_param", 0, "A size parameter which is edited via an enum", 0, dict));
        dd.start(complexCallback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        dynamic_reconfigure::IntParameter int_param;
        int_param.name = "int_param";
        int_param.value = 1;
        srv.request.config.ints.push_back(int_param);

        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = "double_param";
        double_param.value = 0.6;
        srv.request.config.doubles.push_back(double_param);

        dynamic_reconfigure::BoolParameter bool_param;
        bool_param.name = "bool_param";
        bool_param.value = (unsigned char) false;
        srv.request.config.bools.push_back(bool_param);

        dynamic_reconfigure::StrParameter string_param;
        string_param.name = "str_param";
        string_param.value = "Goodbye Home";
        srv.request.config.strs.push_back(string_param);

        dynamic_reconfigure::StrParameter enum_param;
        enum_param.name = "enum_param";
        enum_param.value = "ExtraLarge";
        srv.request.config.strs.push_back(enum_param);

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));

        bool flag = false;
        DDFunc callback = bind(&basicCallback,_1,_2,&flag);
        dd.setCallback(callback);
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_TRUE(flag);

        flag = false;
        dd.clearCallback();
        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_FALSE(flag);
    }

    class InternalClass {
    public:
        inline void internalCallback(const DDMap& map, int level) {}
    };

    /**
     * @brief tests that ddynamic can take member methods as callbacks
     */
    TEST(DDynamicReconfigureTest, memberCallbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDynamicReconfigure dd(nh); // gets our main class running

        dd.start(&InternalClass::internalCallback,new InternalClass);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
    }

    void levelCallback(const DDMap&, int level, int *flag) {
        *flag = level;
    }

    /**
     * @brief tests that ddynamic properly handles param change levels
     */
    TEST(DDynamicReconfigureTest, levelTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        int flag = 0;
        DDFunc callback = bind(&levelCallback, _1, _2, &flag);

        DDynamicReconfigure dd(nh);
        int top = (int) random() % 5 + 5;
        unsigned int or_sum = 0, next;
        for (int i = 1; i < top; i++) {
            next = (unsigned int) random();
            or_sum |= next;
            dd.add(new DDInt((format("param_%d") % i).str(), next,"level_param", 0));
        }
        dd.start(callback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;
        dynamic_reconfigure::IntParameter int_param;
        for (int i = 1; i < top; i++) {
            int_param.name = (format("param_%d") % i).str();
            int_param.value = 1;
            srv.request.config.ints.push_back(int_param);
        }

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(or_sum, flag);

        dd.add(new DDInt("unchanged_param", 1,"unchanged_param", 0)); //u-int max means everything is 1, so the result must also be that.
        dynamic_reconfigure::IntParameter unchanged_param;
        unchanged_param.name = "unchanged_param";
        unchanged_param.value = 1;
        srv.request.config.ints.push_back(unchanged_param);

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(1, flag);

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        ASSERT_EQ(0, flag);
    }

    void badCallback(const DDMap&, int) {
        std::exception e;
        throw e; // NOLINT(cert-err09-cpp,cert-err61-cpp,misc-throw-by-value-catch-by-reference)
    }

    /**
     * @brief tests that ddynamic can properly handle exceptions
     */
    TEST(DDynamicReconfigureTest, badCallbackTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDynamicReconfigure dd(nh); // gets our main class running
        dd.start(badCallback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
        // this is the best way to see exceptions doesn't make the whole thing tumble
    }

    void missingCallback(const DDMap& map, int) {
        ASSERT_EQ(map.end(),map.find("int_param"));
        ASSERT_EQ(map.end(),map.find("double_param"));
        ASSERT_EQ(map.end(),map.find("bool_param"));
        ASSERT_EQ(map.end(),map.find("str_param"));
        ASSERT_EQ(map.end(),map.find("enum_param"));
    }

    /**
     * @brief tests that ddynamic can properly handle missing/unregistered parameters
     */
    TEST(DDynamicReconfigureTest, unknownParamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDynamicReconfigure dd(nh); // gets our main class running
        dd.start(missingCallback);

        ros::AsyncSpinner spinner(1);
        spinner.start();

        dynamic_reconfigure::Reconfigure srv;

        dynamic_reconfigure::IntParameter int_param;
        int_param.name = "int_param";
        int_param.value = 1;
        srv.request.config.ints.push_back(int_param);

        dynamic_reconfigure::DoubleParameter double_param;
        double_param.name = "double_param";
        double_param.value = 0.6;
        srv.request.config.doubles.push_back(double_param);

        dynamic_reconfigure::BoolParameter bool_param;
        bool_param.name = "bool_param";
        bool_param.value = (unsigned char) false;
        srv.request.config.bools.push_back(bool_param);

        dynamic_reconfigure::StrParameter string_param;
        string_param.name = "str_param";
        string_param.value = "Goodbye Home";
        srv.request.config.strs.push_back(string_param);

        dynamic_reconfigure::StrParameter enum_param;
        enum_param.name = "enum_param";
        enum_param.value = "ExtraLarge";
        srv.request.config.strs.push_back(enum_param);

        ASSERT_TRUE(ros::service::call(nh.getNamespace() + "/set_parameters", srv));
    }

    /**
     * @brief tests that ddynamic's stream operator properly works
     */
    TEST(DDynamicReconfigureTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        ros::NodeHandle nh("~");
        DDynamicReconfigure dd(nh); // gets our main class running
        DDInt dd_int("int_param", 0, "An Integer parameter", 0, 50, 100);
        DDDouble dd_double("double_param", 0, "A double parameter", .5, 0, 1);
        DDString dd_string("str_param", 0, "A string parameter", "Hello World");
        DDBool dd_bool("bool_param", 0, "A Boolean parameter", true);
        dd.add(new DDInt(dd_int));
        dd.add(new DDDouble(dd_double));
        dd.add(new DDString(dd_string));
        dd.add(new DDBool(dd_bool));
        map<string, int> dict; {
            dict["Small"] = 0;
            dict["Medium"] = 1;
            dict["Large"] = 2;
            dict["ExtraLarge"] = 3;
        }
        DDEnum dd_enum("enum_param", 0, "A size parameter which is edited via an enum", 0, dict);
        dd.add(new DDEnum(dd_enum));

        stringstream stream, explicit_stream;
        stream << dd;

        explicit_stream << "{" << dd_bool << "," << dd_double << "," << dd_enum << "," << dd_int << "," << dd_string << "}";
        ASSERT_EQ(explicit_stream.str(),stream.str());
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ddynamic_reconfigure_test");

    srand((unsigned int)random());

    return RUN_ALL_TESTS();
}
#pragma clang diagnostic pop