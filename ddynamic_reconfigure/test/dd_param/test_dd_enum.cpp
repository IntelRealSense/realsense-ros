#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ddynamic_reconfigure/param/dd_enum_param.h>

namespace ddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDEnumTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDEnum param1("param1",0,"param1",0,dict);
        DDEnum param2("",0,"","ONE",dict);
        DDEnum param3("\000",0,"\000", 0, dict); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDEnumTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDEnum param("dd_param",0,"dd_param",1,dict);
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v(1);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value("ONE");
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(10);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value("TEN");
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(10.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value(-1);
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens regarding int setValue

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(10)));

        param.setValue(v);
        param.setValue(Value("TEN"));
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens regarding string setValue

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(10)));

        // make sure setValue and sameValue can handle int-string values
        v = Value("10");
        ASSERT_TRUE(param.sameValue(v));
        param.setValue(v);

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(10)));

        // make sure setValue and sameValue can handle non-number non-dictionary strings
        v = Value("TWO");
        // 'two' is not in our dictionary, so we will attempt to place it in there using a hash conversion
        ASSERT_FALSE(param.sameValue(v));
        param.setValue(v);

        ASSERT_TRUE(param.getValue().getType() == "int");
        int hash = (int)boost::hash<string>()("TWO");
        ASSERT_TRUE(param.sameValue(Value(hash)));
    }

    TEST(DDEnumTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        map<string,int> dict;
        dict["ONE"] = 1;
        dict["NEG-ONE"] = -1;
        dict["TEN"] = 10;
        DDEnum param1("param1",0,"param1",1,dict);
        stringstream stream;
        stream << param1;
        ASSERT_EQ(param1.getName() + ":" + param1.getValue().toString(),stream.str());
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    srand((unsigned int)random());

    return RUN_ALL_TESTS();
}