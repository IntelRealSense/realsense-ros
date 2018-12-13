#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ddynamic_reconfigure/param/dd_int_param.h>

namespace ddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDIntTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDInt param1("param1",0,"param1",1);
        DDInt param2("",0,"",1,100);
        DDInt param3("\000",(unsigned int)-1,"param1", 1, -100, -10); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDIntTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDInt param("dd_param",0,"param1",1);
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v(1);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(2);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(2.0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value(3);
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getValue().getType() == "int");
        ASSERT_TRUE(param.sameValue(Value(2)));
    }

    TEST(DDIntTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDInt param1("param1",0,"param1",1);
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