#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ddynamic_reconfigure/param/dd_bool_param.h>

namespace ddynamic_reconfigure {

    /**
     * @brief preliminary test which makes sure we can use the object.
     */
    TEST(DDBoolTest, constructorTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDBool param1("param1",0,"param1",true);
        DDBool param2("",0,"",false);
        DDBool param3("\000",0,"\000",false); // NOLINT(bugprone-string-literal-with-embedded-nul)
    }

    /**
     * @brief a test making sure we can handle all API for handling the values of the param
     */
    TEST(DDBoolTest, valueTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDBool param("dd_param",0,"dd_param",true);
        // we won't do any tests on getLevel or getName, as those are implicit.
        Value v(true);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(1);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_TRUE(param.sameValue(v));

        v = Value(false);
        ASSERT_TRUE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        v = Value(0);
        ASSERT_FALSE(param.sameType(v));
        ASSERT_FALSE(param.sameValue(v));

        param.setValue(v);
        v = Value(true);
        ASSERT_FALSE(param.sameValue(v)); // makes sure anti-aliasing happens

        ASSERT_TRUE(param.getValue().getType() == "bool");
        ASSERT_TRUE(param.sameValue(Value(false)));
    }

    TEST(DDBoolTest, streamTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        DDBool param1("param1",0,"param1",true);
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