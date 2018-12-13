#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ddynamic_reconfigure/dd_value.h>

namespace ddynamic_reconfigure {

    /**
     * @brief a test making sure integer interpretations of value work all around.
     */
    TEST(DDValueTest, intTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // test init properly works
        Value v(0);
        ASSERT_EQ(0,v.toInt());
        ASSERT_EQ(v.getType(),"int");
        v = Value(-1);
        ASSERT_EQ(-1,v.toInt());
        ASSERT_EQ(v.getType(),"int");
        v = Value(INT32_MAX);
        ASSERT_EQ(INT32_MAX,v.toInt());
        ASSERT_EQ(v.getType(),"int");
        v = Value(INT32_MIN);
        ASSERT_EQ(INT32_MIN,v.toInt());
        ASSERT_EQ(v.getType(),"int");
        // test that conversions from any value to this type properly works
        int i = 1;
        v = Value(i + 0.1);
        ASSERT_EQ(i,v.toInt());
        ASSERT_EQ(v.getType(),"double");
        v = Value((bool)i);
        ASSERT_EQ(i,v.toInt());
        ASSERT_EQ(v.getType(),"bool");
        stringstream str; str << i;
        v = Value(str.str());
        ASSERT_EQ(i,v.toInt());
        ASSERT_EQ(v.getType(),"string");
    }

    /**
     * @brief a test making sure double interpretations of value work all around.
     */
    TEST(DDValueTest, doubleTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // test init properly works
        Value v(0.0);
        ASSERT_EQ(0.0,v.toDouble());
        ASSERT_EQ(v.getType(),"double");
        v = Value(-1.0);
        ASSERT_EQ(-1.0,v.toDouble());
        ASSERT_EQ(v.getType(),"double");
        v = Value((double)INT32_MAX);
        ASSERT_EQ((double)INT32_MAX,v.toDouble());
        ASSERT_EQ(v.getType(),"double");
        v = Value((double)INT32_MIN);
        ASSERT_EQ((double)INT32_MIN,v.toDouble());
        ASSERT_EQ(v.getType(),"double");
        v = Value(DBL_MAX);
        ASSERT_EQ(DBL_MAX,v.toDouble());
        ASSERT_EQ(v.getType(),"double");
        v = Value(DBL_MIN);
        ASSERT_EQ(DBL_MIN,v.toDouble());
        ASSERT_EQ(v.getType(),"double");
        v = Value(-DBL_MAX);
        ASSERT_EQ(-DBL_MAX,v.toDouble());
        ASSERT_EQ(v.getType(),"double");
        // test that conversions from any value to this type properly works
        double f = 1.0;
        v = Value((int)f);
        ASSERT_EQ(f,v.toDouble());
        ASSERT_EQ(v.getType(),"int");
        v = Value((int)(f + 0.1));
        ASSERT_NE(f + 0.1,v.toDouble());
        ASSERT_EQ(v.getType(),"int");
        v = Value((bool)f);
        ASSERT_EQ(f,v.toDouble());
        ASSERT_EQ(v.getType(),"bool");
        stringstream str; str << f;
        v = Value(str.str());
        ASSERT_EQ(f,v.toDouble());
        ASSERT_EQ(v.getType(),"string");
        str << ".000";
        v = Value(str.str());
        ASSERT_EQ(f,v.toDouble());
        ASSERT_EQ(v.getType(),"string");
    }

    /**
     * @brief a test making sure boolean interpretations of value work all around.
     */
    TEST(DDValueTest, boolTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
// test init properly works
        Value v(false);
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"bool");
        v = Value(true);
        ASSERT_EQ(true,v.toBool());
        ASSERT_EQ(v.getType(),"bool");
        // test that conversions from any value to this type properly works
        v = Value(1);
        ASSERT_EQ(true,v.toBool());
        ASSERT_EQ(v.getType(),"int");
        v = Value(0);
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"int");
        v = Value(2);
        ASSERT_EQ(true,v.toBool());
        ASSERT_EQ(v.getType(),"int");
        v = Value(-1);
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"int");
        v = Value(1.0);
        ASSERT_EQ(true,v.toBool());
        ASSERT_EQ(v.getType(),"double");
        v = Value(0.0);
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"double");
        v = Value(2.0);
        ASSERT_EQ(true,v.toBool());
        ASSERT_EQ(v.getType(),"double");
        v = Value(-1.0);
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"double");
        v = Value("true");
        ASSERT_EQ(true,v.toBool());
        ASSERT_EQ(v.getType(),"string");
        v = Value("false");
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"string");
        v = Value("not a bool");
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"string");
        v = Value("");
        ASSERT_EQ(false,v.toBool());
        ASSERT_EQ(v.getType(),"string");
    }

    /**
     * @brief a test making sure string interpretations of value work all around.
     */
    TEST(DDValueTest, stringTest) { // NOLINT(cert-err58-cpp,modernize-use-equals-delete)
        // test init properly works
        Value v("normal");
        ASSERT_EQ("normal",v.toString());
        ASSERT_EQ(v.getType(),"string");
        v = Value("\000");
        ASSERT_EQ("\000",v.toString());
        ASSERT_EQ(v.getType(),"string");
        v = Value("");
        ASSERT_EQ("",v.toString());
        ASSERT_EQ(v.getType(),"string");
        string s("How long can I make this string go on for? Well, You would be surprised! I can make it really long, but I won't.");
        v = Value(s);
        ASSERT_EQ(s,v.toString());
        ASSERT_EQ(v.getType(),"string");
        // test that conversions from any value to this type properly works
        v = Value(1);
        ASSERT_EQ("1",v.toString());
        ASSERT_EQ(v.getType(),"int");
        v = Value(-1);
        ASSERT_EQ("-1",v.toString());
        ASSERT_EQ(v.getType(),"int");
        v = Value(1.);
        ASSERT_EQ("1",v.toString());
        ASSERT_EQ(v.getType(),"double");
        v = Value(-1.);
        ASSERT_EQ("-1",v.toString());
        ASSERT_EQ(v.getType(),"double");
        v = Value(1.1);
        ASSERT_EQ("1.1",v.toString());
        ASSERT_EQ(v.getType(),"double");
        v = Value(-1.1);
        ASSERT_EQ("-1.1",v.toString());
        ASSERT_EQ(v.getType(),"double");
        v = Value(true);
        ASSERT_EQ("true",v.toString());
        ASSERT_EQ(v.getType(),"bool");
        v = Value(false);
        ASSERT_EQ("false",v.toString());
        ASSERT_EQ(v.getType(),"bool");
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    srand((unsigned int)random());

    return RUN_ALL_TESTS();
}