#include <gtest/gtest.h>

TEST(realsense2_camera, test1)
{
	std::cout << "Running test1...";
  ASSERT_EQ(4, 2 + 2);
}
TEST(realsense2_camera, test2)
{
  std::cout << "Running test2...";
  ASSERT_EQ(3, 2 + 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

