#include <gtest/gtest.h>
#include <ros/ros.h>

// Declare a test
TEST(TestSuite, testCase1)
{
    EXPECT_TRUE(1==1);
}

// Declare another test
TEST(TestSuite, testCase2)
{
    EXPECT_TRUE(1==1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}