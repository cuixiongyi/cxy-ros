//
// Created by xiongyi on 7/14/16.
//

//
// Created by xiongyi on 7/8/16.
//

//
// Created by xiongyi on 7/6/16.
//

#include <gtest/gtest.h>

#include <gtest/gtest.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <string>
using namespace RD;

class lmicp_test : public testing::Test {
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  lmicp_test() {
    // You can do set-up work for each test here.
  }

  virtual ~lmicp_test() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
    m_pose1.pose.position.x = 100.1;
    m_name1 = "abc";
    m_pose2.pose.position.x = 38.3;
    m_name2 = "bcd";
  }

public:
  std::string m_name1;
  std::string m_name2;
  geometry_msgs::PoseStamped m_pose1;
  geometry_msgs::PoseStamped m_pose2;
};

TEST_F(lmicp_test, worldObject_test) {
    ASSERT_EQ("abc", m_name1);
}

TEST_F(lmicp_test, callback_test) {
    ASSERT_EQ("bcd", m_name2);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
