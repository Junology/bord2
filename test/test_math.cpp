#include <gtest/gtest.h>

class TestMath : public ::testing::Test {
protected:
    static constexpr double threshold_ = 0.00001;

    virtual void SetUp() override {
    }
};

TEST_F(TestMath, Test1)
{
    EXPECT_EQ(3.0*3.0+4.0*4.0-5.0*5.0, 0.0) << "Rounding error.";
}
