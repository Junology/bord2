#include <gtest/gtest.h>

#include "utils.hpp"

using namespace bord2;

TEST(TestBinom, Value)
{
    EXPECT_EQ(binom<>::get(4,0), 1);
    EXPECT_EQ(binom<>::get(4,1), 4);
    EXPECT_EQ(binom<>::get(4,2), 6);
    EXPECT_EQ(binom<>::get(4,3), 4);
    EXPECT_EQ(binom<>::get(4,4), 1);
    EXPECT_EQ(binom<>::get(4,5), 0);
}

TEST(TestBinom, StaticUse)
{
    EXPECT_EQ(std::make_index_sequence<binom<>::get(4,2)>::size(), 6);
}

TEST(TestBinom, Array)
{
    auto arr = binom<>::getArray<double,4,6>();
    ASSERT_EQ(arr.size(), 6);
    EXPECT_EQ(arr[0], 1.0);
    EXPECT_EQ(arr[1], 4.0);
    EXPECT_EQ(arr[2], 6.0);
    EXPECT_EQ(arr[3], 4.0);
    EXPECT_EQ(arr[4], 1.0);
    EXPECT_EQ(arr[5], 0.0);

    auto arr2 = binom<>::getArrayRev<double,3,6>();
    ASSERT_EQ(arr2.size(), 6);
    EXPECT_EQ(arr2[0], 0.0);
    EXPECT_EQ(arr2[1], 0.0);
    EXPECT_EQ(arr2[2], 1.0);
    EXPECT_EQ(arr2[3], 3.0);
    EXPECT_EQ(arr2[4], 3.0);
    EXPECT_EQ(arr2[5], 1.0);
}
