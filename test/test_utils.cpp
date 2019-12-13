#include <gtest/gtest.h>

#include "utils.hpp"

using namespace bord2;

template<size_t... is>
constexpr std::array<size_t,sizeof...(is)> indexArray(std::index_sequence<is...>)
{
    return {is...};
}

TEST(TestTraits, ReversedIndices)
{
    constexpr auto indarr = indexArray(std::make_index_sequence<3>());
    constexpr auto indarr_rev1 = indexArray(make_reversed_index_seq<1>());
    constexpr auto indarr_rev2 = indexArray(make_reversed_index_seq<2>());
    constexpr auto indarr_rev3 = indexArray(make_reversed_index_seq<3>());
    constexpr auto indarr_rev4 = indexArray(make_reversed_index_seq<4>());

    EXPECT_EQ(indarr, (std::array<size_t,3>{0,1,2}));
    EXPECT_EQ(indarr_rev1, (std::array<size_t,1>{0}));
    EXPECT_EQ(indarr_rev2, (std::array<size_t,2>{1,0}));
    EXPECT_EQ(indarr_rev3, (std::array<size_t,3>{2,1,0}));
    EXPECT_EQ(indarr_rev4, (std::array<size_t,4>{3,2,1,0}));
}

TEST(TestPower, Integer)
{
    EXPECT_EQ(cipow(3,0), 1);
    EXPECT_EQ(cipow(3,1), 3);
    EXPECT_EQ(cipow(3,2), 9);
    EXPECT_EQ(cipow(3,3), 27);
    EXPECT_EQ(cipow(3,4), 81);

    EXPECT_EQ(cipow(-2,0), 1);
    EXPECT_EQ(cipow(-2,1), -2);
    EXPECT_EQ(cipow(-2,2), 4);
    EXPECT_EQ(cipow(-2,3), -8);
}

TEST(TestPower, Float)
{
    EXPECT_EQ(cipow(0.5,0), 1.0);
    EXPECT_EQ(cipow(0.5,1), 0.5);
    EXPECT_EQ(cipow(0.5,2), 0.25);
    EXPECT_EQ(cipow(0.5,3), 0.125);
    EXPECT_EQ(cipow(0.5,4), 0.0625);

    EXPECT_EQ(cipow(-0.5,0), 1.0);
    EXPECT_EQ(cipow(-0.5,1), -0.5);
    EXPECT_EQ(cipow(-0.5,2), 0.25);
    EXPECT_EQ(cipow(-0.5,3), -0.125);
    EXPECT_EQ(cipow(-0.5,4), 0.0625);
}

TEST(TestBinom, Value)
{
    EXPECT_EQ(binom<>::get(0,0), 1);
    EXPECT_EQ(binom<>::get(0,1), 0);

    EXPECT_EQ(binom<>::get(1,0), 1);
    EXPECT_EQ(binom<>::get(1,1), 1);
    EXPECT_EQ(binom<>::get(1,2), 0);

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

    auto arr3 = binom<>::getArrayRev<double,1,6>();
    ASSERT_EQ(arr3.size(), 6);
    EXPECT_EQ(arr3[0], 0.0);
    EXPECT_EQ(arr3[1], 0.0);
    EXPECT_EQ(arr3[2], 0.0);
    EXPECT_EQ(arr3[3], 0.0);
    EXPECT_EQ(arr3[4], 1.0);
    EXPECT_EQ(arr3[5], 1.0);

    auto arr4 = binom<>::getArrayRev<double,0,6>();
    ASSERT_EQ(arr4.size(), 6);
    EXPECT_EQ(arr4[0], 0.0);
    EXPECT_EQ(arr4[1], 0.0);
    EXPECT_EQ(arr4[2], 0.0);
    EXPECT_EQ(arr4[3], 0.0);
    EXPECT_EQ(arr4[4], 0.0);
    EXPECT_EQ(arr4[5], 1.0);
}
