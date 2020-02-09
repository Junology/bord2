#include <gtest/gtest.h>

#include "PlTang.hpp"

TEST(TestPlTang, EncodeDecode)
{
    /** Successful case **/
    constexpr PlTang<32,32> pltang(
        "  || |\n"
        "r-JL-J\n"
        "|  r7 \n"
        "L7 LJ \n");

    EXPECT_TRUE(pltang.isvalid());
    EXPECT_EQ(pltang.vlength(), 4);
    EXPECT_EQ(pltang.hlength(), 6);
    EXPECT_EQ(pltang.aarep(),
              "  || |\n"
              "r-JL-J\n"
              "|  r7 \n"
              "L7 LJ \n");

    /** Fail case **/
    // invalid semi-elementary tangles
    constexpr PlTang<32,32> pltang2(
        "|-J L7\n"
        "L     \n");
    EXPECT_FALSE(pltang2.isvalid());

    // length error
    constexpr PlTang<32,32> pltang3(
        "r-JL-J\n"
        "| r7 \n");
    EXPECT_FALSE(pltang3.isvalid());

    // out-of-range; detected statically.
    /*
    constexpr PlTang<32,2> pltang4(
        "  || |",
        "r-JL-J",
        "|  r7 ",
        "L7 LJ ");
    constexpr PlTang<2,32> pltang5(
        "  || |",
        "r-JL-J",
        "|  r7 ",
        "L7 LJ ");
    */
}

TEST(TestPlTang, Equal)
{
    constexpr PlTang<32,32> pltang(
        "  || |\n"
        "r-JL-J\n"
        "|  r7 \n"
        "L7 LJ \n");

    EXPECT_EQ((PlTang<32,32>{}), (PlTang<32,32>{"JL"}));
    EXPECT_EQ((PlTang<32,32>{"JL"}), (PlTang<32,32>{}));
    EXPECT_EQ(pltang, pltang);
    EXPECT_EQ(pltang, (PlTang<32,32>{
            "  || |\n"
            "r-JL-J\n"
            "|  r7 \n"
            "L7 LJ \n"}));

    EXPECT_NE(pltang, (PlTang<32,32>{
            "  || |\n"
            "r-JL-J\n"
            "|     \n"
            "L7    \n"}));
    EXPECT_NE(pltang, (PlTang<32,32>{
            "  || | \n"
            "r-JL-J \n"
            "|  r7  \n"
            "L7 LJ  \n"}));
    EXPECT_NE(pltang, (PlTang<32,32>{
            "  || |\n"
            "r-JL-J\n"
            "|  r7 \n"
            "L7 LJ \n"
            " |    \n"}));
}

TEST(TestPlTang, Slice)
{
    constexpr PlTang<32,32> pltang(
        "  || |\n"
        "r-JL-J\n"
        "|  r7 \n"
        "L7 LJ \n");

    EXPECT_FALSE((pltang.slice<2,2>(0,0).isvalid()))
        << (pltang.slice<2,2>(0,0).aarep());
    EXPECT_FALSE((pltang.slice<2,2>(1,0).isvalid()))
        << (pltang.slice<2,2>(1,0).aarep());
    EXPECT_FALSE((pltang.slice<2,2>(2,0).isvalid()))
        << (pltang.slice<2,2>(2,0).aarep());
    EXPECT_FALSE((pltang.slice<2,2>(3,0).isvalid()))
        << (pltang.slice<2,2>(3,0).aarep());
    EXPECT_FALSE((pltang.slice<2,2>(4,0).isvalid()));

    EXPECT_EQ((pltang.slice<2,3>(3,0).aarep()),
              "| |\n"
              "L-J\n");
    EXPECT_EQ((pltang.slice<2,3>(0,2).aarep()),
              "|  \n"
              "L7 \n");
    EXPECT_EQ((pltang.slice<3,3>(3,1).aarep()),
              "L-J\n"
              "r7 \n"
              "LJ \n");
}

TEST(TestPlTang, Replace)
{
    constexpr PlTang<32,32> pltang(
        " | || \n"
        " L7|L7\n"
        "r-JL-J\n"
        "|  r7 \n"
        "L7 LJ \n");

    /* Success case */ {
        PlTang<32,32> pltang1 = pltang;

        EXPECT_TRUE(pltang1.replace(3, 0, PlTang<2,3>("LJ \nr-7\n")));
        EXPECT_TRUE(pltang1.replace(0, 1, PlTang<3,3>(" | \nrJ \n|  \n")));
        EXPECT_TRUE(pltang1.replace(3, 3, PlTang<2,2>("  \n  \n")));

        EXPECT_EQ(pltang1.aarep(),
                  " | LJ \n"
                  " | r-7\n"
                  "rJ L-J\n"
                  "|     \n"
                  "L7    \n");
    }

    /* Fail case */ {
        PlTang<32,32> pltang2 = pltang;

        // Validity
        EXPECT_FALSE(pltang2.replace(3, 0, PlTang<2,3>{}));

        // Out-of-range
        EXPECT_FALSE(pltang2.replace(4, 1, PlTang<2,3>{}));
        EXPECT_FALSE(pltang2.replace(2, 3, PlTang<3,2>{}));

        // Boundary incompatible
        EXPECT_FALSE(pltang2.replace(3, 0, PlTang<2,3>("L7 \nr7 \n")));
        EXPECT_FALSE(pltang2.replace(0, 1, PlTang<3,2>("|| \n|| \n|| \n")));

        // Failed replace changes nothing.
        EXPECT_EQ(pltang2.aarep(), pltang.aarep());
    }
}

TEST(TestPlTang, InnerPoint)
{
    constexpr auto pt1 = PlTang<3,3>{"|||\nLJ|\nr-J\n"}.takeInnPt();
    EXPECT_EQ(pt1, (std::array<double,2>{0.5,1}));
    constexpr auto pt2 = PlTang<3,3>{"r7|\nLJ|\nr-J\n"}.takeInnPt();
    EXPECT_EQ(pt2, (std::array<double,2>{0.5,1}));
    constexpr auto pt3 = PlTang<3,3>{"L7|\nrJ|\n|rJ\n"}.takeInnPt();
    EXPECT_EQ(pt3, (std::array<double,2>{1,0.5}));
    constexpr auto pt4 = PlTang<3,3>{" ||\nrJ|\n|rJ\n"}.takeInnPt();
    EXPECT_EQ(pt4, (std::array<double,2>{1.5,1}));
}

TEST(TestPlTang, Precomp)
{
    constexpr PlTang<32,32> upper(
        "  || |\n"
        "r-JL-J\n");
    constexpr PlTang<32,32> lower(
        "|  r7 \n"
        "L7 LJ \n");
    constexpr PlTang<32,32> lower2{
        "| L7| \n"
        "L7 LJ \n"};

    EXPECT_EQ(upper.domain(), lower.codomain());
    EXPECT_EQ((PlTang<32,32>(upper).precomp(lower).aarep()),
              "  || |\n"
              "r-JL-J\n"
              "|  r7 \n"
              "L7 LJ \n");

    EXPECT_NE(upper.domain(), lower2.codomain());
    EXPECT_FALSE((PlTang<32,32>(upper).precomp(lower2).isvalid()))
        << (PlTang<32,32>(upper).precomp(lower2).aarep());
}
