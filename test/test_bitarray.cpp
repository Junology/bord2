#include <gtest/gtest.h>

#include "utils.hpp"
#include "utils/BitArray.hpp"

//using namespace bord2;

inline constexpr unsigned char operator "" _uc(unsigned long long x) noexcept
{
    return static_cast< unsigned char >(x);
}

TEST(TestBitArray, Initialize)
{
    constexpr BitArray<8, unsigned char> ba1{0xf2_uc};
    constexpr BitArray<24, unsigned char> ba2(0xff_uc, 0x13_uc, 0x0A_uc);
    constexpr auto ba3 = ba1;
    constexpr auto ba4 = std::move(ba2);

    EXPECT_EQ(ba1, ba3);

    EXPECT_EQ(sizeof(ba1), sizeof(unsigned char));
    EXPECT_EQ(sizeof(ba4), sizeof(unsigned char)*3);

    constexpr BitArray<29, unsigned char> ba5{ba3};
    BitArray<29, unsigned char> ba6{};

    ba6 = ba1;

    EXPECT_EQ(sizeof(ba5), sizeof(unsigned char)*4);
    EXPECT_EQ(ba5, (BitArray<29, unsigned char>(0xf2u)));
}

TEST(TestBitArray, Comparison)
{
    constexpr BitArray<23, unsigned char> ba{0xff_uc, 0x13_uc, 0x0A_uc};
    BitArray<23, unsigned char> bx = ba;

    for(size_t i = 0; i < 23; ++i) {
        EXPECT_EQ(ba == (bx<<i), i==0) << i;
        EXPECT_EQ(ba != (bx<<i), i!=0) << i;
    }
}

TEST(TestBitArray, Test)
{
    constexpr BitArray<24, unsigned char> ba{0xff_uc, 0x13_uc, 0x0A_uc};
    // ba = 0x0A13FF

    // F
    EXPECT_EQ(ba.test(0x00), true);
    EXPECT_EQ(ba.test(0x01), true);
    EXPECT_EQ(ba.test(0x02), true);
    EXPECT_EQ(ba.test(0x03), true);
    // F
    EXPECT_EQ(ba.test(0x04), true);
    EXPECT_EQ(ba.test(0x05), true);
    EXPECT_EQ(ba.test(0x06), true);
    EXPECT_EQ(ba.test(0x07), true);
    // 3
    EXPECT_EQ(ba.test(0x08), true);
    EXPECT_EQ(ba.test(0x09), true);
    EXPECT_EQ(ba.test(0x0A), false);
    EXPECT_EQ(ba.test(0x0B), false);
    // 1
    EXPECT_EQ(ba.test(0x0C), true);
    EXPECT_EQ(ba.test(0x0D), false);
    EXPECT_EQ(ba.test(0x0E), false);
    EXPECT_EQ(ba.test(0x0F), false);
    // A
    EXPECT_EQ(ba.test(0x10), false);
    EXPECT_EQ(ba.test(0x11), true);
    EXPECT_EQ(ba.test(0x12), false);
    EXPECT_EQ(ba.test(0x13), true);
    // 0
    EXPECT_EQ(ba.test(0x14), false);
    EXPECT_EQ(ba.test(0x15), false);
    EXPECT_EQ(ba.test(0x16), false);
    EXPECT_EQ(ba.test(0x17), false);
}

TEST(TestBitArray, Set)
{
    constexpr BitArray<16, unsigned char> ba0 = {0x13_uc, 0x0A_uc};
    BitArray<16, unsigned char> ba{};

    // 3
    ba.set(0x00, true);
    ba.set(0x01 /*, true*/);
    ba.set(0x02, false);
    ba.set(0x03, false);
    // 1
    ba.set(0x04, true);
    ba.set(0x05, false);
    ba.set(0x06, false);
    ba.set(0x07, false);
    // A
    ba.set(0x08, false);
    ba.set(0x09, true);
    ba.set(0x0A, false);
    ba.set(0x0B, true);
    // 0
    ba.set(0x0C, false);
    ba.set(0x0D, false);
    ba.set(0x0E, false);
    ba.set(0x0F, false);

    EXPECT_EQ(ba0, ba);
}

TEST(TestBitArray, BitArithmetic)
{
    constexpr BitArray<16, unsigned char> ba1 = {0x13_uc, 0x0A_uc};
    constexpr BitArray<16, unsigned char> ba2 = {0xFF_uc, 0x13_uc};

    // And
    constexpr auto ba_and = ba1 & ba2;
    EXPECT_EQ(ba_and, (BitArray<16, unsigned char>{0x13_uc, 0x02_uc}));

    // Or
    constexpr auto ba_or = ba1 | ba2;
    EXPECT_EQ(ba_or, (BitArray<16, unsigned char>{0xFF_uc, 0x1B_uc}));

    // Xor
    constexpr auto ba_xor = ba1 ^ ba2;
    EXPECT_EQ(ba_xor, (BitArray<16, unsigned char>{0xEC_uc, 0x19_uc}));
}

TEST(TestBitArray, Shifts)
{
    constexpr BitArray<23, unsigned char> ba(0xff_uc, 0x13_uc, 0x0A_uc);

    constexpr auto bal1 = ba << 3;
    EXPECT_EQ(bal1, (BitArray<23, unsigned char>{0xF8_uc, 0x9F_uc, 0x50_uc}));

    constexpr auto bal2 = ba << 11;
    EXPECT_EQ(bal2, (BitArray<23, unsigned char>{0x00_uc, 0xF8_uc, 0x1F_uc}));

    constexpr auto bal3 = ba << 19;
    EXPECT_EQ(bal3, (BitArray<23, unsigned char>{0x00_uc, 0x00_uc, 0x78_uc}));

    constexpr auto bar1 = ba >> 3;
    EXPECT_EQ(bar1, (BitArray<23, unsigned char>{0x7F_uc, 0x42_uc, 0x01_uc}));

    constexpr auto bar2 = ba >> 11;
    EXPECT_EQ(bar2, (BitArray<23, unsigned char>{0x42_uc, 0x01_uc, 0x00_uc}));

    constexpr auto bar3 = ba >> 19;
    EXPECT_EQ(bar3, (BitArray<23, unsigned char>{0x01_uc}));

    constexpr auto balr = (ba << 8) >> 8;
    EXPECT_EQ(balr, (BitArray<23, unsigned char>{0xff_uc, 0x13_uc, 0x00_uc}));

    constexpr auto barl = (ba >> 8) << 8;
    EXPECT_EQ(barl, (BitArray<23, unsigned char>{0x00_uc, 0x13_uc, 0x0A_uc}));
}

TEST(TestBitArray, PopCount)
{
    constexpr BitArray<23, unsigned char> ba(0xff_uc, 0x13_uc, 0x0A_uc);
    constexpr BitArray<51, unsigned char> bb = ba;

    EXPECT_EQ((BitArray<73, unsigned int>{}.popCount()), 0);
    EXPECT_EQ(ba.popCount(), 13);
    EXPECT_EQ((bb | bb << 23).popCount(), 26);
}

TEST(TestBitArray, Slice)
{
    constexpr BitArray<23, unsigned char> ba(0xff_uc, 0x13_uc, 0x0A_uc);
    constexpr BitArray<51, unsigned char> bb = ba;

    for(size_t i = 0; i < 51; ++i) {
        EXPECT_EQ(bb.slice<23>(i), ba >> i);
    }
}

TEST(TestBitArray, LowHighCuts)
{
    constexpr BitArray<24, unsigned char> ba(0xff_uc, 0x13_uc, 0x0A_uc);
    for(size_t i = 0; i <= 24; ++i) {
        /* High-cut test */
        // F
        EXPECT_EQ(ba.lowpass(i).test(0x00), i > 0x00);
        EXPECT_EQ(ba.lowpass(i).test(0x01), i > 0x01);
        EXPECT_EQ(ba.lowpass(i).test(0x02), i > 0x02);
        EXPECT_EQ(ba.lowpass(i).test(0x03), i > 0x03);
        // F
        EXPECT_EQ(ba.lowpass(i).test(0x04), i > 0x04);
        EXPECT_EQ(ba.lowpass(i).test(0x05), i > 0x05);
        EXPECT_EQ(ba.lowpass(i).test(0x06), i > 0x06);
        EXPECT_EQ(ba.lowpass(i).test(0x07), i > 0x07);
        // 3
        EXPECT_EQ(ba.lowpass(i).test(0x08), i > 0x08);
        EXPECT_EQ(ba.lowpass(i).test(0x09), i > 0x09);
        EXPECT_EQ(ba.lowpass(i).test(0x0A), false);
        EXPECT_EQ(ba.lowpass(i).test(0x0B), false);
        // 1
        EXPECT_EQ(ba.lowpass(i).test(0x0C), i > 0x0C);
        EXPECT_EQ(ba.lowpass(i).test(0x0D), false);
        EXPECT_EQ(ba.lowpass(i).test(0x0E), false);
        EXPECT_EQ(ba.lowpass(i).test(0x0F), false);
        // A
        EXPECT_EQ(ba.lowpass(i).test(0x10), false);
        EXPECT_EQ(ba.lowpass(i).test(0x11), i > 0x11);
        EXPECT_EQ(ba.lowpass(i).test(0x12), false);
        EXPECT_EQ(ba.lowpass(i).test(0x13), i > 0x13);

        /* Low-cut test */
        // F
        EXPECT_EQ(ba.lowcut(i).test(0x00), i <= 0x00);
        EXPECT_EQ(ba.lowcut(i).test(0x01), i <= 0x01);
        EXPECT_EQ(ba.lowcut(i).test(0x02), i <= 0x02);
        EXPECT_EQ(ba.lowcut(i).test(0x03), i <= 0x03);
        // F
        EXPECT_EQ(ba.lowcut(i).test(0x04), i <= 0x04);
        EXPECT_EQ(ba.lowcut(i).test(0x05), i <= 0x05);
        EXPECT_EQ(ba.lowcut(i).test(0x06), i <= 0x06);
        EXPECT_EQ(ba.lowcut(i).test(0x07), i <= 0x07);
        // 3
        EXPECT_EQ(ba.lowcut(i).test(0x08), i <= 0x08);
        EXPECT_EQ(ba.lowcut(i).test(0x09), i <= 0x09);
        EXPECT_EQ(ba.lowcut(i).test(0x0A), false);
        EXPECT_EQ(ba.lowcut(i).test(0x0B), false);
        // 1
        EXPECT_EQ(ba.lowcut(i).test(0x0C), i <= 0x0C);
        EXPECT_EQ(ba.lowcut(i).test(0x0D), false);
        EXPECT_EQ(ba.lowcut(i).test(0x0E), false);
        EXPECT_EQ(ba.lowcut(i).test(0x0F), false);
        // A
        EXPECT_EQ(ba.lowcut(i).test(0x10), false);
        EXPECT_EQ(ba.lowcut(i).test(0x11), i <= 0x11);
        EXPECT_EQ(ba.lowcut(i).test(0x12), false);
        EXPECT_EQ(ba.lowcut(i).test(0x13), i <= 0x13);
    }
}

TEST(TestBitArray, Replace)
{
    constexpr BitArray<23, unsigned char> ba(0b10101010_uc, 0b10101010_uc, 0b10101010_uc);
    constexpr BitArray<7, unsigned char> bb{0b1000001_uc};

    for(size_t i = 0; i < 6; ++i) {
        BitArray<23,unsigned char> bx = ba;

        bx.replace(i, bb);

        EXPECT_EQ(bx.lowpass(i), ba.lowpass(i)) << i;
        EXPECT_EQ(bx.slice<7>(i), bb) << i;
        EXPECT_EQ(bx.lowcut(i+7), ba.lowcut(i+7)) << i;
    }
}

