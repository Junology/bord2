#include <gtest/gtest.h>

#include "utils.hpp"
#include "utils/BitArray.hpp"

//using namespace bord2;

inline constexpr unsigned char operator "" _uc(unsigned long long x) noexcept
{
    return static_cast< unsigned char >(x);
}

template <size_t n, class T, size_t... Is>
constexpr BitArray<n,T> diagBitArray(std::index_sequence<Is...>, T x)
{
    constexpr struct {
        constexpr T operator()(size_t k, T y) const {
            static_cast<void>(k);
            return y;
        }
    } f{};
    return BitArray<n,T>{f(Is,x)...};
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

    // Large size
    constexpr size_t hugenum = 1024;
    constexpr size_t hugebit = sizeof(int64_t)*8*hugenum;
    constexpr auto ba7 = diagBitArray<hugebit,uint64_t>(std::make_index_sequence<hugenum>(), 0x5555555555555555llu);
    EXPECT_EQ(sizeof(ba7), sizeof(int64_t)*hugenum);
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
    constexpr BitArray<73, uint64_t> bc(UINT64_C(0x5555555555555555), UINT64_C(0x5555555555555));
;

    EXPECT_EQ((BitArray<73, unsigned int>{}.popCount()), 0);
    EXPECT_EQ(ba.popCount(), 13);
    EXPECT_EQ((bb | bb << 23).popCount(), 26);
    EXPECT_EQ(bc.popCount(), 37lu);
}

TEST(TestBitArray, CountTrailing0)
{
    constexpr BitArray<73,unsigned int> ba = BitArray<73, unsigned int>{0x7u} << 23;
    constexpr BitArray<73, unsigned int> bb{};

    EXPECT_EQ(bb.countTrail0(), 73) << bb;
    EXPECT_EQ(ba.countTrail0(), 23);
    EXPECT_EQ((ba | ba >> 11).countTrail0(), 12);
}

TEST(TestBitArray, CountTrailing1)
{
    constexpr BitArray<23, unsigned char> ba(0xff_uc, 0x13_uc, 0x0A_uc);
    constexpr BitArray<51, unsigned char> bb = ba;

    EXPECT_EQ((BitArray<73, unsigned int>{}.countTrail1()), 0);
    EXPECT_EQ((BitArray<73, unsigned int>{0x7u}.countTrail1()), 3);
    EXPECT_EQ(ba.countTrail1(), 10);
    EXPECT_EQ((bb | bb << 2).countTrail1(), 13);
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

TEST(TestBitArray, Bool)
{
    constexpr BitArray<91,uint64_t> ba(0x5555555555555555u, 0x5555555555555u);

    EXPECT_TRUE(static_cast<bool>(ba));
    EXPECT_FALSE(static_cast<bool>(ba&(ba >> 1)));
}

TEST(TestBitArray, PopIterator)
{
    constexpr auto bits = BitArray<91,uint16_t>(
        0b1000010001001011u,
        0b0001000000100000u,
        0b0010000000010000u,
        0b0000000010000000u,
        0b0100000000000100u,
        0b0000100000000000u
        );


    std::vector<size_t> indices;

    std::for_each(
        bits.popBegin(), bits.popEnd(),
        [&indices](size_t i) {
            indices.push_back(i);
        } );
    ASSERT_EQ(indices.size(), 13);
    EXPECT_EQ(indices[0], 0);
    EXPECT_EQ(indices[1], 1);
    EXPECT_EQ(indices[2], 3);
    EXPECT_EQ(indices[3], 6);
    EXPECT_EQ(indices[4], 10);
    EXPECT_EQ(indices[5], 15);
    EXPECT_EQ(indices[6], 21);
    EXPECT_EQ(indices[7], 28);
    EXPECT_EQ(indices[8], 36);
    EXPECT_EQ(indices[9], 45);
    EXPECT_EQ(indices[10], 55);
    EXPECT_EQ(indices[11], 66);
    EXPECT_EQ(indices[12], 78);

    constexpr auto bits2 = BitArray<64,uint64_t>(0x3);

    indices.clear();
    std::for_each(
        bits2.popBegin(), bits2.popEnd(),
        [&indices](size_t i) {
            indices.push_back(i);
        } );
    ASSERT_EQ(indices.size(), 2);
    EXPECT_EQ(indices[0], 0);
    EXPECT_EQ(indices[1], 1);

    constexpr auto bits3 = BitArray<64,uint16_t>(0x3);

    indices.clear();
    std::for_each(
        bits3.popBegin(), bits3.popEnd(),
        [&indices](size_t i) {
            indices.push_back(i);
        } );
    ASSERT_EQ(indices.size(), 2);
    EXPECT_EQ(indices[0], 0);
    EXPECT_EQ(indices[1], 1);

    constexpr auto bits4 = BitArray<64,uint16_t>(0x3, 0x0, 0x3);

    indices.clear();
    std::for_each(
        bits4.popBegin(), bits4.popEnd(),
        [&indices](size_t i) {
            indices.push_back(i);
        } );
    ASSERT_EQ(indices.size(), 4);
    EXPECT_EQ(indices[0], 0);
    EXPECT_EQ(indices[1], 1);
    EXPECT_EQ(indices[2], 32);
    EXPECT_EQ(indices[3], 33);

    constexpr auto bits5 = BitArray<64,uint16_t>(0x0, 0x0, 0x3);

    indices.clear();
    std::for_each(
        bits5.popBegin(), bits5.popEnd(),
        [&indices](size_t i) {
            indices.push_back(i);
        } );
    ASSERT_EQ(indices.size(), 2);
    EXPECT_EQ(indices[0], 32);
    EXPECT_EQ(indices[1], 33);

    constexpr auto bits6 = BitArray<64>(0x3);

    indices.clear();
    std::for_each(
        bits6.popBegin(), bits6.popEnd(),
        [&indices](size_t i) {
            indices.push_back(i);
        } );
    ASSERT_EQ(indices.size(), 2);
    EXPECT_EQ(indices[0], 0);
    EXPECT_EQ(indices[1], 1);

    // Constexpr test
    struct TestFunc {
        constexpr size_t operator()(BitArray<91,uint16_t> const& src) const noexcept
        {
            size_t result = 0;
            auto beg = src.popBegin(), end = src.popEnd();
            for(auto itr = beg; itr != end; ++itr) {
                result += *itr;
            }
            return result;
        }
    };
    constexpr TestFunc sumf{};
    constexpr auto tot = sumf(bits);
    EXPECT_EQ(tot, 364);
}

//* An application: Sieve of Eratosthenes
TEST(TestBitArray, Primes)
{
    constexpr size_t max_chunk = 1024;
    constexpr size_t max_num = 8*sizeof(uint_fast64_t)*max_chunk;
    auto tbl = diagBitArray<max_num, uint_fast64_t>(std::make_index_sequence<max_chunk>(), 0x5555555555555555llu) << 3;
    std::vector<size_t> primes{2};

    // Compute all the primes less than max_num
    while(tbl) {
        size_t prime = tbl.countTrail0();
        primes.push_back(prime);
        for(size_t i = prime; i < max_num; i += 2*prime)
            tbl.set(i, false);
    }

    size_t known[] = {
        2, 3, 5, 7, 11, 13, 17, 19, 23, 29,
        31, 37, 41, 43, 47, 53, 59, 61, 67, 71,
        73, 79, 83, 89, 97, 101, 103, 107, 109, 113,
        127, 131, 137, 139, 149, 151, 157, 163, 167, 173,
        179, 181, 191, 193, 197, 199, 211, 223, 227, 229,
        233, 239, 241, 251, 257, 263, 269, 271
    };

    ASSERT_GT(primes.size(), std::distance(std::begin(known), std::end(known)));
    EXPECT_TRUE(std::equal(std::begin(known), std::end(known), primes.begin()));

    // Check if prime
    constexpr struct {
        constexpr bool operator()(size_t n) const noexcept {
            if (n==0 || n==1) return false;

            for(size_t i = 2; i*i <= n; ++i)
                if (n%i == 0) return false;

            return true;
        }
    } is_prime{};

    for(auto n : primes)
        EXPECT_TRUE(is_prime(n)) << n;
}
// */
