#include <gtest/gtest.h>

#include "utils.hpp"

#include <memory>

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

TEST(TestTraits, AllTrue)
{
    constexpr auto result1 = allTrue({false});
    EXPECT_FALSE(result1);

    constexpr auto result2 = allTrue({true});
    EXPECT_TRUE(result2);

    constexpr auto result3 = allTrue({false, true, true});
    EXPECT_FALSE(result3);

    for(unsigned char i = 0; i <= 0xF; ++i) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
        auto result3 = allTrue({i&0x1, i&0x2, i&0x4, i&0x8});
#pragma GCC diagnostic pop
        EXPECT_EQ(result3, i == 0xF);
    }
}

TEST(TestUtilFunction, BitWave)
{
    constexpr auto wave = bitwave<uint16_t>(3);
    EXPECT_EQ(wave, static_cast<uint16_t>(0b0111000111000111));

    EXPECT_EQ(bitwave<uint32_t>(1), static_cast<uint32_t>(0x55555555));
    EXPECT_EQ(bitwave<uint32_t>(2), static_cast<uint32_t>(0x33333333));
    EXPECT_EQ(bitwave<uint32_t>(4), static_cast<uint32_t>(0x0f0f0f0f));
    EXPECT_EQ(bitwave<uint32_t>(8), static_cast<uint32_t>(0x00ff00ff));
    EXPECT_EQ(bitwave<uint32_t>(16), static_cast<uint32_t>(0x0000ffff));
}

TEST(TestUtilFunction, PopCount)
{
    constexpr auto cnt1 = popcount(0x1u);
    constexpr auto cnt2 = popcount(0b1011001110001111u);

    EXPECT_EQ(cnt1, 1u);
    EXPECT_EQ(cnt2, 10u);
}

TEST(TestUtilFunction, CountTrailing0)
{
    constexpr auto cnt1 = counttrail0(0b0u);
    constexpr auto cnt2 = counttrail0(0b1011001110001111u);
    constexpr auto cnt3 = counttrail0(0b100100101000u);

    EXPECT_EQ(cnt1, 8*sizeof(unsigned int));
    EXPECT_EQ(cnt2, 0u);
    EXPECT_EQ(cnt3, 3u);

    // counting does not change the original variable.
    constexpr unsigned int x0 = 0b101100111000u;
    auto x = x0;
    EXPECT_EQ(counttrail0(x), 3u);
    EXPECT_EQ(x, x0);
}

TEST(TestUtilFunction, CountTrailing1)
{
    constexpr auto cnt1 = counttrail1(0b1u);
    constexpr auto cnt2 = counttrail1(0b1011001110001111u);
    constexpr auto cnt3 = counttrail1(0b100100101000u);

    EXPECT_EQ(cnt1, 1u);
    EXPECT_EQ(cnt2, 4u);
    EXPECT_EQ(cnt3, 0u);

    // counting does not change the original variable.
    constexpr unsigned int x0 = 0b1011001110001111u;
    auto x = x0;
    EXPECT_EQ(counttrail1(x), 4u);
    EXPECT_EQ(x, x0);
}

TEST(TestUtilFunction, IntegerPower)
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

TEST(TestUtilFunction, FloatPower)
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

TEST(TestUtilFunction, StringLength)
{
    constexpr auto len0 = strlength("Hello, World!");
    constexpr char tststr[] = "Hello, World!\0Bye!";
    constexpr auto len1 = strlength(tststr);
    constexpr auto len2 = strlength("Hello, World!\0Bye!");

    EXPECT_EQ(len0, len1);
    EXPECT_EQ(len0, len2);
}

template <class T, class U>
inline constexpr T gen_plus(T lhs, U const& rhs)
{
    lhs += rhs;
    return lhs;
}

TEST(TestUtilFunction, AccumMap)
{
    // Simple arithmetics for integers.
    struct Lambda {
        constexpr int operator()(int x) { return x+1; }
    };
    constexpr int nums[] = {1, 1, 2, 3, 5, 8, 13, 21, 34};
    constexpr int res = accumMap(nums, nums+9, Lambda());
    constexpr int resmul = accumMap(nums, nums+9, Lambda(), 1, std::multiplies<int>());
    int init = 2;
    int resplus = accumMap(nums, nums+9, Lambda(), init, std::plus<int>());

    EXPECT_EQ(res, 97);
    EXPECT_EQ(resmul, 27941760);
    EXPECT_EQ(resplus, 99);
    EXPECT_EQ(init, 2);

    // Values with counting copies.
    class Val {
    private:
        int m_val;
        std::shared_ptr<int> m_constrnum;
        std::shared_ptr<int> m_assignnum;

    public:
        Val(int val = 0)
            : m_val(val),
              m_constrnum{std::make_shared<int>(0)},
              m_assignnum{std::make_shared<int>(0)} {}
        Val(Val const& src)
            : m_val(src.m_val),
              m_constrnum(src.m_constrnum),
              m_assignnum(src.m_assignnum)
        { ++(*m_constrnum); }
        Val(Val && src)
            : m_val(src.m_val),
              m_constrnum(std::move(src.m_constrnum)),
              m_assignnum(std::move(src.m_assignnum))
        {}
        int get() const { return m_val; }
        int countConstr() const { return *m_constrnum; }
        int countAssign() const { return *m_assignnum; }
        Val& operator=(Val const& lhs) {
            // std::cout << m_val << " -> " << lhs.m_val << std::endl;
            m_val = lhs.m_val;
            m_constrnum = lhs.m_constrnum;
            m_assignnum = lhs.m_assignnum;
            ++(*m_assignnum);
            return *this;
        }
        Val& operator=(Val && lhs) {
            // std::cout << m_val << " -> " << lhs.m_val << std::endl;
            m_val = lhs.m_val;
            m_constrnum = lhs.m_constrnum;
            m_assignnum = lhs.m_assignnum;
            return *this;
        }
        Val& operator+=(Val const& other) {
            m_val += other.m_val;
            return *this;
        }
        bool operator==(Val const& other) const { return m_val == other.m_val; }
        bool operator!=(Val const& other) const { return m_val != other.m_val; }
    };

    Val vnums[] = {1, 1, 2, 3, 5, 8, 13, 21, 34};
    Val vres = accumMap(vnums, vnums+9, [](Val val){ return val; }, Val{0}, gen_plus<Val,Val>);
    Val vres2 = accumMap(vnums, vnums+9, [](Val val){ return val; }, Val{0}, [](Val lhs, Val const& rhs) { lhs += rhs; return lhs; });
    Val x{2};
    Val vres3 = accumMap(vnums, vnums+9, [](Val val){ return val; }, x, [](Val lhs, Val const& rhs) { lhs += rhs; return lhs; });

    // Count of copy operations.
    // Copy should not happen as long as a move constructor and a move assignment operator are provided.
    EXPECT_EQ(vres, 88);
    EXPECT_EQ(vres.countConstr(), 0);
    EXPECT_EQ(vres.countAssign(), 0);
    EXPECT_EQ(vres2, 88);
    EXPECT_EQ(vres2.countConstr(), 0);
    EXPECT_EQ(vres2.countAssign(), 0);
    EXPECT_EQ(vres3, 90);
    EXPECT_EQ(vres3.countConstr(), 1);
    EXPECT_EQ(vres3.countAssign(), 0);
    EXPECT_EQ(x, 2);

    // Container accumulation
    std::vector<Val> daysm = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    Val daysy = accumMap(daysm.begin(), daysm.end(), [](Val val){return val;}, Val{0}, gen_plus<Val,Val>);
    EXPECT_EQ(daysy, 365);

    auto factors = [](Val n) -> std::vector<Val> {
        bool sign = n.get() < 0;
        int npos = sign ? -n.get() : n.get();
        std::vector<Val> result{};

        for(int i = 1; i <= npos/2; ++i)
            if (npos%i == 0)
                result.emplace_back(i);
        result.emplace_back(npos);
        return result;
    };
    auto concatVects = [](std::vector<Val> lhs, std::vector<Val> const& rhs) -> std::vector<Val> {
        lhs.insert(lhs.end(), rhs.begin(), rhs.end());
        return lhs;
    };
    std::vector<Val> dayfacts = accumMap(daysm.begin(), daysm.end(), factors, {}, concatVects);
    std::vector<Val> shouldbe = {
        1, 31, // 31
        1, 2, 4, 7, 14, 28, //28
        1, 31, //31
        1, 2, 3, 5, 6, 10, 15, 30, //30
        1, 31, //31
        1, 2, 3, 5, 6, 10, 15, 30, //30
        1, 31, //31
        1, 31, //31
        1, 2, 3, 5, 6, 10, 15, 30, //30
        1, 31, //31
        1, 2, 3, 5, 6, 10, 15, 30, //30
        1, 31, //31
    };
    ASSERT_EQ(dayfacts.size(), shouldbe.size());
    for(size_t i = 0; i < dayfacts.size(); ++i) {
        EXPECT_EQ(dayfacts[i].get(), shouldbe[i].get());
    }
    // How many do copy operations occur?
    // Since std::vector uses copy constructors for re-placement, they do but are less than the length of the array.
    EXPECT_LT(daysm[0].countConstr(), daysm.size());
    EXPECT_EQ(daysm[0].countAssign(), 0);
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
