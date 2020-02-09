#include <gtest/gtest.h>

#include "WiredTable.hpp"

TEST(WiredTable, AppendAndAccess)
{
    WiredTable<int> tbl;

    auto idx1 = tbl.append(3);
    auto idx2 = tbl.append(1);
    EXPECT_EQ(tbl.get(idx1), 3);
    EXPECT_EQ(tbl.get(idx2), 1);
    tbl.get(idx1) = 5;
    EXPECT_EQ(tbl.get(idx1), 5);
    EXPECT_EQ(tbl.get(idx2), 1);

    WiredTable<int> const& tbl_ref = tbl;
    EXPECT_EQ(tbl_ref.get(idx1), 5);
    EXPECT_EQ(tbl_ref.get(idx2), 1);
}

TEST(WiredTable, Remove)
{
    WiredTable<int> tbl;

    auto idx1 = tbl.append(3);
    auto idx2 = tbl.append(1);
    auto idx3 = tbl.append(4);

    EXPECT_TRUE(tbl.hasVertexOf(idx1));
    EXPECT_TRUE(tbl.hasVertexOf(idx2));
    EXPECT_TRUE(tbl.hasVertexOf(idx3));

    EXPECT_TRUE((static_cast<bool>(idx2)));
    tbl.remove(idx2);
    EXPECT_FALSE((static_cast<bool>(idx2)));

    EXPECT_EQ(tbl.get(idx1), 3);
    EXPECT_EQ(tbl.get(idx3), 4);

    EXPECT_TRUE(tbl.hasVertexOf(idx1));
    EXPECT_FALSE(tbl.hasVertexOf(idx2));
    EXPECT_TRUE(tbl.hasVertexOf(idx3));
}

TEST(WiredTable, ForEach)
{
    WiredTable<int> tbl;

    auto idx1 = tbl.append(3);
    auto idx2 = tbl.append(1);
    auto idx3 = tbl.append(4);

    int result = 0;
    tbl.foreach(
        [&result](int n, WiredTable<int>::IndexType) {
            result += n;
        } );

    EXPECT_EQ(result, 8);

    tbl.foreach(
        [&](int& n, WiredTable<int>::IndexType) {
            ++n;
        } );

    EXPECT_EQ(tbl.get(idx1), 4);
    EXPECT_EQ(tbl.get(idx2), 2);
    EXPECT_EQ(tbl.get(idx3), 5);    
}

TEST(WiredTable, FindIf)
{
    WiredTable<int> tbl;

    WiredTable<int>::IndexType idx[] = {
        tbl.append(3),
        tbl.append(1),
        tbl.append(4),
        tbl.append(1),
        tbl.append(5),
        tbl.append(9),
        tbl.append(2)
    };

    int n = 0;
    auto idx1 = tbl.find_if(
        [&n](int r) {
            return (n += r) >= 9;
        } );

    ASSERT_TRUE((static_cast<bool>(idx1)));
    EXPECT_EQ(idx1, idx[3]);
    EXPECT_EQ(tbl.get(idx1), 1);

    auto idx2 = tbl.find_if( [](int r) { return r == 7; } );
    EXPECT_FALSE((static_cast<bool>(idx2)));

    auto idces = tbl.find_all_if( [](int r){ return r%2 == 0;} );
    ASSERT_EQ(idces.size(), 2);
    EXPECT_EQ(idces[0], idx[2]);
    EXPECT_EQ(idces[1], idx[6]);
}

TEST(WiredTable, Connection)
{
    WiredTable<int> tbl;

    WiredTable<int>::IndexType idx[] = {
        tbl.append(3),
        tbl.append(1),
        tbl.append(4),
        tbl.append(1),
        tbl.append(5),
        tbl.append(9),
        tbl.append(2)
    };

    // Connect i j if
    //  - j appears later than i;
    //  - the value of j is greater than i.
    tbl.connect(idx[0], idx[2]);
    tbl.connect(idx[0], idx[4]);
    tbl.connect(idx[0], idx[5]);
    tbl.connect(idx[1], idx[2]);
    tbl.connect(idx[1], idx[4]);
    tbl.connect(idx[1], idx[5]);
    tbl.connect(idx[1], idx[6]);
    tbl.connect(idx[2], idx[4]);
    tbl.connect(idx[2], idx[5]);
    tbl.connect(idx[3], idx[4]);
    tbl.connect(idx[3], idx[5]);
    tbl.connect(idx[3], idx[6]);
    tbl.connect(idx[4], idx[5]);

    ASSERT_EQ(tbl.getNexts(idx[0]).size(), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[0])[0]), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[0])[1]), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[0])[2]), 9);

    ASSERT_EQ(tbl.getNexts(idx[1]).size(), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[1])[0]), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[1])[1]), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[1])[2]), 9);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[1])[3]), 2);

    ASSERT_EQ(tbl.getNexts(idx[2]).size(), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[2])[0]), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[2])[1]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[2])[2]), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[2])[3]), 9);

    ASSERT_EQ(tbl.getNexts(idx[3]).size(), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[3])[0]), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[3])[1]), 9);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[3])[2]), 2);

    ASSERT_EQ(tbl.getNexts(idx[4]).size(), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[0]), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[1]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[2]), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[3]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[4]), 9);

    ASSERT_EQ(tbl.getNexts(idx[5]).size(), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[0]), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[1]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[2]), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[3]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[4]), 5);

    ASSERT_EQ(tbl.getNexts(idx[6]).size(), 2);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[6])[0]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[6])[1]), 1);
}

TEST(WiredTable, ConnectRemove)
{
    WiredTable<int> tbl;

    WiredTable<int>::IndexType idx[] = {
        tbl.append(3),
        tbl.append(1),
        tbl.append(4),
        tbl.append(1),
        tbl.append(5),
        tbl.append(9),
        tbl.append(2)
    };

    // Connect i j if
    //  - j appears later than i;
    //  - the value of j is greater than i.
    tbl.connect(idx[0], idx[2]);
    tbl.connect(idx[0], idx[4]);
    tbl.connect(idx[0], idx[5]);
    tbl.connect(idx[1], idx[2]);
    tbl.connect(idx[1], idx[4]);
    tbl.connect(idx[1], idx[5]);
    tbl.connect(idx[1], idx[6]);
    tbl.connect(idx[2], idx[4]);
    tbl.connect(idx[2], idx[5]);
    tbl.connect(idx[3], idx[4]);
    tbl.connect(idx[3], idx[5]);
    tbl.connect(idx[3], idx[6]);
    tbl.connect(idx[4], idx[5]);

    tbl.remove(idx[2]);

    ASSERT_EQ(tbl.getNexts(idx[0]).size(), 2);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[0])[0]), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[0])[1]), 9);

    ASSERT_EQ(tbl.getNexts(idx[1]).size(), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[1])[0]), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[1])[1]), 9);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[1])[2]), 2);

    ASSERT_EQ(tbl.getNexts(idx[3]).size(), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[3])[0]), 5);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[3])[1]), 9);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[3])[2]), 2);

    ASSERT_EQ(tbl.getNexts(idx[4]).size(), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[0]), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[1]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[2]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[4])[3]), 9);

    ASSERT_EQ(tbl.getNexts(idx[5]).size(), 4);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[0]), 3);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[1]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[2]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[5])[3]), 5);

    ASSERT_EQ(tbl.getNexts(idx[6]).size(), 2);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[6])[0]), 1);
    EXPECT_EQ(tbl.get(tbl.getNexts(idx[6])[1]), 1);
}

TEST(WiredTable, Paths)
{
    WiredTable<int> tbl;

    WiredTable<int>::IndexType idx[] = {
        tbl.append(3),
        tbl.append(1),
        tbl.append(4),
        tbl.append(1),
        tbl.append(5)
    };

    /* Connect as follows:
     *  3
     *  |\
     *  4-5
     *  |/|
     *  1 1
     */
    tbl.connect(idx[0], idx[2]);
    tbl.connect(idx[0], idx[4]);
    tbl.connect(idx[1], idx[2]);
    tbl.connect(idx[1], idx[4]);
    tbl.connect(idx[2], idx[4]);
    tbl.connect(idx[3], idx[4]);

    // Compute maximal simple paths beginning from the left bottom 1 in the picture above.
    auto paths = tbl.maxSimplePaths(idx[1]);
    std::vector<std::vector<int>> shouldbes {
        {1, 5, 1},
        {1, 5, 3, 4},
        {1, 5, 4, 3},
        {1, 4, 5, 1},
        {1, 4, 5, 3},
        {1, 4, 3, 5, 1}
    };

    EXPECT_EQ(paths.size(), shouldbes.size());
    for(auto shouldbe : shouldbes) {
        auto itr = std::find(paths.begin(), paths.end(), shouldbe);
        EXPECT_TRUE(itr != paths.end());
    }
}
