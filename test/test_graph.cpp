#include <gtest/gtest.h>

#include "Graph.hpp"

TEST(Graph, Misc)
{
    EXPECT_EQ((Graph<91>::MaxVertices), 91);
    EXPECT_EQ((Graph<91>::MaxEdges), 4095 /* = 91*90/2 */);
}

TEST(Graph, ConstructAndAdd)
{
    Graph<30> graph0;

    EXPECT_FALSE(graph0.inhabited());

    for(size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(graph0.append().second, i);
    }

    EXPECT_TRUE(graph0.inhabited());
    EXPECT_EQ(graph0.size(), 5);

    for(size_t i = 0; i < 5; ++i) {
        EXPECT_TRUE(graph0.isVertex(i));
    }

    for(size_t i = 0; i < 30; ++i) {
        auto graph = graph0;

        EXPECT_EQ(graph.isVertex(i), i < 5);
        EXPECT_EQ(graph.addAt(i), i >= 5);
        EXPECT_TRUE(graph.isVertex(i));
    }

    constexpr Graph<30> graph1(19);

    EXPECT_EQ(graph1.size(), 19);

    for(size_t i = 0; i < 30; ++i) {
        EXPECT_EQ(graph1.isVertex(i), i < 19);
    }

    constexpr BitArray<73> ba{0x5555555555555555lu, 0x5555555555555555lu};
    constexpr Graph<91> graph2(ba);

    EXPECT_TRUE(graph2.inhabited());
    EXPECT_EQ(ba.popCount(), 37);
    EXPECT_EQ(graph2.size(), 37);

    for(size_t i = 0; i < 91; ++i) {
        EXPECT_EQ(graph2.isVertex(i), i%2 == 0 && i < 73) << i;
    }

    constexpr Graph<91> graph3(~BitArray<91>{});
    constexpr Graph<91> graph4(91);
    EXPECT_EQ(graph3, graph4);
}

TEST(Graph, EdgeConnection)
{
    Graph<51> graph(~BitArray<51>{});

    for(size_t i = 0; i < 51; ++i)
        EXPECT_FALSE(graph.connect(i,i));

    for(size_t i = 0; i < 50; ++i) {
        graph.connect(i, i+1);
    }

    for(size_t i = 0; i < 50; ++i) {
        EXPECT_FALSE(graph.isConnected(i, i)) << i;
        EXPECT_TRUE(graph.isConnected(i, i+1)) << i;
        EXPECT_TRUE(graph.isConnected(i+1, i)) << i;
    }

    for(size_t i = 0; i < 50; i += 2) {
        graph.disconnect(i, i+1);
    }

    for(size_t i = 0; i < 50; ++i) {
        EXPECT_EQ(graph.isConnected(i, i+1), i%2 != 0) << i;
        EXPECT_EQ(graph.isConnected(i+1, i), i%2 != 0) << i;
    }

    for(size_t i = 0; i < 50; i += 3) {
        EXPECT_EQ(graph.disconnect(i, i+1), i%2 != 0) << i;
    }

    for(size_t i = 0; i < 50; ++i) {
        EXPECT_EQ(graph.isConnected(i, i+1), i%2 != 0 && i%3 != 0) << i;
        EXPECT_EQ(graph.isConnected(i+1, i), i%2 != 0 && i%3 != 0) << i;
    }
}

TEST(Graph, Remove)
{
    // simple remove
    Graph<31> graph(31);

    for(size_t i = 0; i < 31; ++i) {
        if (i%2  == 0) {
            graph.remove(i);
        }
    }

    EXPECT_EQ(graph, Graph<31>(BitArray<31>{0x55555555u} << 1));

    for(size_t i = 0; i < 31; ++i) {
        if (i%3 == 0) {
            EXPECT_EQ(graph.remove(i), i%2 != 0);
        }
    }

    // remove and edges
    Graph<31> graph1(31);

    for(size_t i = 0; i < 31; ++i) {
        for(size_t j = 1; j <= i; ++j)
            graph1.connect(i, i+j);
    }

    for(size_t i = 0; i < 31; ++i) {
        for(size_t j = i; j < 31; ++j) {
            EXPECT_EQ(graph1.isConnected(i, j), 0 < j-i && j-i <= i);
        }
    }

    EXPECT_TRUE(graph1.remove(16));

    for(size_t i = 0; i < 31; ++i) {
        for(size_t j = i; j < 31; ++j) {
            EXPECT_EQ(graph1.isConnected(i, j), i != 16 && j != 16 && 0 < j-i && j-i <= i);
        }
    }
}

TEST(Graph, ForEach)
{
    constexpr size_t smpl = 256;
    Graph<smpl> graph(smpl);

    for(size_t i = 0; i < smpl; i+=2) {
        graph.remove(i);
    }

    for(size_t i = 0; i < smpl; i += 3) {
        graph.remove(i);
    }

    graph.forEachVertex(
        [&](size_t i){
            EXPECT_TRUE(i%2 != 0 && i%3 != 0);
        });
}

TEST(Graph, Components)
{
    Graph<256> graph(15);

    /*
     *  0-10
     *   / \
     *  4-3-9
     *  |
     *  5-13
     *   \|
     *    7
     */
    graph.connect(0,10);
    graph.connect(10,9);
    graph.connect(9,3);
    graph.connect(3,4);
    graph.connect(4,10);
    graph.connect(4,5);
    graph.connect(5,13);
    graph.connect(13,7);
    graph.connect(7,5);

    /*
     * 8-2-1
     *   |
     *   14
     */
    graph.connect(8,2);
    graph.connect(2,1);
    graph.connect(1,14);

    /*
     * 12
     *  |\
     * 11-6
     */
    graph.connect(12,6);
    graph.connect(6,11);

    std::vector<Graph<256>> components{};

    while(graph.inhabited()) {
        components.push_back(graph.trimComponent());
    }

    EXPECT_EQ(components.size(), 3);

    EXPECT_EQ(components[0].size(), 8);
    EXPECT_TRUE(components[0].isVertex(0));
    EXPECT_TRUE(components[0].isVertex(3));
    EXPECT_TRUE(components[0].isVertex(4));
    EXPECT_TRUE(components[0].isVertex(5));
    EXPECT_TRUE(components[0].isVertex(7));
    EXPECT_TRUE(components[0].isVertex(9));
    EXPECT_TRUE(components[0].isVertex(10));
    EXPECT_TRUE(components[0].isVertex(13));

    EXPECT_EQ(components[1].size(), 4);
    EXPECT_TRUE(components[1].isVertex(1));
    EXPECT_TRUE(components[1].isVertex(2));
    EXPECT_TRUE(components[1].isVertex(8));
    EXPECT_TRUE(components[1].isVertex(14));

    EXPECT_EQ(components[2].size(), 3);
    EXPECT_TRUE(components[2].isVertex(6));
    EXPECT_TRUE(components[2].isVertex(11));
    EXPECT_TRUE(components[2].isVertex(12));
}
