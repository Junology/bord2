#include <gtest/gtest.h>

#include "GenericGraph.hpp"

#include <map>

TEST(GenericGraph, ConstructAndAdd)
{
    // Test size
    constexpr size_t vertex_num = 10;

    // Simple set
    GenericGraph<std::set<size_t>> graph0;
    size_t keys0[vertex_num];

    EXPECT_FALSE(graph0.inhabited());

    for(size_t i = 0; i < vertex_num; ++i) {
        keys0[i] = graph0.append();
    }

    EXPECT_TRUE(graph0.inhabited());
    EXPECT_EQ(graph0.size(), vertex_num);

    for(size_t i = 0; i < vertex_num; ++i) {
        EXPECT_TRUE(graph0.isVertex(keys0[i]));
    }

    for(size_t i = 0; i < 2*vertex_num; ++i) {
        EXPECT_EQ(
            graph0.isVertex(i),
            std::find(std::begin(keys0), std::end(keys0), i) != std::end(keys0));
    }

    // Map
    GenericGraph<std::map<int,std::string>> graph1;
    int keys1[vertex_num];

    for(size_t i = 0; i < vertex_num; ++i) {
        keys1[i] = graph1.append(std::string(":") + std::to_string(i));
    }

    for(size_t i = 0; i < vertex_num; ++i) {
        EXPECT_TRUE(graph1.isVertex(keys1[i]));
        EXPECT_TRUE(
            graph1.at(keys1[i]) == std::string(":") + std::to_string(i));
    }

    for(size_t i = 0; i < 2*vertex_num; ++i) {
        EXPECT_EQ(
            graph1.isVertex(i),
            std::find(std::begin(keys1), std::end(keys1), i) != std::end(keys1));
    }

    EXPECT_EQ(graph1.at(3), ":3");
}

TEST(GenericGraph, EdgeConnection)
{
    // Test size
    constexpr size_t vertex_num = 10;

    // Simple set
    GenericGraph<std::set<size_t>> graph;
    size_t keys[vertex_num];

    for(size_t i = 0; i < vertex_num; ++i) {
        keys[i] = graph.append();
    }


    for(size_t i = 1; i < vertex_num; ++i) {
        graph.connect(keys[i-1], keys[i]);
    }

    for(size_t i = 1; i < vertex_num; ++i) {
        EXPECT_FALSE(graph.isConnected(keys[i], keys[i])) << i;
        EXPECT_TRUE(graph.isConnected(keys[i-1], keys[i])) << i;
        EXPECT_TRUE(graph.isConnected(keys[i], keys[i-1])) << i;
    }

    for(size_t i = 1; i < vertex_num; i += 2) {
        graph.disconnect(keys[i-1], keys[i]);
    }

    for(size_t i = 1; i < vertex_num; ++i) {
        EXPECT_EQ(graph.isConnected(keys[i-1], keys[i]), (i-1)%2 != 0) << i;
        EXPECT_EQ(graph.isConnected(keys[i], keys[i-1]), (i-1)%2 != 0) << i;
    }

    for(size_t i = 1; i < vertex_num; i += 3) {
        EXPECT_EQ(graph.disconnect(keys[i-1], keys[i]), (i-1)%2 != 0) << i;
    }

    for(size_t i = 1; i < vertex_num; ++i) {
        EXPECT_EQ(graph.isConnected(keys[i-1], keys[i]), (i-1)%2 != 0 && (i-1)%3 != 0) << i;
        EXPECT_EQ(graph.isConnected(keys[i], keys[i-1]), (i-1)%2 != 0 && (i-1)%3 != 0) << i;
    }
}

TEST(GenericGraph, Remove)
{
    // Test size
    constexpr size_t vertex_num = 10;

    // Simple set
    GenericGraph<std::set<size_t>> graph0;
    size_t keys0[vertex_num];

    for(size_t i = 0; i < vertex_num; ++i)
        keys0[i] = graph0.append();

    for(size_t i = 0; i < vertex_num; i+=2) {
            graph0.remove(keys0[i]);
    }

    for(size_t i = 0; i < vertex_num; ++i) {
        EXPECT_EQ(graph0.isVertex(keys0[i]), i%2 != 0);
    }

    // remove and edges
    GenericGraph<std::set<size_t>> graph1;
    size_t keys1[vertex_num];

    for(size_t i = 0; i < vertex_num; ++i)
        keys1[i] = graph1.append();

    for(size_t i = 0; i < vertex_num; ++i) {
        for(size_t j = i+1; j < std::min(2*i, vertex_num); ++j)
            graph1.connect(keys1[i], keys1[j]);
    }

    for(size_t i = 0; i < vertex_num; ++i) {
        for(size_t j = i+1; j < vertex_num; ++j) {
            EXPECT_EQ(graph1.isConnected(keys1[i], keys1[j]), j-i < i);
        }
    }

    EXPECT_TRUE(graph1.remove(vertex_num/2));
    keys1[vertex_num/2] = graph1.append();

    for(size_t i = 0; i < vertex_num; ++i) {
        for(size_t j = i+1; j < vertex_num; ++j) {
            EXPECT_EQ(graph1.isConnected(keys1[i], keys1[j]),
                      i != vertex_num/2 && j != vertex_num/2 && j-i < i)
                << i << j << std::endl;
        }
    }
}

TEST(GenericGraph, ForEach)
{
    constexpr size_t vertex_num = 10;

    GenericGraph<std::map<int,std::string>> graph;

    for(size_t i = 0; i < vertex_num; ++i) {
        graph.append(std::string(":") + std::to_string(i));
    }

    graph.forEachVertex(
        [&](std::pair<int, std::string> vertex){
            EXPECT_EQ(vertex.second,
                      std::string(":") + std::to_string(vertex.first));
        });

    graph.forEachVertex(
        [&graph](std::pair<int, std::string> vertex) {
            graph.at(vertex.first)[0] = '+';
        });

    graph.forEachVertex(
        [&](std::pair<int, std::string> vertex){
            EXPECT_EQ(vertex.second,
                      std::string("+") + std::to_string(vertex.first));
        });
}

TEST(GenericGraph, FindKey)
{
    constexpr size_t vertex_num = 1024;
    using key_type = int;

    GenericGraph<std::map<key_type, std::string>> graph;
    key_type keys[vertex_num];

    for(size_t i = 0; i < vertex_num; ++i) {
        std::ostringstream oss;
        oss << BitArray<10>(i);
        keys[i] = graph.append(oss.str());
    }

    auto kp = graph.findKey(
        [](std::pair<key_type, std::string> elem) -> bool {
            return elem.second[2] == '1';
        } );

    ASSERT_TRUE(kp);
    EXPECT_EQ(kp->first, keys[128]) << kp->second;

    auto kp2 = graph.findKey(
        [](std::pair<key_type, std::string> elem) -> bool {
            return elem.second.size() > 10;
        } );

    EXPECT_FALSE(kp2);
}

TEST(GenericGraph, Components)
{
    GenericGraph<std::set<size_t>> graph;

    for(size_t i = 0; i < 15; ++i)
        graph.append();

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

    std::vector<decltype(graph)> components{};

    while(graph.inhabited()) {
        auto key = graph.minkey();
        components.push_back(graph.trimComponent(key));
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

TEST(GenericGraph, ComponentsWithMap)
{
    GenericGraph<std::map<int,std::string>> graph;

    for(size_t i = 0; i < 15; ++i)
        graph.append(std::string("Hello, ") + std::to_string(i));

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

    std::vector<decltype(graph)> components{};

    while(graph.inhabited()) {
        auto key = graph.minkey();
        components.push_back(graph.trimComponent(key));
    }

    EXPECT_EQ(components.size(), 3);

    EXPECT_EQ(components[0].size(), 8);
    EXPECT_TRUE(components[0].isVertex(0));
    EXPECT_EQ(components[0].at(0), "Hello, 0");
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

TEST(GenericGraph, TrackPath)
{
    constexpr size_t vertex_num = 12;
    GenericGraph<std::set<int>> graph;
    int keys[vertex_num];

    for(size_t i = 0; i < vertex_num; ++i)
        keys[i] = graph.append();

    // 0-1-2-3  7-8
    //   | |\|  | |
    //   4-5-6  9-10
    graph.connect(keys[0], keys[1]);
    graph.connect(keys[1], keys[2]);
    graph.connect(keys[1], keys[4]);
    graph.connect(keys[2], keys[3]);
    graph.connect(keys[2], keys[5]);
    graph.connect(keys[2], keys[6]);
    graph.connect(keys[3], keys[6]);
    graph.connect(keys[4], keys[5]);
    graph.connect(keys[5], keys[6]);

    graph.connect(keys[7], keys[8]);
    graph.connect(keys[7], keys[9]);
    graph.connect(keys[8], keys[10]);
    graph.connect(keys[9], keys[10]);

    std::vector<int> path1{};
    // The path should be 0-1-2-3-4-6-5-4..1 (loop)
    auto next1 = graph.trackPath(
        keys[0],
        [&path1](int k) {
            path1.push_back(k);
        });

    EXPECT_TRUE(next1.first);
    EXPECT_EQ(next1.second, keys[1]);
    ASSERT_EQ(path1.size(), 7);
    EXPECT_EQ(path1[0], keys[0]);
    EXPECT_EQ(path1[1], keys[1]);
    EXPECT_EQ(path1[2], keys[2]);
    EXPECT_EQ(path1[3], keys[3]);
    EXPECT_EQ(path1[4], keys[6]);
    EXPECT_EQ(path1[5], keys[5]);
    EXPECT_EQ(path1[6], keys[4]);

    std::vector<int> path2{};
    // The path should be 6-2-1-0
    auto next2 = graph.trackPath(
        keys[6],
        [&path2](int k) {
            path2.push_back(k);
        });

    EXPECT_FALSE(next2.first);
    ASSERT_EQ(path2.size(), 4);
    EXPECT_EQ(path2[0], keys[6]);
    EXPECT_EQ(path2[1], keys[2]);
    EXPECT_EQ(path2[2], keys[1]);
    EXPECT_EQ(path2[3], keys[0]);

    std::vector<int> path3{};
    // The path should be 9-7-8-10..9 (loop)
    auto next3 = graph.trackPath(
        keys[9],
        [&path3](int k) {
            path3.push_back(k);
        });

    EXPECT_TRUE(next3.first);
    EXPECT_EQ(next3.second, keys[9]);
    ASSERT_EQ(path3.size(), 4);
    EXPECT_EQ(path3[0], keys[9]);
    EXPECT_EQ(path3[1], keys[7]);
    EXPECT_EQ(path3[2], keys[8]);
    EXPECT_EQ(path3[3], keys[10]);
}
