#include <gtest/gtest.h>

#include <vector>
#include "math/PlaneGeometry.hpp"

TEST(Bezier2D, HullFaceNone)
{
    // Nothing will happen for Bezier curve of degree 0 (i.e. just a point).
    auto hull1 = getConvexHull(
        std::vector<Eigen::Vector2d>{Eigen::Vector2d(0.0, 0.0)}
        );
    EXPECT_TRUE(hull1.empty());

    // Nothing will happen since there the convex hull is 0-dimensional.
    auto hull2 = getConvexHull(
        { Eigen::Vector2d(0.0, 0.0),
          Eigen::Vector2d(0.0, 0.0),
          Eigen::Vector2d(0.0, 0.0)
         } );
    EXPECT_TRUE(hull2.empty());
}

TEST(Bezier2D, HullFaceLinear)
{
    // Test for lines
    constexpr size_t smp = 10;
    for(size_t i = 0; i < smp; ++i) {
        double t = (2*i*M_PI)/smp;
        Eigen::Matrix2d rmat;
        rmat << cos(t), -sin(t), sin(t), cos(t);
        Eigen::Vector2d p0 = rmat*Eigen::Vector2d(-0.5, 0.0);
        Eigen::Vector2d p1 = rmat*Eigen::Vector2d(1.0, 0.0);
        auto bndpath = getConvexHull({p0,p1});

        ASSERT_GE(bndpath.size(), 2)
            << "i=" << i << ", t=" << t << std::endl;
        auto itr = std::find(bndpath.begin(), bndpath.end(), p0);
        EXPECT_NE(itr, bndpath.end())
            << "p0 not found: " << p0.adjoint() << std::endl;
        itr = std::find(bndpath.begin(), bndpath.end(), p1);
        EXPECT_NE(itr, bndpath.end())
            << "p1 not found: " << p1.adjoint() << std::endl;
        if(bndpath.size() > 2) {
            ADD_FAILURE()
                << "i=" << i << ", t=" << t << std::endl
                << "p0= " << p0.adjoint() << std::endl
                << "p1= " << p1.adjoint() << std::endl
                << bndpath[2].adjoint() << std::endl;
        }
    }
}

TEST(Bezier2D, HullFaceHigher)
{
    // Test for a higher degree.
    auto bez2d = std::vector<Eigen::Vector2d>{
        Eigen::Vector2d(0.0, 0.0), // 0
        Eigen::Vector2d(1.0, 0.5), // 1
        Eigen::Vector2d(2.0, 0.0), // 2
        Eigen::Vector2d(1.5, 1.0), // 3
        Eigen::Vector2d(2.0, 2.0), // 4
        Eigen::Vector2d(1.0, -1.0),// 5
        Eigen::Vector2d(0.0, 2.0), // 6
        Eigen::Vector2d(0.5, 1.0), // 7
        Eigen::Vector2d(2.0, 0.0)  // 8
    };
    auto bndpath = getConvexHull(bez2d);

    ASSERT_EQ(bndpath.size(), 5);
    EXPECT_EQ(bndpath[0], bez2d[5])
        << bndpath[0].adjoint() << std::endl;
    EXPECT_EQ(bndpath[1], bez2d[2])
        << bndpath[1].adjoint() << std::endl;
    EXPECT_EQ(bndpath[2], bez2d[4])
        << bndpath[2].adjoint() << std::endl;
    EXPECT_EQ(bndpath[3], bez2d[6])
        << bndpath[3].adjoint() << std::endl;
    EXPECT_EQ(bndpath[4], bez2d[0])
        << bndpath[4].adjoint() << std::endl;
}
