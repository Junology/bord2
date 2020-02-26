#include <gtest/gtest.h>

#include <Eigen/Dense>
#include "BezierScheme.cpp"

void PrintTo(BezierSequence const& bezseq, ::std::ostream *os)
{
    *os << std::endl;

    for(auto& bez : bezseq) {
        *os << bez.source().adjoint() << std::endl;
        *os << " -> " << bez.target().adjoint() << std::endl;
    }
}

TEST(BezierScheme, Raw)
{
    BezierScheme scheme;

    scheme.moveTo(Eigen::Vector3d(1.0, 0.5, -1.0));
    scheme.lineTo(Eigen::Vector3d(-1.0, 0.5, 1.0));

    scheme.moveTo(Eigen::Vector3d(-0.5, 1.0, -1.0));
    scheme.lineTo(Eigen::Vector3d(-0.5, -1.0, 1.0));

    scheme.moveTo(Eigen::Vector3d(-1.0, -0.5, -1.0));
    scheme.qbezierTo(Eigen::Vector3d(0.5, -0.5, 0.0),
                     Eigen::Vector3d(0.5, 1.0, 1.0) );

    auto raw = scheme.getRaw();
    ASSERT_EQ(raw.size(), 3);
    EXPECT_EQ(raw[0].size(), 1);
    EXPECT_EQ(raw[0][0].source(), Eigen::Vector3d(1.0, 0.5, -1.0));
    EXPECT_EQ(raw[0][0].target(), Eigen::Vector3d(-1.0, 0.5, 1.0));
    EXPECT_EQ(raw[1].size(), 1);
    EXPECT_EQ(raw[1][0].source(), Eigen::Vector3d(-0.5, 1.0, -1.0));
    EXPECT_EQ(raw[1][0].target(), Eigen::Vector3d(-0.5, -1.0, 1.0));
    EXPECT_EQ(raw[2].size(), 1);
    EXPECT_EQ(raw[2][0].source(), Eigen::Vector3d(-1.0, -0.5, -1.0));
    EXPECT_EQ(raw[2][0].target(), Eigen::Vector3d(0.5, 1.0, 1.0));
}

TEST(BezierScheme, Cutout)
{
    using BezVarType = BezierVariant<Eigen::Vector3d, 0, 1, 2, 3>;

    BezVarType bez0(
        std::integral_constant<size_t,1>(),
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0) );
    BezVarType bez1(
        std::integral_constant<size_t,2>(),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 1.0, 1.0),
        Eigen::Vector3d(0.0, 1.0, 0.0) );
    BezVarType bez2(
        std::integral_constant<size_t,3>(),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(-1.0, 1.0, 1.0),
        Eigen::Vector3d(-1.0, -1.0, 1.0),
        Eigen::Vector3d(0.0, -1.0, 0.0) );

    BezierSequence bezseq = {bez0, bez1, bez2};

    std::set<BezierCutter> cutters;

    /*** First test ***/
    std::vector<BezierSequence> result;

    cutters.insert(BezierCutter{1, 0.25});
    cutters.insert(BezierCutter{1, 0.75});
    cutoutBezSeq(bezseq, cutters, result);
    
    ASSERT_EQ(result.size(), 3);
    ASSERT_EQ(result[0].size(), 2);
    EXPECT_EQ(result[0][0].source(), bez0.source());
    EXPECT_EQ(result[0][0].target(), bez0.target());
    EXPECT_EQ(result[0][1].source(), bez1.source());
    EXPECT_EQ(result[0][1].target(), bez1.eval(0.25));
    ASSERT_EQ(result[1].size(), 1);
    EXPECT_EQ(result[1][0].source(), bez1.eval(0.25));
    EXPECT_LT((result[1][0].target() - bez1.eval(0.75)).norm(), 1e-12)
        << result[1][0].target().adjoint()
        << std::endl
        << bez1.eval(0.75).adjoint()
        << std::endl;
    ASSERT_EQ(result[2].size(), 2);
    EXPECT_LT((result[2][0].source() - bez1.eval(0.75)).norm(), 1e-12)
        << result[2][0].source().adjoint()
        << std::endl
        << bez1.eval(0.75).adjoint()
        << std::endl;
    EXPECT_EQ(result[2][0].target(), bez1.target());
    EXPECT_EQ(result[2][1].source(), bez2.source());
    EXPECT_EQ(result[2][1].target(), bez2.target());

    /*** Second test ***/
    result.clear();
    cutters.insert(BezierCutter{0, 0.5});
    cutoutBezSeq(bezseq, cutters, result);
    ASSERT_EQ(result.size(), 4);
    ASSERT_EQ(result[0].size(), 1);
    EXPECT_EQ(result[0][0].source(), bez0.source());
    EXPECT_EQ(result[0][0].target(), bez0.eval(0.5));
    ASSERT_GE(result[1].size(), 2);
    EXPECT_EQ(result[1][0].source(), bez0.eval(0.5));
    EXPECT_EQ(result[1][0].target(), bez0.target());
    EXPECT_EQ(result[1][1].source(), bez1.source());
    EXPECT_EQ(result[1][1].target(), bez1.eval(0.25));
    if(result[1].size() > 2) {
        ADD_FAILURE();
        for(auto& bez : result[1]) {
            std::cout << bez.source().adjoint()
                      << std::endl << " -> "
                      << bez.target().adjoint()
                      << std::endl;
        }
    }
    ASSERT_EQ(result[2].size(), 1);
    EXPECT_EQ(result[2][0].source(), bez1.eval(0.25));
    EXPECT_LT((result[2][0].target() - bez1.eval(0.75)).norm(), 1e-12)
        << result[2][0].target().adjoint()
        << std::endl
        << bez1.eval(0.75).adjoint()
        << std::endl;
    ASSERT_EQ(result[3].size(), 2);
    EXPECT_LT((result[3][0].source() - bez1.eval(0.75)).norm(), 1e-12)
        << result[2][0].source().adjoint()
        << std::endl
        << bez1.eval(0.75).adjoint()
        << std::endl;
    EXPECT_EQ(result[3][0].target(), bez1.target());
    EXPECT_EQ(result[3][1].source(), bez2.source());
    EXPECT_EQ(result[3][1].target(), bez2.target());

    /*** Third test ***/
    result.clear();
    cutters.insert(BezierCutter{2, 0.5});
    cutoutBezSeq(bezseq, cutters, result);
    ASSERT_EQ(result.size(), 5);
    ASSERT_EQ(result[0].size(), 1);
    EXPECT_EQ(result[0][0].source(), bez0.source());
    EXPECT_EQ(result[0][0].target(), bez0.eval(0.5));
    ASSERT_GE(result[1].size(), 2);
    EXPECT_EQ(result[1][0].source(), bez0.eval(0.5));
    EXPECT_EQ(result[1][0].target(), bez0.target());
    EXPECT_EQ(result[1][1].source(), bez1.source());
    EXPECT_EQ(result[1][1].target(), bez1.eval(0.25));
    if(result[1].size() > 2) {
        ADD_FAILURE();
        for(auto& bez : result[1]) {
            std::cout << bez.source().adjoint()
                      << std::endl << " -> "
                      << bez.target().adjoint()
                      << std::endl;
        }
    }
    ASSERT_EQ(result[2].size(), 1);
    EXPECT_EQ(result[2][0].source(), bez1.eval(0.25));
    EXPECT_LT((result[2][0].target() - bez1.eval(0.75)).norm(), 1e-12)
        << result[2][0].target().adjoint()
        << std::endl
        << bez1.eval(0.75).adjoint()
        << std::endl;
    ASSERT_EQ(result[3].size(), 2);
    EXPECT_LT((result[3][0].source() - bez1.eval(0.75)).norm(), 1e-12)
        << result[2][0].source().adjoint()
        << std::endl
        << bez1.eval(0.75).adjoint()
        << std::endl;
    EXPECT_EQ(result[3][0].target(), bez1.target());
    EXPECT_EQ(result[3][1].source(), bez2.source());
    EXPECT_EQ(result[3][1].target(), bez2.eval(0.5));
    ASSERT_EQ(result[4].size(), 1);
    EXPECT_EQ(result[4][0].source(), bez2.eval(0.5));
    EXPECT_EQ(result[4][0].target(), bez2.target());
}

TEST(BezierScheme, ProjectOneParOne)
{
    BezierScheme scheme;

    scheme.moveTo(Eigen::Vector3d(1.0, 0.5, -1.0));
    scheme.lineTo(Eigen::Vector3d(-1.0, 0.5, 1.0));

    scheme.moveTo(Eigen::Vector3d(-0.5, 1.0, -1.0));
    scheme.lineTo(Eigen::Vector3d(-0.5, -1.0, 1.0));

    scheme.moveTo(Eigen::Vector3d(-1.0, -0.5, -1.0));
    scheme.qbezierTo(Eigen::Vector3d(0.5, -0.5, 0.0),
                     Eigen::Vector3d(0.5, 1.0, 1.0) );

    auto result = scheme.getProject(Eigen::Vector3d(0.0, 0.0, 1.0)).get();

    if(result.size() != 6) {
        std::cout << "Size: " << result.size() << std::endl;
        for(auto& bezseq : result) {
            PrintTo(bezseq, &std::cout);
        }
        FAIL();
    }

    ASSERT_EQ(result[0].size(), 1);
    EXPECT_EQ(result[0][0].source(), Eigen::Vector3d(1.0, 0.5, -1.0))
        << result[0][0].source().adjoint();
    ASSERT_EQ(result[1].size(), 1);
    EXPECT_EQ(result[1][0].target(), Eigen::Vector3d(-1.0, 0.5, 1.0))
        << result[1][0].target().adjoint();
    ASSERT_EQ(result[2].size(), 1);
    EXPECT_EQ(result[2][0].source(), Eigen::Vector3d(-0.5, 1.0, -1.0))
        << result[2][0].target().adjoint();
    ASSERT_EQ(result[3].size(), 1);
    EXPECT_EQ(result[3][0].target(), Eigen::Vector3d(-0.5, -1.0, 1.0))
        << result[3][0].target().adjoint();
    ASSERT_EQ(result[4].size(), 1);
    EXPECT_EQ(result[4][0].source(), Eigen::Vector3d(-1.0, -0.5, -1.0))
        << result[4][0].target().adjoint();
    ASSERT_EQ(result[5].size(), 1);
    EXPECT_EQ(result[5][0].target(), Eigen::Vector3d(0.5, 1.0, 1.0))
        << result[5][0].target().adjoint();
}
