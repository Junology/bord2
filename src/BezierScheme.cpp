/*!
 * \file BezierScheme.cpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 23, 2020: created
 */

#include "BezierScheme.hpp"

#include <set>
#include <thread>
#include <mutex>

#include <iostream>

using BezierSequence = typename BezierScheme::BezierSequence;

//* Debug
static std::mutex ostr_mutex;
// */

/*********************************************************!
 * Information on cutting a sequence of Bezier curves.
 *********************************************************/
struct BezierCutter {
    size_t index;
    double param;

    //! \name Comparison operators.
    //@{
    constexpr bool operator==(BezierCutter const& rhs) const noexcept {
        return index == rhs.index && param == rhs.param;
    }

    constexpr bool operator<(BezierCutter const& rhs) const noexcept {
        return index < rhs.index || (index == rhs.index && param < rhs.param);
    }

    constexpr bool operator>(BezierCutter const& rhs) const noexcept {
        return rhs < *this;
    }

    constexpr bool operator<=(BezierCutter const& rhs) const noexcept {
        return !(*this > rhs);
    }

    constexpr bool operator>=(BezierCutter const& rhs) const noexcept {
        return !(*this < rhs);
    }
    //@}
};


/**********************************!
 * Cut a sequence of Bezier curves
 **********************************/
void cutoutBezSeq(BezierSequence const& src,
                  std::set<BezierCutter> const& cutters,
                  std::vector<BezierSequence> & out)
{
    BezierCutter lastcut{0, 0.0};
    bool carryover = false;

    // Traverse all cuts.
    for(auto& cut : cutters) {
        if(cut <= lastcut)
            continue;

        // Append the Bezier curves in the given sequence that are irrelevant to cutting.
        if (carryover) {
            if (lastcut.index + 1 < cut.index)
                out.back().insert(
                    out.back().end(),
                    std::next(src.begin(), lastcut.index+1),
                    std::next(src.begin(), cut.index));
        }
        else {
            out.emplace_back(
                std::next(src.begin(), lastcut.index),
                std::next(src.begin(), cut.index));
        }

        // Divide the Bezier curve in the specified index.
        auto bezdiv = (lastcut.index == cut.index && carryover)
            ? out.back().back().divide(
                (cut.param - lastcut.param)/(1.0 - lastcut.param) )
            : src[cut.index].divide(cut.param);

        // Append the first half.
        if(cut.param > 0.0) {
            if(lastcut.index == cut.index && carryover) {
                out.back().back() = bezdiv.first;
            }
            else {
                out.back().push_back(bezdiv.first);
            }
        }

        // If the division actually divides the curve, carry over the latter half to the next step.
        if(cut.param < 1.0) {
            out.emplace_back(1, bezdiv.second);
            carryover = true;
            lastcut = cut;
        }
        else {
            carryover = false;
            lastcut = {cut.index+1, 0.0};
        }
    }

    // If there are remaining parts, add them.
    if(carryover && lastcut.index+1 < src.size()) {
        out.back().insert(
            out.back().end(),
            std::next(src.begin(), lastcut.index+1),
            src.end() );
    }
    else if (!carryover && lastcut.index < src.size()) {
        out.emplace_back(
            std::next(src.begin(), lastcut.index),
            src.end() );
    }
}


/*******************************************************************!
 * Computes cuttings of an under-strand of crossing Bezier curves.
 * \param f A function-like object computing the "height" of signature
 *   > double(bool,BezierCutter const&)
 * where the first parameter indicates which we are looking at; LHS (true) or RHS (false).
 *******************************************************************/
template <class BezierT, class F>
auto crosscut(std::vector<BezierT> const& lhs,
              std::vector<BezierT> const& rhs,
              F const& f,
              std::atomic_bool const& flag = std::atomic_bool{true}
    )
    -> std::pair<std::set<BezierCutter>,std::set<BezierCutter>>
{
    static_assert(
        std::is_same<typename BezierTraits<BezierT>::vertex_type, Eigen::Vector2d>::value,
        "The control points must be of type Eigen::Vector2d.");

    bool to_be_continued = true;
    auto result = std::make_pair(
        std::set<BezierCutter>{}, std::set<BezierCutter>{} );

    // Traverse all parings.
    for(size_t i = 0; i < lhs.size() && to_be_continued; ++i) {
        for(size_t j = 0; j < rhs.size() && to_be_continued; ++j) {
            auto crosses = intersect<12,3>(
                lhs[i], rhs[j],
                [&flag]() -> bool {
                    return flag.load(std::memory_order_acquire);
                } );

            for(auto& params : crosses) {
                // Compute the heights at the crossing in the projections.
                double lhei = f(true, BezierCutter{i, params.first});
                double rhei = f(false, BezierCutter{j, params.second});

                // Append the parameter of the under-strand.
                if (lhei < rhei)
                    result.first.insert(BezierCutter{i, params.first});
                else
                    result.second.insert(BezierCutter{j, params.second});
            }

            to_be_continued = flag.load(std::memory_order_acquire);
        }
    }

    return result;
}


/*******************************************************************!
 * Computes cuttings of an under-strand in self-crossing.
 * \param f A function-like object computing the "height" of signature
 *   > double(BezierCutter const&)
 *******************************************************************/
template <class BezierT, class F>
auto crosscut_self(std::vector<BezierT> const& bezseq,
                   F const& f,
                   std::atomic_bool const &flag = std::atomic_bool{true}
    )
    -> std::set<BezierCutter>
{
    static_assert(
        std::is_same<typename BezierTraits<BezierT>::vertex_type, Eigen::Vector2d>::value,
        "The control points must be of type Eigen::Vector2d.");

    bool to_be_continued = true;
    std::set<BezierCutter> result{};

    // Traverse all parings.
    for(size_t i = 0; i < bezseq.size() && to_be_continued; ++i) {
        for(size_t j = i+1; j < bezseq.size() && to_be_continued; ++j) {
            auto crosses = intersect<12,3>(
                bezseq[i], bezseq[j],
                [&flag]() -> bool {
                    return flag.load(std::memory_order_acquire);
                });

            for(auto& params : crosses) {
                // Compute the heights at the crossing in the projections.
                double lhei = f(BezierCutter{i, params.first});
                double rhei = f(BezierCutter{j, params.second});

                // Append the parameter of the under-strand.
                if (lhei < rhei)
                    result.insert(BezierCutter{i, params.first});
                else
                    result.insert(BezierCutter{j, params.second});
            }

            to_be_continued = flag.load(std::memory_order_acquire);
        }
    }

    return result;
}


/*********************************************************************!
 * Implementation of getProject member function of BezierScheme class
 * This function should return as soon as possible when the value of the argument flag becomes false.
 *********************************************************************/
auto getProject_impl(
    std::vector<BezierSequence> const& bezseqs,
    Eigen::Matrix3d const& basis,
    std::atomic_bool const& flag = std::atomic_bool{true}
    )
    -> std::vector<BezierSequence>
{
    // Type aliases.
    using BezierVarType = BezierScheme::BezierVarType;
    using BezierProjected = typename BezierTraits<BezierVarType>::template converted_type<Eigen::Vector2d>;

    // 2d projected Bezier sequences
    std::vector<std::vector<BezierProjected>> bezseqs_2d(
        bezseqs.size(),
        std::vector<BezierProjected>{} );
    for(size_t i = 0; i < bezseqs.size(); ++i) {
        std::transform(
            bezseqs[i].begin(), bezseqs[i].end(),
            std::back_inserter(bezseqs_2d[i]),
            [&basis](BezierVarType const& bez3d) -> BezierProjected {
                return bez3d.convert(
                    [&basis](Eigen::Vector3d const& v) -> Eigen::Vector2d {
                        return basis.block<2,3>(0,0) * v;
                    } );
            } );
    }

    std::vector<std::set<BezierCutter>> cutters(
        bezseqs.size(), std::set<BezierCutter>{} );

    //* Debug
    std::cout << __FILE__":" << __LINE__ << std::endl;
    std::cout << "In Thread: " << std::this_thread::get_id() << std::endl;
    // */

    /* Compute the self-crossings concurrently.*/
    {
        std::vector<std::thread> workers;

        for(size_t i = 0; i < bezseqs.size(); ++i) {
            workers.emplace_back(
                [&cuts=cutters[i],
                 &bezseq_2d=bezseqs_2d[i],
                 &bezseq=bezseqs[i],
                 &basis,
                 &flag,
                 /* Debug */ i]
                {
                    //* Debug
                    {
                        std::lock_guard<std::mutex> _(ostr_mutex);
                        std::cout << __FILE__":" << __LINE__ << std::endl;
                        std::cout << "Launch the Thread: "
                                  << std::this_thread::get_id() << std::endl;
                        std::cout << "Computing self-crossings of " << i << std::endl;
                    }
                    // */

                    auto selfcuts = crosscut_self(
                        bezseq_2d,
                        [&bezseq, &basis](BezierCutter const& cut) -> double
                        {
                            return basis.row(2) * bezseq[cut.index].eval(cut.param);
                        }, flag );
                    cuts.insert(
                        std::make_move_iterator(selfcuts.begin()),
                        std::make_move_iterator(selfcuts.end()));

                    //* Debug
                    {
                        std::lock_guard<std::mutex> _(ostr_mutex);
                        std::cout << __FILE__":" << __LINE__ << std::endl;
                        std::cout << "Finish the Thread: "
                                  << std::this_thread::get_id() << std::endl;
                        std::cout << "Finish self-crossings of " << i << std::endl;
                    }
                    // */
                } );
        }

        for(auto& worker : workers)
            worker.join();
    }

    /*** Compute crossings of different strands. ***/
    {
        // Mutexex for Bezier sequences.
        std::vector<std::mutex> mutexes(bezseqs.size());
        // Thread computing crossings of Bezier sequences.
        std::vector<std::thread> workers;
        workers.reserve(bezseqs.size()*(bezseqs.size()-1)/2);

        for(size_t stride = 1; stride < bezseqs.size(); ++stride) {
            for(size_t i = 0; i+stride < bezseqs.size(); ++i) {
                /* Debug
                std::cout << __FILE__":" << __LINE__ << std::endl;
                std::cout << "i=" << i
                          << " vs i+stride=" << i+stride
                          << std::endl;
                // */
                workers.emplace_back(
                    [&cutsL = cutters[i], &cutsR = cutters[i+stride],
                     &lhs2d = bezseqs_2d[i], &rhs2d = bezseqs_2d[i+stride],
                     &lhs = bezseqs[i], &rhs = bezseqs[i+stride],
                     &basis,
                     &mutexL = mutexes[i], &mutexR = mutexes[i+stride],
                     &flag,
                     /*Debug*/ i, /*Debug*/ j=i+stride]()
                    {
                        //* Debug
                        {
                            std::lock_guard<std::mutex> _(ostr_mutex);
                            std::cout << __FILE__":" << __LINE__ << std::endl;
                            std::cout << "Launch the Thread: "
                                      << std::this_thread::get_id() << std::endl;
                            std::cout << "Computing crossings of " << i << " and " << j << std::endl;
                        }
                        // */
                        auto crscuts = crosscut(
                            lhs2d, rhs2d,
                            [&lhs, &rhs, &basis](bool flag, BezierCutter const& cut) -> double {
                                return basis.row(2)*(flag ? lhs[cut.index].eval(cut.param) : rhs[cut.index].eval(cut.param));
                            }, flag);

                        {
                            std::lock_guard<std::mutex> gurdL(mutexL);
                            cutsL.insert(
                                std::make_move_iterator(crscuts.first.begin()),
                                std::make_move_iterator(crscuts.first.end()) );
                        }
                        {
                            std::lock_guard<std::mutex> gurdR(mutexR);
                            cutsR.insert(
                                std::make_move_iterator(crscuts.second.begin()),
                                std::make_move_iterator(crscuts.second.end()) );
                        }
                        //* Debug
                        {
                            std::lock_guard<std::mutex> _(ostr_mutex);
                            std::cout << __FILE__":" << __LINE__ << std::endl;
                            std::cout << "Finish the Thread: "
                                      << std::this_thread::get_id() << std::endl;
                            std::cout << "Finish " << i << " and " << j << std::endl;
                        }
                        // */
                    } );
            }
        }
        // Wait for the computations.
        for(auto& worker : workers)
            worker.join();
    }

    // Cut-out the sequences of Bezier curves and write the results
    std::vector<BezierSequence> result{};
    result.reserve(bezseqs.size());
    for(size_t i = 0; i < bezseqs.size() && flag.load(std::memory_order_acquire); ++i) {
        /* Debug
        std::cout << __FILE__":" << __LINE__ << std::endl;
        std::cout << "i=" << i
                  << " #cuts=" << cutters[i].size()
                  << std::endl;
        // */
        cutoutBezSeq(bezseqs[i], cutters[i], result);
    }

    return result;
}


/******************************
 * BezierScheme::getProject
 ******************************/
auto BezierScheme::getProject(
    Eigen::Matrix<double,2,3> const& prmat,
    std::function<void(void)> fun
    ) const
    -> std::future<std::vector<BezierSequence>>
{
    std::promise<std::vector<BezierSequence>> p;
    auto fut = p.get_future();

    Eigen::RowVector3d kernel = prmat.row(0).cross(prmat.row(1));

    // If the depth vector is the zero vector, return immediately.
    if (kernel.isZero()) {
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Projection direction is zero vector." << std::endl;

        p.set_value({});
        return fut;
    }

    // Compute a projection matrix onto the plane.
    // Notice that the result depends not on this projection but only on the kernel.
    Eigen::Matrix3d basis;

    kernel.normalize();
    basis.block<2,3>(0,0) = prmat;
    basis.row(2) = kernel;

    terminate();
    m_to_be_computed.store(true);
    m_computer = std::thread(
        [basis, bezseqs=m_bezseqs, p=std::move(p), f=std::move(fun), &flag=m_to_be_computed]() mutable
        {
            p.set_value(getProject_impl(bezseqs, basis, flag));

            // Invoke f() if the computation successfully finished.
            if(flag.load(std::memory_order_acquire))
                f();
        } );

    return fut;
}

