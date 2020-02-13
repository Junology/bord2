/*!
 * \file PlTangMove.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 27, 2020: created
 */

#pragma once

#include <cstring>
#include "PlTang.hpp"
#include "Graph.hpp"

/*!
 * Move of local tangles.
 */
template <size_t R, size_t C>
class PlTangMove {
public:
    enum : size_t {
        rows = R,
        cols = C
    };

    struct MoveElem {
        PlTangMove<R,C> move;
        size_t x, y;
    };

    using MoveSeq = std::vector<MoveElem>;

    //! Graph carrying the information how elementary cells in tangles are connected in the move (or bordism more precisely).
    //! Each graph is supposed to have exactly 2*R*C vertices; first R*C ones represent cells in the "before" tangle while the others in the "after" tangle.
    //! In either case, use row-major ordering on cells.
    using ConnectionGraph = Graph<2*R*C>;

private:
    char const* m_name;
    PlTang<R,C> m_tangBefore;
    PlTang<R,C> m_tangAfter;
    ConnectionGraph m_graph;

public:
    constexpr PlTangMove() noexcept
        : m_name{nullptr}, m_tangBefore{}, m_tangAfter{}
    {}

    template<size_t n>
    constexpr PlTangMove(char const (&name)[n], PlTang<R,C> const& before, PlTang<R,C> const& after, ConnectionGraph graph) noexcept
      : m_name(name),
        m_tangBefore(!after.isvalid() || before.codomain() != after.codomain() || before.domain() != after.domain() ? PlTang<R,C>{} : before),
        m_tangAfter(!before.isvalid() || before.codomain() != after.codomain() || before.domain() != after.domain() ? PlTang<R,C>{} : after),
        m_graph(graph)
    {}

    constexpr PlTangMove(PlTangMove const&) noexcept = default;
    constexpr PlTangMove(PlTangMove &&) noexcept = default;

    constexpr PlTangMove& operator=(PlTangMove const&) noexcept = default;
    constexpr PlTangMove& operator=(PlTangMove &&) noexcept = default;

    constexpr char const* getName() const noexcept {
        return m_name;
    }

    constexpr PlTang<R,C> getBefore() const noexcept {
        return m_tangBefore;
    }

    constexpr PlTang<R,C> getAfter() const noexcept {
        return m_tangAfter;
    }

    constexpr ConnectionGraph getGraph() const noexcept {
        return m_graph;
    }

    constexpr PlTangMove<R,C> getReversed() const noexcept {
        PlTangMove<R,C> result = *this;

        // Revert move.
        std::swap(result.m_tangBefore, result.m_tangAfter);

        // Revert connection graph.
        Graph<2*R*C> graph(2*R*C);

        m_graph.forEachEdge(
            [&graph](size_t i, size_t j) {
                size_t ibar = i>=R*C ? i-R*C : i+R*C;
                size_t jbar = j>=R*C ? j-R*C : j+R*C;
                graph.connect(ibar, jbar);
            } );
        result.m_graph = graph;

        return result;
    }

    constexpr bool isvalid() const noexcept {
        return m_tangBefore.isvalid();
    }

    //! Comparison operator based on the lexicographical order on the names.
    constexpr bool operator<(PlTangMove<R,C> const& other) const noexcept {
        return std::strcmp(m_name, other.m_name) < 0;
    }

    constexpr bool operator<=(PlTangMove<R,C> const& other) const noexcept {
        return std::strcmp(m_name, other.m_name) <= 0;
    }

    constexpr bool operator>(PlTangMove<R,C> const& other) const noexcept {
        return std::strcmp(m_name, other.m_name) > 0;
    }

    constexpr bool operator>=(PlTangMove<R,C> const& other) const noexcept {
        return std::strcmp(m_name, other.m_name) >= 0;
    }
};

//! Find the first move in a sequence that does not commute with former moves.
//! \param begin The iterator pointing to the begining of the sequence.
//! \param end The iterator pointing to the end of the sequence.
//! \return The iterator pointing to the first move not commuting with former moves if found, or the end of the sequence otherwise.
template <class T, size_t MVR, size_t MVC>
constexpr
auto
noncommHead(T const& tangle,
            typename PlTangMove<MVR,MVC>::MoveSeq::iterator begin,
            typename PlTangMove<MVR,MVC>::MoveSeq::iterator const& end) noexcept
    -> std::pair<typename PlTangMove<MVR,MVC>::MoveSeq::iterator, std::vector<bool>>
{
    auto result = std::make_pair(
        begin,
        std::vector<bool>(tangle.hlength()*tangle.vlength(), false));

    while(result.first != end) {
        auto &mv = *result.first;
        auto &mvtbl = result.second;

        if (!mvtbl[mv.x + mv.y*tangle.hlength()]
            && !mvtbl[(mv.x+1) + mv.y*tangle.hlength()]
            && !mvtbl[mv.x + (mv.y+1)*tangle.hlength()]
            && !mvtbl[(mv.x+1) + (mv.y+1)*tangle.hlength()])
        {
            mvtbl[mv.x + mv.y*tangle.hlength()] = true;
            mvtbl[(mv.x+1) + mv.y*tangle.hlength()] = true;
            mvtbl[mv.x + (mv.y+1)*tangle.hlength()] = true;
            mvtbl[(mv.x+1) + (mv.y+1)*tangle.hlength()] = true;

            ++result.first;
        }
        else {
            break;
        }
    }
    // Hopefully RVO works.
    return result;
}

