/*!
 * \file PlTangMove.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 27, 2020: created
 */

#pragma once

#include <map>
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

    constexpr size_t hlength() const noexcept {
        return m_tangBefore.hlength();
    }

    constexpr size_t vlength() const noexcept {
        return m_tangBefore.vlength();
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

    //! Mapping each vertex to a value.
    template <
        class F,
        class T = decltype(std::declval<F>()(std::declval<size_t>(),
                                             std::declval<size_t>(),
                                             std::declval<char>()).second)
        >
    auto mapVertexIf(F const& f)
        noexcept(noexcept(std::declval<F>()(std::declval<size_t>(),std::declval<size_t>(),std::declval<char>())))
        -> std::map<size_t, T>
    {
        static_assert(
            std::is_convertible<decltype(f(std::declval<size_t>(),std::declval<size_t>(),std::declval<char>()).first), bool>::value,
            "The argument f must be of signature of the form std::pair<bool,T>(size_t,size_t,char)");

        std::map<size_t, T> result;

        m_tangBefore.traverse(
            [&result, &f, this](size_t i, size_t j, char c) {
                auto res = f(i,j,c);
                if(res.first)
                    result.emplace_hint(
                        result.end(),
                        i+j*hlength(),
                        res.second);
            } );
        m_tangAfter.traverse(
            [&result, &f, this](size_t i, size_t j, char c) {
                auto res = f(i,j,c);
                if(res.first)
                    result.emplace_hint(
                        result.end(),
                        i+j*hlength() + hlength()*vlength(),
                        res.second);
            } );

        return result;
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

//! Applying a move to a planar tangle.
template<size_t R, size_t C, class M>
constexpr
void applyMove(PlTang<R,C>& pltang, M const& mv) noexcept
{
    pltang.replace(mv.x, mv.y, mv.move.getAfter());
}

//! Find the first move in a sequence that does not commute with former moves.
//! \param begin The iterator pointing to the begining of the sequence.
//! \param end The iterator pointing to the end of the sequence.
//! \return The iterator pointing to the first move not commuting with former moves if found, or the end of the sequence otherwise.
template <size_t R, size_t C, class Iterator>
auto
noncommHead(PlTang<R,C> const& tangle, Iterator begin, Iterator const& end) noexcept
    -> std::pair<Iterator, std::vector<bool>>
{
    auto result = std::make_pair(
        begin,
        std::vector<bool>(tangle.hlength()*tangle.vlength(), false));

    while(result.first != end) {
        auto &mv = *result.first;
        auto &mvtbl = result.second;

        bool accumflag = true;

        for(size_t i = 0; i < mv.move.hlength(); ++i)
            for(size_t j = 0; j < mv.move.vlength(); ++j)
                accumflag &= !mvtbl[(mv.x+i) + (mv.y+j)*tangle.hlength()];

        if (accumflag) {
            for(size_t i = 0; i < mv.move.hlength(); ++i)
                for(size_t j = 0; j < mv.move.vlength(); ++j)
                    mvtbl[(mv.x+i) + (mv.y+j)*tangle.hlength()] = true;

            ++result.first;
        }
        else {
            break;
        }
    }
    // Hopefully RVO works.
    return result;
}

