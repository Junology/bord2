/**
 * \file Graph.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 5, 2019: created
 */

#pragma once

#include <utility>
#include "utils/BitArray.hpp"

#include <iostream> // Debug

/*!
 * \brief The class representing abstract (finite) graphs without self-loops.
 * \detail We here refer "graphs" as pairs (V,E) of a set V and a subset E of VxV stable under transposition.
 * An element of E is called a *self-loop* if it is of the form (v,v) for an element v of V.
 * In other words, if (V,E) is an abstract graph without self-loop, E is thought of as a set of subsets of V of cardinality exactly 2.
 *
 * We represent the set V just as a subset of the set {0,1,...,N-1}.
 * As for edges E, we encode each pair (v,w) into a non-negative integer r as follows: we first assume v < w by transposition if necessary.
 * Then, r = v + w*(w+1)/2.
 * Hence, E is represented as a set of non-negative integers 0 <= r < n*(n+1)/2, which is realized in terms of BitArray<n*(n+1)/2>.
 * \tparam N Maximum number of vertices; hence 0 <= n <= N.
 */
template <size_t N, std::enable_if_t<(N>0),int> = 0>
class Graph
{
    // Friendship has nothing to do with the sizes.
    template <size_t M, std::enable_if_t<(M>0),int>>
    friend class Graph;

public:
    enum : size_t {
        MaxVertices = N,
        MaxEdges = N*(N-1)/2
    };

private:
    //! The set of vertices as a subset of {0,1,...,N}.
    BitArray<MaxVertices> m_vertices{};
    BitArray<MaxEdges> m_edges{};

public:
    constexpr Graph() noexcept = default;
    constexpr Graph(Graph const& src) noexcept = default;

    //! Construct a graph with a given number of vertices.
    constexpr Graph(size_t n) noexcept
      : m_vertices((~BitArray<N>{}).lowpass(n)) {}

    //! Construct a graph from a BitArray as a set of vertices.
    template <size_t M, std::enable_if_t<(M<=N),int> = 0>
    constexpr Graph(BitArray<M> const &vertices) noexcept
      : m_vertices(vertices) {};

    //! Copy from a smaller graph.
    template <size_t M, std::enable_if_t<(M<N),int> = 0>
    constexpr Graph(Graph<M> const& src) noexcept
      : m_vertices(src.m_vertices), m_edges(src.m_edges) {}

    ~Graph() noexcept= default;

    //! Get the number of vertices in the graph.
    constexpr size_t size() const noexcept {
        return m_vertices.popCount();
    }

    //! Check if the graph is inhabited.
    constexpr bool inhabited() const noexcept {
        return static_cast<bool>(m_vertices);
    }

    //! Check if a number is associated to a vertex of the graph.
    constexpr bool isVertex(size_t v) const noexcept {
        return m_vertices.test(v);
    }

    //! Check if two vertices are connected by an edge.
    //! \note Notice that self-loops are prohibited.
    constexpr bool isConnected(size_t v, size_t w) const noexcept {
        return v != w && m_edges.test(encodePair(v,w));
    }

    //! Get the vertex with the minimum index.
    //! If the graph is empty, the returned value exceeds MaxVertices.
    constexpr size_t first() const noexcept {
        return m_vertices.countTrail0();
    }

    //! Append a vertex to the graph.
    //! \return If the first component is true, the second is the index of the new vertex; otherwise, the number of vertices exceeds the maximum, and the set of vertices do not change.
    constexpr std::pair<bool,size_t> append() noexcept {
        size_t i = m_vertices.countTrail1();

        if(i >= MaxVertices)
            return {false, 0};

        m_vertices.set(i);
        return {true, i};
    }

    //! Add a vertex at the given index.
    //! \return Whether a new vertex is added or not.
    constexpr bool addAt(size_t idx) noexcept {
        if (m_vertices.test(idx))
            return false;

        m_vertices.set(idx);

        return true;
    }

    //! Remove a vertex
    constexpr bool remove(size_t v) noexcept {
        if (!isVertex(v))
            return false;

        // Remove the vertex.
        m_vertices.set(v, false);

        // Remove edges involved with removal of v.
        m_edges = m_edges & ~genEdgesWith(v);

        return true;
    }

    //! Add an edge connecting two vertices.
    //! \return Whether a new edge is added or not.
    constexpr bool connect(size_t v, size_t w) noexcept {
        // Check if v and w are in fact vertices.
        if (v == w || !isVertex(v) || !isVertex(w))
            return false;

        size_t r = encodePair(v, w);

        if(m_edges.test(r))
            return false;
        else {
            m_edges.set(r);
            return true;
        }
    }

    //! Remove an edge connecting two vertices.
    //! \return Whether an edge is removed or not.
    constexpr bool disconnect(size_t v, size_t w) noexcept {
        // Check if v and w are in fact vertices.
        if (!isVertex(v) || !isVertex(w))
            return false;

        size_t r = encodePair(v, w);

        if(m_edges.test(r)) {
            m_edges.set(r, false);
            return true;
        }
        else {
            return false;
        }
    }

    //! Traverse all the indices of vertices (from smaller to larger)
    //! \param f A function-like object admitting a single argument of type std::size_t; in other words, the following should type-check:
    //!   >> f(std::declval<std::size_t>());
    template <class F>
    constexpr void forEachVertex(F&& f) noexcept(noexcept(f(std::declval<std::size_t>()))) {
        auto vertices = m_vertices;
        size_t idx = 0;

        while(vertices) {
            size_t r = vertices.countTrail0();
            /*
            f(r);
            vertices = vertices.lowcut(r+1);
            */
            idx += r;
            f(idx);
            ++idx;
            vertices = vertices >> (r+1);
        }
    }

    //! Trim a connected component to which the minimum vertex belongs.
    //! If there is no vertex, the function just returns the empty graph.
    constexpr Graph<N> trimComponent() noexcept {
        // Compute the index of the minimum vertex.
        auto v0 = m_vertices.countTrail0();

        if (v0 >= MaxVertices)
            return Graph<N>{};

        // The result.
        Graph<N> result{};
        BitArray<MaxVertices> vertices{};

        // Move the first vertex.
        vertices.set(v0);
        m_vertices.set(v0, false);
        // The result should contain edges with v0 as an end.
        result.m_edges = genEdgesWith(v0) & m_edges;

        // Find companions.
        do {
            result.m_vertices = vertices;
            forEachVertex(
                [this, &vertices, &result](size_t v) {
                    // Edges containing the vertex v.
                    auto e = genEdgesWith(v);
                    if (e & result.m_edges) {
                        vertices.set(v);
                        m_vertices.set(v, false);
                        result.m_edges |= e & m_edges;
                    }
                });
        } while(result.m_vertices != vertices);

        return result;
    }

    /*!
     * \section Operator overloads.
     */
    constexpr Graph& operator=(Graph const& src) noexcept = default;

    template <size_t M, std::enable_if_t<(M<N),int> = 0>
    constexpr Graph& operator=(Graph<M> const& src) noexcept {
        m_vertices = src.m_vertices;
        m_edges = src.m_edges;
        return *this;
    }

    constexpr bool operator==(Graph const& other) const noexcept {
        return m_vertices == other.m_vertices
            && m_edges == other.m_edges;
    }


    /*!
     * \section Static utility functions
     */
protected:
    //! Encode a pair of vertices into an integer.
    //! \pre This function assumes m != n.
    static constexpr size_t encodePair(size_t m, size_t n) noexcept {
        if (m > n)
            std::swap(m, n);
        return m + n*(n-1)/2;
    }

    //! Decode a pair of vertices from an integer.
    static constexpr std::pair<size_t, size_t> decodePair(size_t code) noexcept {
        size_t t = 1, r = 0;
        while( r+t <= code ) {
            r += t;
            ++t;
        }

        return {code-r, t};
    }

    static constexpr BitArray<MaxEdges> genEdgesFrom0() noexcept {
        BitArray<MaxEdges> result{};
        // <01>
        // <02> 12
        // <03> 13 23
        // <04> 14 24 34
        // ...
        // -> indices of 0n = 0 1 3 6 .. 
        size_t n = 0;
        for(size_t i = 0; i < N-1; ++i) {
            n += i;
            result.set(n);
        }
        return result;
    }

    static constexpr BitArray<MaxEdges> genEdgesFrom(size_t v) noexcept {
        constexpr auto edges_from_0 = genEdgesFrom0();
        return v > 0
            ? edges_from_0.lowcut(encodePair(0,v)) << v
            : edges_from_0;
    }

    static constexpr BitArray<MaxEdges> genEdgesTo(size_t v) noexcept {
        return v > 0
            ? BitArray<MaxEdges>((~BitArray<MaxVertices>{}).lowpass(v)) << encodePair(0,v)
            : BitArray<MaxEdges>{};
    }

    static constexpr BitArray<MaxEdges> genEdgesWith(size_t v) noexcept {
        return genEdgesFrom(v) | genEdgesTo(v);
    }
};
