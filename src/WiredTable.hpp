/*!
 * \file WiredTable.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 29, 2020: created
 */

#pragma once

#include <type_traits>
#include <algorithm>
#include <vector>
#include <list>
#include <stdexcept>

#include "utils.hpp"
#include "ColdPtr.hpp"

template <class T, class=void>
struct has_maxvalence : public std::false_type {};

template <class T>
struct has_maxvalence<T,bord2::void_t<decltype(std::declval<T>().maxVelence())>>
    : public std::true_type {};

/*!
 * The class stores a seqeunce of data as if they are "vertices" in a wired network.
 * One can access each data by the index associated with.
 * \tparam V The type of data stored. If the class has a member function maxValence() whose return value can be converted to int, it will be used to give an upper-bound of the number of edges attached to a vertex. \see max_valence
 */
template <class V>
class WiredTable
{
public:
    using VertexType = V;

protected:
    struct NodeType;

public:
    //! The type of indices to be associated with vertices.
    //! It is supposed that it has operator bool to check if it is valid or not.
    struct IndexType : public ColdPtr<NodeType> {
        friend WiredTable<V>;
        using ColdPtr<NodeType>::ColdPtr;
    };

protected:
    struct NodeType {
        VertexType v;
        std::vector<IndexType> branch;
    };

private:
    //! A list of vertices.
    std::list<NodeType> m_verts{};

public:
    //! Check if an index is a valid one.
    bool hasVertexOf(IndexType const index) const noexcept
    {
        for(auto& vert : m_verts) {
            if (index == &vert)
                return true;
        }

        return false;
    }

    //! Get the reference to an element in a specific index.
    //! If index is invalid
    VertexType& get(IndexType const index) {
        if(!index) throw std::out_of_range{"Invalid index"};
        return index->v;
    }

    //! Get the reference to an element in a specific index.
    //! This does not check if index is valid or not.
    VertexType const& get(IndexType const index) const {
        if(!index) throw std::out_of_range{"Invalid index"};
        return index->v;
    }

    //! Append a new vertex together with an attribute to the table.
    //! \return The index of the vertex in the table.
    //! \wraning Although the type of the index may be a pointer, the user must not try to de-reference it.
    IndexType append(VertexType const& vert) noexcept
    {
        m_verts.push_back({vert, std::vector<IndexType>{}});
        return &m_verts.back();
    }

    //! Remove a vertex from the table.
    //! \param index The index of a vertex to be removed.
    //! \retval true The element is successfully removed. In this case, the variable in the argument will be invalidate.
    //! \retval false The function failed to find an element of the given index. No side effects.
    bool remove(IndexType &index) noexcept
    {
        bool retval = false;
        for(auto itr = m_verts.begin(); itr != m_verts.end(); ) {
            // If found
            if (index == &(*itr)) {
                retval = true;
                itr = m_verts.erase(itr);
            }
            // Otherwise, remove any connections to the vertex
            // Thanks to the implementation of \see connect(), we may assume that there are at most one edge between each pair of vertices.
            else {
                auto conn = std::find(itr->branch.begin(), itr->branch.end(), index);
                if (conn != itr->branch.end())
                    itr->branch.erase(conn);

                ++itr;
            }
        }

        if (retval)
            index.invalidate();

        return retval;
    }

    //! Traversal access to vertices with indices.
    //! \param f Any function-like object so that the following type checks:
    //!   >> f(std::declval<VertexType>(), std::declval<IndexType>());
    template <class F>
    void foreach(F const& f) {
        for(auto& vert : m_verts)
            f(vert.v, IndexType{&vert});
    }

    //! Traversal access to vertices with indices.
    template <class F>
    void foreach(F const& f) const {
        for(auto& vert : m_verts)
            f(vert.v, IndexType{&vert});
    }

    //! Find the index of the first vertex satisfying a given condition.
    //! \param f Any function-like object so that the following type checks:
    //!   >> static_cast<bool>(f(std::declval<VertexType>()));
    template <class F>
    IndexType find_if(F const& f) {
        for(auto& vert : m_verts)
            if (f(vert.v))
                return IndexType{&vert};

        return IndexType{};
    }

    //! Find all the indices of vertices satisfying a given condition.
    //! \param f Any function-like object so that the following type checks:
    //!   >> static_cast<bool>(f(std::declval<VertexType>()));
    template <class F>
    std::vector<IndexType> find_all_if(F const& f) {
        std::vector<IndexType> result{};

        for(auto& vert : m_verts)
            if (f(vert.v))
                result.push_back(IndexType{&vert});

        return std::move(result);
    }

    //! Add an edge connecting two vertices with the indices i and j.
    //! \return Whether an edge is successfully added.
    bool connect(IndexType &i, IndexType &j) noexcept {
        // Check if i is already connected to j.
        auto itr = std::find(i->branch.begin(), i->branch.end(), j);
        if(itr != i->branch.end())
            return false;

        // Check if j is already connected to i.
        itr = std::find(j->branch.begin(), j->branch.end(), i);
        if(itr != j->branch.end())
            return false;

        // Valence check.
        int imax = max_valence(i->v);
        int jmax = max_valence(j->v);

        if((imax >= 0 && i->branch.size() + 1 >= static_cast<size_t>(imax))
           || (jmax >= 0 && j->branch.size() + 1 >= static_cast<size_t>(jmax)) )
            return false;

        // If everything is OK, then add an edge
        i->branch.emplace_back(IndexType{j});
        j->branch.emplace_back(IndexType{i});

        return true;
    }

    std::vector<IndexType>& getNexts(IndexType &i) noexcept {
        return i->branch;
    }

    std::vector<std::vector<VertexType>> maxSimplePaths(IndexType &root) noexcept {
        using PathType = std::vector<IndexType>;
        using VertexPath = std::vector<VertexType>;
        std::pair<std::vector<PathType>, std::vector<VertexPath>> result{
            {std::vector<IndexType>{root}},
            {}
        };
        auto concatVect = [](std::vector<PathType> lhs, std::vector<PathType> const& rhs) -> std::vector<PathType> {
            lhs.insert(lhs.end(), rhs.begin(), rhs.end());
            return lhs;
        };
        while(!result.first.empty()) {
            result.first = bord2::accumMap(
                std::make_move_iterator(result.first.begin()),
                std::make_move_iterator(result.first.end()),
                [&result](PathType const& path) -> std::vector<PathType> {
                    // Candidates of next vertices.
                    std::vector<IndexType> nexts = path.back()->branch;
                    // Remove already passed vertices from candidates.
                    auto remtop = std::remove_if(
                        nexts.begin(), nexts.end(),
                        [&path](IndexType const& idx) {
                            auto itr = std::find(path.begin(), path.end(), idx);
                            return itr != path.end();
                        });
                    nexts.erase(remtop, nexts.end());
                    // If the current vertex is an end, append it to the result list.
                    if (nexts.empty()) {
                        result.second.push_back(std::vector<VertexType>{});
                        for(auto& idx : path)
                            result.second.back().push_back(idx->v);
                        return std::vector<PathType>{};
                    }
                    std::vector<PathType> newpaths{};
                    std::for_each(
                        std::make_move_iterator(nexts.begin()),
                        std::make_move_iterator(nexts.end()),
                        [&path, &newpaths](IndexType const& idx) -> void {
                            newpaths.push_back(path);
                            newpaths.back().push_back(idx);
                        });
                    return newpaths;
                },
                std::vector<PathType>{},
                concatVect );
        }
        return result.second;
    }

protected:
    //! Get the maximum number of branches that a vertex admits.
    //! A negative number will be understood as the (positive) infinity.
    //! If the class VertexType has a member function maxValence(), it will be used; Otherwise, this just returns -1.
    template <
        class T,
        std::enable_if_t<
            std::is_same<std::remove_cv_t<std::remove_reference_t<T>>, VertexType>::value
            && has_maxvalence<VertexType>::value,
            int
            > = 0
    >
    static int max_valence(T const& vert)
    {
        return vert.maxValence();
    }

    template <
        class T,
        std::enable_if_t<
            std::is_same<std::remove_cv_t<std::remove_reference_t<T>>, VertexType>::value
            && !has_maxvalence<VertexType>::value,
            int
            > = 0
        >
    static int max_valence(T const& vert)
    {
        return -1;
    }
};
