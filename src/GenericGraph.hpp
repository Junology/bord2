/**
 * \file GenericGraph.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 8, 2019: created
 */

#pragma once

#include <type_traits>
#include <utility>
#include <set>
#include <map>
#include <memory>

#include "utils/BitArray.hpp"

// #include <iostream> // Debug

/*!
 * \addgroup Type traits
 * \@{
 */
template <class T, class K, class=void>
struct has_at : public std::false_type {
    using type = void;
    static type apply(T&, K){}
};

template <class T, class K>
struct has_at<T,K, bord2::void_t<decltype(std::declval<T>().at(std::declval<K>()))>>
    : public std::true_type {
    using type = decltype(std::declval<T>().at(std::declval<K>()));
    static type apply(T& obj, K key){ return obj.at(key); }
};
/*! \@} */

//! Key manager class
template <class K>
class IndexHolder{
    static_assert(std::is_convertible<K, size_t>::value && std::is_convertible<size_t, K>::value,
                  "The template parameter type is not mutually convertible with std::size_t.");
public:
    enum : size_t { chunkbits = 64 };
    using key_type = K;
    using chunk_type = BitArray<chunkbits>;

private:
    std::vector<chunk_type> m_tbl{};

public:
    bool empty() const noexcept {
        return std::none_of(
            m_tbl.begin(), m_tbl.end(),
            [](chunk_type const& chunk){
                return static_cast<bool>(chunk);
            });
    }

    bool test(key_type key) const noexcept {
        std::size_t gpos = static_cast<size_t>(key) / chunkbits;
        std::size_t lpos = static_cast<size_t>(key) % chunkbits;

        return (gpos < m_tbl.size() && m_tbl[gpos].test(lpos));
    }

    key_type generate() noexcept {
        size_t r = 0;
        size_t lpos;
        while(r < m_tbl.size()) {
            lpos = m_tbl[r].countTrail1();
            if(lpos < chunkbits)
                break;
            ++r;
        }

        if(r == m_tbl.size()) {
            m_tbl.emplace_back();
            lpos = 0;
        }

        m_tbl[r].set(lpos);

        return static_cast<key_type>(lpos + chunkbits*r);
    }

    bool reserve(key_type key) noexcept {
        std::size_t gpos = static_cast<size_t>(key) / chunkbits;
        std::size_t lpos = static_cast<size_t>(key) % chunkbits;
        if(gpos >= m_tbl.size())
            m_tbl.resize(gpos+1);

        if (m_tbl[gpos].test(lpos))
            return false;
        else {
            m_tbl[gpos].set(lpos);
            return true;
        }
    }

    void release(key_type key) noexcept {
        std::size_t gpos = static_cast<size_t>(key) / chunkbits;
        std::size_t lpos = static_cast<size_t>(key) % chunkbits;

        if(gpos < m_tbl.size())
            m_tbl[gpos].set(lpos, false);
    }

    key_type take() const noexcept {
        for(size_t i = 0; i < m_tbl.size(); ++i) {
            if (m_tbl[i])
                return static_cast<key_type>(i*chunkbits + m_tbl[i].countTrail0());
        }

        return key_type{};
    }
};

/*!
 * Graph whose vertices are elements of an ordered container such as std::set or std::map.
 * \tparam Set An ordered container in order to which "the set of vertices" is realized. The type should be a "container" (see Container concept in the C++20 standard) and have the following members:
 *   - member-type *vertex_type* which is default-constructible;
 *   - member-type *key_type* which is default-constructible;
 *   - member-function *emplace* whose first argument admits values of key_type;
 *   - member-function *find* returning iterators to access elements via key;
 *   - member-function *erase* to remove elements via keys (not via iterators).
 * \tparam KeyHolder A type used to generate and manage keys of vertices. It must be default-constructible and have the following member functions:
 *   - *empty* to check if there is no key;
 *   - *test* to check if a key is in use;
 *   - *generate* to generate a new key;
 *   - *reserve* to reserve a specific key;
 *   - *release* called in removal of vertices.
 * To use the default KeyHolder, i.e. \sa IndexHolder, make sure that key_type is mutually convertible with std::size_t.
 */
template <class Set, class KeyHolder=IndexHolder<typename Set::key_type>>
class GenericGraph
{
    static_assert(std::is_default_constructible<Set>::value,
                  "The template parameter type is not default constructible.");
    static_assert(std::is_default_constructible<typename Set::key_type>::value,
                  "key_type is not default constructible.");
    static_assert(std::is_default_constructible<KeyHolder>::value,
                  "The keygenrator is not default constructible.");
    static_assert(std::is_copy_constructible<KeyHolder>::value,
                  "The keygenerator is not copy-constructible.");

public:
    using vertex_type = Set;
    using value_type = typename Set::value_type;
    using key_type = typename Set::key_type;

private:
    vertex_type m_vertices{};
    std::set<std::pair<key_type,key_type>> m_edges{};
    KeyHolder m_keyholder{};

public:
    /*
     * There are only compiler-generated constructors and destructors.
     */

    //! Get the number of vertices in the graph.
    constexpr size_t size() const noexcept {
        return m_vertices.size();
    }

    //! Check if the graph is inhabited.
    constexpr bool inhabited() const noexcept {
        return m_vertices.size() > 0;
    }

    //! Check if a key is associated with a vertex.
    bool isVertex(key_type key) const noexcept {
        return m_vertices.find(key) != m_vertices.end();
    }

    //! Check if two vertices are connected by an edge.
    //! \note Notice that self-loops are prohibited.
    bool isConnected(key_type key0, key_type key1) const noexcept {
        return (key0 != key1)
            && (m_edges.find(makeOrdered(key0, key1)) != m_edges.cend());
    }

    //! Get the minimum key associated to a vertex.
    key_type minkey() const noexcept {
        return m_keyholder.take();
    }

    //! Access to a vertex via key.
    //! This is available only when vertex_type has a member at(); e.g. std::map.
    auto at(key_type key)
        -> typename has_at<vertex_type, key_type>::type {
        return has_at<vertex_type, key_type>::apply(m_vertices, key);
    }

    auto at(key_type key) const
        -> typename has_at<const vertex_type, key_type>::type {
        return has_at<const vertex_type, key_type>::apply(m_vertices, key);
    }

    template<class T=void>
    void at(...) const {}

    //! Append a new vertex.
    template <class... Args>
    key_type append(Args&&... args) noexcept {
        key_type key = m_keyholder.generate();
        m_vertices.emplace(key, std::forward<Args>(args)...);

        return key;
    }

    //! Remove a vertex.
    bool remove(key_type key) noexcept {
        if (m_vertices.erase(key) != 0) {
            m_keyholder.release(key);
            for(auto eitr = m_edges.begin(); eitr != m_edges.end(); ) {
                if (eitr->first == key || eitr->second == key)
                    eitr = m_edges.erase(eitr);
                else
                    ++eitr;
            }
            return true;
        }
        else
            return false;
    }

    //! Make an edge
    bool connect(key_type key0, key_type key1) noexcept {
        if (!isVertex(key0) || !isVertex(key1))
            return false;

        return m_edges.insert(makeOrdered(key0, key1)).second;
    }

    //! Remove an edge
    bool disconnect(key_type key0, key_type key1) noexcept {
        return m_edges.erase(makeOrdered(key0, key1)) != 0;
    }

    //! Traverse the keys of all the vertices (from smaller to larger)
    //! \param f A function-like object admitting a single argument of type value_type; in other words, the following should type-check:
    //!   >> f(std::declval<value_type>());
    template <class F>
    void forEachVertex(F&& f) noexcept(noexcept(f(std::declval<value_type>())))
    {
        std::for_each(m_vertices.begin(), m_vertices.end(), std::forward<F>(f));
    }

    //! Find a vertex that satisfies a given condition.
    //! \param f A function-like object admitting a single argument of type value_type and returning bool.
    //! \retval {flag,key} A pair of a flag indicating success or not and, if flag is true, a key to the first vertex satisfying the condition.
    //! \warning f must not change the given value.
    template <class F>
    value_type const* findKey(F&& f) noexcept(noexcept(f(std::declval<const value_type>())))
    {
        auto itr = std::find_if(m_vertices.begin(),
                                m_vertices.end(),
                                std::forward<F>(f));
        // Found
        if (itr != m_vertices.end())
            return &(*itr);
        // Not found
        else
            return nullptr;
    }

    //! Find a univalent vertex in the graph.
    std::pair<bool, key_type> findEnd() const noexcept {
        KeyHolder holder = m_keyholder;
        std::map<key_type, size_t> cntmap;

        // Add every reserved keys to a map.
        while(!holder.empty()) {
            auto key = holder.take();
            cntmap.emplace_hint(cntmap.end(), key, 0);
            holder.release(key);
        }

        // Count how many times a vertex appears as an end of edges.
        std::for_each(
            m_edges.begin(), m_edges.end(),
            [&cntmap](std::pair<key_type, key_type> const& e) {
                ++(cntmap[e.first]);
                ++(cntmap[e.second]);
            } );

        // Find a vertex appearing only once.
        auto itr = std::find_if(
            cntmap.begin(), cntmap.end(),
            [](std::pair<key_type, size_t> const& v) {
                return v.second == 1;
            } );

        return (itr == cntmap.end())
            ? std::pair<bool, key_type>{false, key_type{}}
            : std::pair<bool, key_type>{true, itr->first};
    }

    //! Trim a connected component containing the vertex associated with the given key.
    //! If the key is not associated to any verteices, the function just returns the empty graph.
    GenericGraph trimComponent(key_type key0) noexcept {
        // If key0 is invalid, return the empty one.
        if (!isVertex(key0))
            return GenericGraph{};

        // The result.
        GenericGraph result{};

        // Move the first vertex.
        auto vitr0 = m_vertices.find(key0);
        if(vitr0 != m_vertices.end()) {
            result.m_vertices.emplace(*vitr0);
            result.m_keyholder.reserve(key0);
            m_vertices.erase(vitr0);
            m_keyholder.release(key0);
        }
        else {
            /* Debug
            std::cout << __FILE__": " << __LINE__ << std::endl;
            std::cout << "Invalid key: " << key0 << std::endl;
            std::cout << "Vertices are as follows:" << std::endl;
            for(auto vert : m_vertices) {
                std::cout << vert << std::endl;
            }
            // */
        }

        // Find companions.
        bool flag = true;
        while(flag) {
            flag = false;

            /* Debug
            std::cout << __FILE__": " << __LINE__ << std::endl;
            std::cout << "Remaining vertices: " << m_vertices.size() << std::endl;
            std::cout << "Remaining edges: " << m_edges.size() << std::endl;
            // */

            // Traverse all the edges in the current graph.
            for(auto eitr = m_edges.begin(); eitr != m_edges.end(); ) {
                // Find vertices in the current result graph which are ends of the edge.
                auto keypair_itr = std::make_pair(
                    result.m_vertices.find(eitr->first),
                    result.m_vertices.find(eitr->second));

                // Move the edge and the vertex in the other end.
                if(keypair_itr.first != result.m_vertices.end()) {
                    // Move the vertex.
                    auto vitr = m_vertices.find(eitr->second);
                    if (vitr != m_vertices.end()) {
                        result.m_vertices.insert(*vitr);
                        result.m_keyholder.reserve(eitr->second);
                        m_vertices.erase(vitr);
                        m_keyholder.release(eitr->second);
                        flag = true;
                    }

                    // Move the edge
                    result.m_edges.insert(*eitr);
                    eitr = m_edges.erase(eitr);
                }
                else if(keypair_itr.second != result.m_vertices.end()) {
                    // Move the vertex.
                    auto vitr = m_vertices.find(eitr->first);
                    if (vitr != m_vertices.end()) {
                        result.m_vertices.insert(*vitr);
                        result.m_keyholder.reserve(eitr->first);
                        m_vertices.erase(vitr);
                        m_keyholder.release(eitr->first);
                        flag = true;
                    }

                    // Move the edge
                    result.m_edges.insert(*eitr);
                    eitr = m_edges.erase(eitr);
                }
                else {
                    ++eitr;
                }
            }
        }

        return result;
    }

    //! Iterate a given function to which vertices along a maximal simple (i.e. loop-less) path are passed.
    //! \param key The key associated to a vertex where the path begins.
    //! \param f A function-like object, the same as the one in the function \sa forEachVertex.
    //! \return The first component indicates whether the path may continue to vertices which have already been passed through. In this case, the second component is the key to one of "possible-next" vertices.
    template <class F>
    std::pair<bool,key_type> trackPath(key_type key, F const& f) {
        // Check if key is valid.
        if (!isVertex(key))
            return {false, key};

        // Key-holder to remember vertices that a path have been passed through.
        KeyHolder holder{};
        std::unique_ptr<key_type> prev{};

        while(!holder.test(key)) {
            // Execute a function on the head vertex.
            f(*m_vertices.find(key));
            holder.reserve(key);

            // Flag indicating a vertex is found whether or not it is
            bool foundany = false;
            key_type key_next;
            // Traverse all the edges
            for(auto const &e : m_edges) {
                // Check if the edge contains the current key.
                auto maybe = iftheother(key, e);

                // We are not interested in edges which do not contain the vertex associated with the current key or ones one of whose ends is the vertex passed through in the previous step.
                if (!maybe.first || (prev && *prev == maybe.second))
                    continue;

                // Update the key
                key_next = maybe.second;
                foundany = true;

                if (!holder.test(key_next))
                    break;
            }

            // In case there is no candidate for the next, it means we reach a univalent vertex.
            if(!foundany)
                return {false, key_type{}};

            // Update loop data.
            if(!prev)
                prev.reset(new key_type(key));
            else
                *prev = key;
            key = key_next;
        }

        return {true, key};
    }

    //! Iterate a given function to which vertices along a maximal simple (i.e. loop-less) path are passed.
    //! This version visits a vertex twice if it is a base-point of a loop.
    //! \param key The key associated to a vertex where the path begins.
    //! \param f A function-like object, the same as the one in the function \sa forEachVertex.
    //! \return Whether a loop was found or not.
    template <class F>
    bool trackPathCyc(key_type key, F const& f) {
        auto maynext = trackPath(key, f);
        if (maynext.first)
            f(*m_vertices.find(maynext.second));
        return maynext.first;
    }

protected:
    inline static constexpr std::pair<key_type, key_type> makeOrdered(key_type key0, key_type key1) noexcept {
        return (key0 <= key1)
            ? std::make_pair(key0, key1)
            : std::make_pair(key1, key0);
    }

    inline static constexpr std::pair<bool, key_type> iftheother(key_type key, std::pair<key_type,key_type> const& p) noexcept {
        if (p.first == key)
            return {true, p.second};
        else if( p.second == key)
            return {true, p.first};
        else
            return {false, key};
    }
};

//! Special function for graphs with mapping vertices.
//! This function first searches a vertex which is mapped to a given value.
//! If found, it returns the key; otherwise, it appends the given value as a new vertex and returns the key.
//! In the search, given comparison function is used; default is std::equal_to.
template <class K, class T, class U, class Comp = std::equal_to<T>>
K findAppend(GenericGraph<std::map<K,T>> &graph, U const& x, Comp const& comp = std::equal_to<T>()) noexcept
{
    static_assert(std::is_convertible<U,T>::value, "The type of the given value cannot be converted into mapped_type.");

    auto maybeval = graph.findKey(
        [&x, &comp](std::pair<K,T> const& val) {
            return comp(val.second, x);
        } );
    if (maybeval)
        return maybeval->first;
    else
        return graph.append(x);
}
