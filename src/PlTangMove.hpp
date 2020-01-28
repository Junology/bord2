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

/*!
 * Move of local tangles.
 */
template <size_t R, size_t C>
class PlTangMove {
private:
    char const* m_name;
    PlTang<R,C> m_tangBefore;
    PlTang<R,C> m_tangAfter;

public:
    PlTangMove() = delete;

    template<size_t n>
    constexpr PlTangMove(char const (&name)[n], PlTang<R,C> const& before, PlTang<R,C> const& after) noexcept
      : m_name(name),
        m_tangBefore(!after.isvalid() || before.codomain() != after.codomain() || before.domain() != after.domain() ? PlTang<R,C>{} : before),
        m_tangAfter(!before.isvalid() || before.codomain() != after.codomain() || before.domain() != after.domain() ? PlTang<R,C>{} : after)
    {}

    constexpr PlTangMove(PlTangMove<R,C> const&) noexcept = default;
    constexpr PlTangMove(PlTangMove<R,C> &&) noexcept = default;

    ~PlTangMove() noexcept = default;

    constexpr char const* getName() const noexcept {
        return m_name;
    }

    constexpr PlTang<R,C> getBefore() const noexcept {
        return m_tangBefore;
    }

    constexpr PlTang<R,C> getAfter() const noexcept {
        return m_tangAfter;
    }

    constexpr PlTangMove<R,C> getReversed() const noexcept {
        PlTangMove<R,C> result = *this;
        PlTang<R,C> aux = result.m_tangBefore;
        result.m_tangBefore = m_tangAfter;
        result.m_tangAfter = aux;

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

