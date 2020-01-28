/*!
 * \file StrLnView.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 5, 2020: created
 */

#pragma once

#include "../utils.hpp"

/*!
 * A variant of string_view in C++17.
 * The class focuses on the line-wise access to strings.
 * The lifetime of an instance is no longer than that of the string refered to.
 */
class StrLnView {
private:
    //! A pointer to a null terminated string.
    char const *m_str;
    //! The total length of the string; in other words, a distance to '\0' in the string.
    std::size_t m_length;
    //! The smallest index with m_cursor <= m_linecursor such that m_linecursor == m_length or m_str[m_linecursor] = '\n'.
    std::size_t m_eolcursor;
    //! The current head.
    std::size_t m_cursor;

public:
    template<
        class T,
        std::enable_if_t<
            std::is_same<std::decay_t<T>, char const*>::value,
            int
            > = 0
        >
    constexpr StrLnView(T&& str) noexcept
      : m_str(str),
        m_length(bord2::strlength(str)),
        m_eolcursor(0),
        m_cursor(0)
    {
        while(m_eolcursor < m_length && m_str[m_eolcursor] != '\n')
            ++m_eolcursor;
    }

    constexpr StrLnView() noexcept
      : m_str(nullptr),
        m_length(0),
        m_eolcursor(0),
        m_cursor(0)
    {}

    constexpr StrLnView(StrLnView const&) noexcept = default;
    constexpr StrLnView(StrLnView &&) noexcept = default;

    ~StrLnView() = default;

    constexpr bool empty() const noexcept
    {
        return !m_str || m_cursor == m_length;
    }

    constexpr std::size_t remainLn() const noexcept
    {
        return m_eolcursor - m_cursor;
    }

    constexpr char const* c_str() const noexcept
    {
        return m_str + m_cursor;
    }

    // Move to the successor line.
    constexpr bool advLn() noexcept
    {
        if(m_eolcursor == m_length) {
            m_cursor = m_length;
            return false;
        }

        // move to the head of the next line.
        ++m_eolcursor;
        m_cursor = m_eolcursor;

        while(m_eolcursor < m_length && m_str[m_eolcursor] != '\n')
            ++m_eolcursor;

        return true;
    }

    constexpr operator bool() const noexcept
    {
        return !empty();
    }

    constexpr bool operator!() const noexcept
    {
        return empty();
    }
};
