/*!
 * \file ColdPtr.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 3, 2020: created
 */

#pragma once

/*!
 * A class that looks like a pointer with de-referencing operations are not public but protected.
 * It is used as an ordinary pointer in a class that is a friend of a class derived from this; e.g.
 *   >> class C {
 *   >>     class : public ColdPtr<int> {
 *   >>          friend class C;
 *   >>          using ColdPtr<int>::ColdPtr;
 *   >>     } m_ptr{};
 *   >> public:
 *   >>     ...
 *   >> };
 * \note The type const ColdPtr<T> behaves not as T * const but as T const*.
 */
template <class T>
class ColdPtr
{
public:
    using element_type = T;
    using pointer = element_type*;
    using const_pointer = element_type const*;

private:
    pointer m_ptr{nullptr};

protected:
    constexpr ColdPtr(pointer ptr) noexcept : m_ptr(ptr) {}

    //! Get the raw pointer.
    constexpr pointer get() const noexcept { return m_ptr; }

    /*!
     * \name De-referencing operators.
     */
    //@{
    //constexpr element_type& operator*() noexcept { return *m_ptr; }
    constexpr element_type & operator*() const noexcept { return *m_ptr; }
    constexpr pointer operator->() const noexcept { return m_ptr; }
    //constexpr const_pointer operator->() const noexcept { return m_ptr; }
    //@}

    //! Direct comparison with a raw pointer.
    constexpr bool operator==(const_pointer ptr) const noexcept { return m_ptr == ptr; }
    //! Direct comparison with a raw pointer.
    constexpr bool operator!=(const_pointer ptr) const noexcept { return m_ptr != ptr; }

public:
    //! Invalid instances are publicly constructible.
    constexpr ColdPtr() noexcept = default;

    constexpr ColdPtr(ColdPtr<T> const&) noexcept = default;
    constexpr ColdPtr(ColdPtr<T> &&src) noexcept = default;

    constexpr ColdPtr& operator=(ColdPtr<T> const&) noexcept = default;
    constexpr ColdPtr& operator=(ColdPtr<T> &&src) noexcept = default;

    //! Nothing to do in the destructor.
    ~ColdPtr() noexcept = default;

    //! Invalidate the pointer.
    constexpr void invalidate() noexcept { m_ptr = nullptr; }

    /*!
     * \name Comparison and validity checkers
     */
    //@{
    constexpr bool operator!() const noexcept { return !m_ptr; }
    constexpr operator bool() { return m_ptr; }
    constexpr bool operator==(ColdPtr<T> const& index) const noexcept {
        return m_ptr == index.m_ptr;
    }
    constexpr bool operator!=(ColdPtr<T> const& index) const noexcept {
        return m_ptr != index.m_ptr;
    }
    //@}
};
