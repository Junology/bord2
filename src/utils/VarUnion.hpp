/*!
 * \file VarUnion.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February, 20 2020: created
 */

#pragma once

#include "../utils.hpp"

//! Union of types given as variadic template arguments.
//! In contrast to std::variant in C++17, this class does not store any information about an active member as a member variable.
//! \warning All types must be trivially destructible.
template<class... Ts>
union VarUnion;

template<class T>
union VarUnion<T> {
    static_assert(std::is_trivially_destructible<T>::value,
                  "The element type is not trivially destructible.");

    using union_head = T;
    union_head head;
    char _dummy;

    template<size_t n>
    using elem_type = std::tuple_element_t<n,T>;

    constexpr VarUnion() : _dummy{0} {}

    template<size_t n, class...Args, std::enable_if_t<n==0,int> = 0>
    constexpr VarUnion(std::integral_constant<size_t, n>, Args&&...args)
    : head(std::forward<Args>(args)...) {}

    template<size_t n>
    constexpr union_head& get() noexcept {
        static_assert(n==0, "Index out of range.");
        return head;
    }

    template<size_t n>
    constexpr union_head const& get() const noexcept {
        return const_cast<VarUnion<T>*>(this)->get<n>();
    }
};

template<class T0, class T1, class...Ts>
union VarUnion<T0,T1,Ts...> {
    static_assert(std::is_trivially_destructible<T0>::value,
                  "The element type is not trivially destructible.");

    using union_head = T0;
    using union_tail = VarUnion<T1,Ts...>;

    template<size_t n>
    using elem_type = std::tuple_element_t<n,std::tuple<T0,T1,Ts...>>;

    union_head head;
    union_tail tail;

    constexpr VarUnion() : tail{} {}

    template<size_t n, class...Args, std::enable_if_t<n==0,int> = 0>
    constexpr VarUnion(std::integral_constant<size_t,n>, Args&&... args)
        : head(args...)
    {}

    template<size_t n, class...Args, std::enable_if_t<(n>0),int> = 0>
    constexpr VarUnion(std::integral_constant<size_t,n>, Args&&...args)
        : tail(std::integral_constant<size_t,n-1>(), std::forward<Args>(args)...)
    {}

    template<size_t n, std::enable_if_t<n==0,int> = 0>
    constexpr union_head& get() noexcept {
        return head;
    }

    template<size_t n, std::enable_if_t<(n>0),int> = 0>
    constexpr auto get() noexcept -> elem_type<n>& {
        return tail.template get<n-1>();
    }

    template<size_t n>
    constexpr auto get() const noexcept -> elem_type<n> const& {
        return const_cast<VarUnion<T0,T1,Ts...>*>(this)->get<n>();
    }
};
