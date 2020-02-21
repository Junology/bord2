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

template <>
union VarUnion<>
{
    constexpr VarUnion() noexcept = default;
    constexpr VarUnion(VarUnion const&) = default;
    constexpr VarUnion(size_t, VarUnion const&) {}

    // Only *_dynamic() functions are defined so that they always fail.
    constexpr bool assign_dynamic(size_t, VarUnion const&) noexcept {
        return false;
    }

    template <class...Args>
    bool reconstruct_dynamic(size_t, Args&&...) noexcept {
        return false;
    }
};

template<class T0, class... Ts>
union VarUnion<T0, Ts...> {
    static_assert(std::is_trivially_destructible<T0>::value,
                  "The element type is not trivially destructible.");

    //! The number of members.
    enum : size_t { num_members = 1 + sizeof...(Ts) };

    using union_head = T0;
    using union_tail = VarUnion<Ts...>;

    //! Tail member
    union_tail tail;
    //! Head member
    union_head head;

    template<size_t n>
    using elem_type = std::tuple_element_t<n,std::tuple<T0,Ts...>>;

    /*!
     * \name Constructors
     */
    //@{
    constexpr VarUnion() : tail{} {}

    constexpr VarUnion(VarUnion const&) = default;
    constexpr VarUnion(VarUnion &&) = default;

    //! Copy construction statically specifying the index of the active member; base case.
    template<size_t n, std::enable_if_t<n==0,int> = 0>
    constexpr VarUnion(std::integral_constant<size_t,n>, VarUnion const& src) noexcept
        : head(src.head)
    {}

    //! Copy construction statically specifying the index of the active member; successor case.
    template<size_t n, std::enable_if_t<(0<n && n<num_members),int> = 0>
    constexpr VarUnion(std::integral_constant<size_t,n>, VarUnion const& src) noexcept
        : tail(std::integral_constant<size_t,n-1>(), src.tail)
    {}

    //! Move construction statically specifying the index of the active member; base case.
    template<size_t n, std::enable_if_t<n==0,int> = 0>
    constexpr VarUnion(std::integral_constant<size_t,n>, VarUnion && src) noexcept
        : head(std::move(src.head))
    {}

    //! Move construction statically specifying the index of the active member; successor case.
    template<size_t n, std::enable_if_t<(0<n && n<num_members),int> = 0>
    constexpr VarUnion(std::integral_constant<size_t,n>, VarUnion && src) noexcept
        : tail(std::integral_constant<size_t,n-1>(), std::move(src.tail))
    {}

    //! Copy construction dynamically specifying the index of the active member; base case.
    //! This uses placement new to initialize the member, so it cannot be constexpr.
    VarUnion(size_t n, VarUnion const& src) noexcept
        : tail{}
    {
        if (n==0)
            new(&head) union_head{src.head};
        if (0 < n && n < num_members)
            new(&tail) union_tail(static_cast<size_t>(n-1), src.tail);
    }

    //! Move construction dynamically specifying the index of the active member; base case.
    //! This uses placement new to initialize the member, so it cannot be constexpr.
    VarUnion(size_t n, VarUnion && src) noexcept
        : tail{}
    {
        if (n==0)
            new(&head) union_head{std::move(src.head)};
        if (0 < n && n < num_members)
            new(&tail) union_tail(n-1, std::move(src.tail));
    }

    //! Direct construction of a member specifing the index.
    template<size_t n, class...Args, std::enable_if_t<n==0,int> = 0>
    constexpr VarUnion(std::integral_constant<size_t, n>, Args&&...args)
        : head(std::forward<Args>(args)...) {}

    template<size_t n, class...Args, std::enable_if_t<(0<n && n<num_members),int> = 0>
    constexpr VarUnion(std::integral_constant<size_t, n>, Args&&...args)
        : tail(std::integral_constant<size_t,n-1>(), std::forward<Args>(args)...) {}
    //@}
    /*** End of constructors **/

    //! Reference access to a member specified by its index.
    //! \warning The author is not sure if it is safely used for non-active members. For this purpose, he recommends using \sa assgin instead.
    template<size_t n, std::enable_if_t<n==0,int> = 0>
    constexpr union_head& get(std::integral_constant<size_t,n> = {}) noexcept {
        return head;
    }

    template<size_t n, std::enable_if_t<(0<n && n<num_members),int> = 0>
    constexpr auto get(std::integral_constant<size_t,n> = {}) noexcept
        -> elem_type<n>& {
        return tail.get(std::integral_constant<size_t,n-1>());
    }

    //! Const reference access to a member specified by its index.
    template<size_t n>
    constexpr auto get(std::integral_constant<size_t,n> = {}) const noexcept
        -> elem_type<n> const&
    {
        return const_cast<VarUnion*>(this)->get(std::integral_constant<size_t,n>());
    }

    //! Assignment a value to a member.
    //! This function causes the move-assignment operator followed by a constructor.
    //! If you are not comfortable with this behavior, use \sa reconstruct instead.
    template <size_t n, class...Args, std::enable_if_t<n==0,int> = 0>
    constexpr void assign(std::integral_constant<size_t,n>, Args&&...args) noexcept {
        head = T0(std::forward<Args>(args)...);
    }

    template <size_t n, class...Args, std::enable_if_t<(0<n && n<num_members),int> = 0>
    constexpr void assign(std::integral_constant<size_t,n>, Args&&...args) noexcept {
        tail.assign(std::integral_constant<size_t,n-1>(), std::forward<Args>(args)...);
    }

    //! Non-statically checked copy; indeed, the range check will not be performed in compile-time if the function is not constexpr.
    //! \param n The index of the current active member; the function just believe it.
    //! \retval true The copy was proceeded.
    //! \retval false The index is out of range; in this case, no operation will be proceeded.
    constexpr bool assign_dynamic(size_t n, VarUnion const& src) noexcept {
        if (n==0) {
            head = src.head;
            return true;
        }
        else if (n < num_members) {
            tail.assign_dynamic(n-1, src.tail);
            return true;
        }
        else {
            return false;
        }
    }

    //! Non-statically checked move; indeed, the range check will not be performed in compile-time if the function is not constexpr.
    //! \param n The index of the current active member; the function just believe it.
    //! \retval true The copy was proceeded.
    //! \retval false The index is out of range; in this case, no operation will be proceeded.
    constexpr bool assign_dynamic(size_t n, VarUnion && src) noexcept {
        if (n==0) {
            head = std::move(src.head);
            return true;
        }
        else if (n < num_members) {
            tail.assign_dynamic(n-1, std::move(src.tail));
            return true;
        }
        else {
            return false;
        }
    }

    //! Change an active member using placement new; so this function cannot be constexpr.
    //! Note that, since we assume all the member types are trivially destructible, we do not need to care about the current active member.
    template <size_t n, class...Args, std::enable_if_t<n==0,int> = 0>
    void reconstruct(std::integral_constant<size_t,n>, Args&&...args) noexcept
    {
        new(&head) T0(std::forward<Args>(args)...);
    }

    template <size_t n, class...Args, std::enable_if_t<(0<n && n<num_members),int> = 0>
    void reconstruct(std::integral_constant<size_t,n>, Args&&...args) noexcept
    {
        tail.reconstruct(std::integral_constant<size_t,n-1>(), std::forward<Args>(args)...);
    }

    //! Non-statically checked reconstruction; indeed the range check will not be performed in compile-time in case the function is not constexpr.
    template <class...Args>
    bool reconstruct_dynamic(size_t n, Args&&...args) noexcept {
        if(n==0) {
            new(&head) T0(std::forward<Args>(args)...);
            return true;
        }
        else if (n < num_members) {
            tail.reconstruct_dynamic(n-1, std::forward<Args>(args)...);
            return true;
        }
        else {
            return false;
        }
    }
};
