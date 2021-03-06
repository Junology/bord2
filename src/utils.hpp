/*!
 * \file utils.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 9, 2019: created
 */

#pragma once

#include <type_traits>
#include <utility>
#include <limits>
#include <tuple>
#include <array>
#include <vector>

namespace bord2 {

/***********************************!
 * \section Hints to the compilers
 ************************************/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-noreturn"
[[noreturn]] inline constexpr void unreachable() noexcept {
#if defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5))
    __builtin_unreachable();
#elseif defined(_MSC_VER)
    __assume(false);
#endif
}
#pragma GCC diagnostic pop

#define BORD2_ASSUME(x) do{ if(!x) ::bord2::unreachable(); } while(0)

/************************************!
 * \section Convenient type traits
 ************************************/

template <class...>
using void_t = void;

template <
    class T,
    std::enable_if_t<
        !std::is_reference<T>::value
        && std::is_default_constructible<T>::value,int
        > = 0
    >
constexpr T absurd [[gnu::cold]]() noexcept {
    /* NOT REACHED */
    return T{};
}

template <class T, std::enable_if_t<!std::is_default_constructible<T>::value,int> = 0>
[[noreturn]] constexpr T absurd [[gnu::cold]]() noexcept {
    /* NOT REACHED */
    /*
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
    return *static_cast<T*>(nullptr);
#pragma GCC diagnostic pop
    */
    unreachable();
    // throw(std::exception());
}

constexpr bool allTrue(std::initializer_list<bool>&& flags) noexcept {
    for(auto& flag : flags)
        if (!flag) return false;
    return true;
}

//! Check if a class is publically derived from a template class
template <template <class...> class TBase, class Derived>
class is_pubbase_of_template_impl {
private:
    template <class... Ts>
    static std::true_type is_pubbase_of_template_implfunc(TBase<Ts...> const*);
    // C style variadic length argument.
    static std::false_type is_pubbase_of_template_implfunc(...);

public:
    using type = decltype(is_pubbase_of_template_implfunc(std::declval<Derived*>()));
};

template <template <class...> class TBase, class Derived>
using is_pubbase_of_template = typename is_pubbase_of_template_impl<TBase, Derived>::type;

//! make_index_sequence reversed
template <std::size_t N>
class make_reversed_index_seq_impl{
private:
    template <std::size_t... is>
    static constexpr auto reverse_impl(std::index_sequence<is...>)
        -> std::index_sequence<(sizeof...(is)-is-1)...>;

public:
    using type = decltype(reverse_impl(std::declval<std::make_index_sequence<N> >()));
};

template <std::size_t N>
using make_reversed_index_seq = typename make_reversed_index_seq_impl<N>::type;


/*******************************************!
 * \section Compile-time utility functions
 *******************************************/
template <class T>
constexpr T bitwave(size_t width) noexcept
{
    if (width == 0)
        return T{};

    T result = static_cast<T>(~static_cast<T>(0u)) >> (sizeof(T)*8 - width);

    for(width *= 2; width < sizeof(T)*8; width *= 2) {
        result |= result << width;
    }

    return result;
}

//! Population-count (aka. Hamming weight).
template <class T>
constexpr std::remove_reference_t<std::remove_cv_t<T> > popcount(T x) noexcept
{
    // I hope the compiler unfolds the loop.
    for(size_t i = 1; i < std::numeric_limits<T>::digits; i <<= 1) {
        x = (x & bitwave<T>(i)) + ( (x>>i) & bitwave<T>(i) );
    }

    return x;
}

//! Count trailing ones (using the population count above).
template <class T>
constexpr std::remove_reference_t<std::remove_cv_t<T>> counttrail1(T x) noexcept
{
    // All we have to do is just counting bits after clearing all bits but the trailing 1s.
    return popcount<T>((~x & (x+1))-1);
}

//! Count trailing zeros (using the population count above).
template <class T>
constexpr std::remove_reference_t<std::remove_cv_t<T>> counttrail0(T x) noexcept
{
    return popcount<T>(~x & (x-1));
}

//! Compile-time non-negative integer power
//! It is verified that this function is faster than std::pow.
template<class T>
inline constexpr T cipow(T x, unsigned int n) noexcept
{
    T result = 1;

    while(n) {
        if(n&0x1)
            result *= x;
        x *= x;
        n >>= 1;
    }

    return result;

    // The following version is faster than the above under
    //   > g++ --std=c++14 -O2 -march=native
    /* 
    if (n==0)
        return 1;
    else
        return ((n&0x1) ? x : 1)*cipow(x*x,n>>1);
    */
}

//! Compile-time string length
template <std::size_t N>
constexpr std::size_t strlength(char const (&str)[N]) noexcept
{
    std::size_t len = 0;
    while(str[len] && len < N)
        ++len;

    return len;
}

template <
    class T,
    std::enable_if_t<
        std::is_same<T,char const*>::value,
        int
        > = 0
    >
constexpr std::size_t strlength(T str) noexcept
{
    std::size_t len = 0;
    while(str[len])
        ++len;

    return len;
}

template <class... Ts>
constexpr void ignoreall(Ts&&...) noexcept
{}

template <size_t... is, class... Ts>
constexpr void tupbind_impl(std::index_sequence<is...>, std::tuple<Ts...> const &tup, std::decay_t<Ts>&... var) noexcept
{
    ignoreall(std::get<is>(std::tie(var...))=std::get<is>(tup)...);
}

template <class... Ts>
constexpr void tupbind(std::tuple<Ts...> const &tup, std::decay_t<Ts>&... var) noexcept
{
    tupbind_impl(std::index_sequence_for<Ts...>(), tup, var...);
}

template <std::size_t x, std::size_t y>
struct firstoftwo {
    enum : std::size_t { value = x };
};


/************************!
 * \section Algorithms
 ************************/
struct NullOutIterator
    : std::iterator<std::output_iterator_tag, NullOutIterator>
{
    template<typename T>
    constexpr NullOutIterator& operator=(T const&) noexcept { return *this; }

    constexpr NullOutIterator& operator++() noexcept { return *this; }

    constexpr NullOutIterator operator+=(int) noexcept { return *this; }
    constexpr NullOutIterator& operator*() noexcept { return *this; }
    constexpr NullOutIterator const& operator*() const noexcept { return *this; }
};

template<
    class InputIterator,
    class F,
    class Target = std::result_of_t<
        F&&(std::remove_cv_t<std::remove_reference_t<decltype(*std::declval<InputIterator>())>>&&)
        >,
    class BinOp = std::plus<Target>
    >
constexpr inline auto accumMap(
    InputIterator first,
    InputIterator last,
    F&& f,
    Target init = Target{},
    BinOp&& bin = std::plus<Target>()
    ) noexcept(std::is_nothrow_assignable<Target, Target>::value)
    -> Target
{
    while(first != last) {
        init = bin(std::move(init), f(*first));
        ++first;
    }

    return init;
}


/****************************!
 * \section Combinatorials
 ****************************/

//! Binomial coefficients
template<
    class T = std::size_t,
    class = std::enable_if_t<
        std::is_integral<T>::value && !std::is_same<T,bool>::value
        >
    >
struct binom {
    static constexpr T get(T n, T k)
    {
        if (k < 0 || k > n)
            return 0;

        if (k > n-k)
            k = n-k;

        if (k == 0)
            return 1;

        T result = 1;

        for (T i = 0; i < k; ++i)
        {
            result *= n-i;
            result /= (i+1);
        }

        return result;
    }

    template<class U, std::size_t n, std::size_t N=n+1>
    static constexpr std::array<U,N> getArray()
    {
        return getArray_impl<U,n>(std::make_index_sequence<N>());
    }

    template<class U, std::size_t n, std::size_t N=n+1>
    static constexpr std::array<U,N> getArrayRev()
    {
        return getArray_impl<U,n>(bord2::make_reversed_index_seq<N>());
    }

private:
    template<class U, std::size_t n, std::size_t... is>
    static constexpr auto getArray_impl(std::index_sequence<is...> const &dummy)
        -> std::array<U,sizeof...(is)>
    {
        return {static_cast<U>(get(n,is))...};
    }
};

} // end namespace bord2
