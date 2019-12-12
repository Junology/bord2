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

namespace bord2 {

/************************************!
 * \section Convenient type traits
 ************************************/

//! The type which verifies all the template parameters are true.
template <bool... bs>
struct all_of;

template <>
struct all_of<> : std::true_type {};

template <bool... bs>
struct all_of<false, bs...> : std::false_type {};

template <bool... bs>
struct all_of<true, bs...> : all_of<bs...> {};

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
template <size_t N>
class make_reversed_index_seq_impl{
private:
    template <size_t... is>
    static constexpr auto reverse_impl(std::index_sequence<is...>)
        -> std::index_sequence<(sizeof...(is)-is-1)...>;

public:
    using type = decltype(reverse_impl(std::declval<std::make_index_sequence<N> >()));
};

template <size_t N>
using make_reversed_index_seq = typename make_reversed_index_seq_impl<N>::type;

/****************************!
 * \section Combinatorials
 ****************************/

//! Binomial coefficients
template<
    class T = size_t,
    class = std::enable_if_t<
        std::is_integral<T>::value && !std::is_same<T,bool>::value
        >
    >
struct binom {
    static constexpr T get(T n, T k)
    {
        T result = 1;

        if (k < 0 || k > n)
            return 0;

        if (k > n-k)
            k = n-k;

        if (k == 0)
            return 1;

        for (T i = 0; i < k; ++i)
        {
            result *= n-i;
            result /= (i+1);
        }

        return result;
    }

    template<class U, size_t n, size_t N=n+1>
    static constexpr std::array<U,N> getArray()
    {
        return getArray_impl<U,n>(std::make_index_sequence<N>());
    }

    template<class U, size_t n, size_t N=n+1>
    static constexpr std::array<U,N> getArrayRev()
    {
        return getArray_impl<U,n>(bord2::make_reversed_index_seq<N>());
    }

private:
    template<class U, size_t n, size_t... is>
    static constexpr auto getArray_impl(std::index_sequence<is...> const &dummy)
        -> std::array<U,sizeof...(is)>
    {
        return {static_cast<U>(get(n,is))...};
    }
};

} // end namespace bord2
