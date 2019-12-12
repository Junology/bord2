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
#include <vector>

namespace bord2 {

/************************************
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

    static std::false_type is_pubbase_of_template_implfunc(...);

public:
    using type = decltype(is_pubbase_of_template_implfunc(std::declval<Derived*>()));
};

template <template <class...> class TBase, class Derived>
using is_pubbase_of_template = typename is_pubbase_of_template_impl<TBase, Derived>::type;

} // end namespace bord2
