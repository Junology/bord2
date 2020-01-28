/*!
 * \file misc.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 21, 2019: created
 */

#pragma once

#include <type_traits>
#include <Eigen/Dense>

/*************************
 *** Utility functions ***
 *************************/

/*
double cross2D(Eigen::Ref<Eigen::Vector2d> const &x, Eigen::Ref<Eigen::Vector2d> const &y)
{
    return x(0)*y(1)-x(1)*y(0);
}
*/

inline double cross2D(Eigen::Vector2d &&x, Eigen::Vector2d &&y)
{
    return x(0)*y(1)-x(1)*y(0);
}

template<class T, int n, size_t... is>
Eigen::Matrix<T,n+1,1> affWrap_impl(Eigen::Matrix<T,n,1> const &mat, std::index_sequence<is...>)
{
    return Eigen::Matrix<T,n+1,1>{mat(is)...,1.0};
}

template<class T,int n>
Eigen::Matrix<T,n+1,1> affWrap(Eigen::Matrix<T,n,1> const &mat)
{
    return affWrap_impl(mat, std::make_index_sequence<n>());
}
