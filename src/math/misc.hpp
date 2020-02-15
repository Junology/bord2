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

//! Check if two triangles overwraps with each other.
//! Based on the idea suggested in https://stackoverflow.com/questions/2778240/detection-of-triangle-collision-in-2d-space.
inline bool trianglesIntersection2D(std::array<Eigen::Vector2d,3> const &t1_, std::array<Eigen::Vector2d,3> const &t2_)
{
    /*** Preparation ***/
    // Rotation matrix of M_PI/2 in radian.
    Eigen::Matrix<double,2,2> mat;
    mat << 0, 1, -1, 0;

    // We may assume the vertices are given counter-clockwisely around triangles.
    bool is_ccwise1 = static_cast<double>((t1_[1]-t1_[0]).adjoint()*mat*(t1_[2]-t1_[0])) > 0;
    bool is_ccwise2 = static_cast<double>((t2_[1]-t2_[0]).adjoint()*mat*(t2_[2]-t2_[0])) > 0;
    std::array<std::array<Eigen::Vector2d const*, 3>,2> t{
        &(t1_[0]),
        is_ccwise1 ? &(t1_[1]) : &(t1_[2]),
        is_ccwise1 ? &(t1_[2]) : &(t1_[1]),
        &(t2_[0]),
        is_ccwise2 ? &(t2_[1]) : &(t2_[2]),
        is_ccwise2 ? &(t2_[2]) : &(t2_[1])
    };

    /*** The algorithm begins here ***/

    // Find an edge in one triangle which separates the opposite vertex and the vertices of the other triangle.
    // If found, this means two triangles are disjoint.
    for(size_t i = 0; i < 3; ++i) {
        Eigen::RowVector2d rv = (*(t[0][(i+1)%3])-*(t[0][i])).adjoint()*mat;
        if (static_cast<double>(rv*(*(t[1][0])-*(t[0][i]))) < 0
            && static_cast<double>(rv*(*(t[1][1])-*(t[0][i]))) < 0
            && static_cast<double>(rv*(*(t[1][2])-*(t[0][i]))) < 0)
            return false;
    }

    for(size_t i = 0; i < 3; ++i) {
        Eigen::RowVector2d rv = (*(t[1][(i+1)%3])-*(t[1][i])).adjoint()*mat;
        if (static_cast<double>(rv*(*(t[0][0])-*(t[1][i]))) < 0
            && static_cast<double>(rv*(*(t[0][1])-*(t[1][i]))) < 0
            && static_cast<double>(rv*(*(t[0][2])-*(t[1][i]))) < 0)
            return false;
    }

    return true;
}
