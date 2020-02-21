/*!
 * \file PlainGeometry.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 20, 2020: created
 */

#pragma once

#include <vector>
#include <Eigen/Dense>

namespace {
enum Orientation {
    OnLine,
    Clockwise,
    CntrClockwise
};

Orientation orientation(
    Eigen::Vector2d const& p0,
    Eigen::Vector2d const& p1,
    Eigen::Vector2d const& p2) noexcept;

//! Compute the convex hull.
//! \param A set of vertices.
//! \return A list of control points spanning the hull. They are stored in the counter-clockwise order.
std::vector<Eigen::Vector2d> getConvexHull(
    std::vector<Eigen::Vector2d> const& vs) noexcept;

//! Check if the convex hull of control points overlaps with that of another Bezier curve.
//! Here, so-called *The Separating Axis Theorem* is used.
bool hasHullIntersection(
    std::vector<Eigen::Vector2d> const& lhs,
    std::vector<Eigen::Vector2d> const& rhs) noexcept;

}
