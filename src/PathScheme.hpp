/*!
 * \file PathScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#pragma once

#include <array>

namespace bord2 {

//! The list of colors (borrowed from TikZ).
enum PathColor {
    Black,
    White,
    Red,
    Green,
    Blue,
    Cyan,
    Magenta,
    Yellow,
    Gray,
    Brown,
    Lime,
    Olive,
    Orange,
    Pink,
    Purple,
    Teal,
    Violet
};

//! Stroke patterns
enum StrokePattern {
    Solid,
    Dotted,
    Dashed
};

} // end namespace bord2

//! Interface class providing path-based rendering.
template<class T>
class PathScheme {
public:
    using vertex_type = T;

    PathScheme() = default;
    virtual ~PathScheme() = default;

    //* Scheme data query
    using BBoxT = typename std::array<double,4>;
    //! Get the bounding box of the drawing area.
    //! \return The array of the form {left, top, right, bottom}.
    virtual BBoxT getBBox() const = 0;

    virtual std::array<double,2> getCenter() const {
        auto bbox = getBBox();
        return {(bbox[0]+bbox[2])/2.0, (bbox[1]+bbox[3])/2.0};
    }

    //* Pens and Brushes
    virtual void setPen(double wid, bord2::PathColor col = bord2::Black, bord2::StrokePattern pat = bord2::Solid) = 0;
    virtual void setBrush(bord2::PathColor col) = 0;

    //* Drawing paths.
    //! Stroke and flush.
    virtual void stroke() = 0;
    //! Stroke without flush
    virtual void strokePres() = 0;
    //! Fill and flush.
    virtual void fill() = 0;
    //! Fill without flush
    virtual void fillPres() = 0;

    //* Path elements.
    //! Move the current position.
    virtual void moveTo(vertex_type const &p) = 0;
    //! Move the current position drawing a line from the old.
    virtual void lineTo(vertex_type const &p) = 0;
    //! Move the current position drawing a cubic Bezier curve.
    virtual void bezierTo(vertex_type const &c1, vertex_type const &c2, vertex_type const &p) = 0;
    //! Move the current position drawing a quadratic Bezier curve.
    virtual void qbezierTo(vertex_type const &c, vertex_type const &p) = 0;
    //! Close path.
    virtual void closePath() = 0;
};

//! Base class for figures which draw themselves using PathScheme.
template <class T>
class PathFigure {
public:
    using vertex_type = T;
    using SchemeType = PathScheme<vertex_type>;

    PathFigure() = default;
    virtual ~PathFigure() = default;

    virtual void draw(SchemeType&) const = 0;
};
