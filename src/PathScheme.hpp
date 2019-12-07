/*!
 * \file PathScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#pragma once

#include <array>

//! Interface class providing path-based rendering.
class PathScheme {
public:
    //! The list of colors (borrowed from TikZ).
    enum Color {
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
    virtual void setPen(double wid, Color col = Black, StrokePattern pat = Solid) = 0;
    virtual void setBrush(Color col) = 0;

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
    virtual void moveTo(double x, double y) = 0;
    //! Move the current position drawing a line from the old.
    virtual void lineTo(double x, double y) = 0;
    //! Move the current position drawing a cubic Bezier curve.
    virtual void bezierTo(double c1x, double c1y, double c2x, double c2y, double x, double y) = 0;
    //! Move the current position drawing a quadratic Bezier curve.
    virtual void qbezierTo(double cx, double cy, double x, double y) = 0;
    //! Close path.
    virtual void closePath() = 0;

    //* Commonly used shapes.
    void rectangle(double lx, double ty, double rx, double by) {
        moveTo(lx, ty);
        lineTo(lx, by);
        lineTo(rx, by);
        lineTo(rx, ty);
        closePath();
    }
};

//! Base class for figures which draw themselves using PathScheme.
class PathFigure {
public:
    PathFigure() = default;
    virtual ~PathFigure() = default;

    virtual void draw(PathScheme&) const = 0;
};
