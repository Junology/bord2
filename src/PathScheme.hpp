/*!
 * \file PathScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#pragma once

#include <array>
#include <memory>
#include <functional>

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

    virtual bool isvalid() const noexcept = 0;

    //* Pens and Brushes
    virtual void setPen(double wid, bord2::PathColor col = bord2::Black, bord2::StrokePattern pat = bord2::Solid) = 0;
    virtual void setBrush(bord2::PathColor col) = 0;

    //! Translate the origin.
    virtual void translate(vertex_type const& p) = 0;

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

//! Adapter connecting PathScheme with different vertex_type.
//! \tparam U The vertex_type of an underlying PathScheme.
//! \tparam V The target vertex_type.
template <class U, class V>
class AdapterScheme : public PathScheme<V>
{
public:
    using BBoxT = typename PathScheme<V>::BBoxT;
    using source_vertex_type = U;
    using target_vertex_type = V;

private:
    std::unique_ptr<PathScheme<U>> m_scheme;
    std::function<U(V)> m_f;

public:

    //! Constructor
    //! \param scheme A pointer to an instance of the underlying scheme. Note that it is stored in std::unique_ptr, so the user must not delete it. Ideally, it is passed directly by new operator. If you reuse the pointer, \see release().
    //! \param f An adapter of two vertex_type.
    template <class F>
    AdapterScheme(PathScheme<U> *p_scheme, F&& f)
        : m_scheme(p_scheme), m_f(std::forward<F>(f))
    {}

    //! Move constructive
    AdapterScheme(AdapterScheme &&src)
        : m_scheme(std::move(src.m_scheme)), m_f(std::move(src.m_f))
    {}

    //! Destructor.
    virtual ~AdapterScheme() = default;

    //! Release the pointer.
    //! This function just calls std::unique_ptr::release().
    PathScheme<U>* release() {
        return m_scheme.release();
    }

    //* Implementations of pure virtual functions
    virtual BBoxT getBBox() const override { return m_scheme->getBBox(); }

    virtual std::array<double,2> getCenter() const override {
        return m_scheme->getCenter();
    }

    virtual bool isvalid() const noexcept override {
        return m_scheme && m_scheme->isvalid();
    }

    virtual void setPen(double wid, bord2::PathColor col = bord2::Black, bord2::StrokePattern pat = bord2::Solid) override {
        m_scheme->setPen(wid, col, pat);
    }

    virtual void setBrush(bord2::PathColor col) override {
        m_scheme->setBrush(col);
    }

    virtual void translate(target_vertex_type const &p) override {
        m_scheme->translate(m_f(p));
    }

    virtual void stroke() override { m_scheme->stroke(); }
    virtual void strokePres() override { m_scheme->strokePres(); }
    virtual void fill() override { m_scheme->fill(); }
    virtual void fillPres() override { m_scheme->fillPres(); }

    virtual void moveTo(target_vertex_type const &p) {
        m_scheme->moveTo(m_f(p));
    }

    virtual void lineTo(target_vertex_type const &p) {
        m_scheme->lineTo(m_f(p));
    }

    virtual void bezierTo(target_vertex_type const &c1, target_vertex_type const &c2, target_vertex_type const &p) {
        m_scheme->bezierTo(m_f(c1), m_f(c2), m_f(p));
    }

    virtual void qbezierTo(target_vertex_type const &c, target_vertex_type const &p) {
        m_scheme->qbezierTo(m_f(c), m_f(p));
    }

    virtual void closePath() { m_scheme->closePath(); }
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
