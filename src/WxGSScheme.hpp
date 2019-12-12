/*!
 * \file WxGSScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include <utility>
#include <array>
#include <memory>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#    include <wx/graphics.h>
#endif

#include "PathScheme.hpp"

class WxGSScheme : public PathScheme<std::array<double,2> >
{
private:
    std::unique_ptr<wxGraphicsContext> mp_gc;
    std::unique_ptr<wxGraphicsPath> mp_path;

public:
    WxGSScheme(const wxWindowDC &dc);
    virtual ~WxGSScheme() = default;

    void clear();

    //* Overriding methods.
    //** Scheme data query
    BBoxT getBBox() const override {
        BBoxT sz{0.0, 0.0, 0.0, 0.0};
        if(mp_gc) {
            mp_gc->GetSize(&(sz[2]), &(sz[3]));
        }
        return sz;
    }

    //** Pens and Brushes
    void setPen(double wid, bord2::PathColor col = bord2::Black, bord2::StrokePattern pat = bord2::Solid) override;
    void setBrush(bord2::PathColor col) override;

    //** Drawing paths.
    //! Stroke and flush.
    void stroke() override {
        strokePres();
        clear();
    }
    //! Stroke without flush
    void strokePres() override;
    //! Fill and flush.
    void fill() override {
        fillPres();
        clear();
    }
    //! Fill without flush
    void fillPres() override;

    //* Path elements.
    //! Move the current position.
    virtual void moveTo(vertex_type const &p) override;
    //! Move the current position drawing a line from the old.
    virtual void lineTo(vertex_type const &p) override;
    //! Move the current position drawing a cubic Bezier curve.
    virtual void bezierTo(vertex_type const &c1, vertex_type const &c2, vertex_type const &p) override;
    //! Move the current position drawing a quadratic Bezier curve.
    virtual void qbezierTo(vertex_type const &c, vertex_type const &p) override;
    //! Close path.
    virtual void closePath() override;
};
