/*!
 * \file WxGSScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#pragma once

#include <utility>
#include <memory>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#    include <wx/graphics.h>
#endif

#include "PathScheme.hpp"

class WxGSScheme : public PathScheme
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
    std::pair<double,double> getCenter() override {
        auto sz = getSize();
        return {sz.first/2.0, sz.second/2.0};
    }
    std::pair<double,double> getSize() override {
        std::pair<double,double> sz(0.0, 0.0);
        if(mp_gc) {
            mp_gc->GetSize(&sz.first, &sz.second);
        }
        return sz;
    }

    //** Pens and Brushes
    void setPen(double wid, Color col = Black, StrokePattern pat = Solid) override;
    void setBrush(Color col) override;

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

    //** Path elements.
    //! Move the current position.
    void moveTo(double x, double y) override;
    //! Move the current position drawing a line from the old.
    void lineTo(double x, double y) override;
    //! Move the current position drawing a cubic Bezier curve.
    void bezierTo(double c1x, double c1y, double c2x, double c2y, double x, double y) override;
    //! Move the current position drawing a quadratic Bezier curve.
    void qbezierTo(double cx, double cy, double x, double y) override;
    //! Close path.
    void closePath() override;
};
