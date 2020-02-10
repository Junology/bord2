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
//#    include <wx/gdicmn.h>
#endif

#include "PathScheme.hpp"

//! Query wxColour from color table.
wxColour getWxColor(const bord2::PathColor &col);

//! Implementation of PathScheme in terms of wxGraphicsContext.
class WxGSScheme : public PathScheme<std::array<double,2> >
{
private:
    std::unique_ptr<wxGraphicsContext> mp_gc;
    std::unique_ptr<wxGraphicsPath> mp_path;

public:
    //! Constructor
    WxGSScheme(const wxWindowDC &dc)
        : mp_gc(wxGraphicsContext::Create(dc)), mp_path()
    {
        if(mp_gc) {
            mp_gc->SetBrush(*wxWHITE_BRUSH);
            mp_gc->SetPen(*wxBLACK_PEN);
        }

        WxGSScheme::clear();
    }

    virtual ~WxGSScheme() = default;

    //! Clear paths.
    void clear() {
        if(mp_gc)
            mp_path = std::make_unique<wxGraphicsPath>(mp_gc->CreatePath());
    }


    //* Overriding methods.
    //** Scheme data query
    BBoxT getBBox() const override {
        BBoxT sz{0.0, 0.0, 0.0, 0.0};
        if(mp_gc) {
            mp_gc->GetSize(&(sz[2]), &(sz[3]));
        }
        return sz;
    }

    //! Check if the instance is valid or not.
    bool isvalid() const noexcept override {
        return static_cast<bool>(mp_gc);
    }

    //** Pens and Brushes
    void setPen(double wid, bord2::PathColor col = bord2::Black, bord2::StrokePattern pat = bord2::Solid) override {
        wxPenStyle wxpen;

        switch (pat) {
        case bord2::Solid:
            wxpen = wxPENSTYLE_SOLID;
            break;

        case bord2::Dotted:
            wxpen = wxPENSTYLE_DOT;
            break;

        case bord2::Dashed:
            wxpen = wxPENSTYLE_SHORT_DASH;
            break;

        default:
            std::cerr << "Unknown brush: " << pat
                      << " (" << __FILE__ << "," << __LINE__ << ")"
                      << std::endl;
            return;
        }

        mp_gc->SetPen(wxPen(getWxColor(col), static_cast<int>(wid), wxpen));
    }

    void setBrush(bord2::PathColor col) override {
        mp_gc->SetBrush(wxBrush(getWxColor(col)));
    }


    void translate(std::array<double,2> const& p) override {
        mp_gc->Translate(p[0], p[1]);
    }

    void save() override {
        mp_gc->PushState();
    }

    void restore() override {
        mp_gc->PopState();
    }

    //** Drawing paths.
    //! Stroke and flush.
    void stroke() override {
        strokePres();
        clear();
    }

    //! Stroke without flush
    void strokePres() override {
        mp_gc->StrokePath(*mp_path);
    }

    //! Fill and flush.
    void fill() override {
        fillPres();
        clear();
    }

    //! Fill without flush
    void fillPres() override {
        mp_gc->FillPath(*mp_path);
    }

    /* Path elements.
    //! Move the current position.
    virtual void moveTo(vertex_type const &p) override {
        mp_path->MoveToPoint(p[0], p[1]);
    }

    //! Move the current position drawing a line from the old.
    virtual void lineTo(vertex_type const &p) override {
        mp_path->AddLineToPoint(p[0],p[1]);
    }

    //! Move the current position drawing a cubic Bezier curve.
    virtual void bezierTo(vertex_type const &c1, vertex_type const &c2, vertex_type const &p) override {
        mp_path->AddCurveToPoint(c1[0], c1[1], c2[0], c2[1], p[0], p[1]);
    }

    //! Move the current position drawing a quadratic Bezier curve.
    virtual void qbezierTo(vertex_type const &c, vertex_type const &p) override {
        mp_path->AddQuadCurveToPoint(c[0], c[1], p[0], p[1]);
    }

    //! Close path.
    virtual void closePath() override {
        mp_path->CloseSubpath();
    }
    */

protected:
    using PathElement = PathScheme<vertex_type>::PathElement;

    virtual void putPathElement(PathElement const& elem) override;
};
