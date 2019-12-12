/*!
 * \file WxGSScheme.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#define _USE_MATH_DEFINES

#include "WxGSScheme.hpp"

#include <iostream>
#include <cmath>
#include <wx/gdicmn.h>

/******************************
 *** Utility functions.
 ******************************/
//! Convert Color to wxColour
wxColour getWxColor(const bord2::PathColor &col)
{
    switch (col) {
    case bord2::Black:
        return *wxBLACK;

    case bord2::White:
        return *wxWHITE;

    case bord2::Red:
        return *wxRED;

    case bord2::Green:
        return *wxGREEN;

    case bord2::Blue:
        return *wxBLUE;

    case bord2::Cyan:
        return *wxCYAN;

    case bord2::Magenta:
        return wxTheColourDatabase->Find("MAGENTA");

    case bord2::Yellow:
        return *wxYELLOW;

    case bord2::Gray:
        return wxTheColourDatabase->Find("GREY");

    case bord2::Brown:
        return wxTheColourDatabase->Find("BROWN");

    case bord2::Lime:
        return wxTheColourDatabase->Find("LIME GREEN");

    case bord2::Olive:
        return wxTheColourDatabase->Find("DARK OLIVE GREEN");

    case bord2::Orange:
        return wxTheColourDatabase->Find("ORANGE");

    case bord2::Pink:
        return wxTheColourDatabase->Find("PINK");

    case bord2::Purple:
        return wxTheColourDatabase->Find("PURPLE");

    case bord2::Teal:
        return wxColour(0, 0x80, 0x80);

    case bord2::Violet:
        return wxTheColourDatabase->Find("VIOLET");

    default:
        std::cerr << "Unknown color: " << col
                  << " (" << __FILE__ << "," << __LINE__ << ")"
                  << std::endl;
        return wxColour();
    }
}


/********************************
 *** Non-derived methods.
 ********************************/
//! Constructor
WxGSScheme::WxGSScheme(const wxWindowDC &dc)
    : mp_gc(wxGraphicsContext::Create(dc)), mp_path()
{
    mp_gc->SetBrush(*wxWHITE_BRUSH);
    mp_gc->SetPen(*wxBLACK_PEN);
    WxGSScheme::clear();
}

//! Clear paths.
void WxGSScheme::clear()
{
    if(mp_gc)
        mp_path = std::make_unique<wxGraphicsPath>(mp_gc->CreatePath());
}


/*********************************
 ***  Overriding methods
 *********************************/
//** Pens and Brushes
void WxGSScheme::setPen(double wid, bord2::PathColor col, bord2::StrokePattern pat)
{
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

void WxGSScheme::setBrush(bord2::PathColor col)
{
    mp_gc->SetBrush(wxBrush(getWxColor(col)));
}

//* Drawing paths.
//! Stroke without flush
void WxGSScheme::strokePres()
{
    mp_gc->StrokePath(*mp_path);
}

//! Fill without flush
void WxGSScheme::fillPres()
{
    mp_gc->FillPath(*mp_path);
}

//* Path elements.
//! Move the current position.
void WxGSScheme::moveTo(WxGSScheme::vertex_type const &p)
{
    mp_path->MoveToPoint(p[0], p[1]);
}

//! Move the current position drawing a line from the old.
void WxGSScheme::lineTo(WxGSScheme::vertex_type const &p)
{
    mp_path->AddLineToPoint(p[0],p[1]);
}

//! Move the current position drawing a cubic Bezier curve.
void WxGSScheme::bezierTo(vertex_type const &c1, vertex_type const &c2, vertex_type const &p)
{
    mp_path->AddCurveToPoint(c1[0], c1[1], c2[0], c2[1], p[0], p[1]);
}

//! Move the current position drawing a quadratic Bezier curve.
void WxGSScheme::qbezierTo(vertex_type const &c, vertex_type const &p)
{
    mp_path->AddQuadCurveToPoint(c[0], c[1], p[0], p[1]);
}

//! Close path.
void WxGSScheme::closePath()
{
    mp_path->CloseSubpath();
}

