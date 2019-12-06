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
wxColour getWxColor(const WxGSScheme::Color &col)
{
    switch (col) {
    case WxGSScheme::Black:
        return *wxBLACK;

    case WxGSScheme::White:
        return *wxWHITE;

    case WxGSScheme::Red:
        return *wxRED;

    case WxGSScheme::Green:
        return *wxGREEN;

    case WxGSScheme::Blue:
        return *wxBLUE;

    case WxGSScheme::Cyan:
        return *wxCYAN;

    case WxGSScheme::Magenta:
        return wxTheColourDatabase->Find("MAGENTA");

    case WxGSScheme::Yellow:
        return *wxYELLOW;

    case WxGSScheme::Gray:
        return wxTheColourDatabase->Find("GREY");

    case WxGSScheme::Brown:
        return wxTheColourDatabase->Find("BROWN");

    case WxGSScheme::Lime:
        return wxTheColourDatabase->Find("LIME GREEN");

    case WxGSScheme::Olive:
        return wxTheColourDatabase->Find("DARK OLIVE GREEN");

    case WxGSScheme::Orange:
        return wxTheColourDatabase->Find("ORANGE");

    case WxGSScheme::Pink:
        return wxTheColourDatabase->Find("PINK");

    case WxGSScheme::Purple:
        return wxTheColourDatabase->Find("PURPLE");

    case WxGSScheme::Teal:
        return wxColour(0, 0x80, 0x80);

    case WxGSScheme::Violet:
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
void WxGSScheme::setPen(double wid, WxGSScheme::Color col, WxGSScheme::StrokePattern pat)
{
    wxPenStyle wxpen;

    switch (pat) {
    case Solid:
        wxpen = wxPENSTYLE_SOLID;
        break;

    case Dotted:
        wxpen = wxPENSTYLE_DOT;
        break;

    case Dashed:
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

void WxGSScheme::setBrush(Color col)
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
void WxGSScheme::moveTo(double x, double y)
{
    mp_path->MoveToPoint(x, y);
}

//! Move the current position drawing a line from the old.
void WxGSScheme::lineTo(double x, double y)
{
    mp_path->AddLineToPoint(x,y);
}

//! Move the current position drawing a cubic Bezier curve.
void WxGSScheme::bezierTo(double c1x, double c1y, double c2x, double c2y, double x, double y)
{
    mp_path->AddCurveToPoint(c1x, c1y, c2x, c2y, x, y);
}

//! Move the current position drawing a quadratic Bezier curve.
void WxGSScheme::qbezierTo(double cx, double cy, double x, double y)
{
    mp_path->AddQuadCurveToPoint(cx, cy, x, y);
}

//! Close path.
void WxGSScheme::closePath()
{
    mp_path->CloseSubpath();
}

