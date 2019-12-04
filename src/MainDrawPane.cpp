/*!
 * \file MainDrawPane.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include <memory>
#include <wx/graphics.h>

#include "MainDrawPane.hpp"

BEGIN_EVENT_TABLE(MainDrawPane, wxPanel)

EVT_PAINT(MainDrawPane::OnPaint)

END_EVENT_TABLE()

//! Constructor
MainDrawPane::MainDrawPane(wxFrame *parent)
  : wxPanel(parent)
{
}

//! Paint Event handler
void MainDrawPane::OnPaint(wxPaintEvent &event)
{
    MainDrawPane::render(wxPaintDC(this));
}

//! The implementation of rendering.
void MainDrawPane::render(wxWindowDC &&dc)
{
    // Clear
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();

    std::unique_ptr<wxGraphicsContext> p_gc(wxGraphicsContext::Create(dc));

    if(!p_gc)
        throw Error::FailedCreatingGC;

    p_gc->SetPen(*wxRED_PEN);
    p_gc->SetBrush(*wxGREEN_BRUSH);

    wxGraphicsPath path = p_gc->CreatePath();
    path.MoveToPoint(400.0, 400.0);
    path.AddQuadCurveToPoint(500.0, 400.0, 500.0, 300.0);
    path.AddQuadCurveToPoint(500.0 ,200.0, 400.0, 200.0);
    path.AddQuadCurveToPoint(300.0, 200.0, 300.0, 300.0);
    path.AddQuadCurveToPoint(300.0, 400.0, 400.0, 400.0);
    path.CloseSubpath();
    path.AddCircle(400.0, 300.0, 120.0);
    p_gc->StrokePath(path);
    p_gc->FillPath(path);
}
