/*!
 * \file MainDrawPane.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include <iostream>
#include <memory>
#include <wx/graphics.h>

#include "MainDrawPane.hpp"
#include "figures/figures.hpp"

BEGIN_EVENT_TABLE(MainDrawPane, wxPanel)

EVT_PAINT(MainDrawPane::OnPaint)
EVT_KEY_DOWN(MainDrawPane::OnKeyDown)

END_EVENT_TABLE()

//! Constructor
MainDrawPane::MainDrawPane(wxFrame *parent)
    : wxPanel(parent), mp_fig3d(), m_elev(0.0), m_azim(0.0)
{
    mp_fig3d.reset(
        new QBezierLine3D(
            {200.0, 0.0, 400.0},
            {300.0, 0.0, 300.0},
            {400.0, 0.0, 400.0}
            )
        );
    mp_fig3d->setFocus(400.0, 0.0, 300.0);
    mp_fig3d->setAngles(m_elev, m_azim);
}

//! Paint Event handler
void MainDrawPane::OnPaint(wxPaintEvent &event)
{
    MainDrawPane::render(wxPaintDC(this));
}

//! KeyDown event handler
void MainDrawPane::OnKeyDown(wxKeyEvent &event)
{
    /* Debug
    std::cout << "Debug (" << __FILE__ << ":" << __LINE__ << ")" << std::endl;
    std::cout << event.GetKeyCode() << std::endl;
    // */

    switch(event.GetKeyCode()) {
    case WXK_LEFT:
        m_azim -= 3.0;
        break;

    case WXK_RIGHT:
        m_azim += 3.0;
        break;

    case WXK_UP:
        m_elev += 3.0;
        break;

    case WXK_DOWN:
        m_elev -= 3.0;
        break;
    }

    mp_fig3d->setAngles(m_elev, m_azim);
    this->Refresh();
}

//! The implementation of rendering.
void MainDrawPane::render(wxWindowDC &&dc)
{
    // Clear
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();

    WxGSScheme wxgs(dc);
    mp_fig3d->draw(wxgs);
}
