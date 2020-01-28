/*!
 * \file MainDrawPane.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include "MainDrawPane.hpp"

#include <iostream>
#include <memory>
#include <wx/graphics.h>

#include "WxGSScheme.hpp"
#include "OrthoSpatialScheme.hpp"

#include "figures/figures.hpp"

BEGIN_EVENT_TABLE(MainDrawPane, wxPanel)

EVT_PAINT(MainDrawPane::OnPaint)
EVT_KEY_DOWN(MainDrawPane::OnKeyDown)

END_EVENT_TABLE()

//! Constructor
MainDrawPane::MainDrawPane(wxWindow *parent)
  : wxPanel(parent),
//    mp_fig3d(),
    m_figs(),
    m_elev(0.0),
    m_azim(0.0),
    m_focus({400.0, 0.0, 300.0})
{
    m_figs.emplace_back(
        new QBezierLine3D(
            {200.0, 0.0, 400.0},
            {300.0, 0.0, 300.0},
            {400.0, 0.0, 400.0}
            )
        );

    m_figs.emplace_back(
        new QBezierTriangle(
            {600.0, 0.0, 300.0},
            {400.0, 200.0, 300.0},
            {400.0, 0.0, 500.0},
            {400.0, 200.0, 500.0},
            {600.0, 0.0, 500.0},
            {600.0, 200.0, 300.0} ));
    m_figs.emplace_back(
        new QBezierTriangle(
            {200.0, 0.0, 300.0},
            {400.0, 200.0, 300.0},
            {400.0, 0.0, 500.0},
            {400.0, 200.0, 500.0},
            {200.0, 0.0, 500.0},
            {200.0, 200.0, 300.0} ));
    m_figs.emplace_back(
        new QBezierTriangle(
            {600.0, 0.0, 300.0},
            {400.0, -200.0, 300.0},
            {400.0, 0.0, 500.0},
            {400.0, -200.0, 500.0},
            {600.0, 0.0, 500.0},
            {600.0, -200.0, 300.0} ));
    m_figs.emplace_back(
        new QBezierTriangle(
            {200.0, 0.0, 300.0},
            {400.0, -200.0, 300.0},
            {400.0, 0.0, 500.0},
            {400.0, -200.0, 500.0},
            {200.0, 0.0, 500.0},
            {200.0, -200.0, 300.0} ));
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

    double t = M_PI*m_elev/180.0;
    double u = M_PI*m_azim/180.0;

    Eigen::Matrix<double,2,3> projmat;
    projmat <<
        cos(u),       -sin(u), 0,
        // cos(t)*sin(u), cos(t)*cos(u), -sin(t),
        sin(t)*sin(u), sin(t)*cos(u), cos(t);

    // Inform the change of the angles.
//    mp_fig3d->updateProjector(projmat);
    for(auto& fig : m_figs)
        fig->updateProjector(projmat);

    this->Refresh();
}

//! The implementation of rendering.
void MainDrawPane::render(wxWindowDC &&dc)
{
    // Clear
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();

    OrthoSpatialScheme<WxGSScheme> wxgs(m_focus, m_elev, m_azim, std::forward<wxWindowDC>(dc));
//    mp_fig3d->draw(wxgs);
    for(auto &fig : m_figs)
        fig->draw(wxgs);
}
