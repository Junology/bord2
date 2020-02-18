/*!
 * \file Figure3DView.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include "Figure3DView.hpp"

#include <wx/graphics.h>

#include "WxGSScheme.hpp"
#include "ProjSpatialScheme.hpp"

#include "figures/figures.hpp"

#include <iostream> // For Debug

IMPLEMENT_DYNAMIC_CLASS(Figure3DView, wxPanel)

BEGIN_EVENT_TABLE(Figure3DView, wxPanel)

EVT_PAINT(Figure3DView::OnPaint)
EVT_KEY_DOWN(Figure3DView::OnKeyDown)

END_EVENT_TABLE()


/*************************
 *** Utility functions ***
 *************************/
Eigen::Matrix<double,2,3> getOrthoProjMat(double elev, double azim) noexcept
{
    double t = M_PI*elev/180.0;
    double u = M_PI*azim/180.0;

    Eigen::Matrix<double,2,3> projmat;
    projmat <<
        cos(u),       -sin(u), 0,
        // cos(t)*sin(u), cos(t)*cos(u), -sin(t),
        sin(t)*sin(u), sin(t)*cos(u), cos(t);

    return projmat;
}

Eigen::Matrix<double,2,3> getCabinetProjMat(double elev, double azim) noexcept
{
    double t = M_PI*elev/180.0;
    double u = M_PI*azim/180.0;

    Eigen::Matrix<double,2,3> projmat;
    projmat <<
        1.0, sin(elev)*cos(azim), 0.0,
        0.0, sin(elev)*sin(azim), 1.0;

    return projmat;
}

/************************
 *** Member functions ***
 ************************/

//! Paint Event handler
void Figure3DView::OnPaint(wxPaintEvent &event) {
    render(wxPaintDC(this));
}

//! KeyDown event handler
void Figure3DView::OnKeyDown(wxKeyEvent &event)
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
    switch(m_prmode) {
    case ProjectionMode::Orthographic:
        projmat = getOrthoProjMat(m_elev, m_azim);
        break;

    case ProjectionMode::Cabinet:
        projmat = getCabinetProjMat(m_elev, m_azim);
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Unknown projection mode: " << m_prmode << std::endl;
        return;
    }

    // Inform the change of the angles.
    if (mp_fig)
        mp_fig->updateProjector(projmat);

    this->Refresh();
}

//! The implementation of rendering.
void Figure3DView::render(wxWindowDC &&dc)
{
    // If no figure loaded, just fill the area with gray brush.
    if(!mp_fig) {
        dc.SetBackground(*wxGREY_BRUSH);
        dc.Clear();
        return;
    }

    // Clear
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();

    ProjSpatialScheme<WxGSScheme> wxgsOrtho{
        Eigen::Vector3d{m_focus[0], m_focus[1], m_focus[2]}, std::move(dc)};
    switch(m_prmode) {
    case ProjectionMode::Orthographic:
        wxgsOrtho.ortho(m_elev, m_azim);
        break;

    case ProjectionMode::Cabinet:
        wxgsOrtho.cabinet(m_azim, sin(M_PI*m_elev/180.0));
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Unknown projection mode: " << m_prmode << std::endl;
        return;
    }

    mp_fig->draw(wxgsOrtho);
}
