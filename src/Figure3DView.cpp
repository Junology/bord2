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

enum : long{
    FIG3DVIEW_EVENT_LOWEST = wxID_HIGHEST,
    FIG3DVIEW_EVENT_COMPLETE
};

IMPLEMENT_DYNAMIC_CLASS(Figure3DView, wxPanel)

BEGIN_EVENT_TABLE(Figure3DView, wxPanel)
    EVT_PAINT(Figure3DView::OnPaint)
    EVT_KEY_DOWN(Figure3DView::OnKeyDown)
    EVT_THREAD(FIG3DVIEW_EVENT_COMPLETE, Figure3DView::OnBezierReady)
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

// Get the projection matrix.
Eigen::Matrix<double,2,3> Figure3DView::getPrMatrix() const noexcept
{
    switch(m_prmode) {
    case ProjectionMode::Orthographic:
        return getOrthoProjMat(m_elev, m_azim);

    case ProjectionMode::Cabinet:
        return getCabinetProjMat(m_elev, m_azim);

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Unknown projection mode: " << m_prmode << std::endl;
        return {};
    }
}

// Update the buffer of Bezier sequences.
void Figure3DView::updateBuffer()
{
    // If no figure, do nothing.
    if(!mp_fig)
        return;

    m_bezsch.clear();
    mp_fig->draw(m_bezsch);
    m_bezseq_incomputation
        = m_bezsch.getProject(
            getPrMatrix(),
            [this] {
                auto const tid = std::this_thread::get_id();
                wxThreadEvent evt;
                evt.SetId(FIG3DVIEW_EVENT_COMPLETE);
                evt.SetInt(std::hash<std::thread::id>()(tid));
                this->QueueEvent(evt.Clone());
            } );
    m_bezseq = m_bezsch.moveRaw();
}

void Figure3DView::setProjMode(ProjectionMode prmode) noexcept {
    if (m_prmode == prmode)
        return;

    m_prmode = prmode;
    mp_fig->updateProjector(getPrMatrix());
    updateBuffer();
    this->Refresh();
}

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

    // Inform the change of the angles.
    if (mp_fig)
        mp_fig->updateProjector(getPrMatrix());

    updateBuffer();
    this->Refresh();
}

//! Called when the computation is finished
void Figure3DView::OnBezierReady(wxThreadEvent &event)
{
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

    ProjSpatialScheme<WxGSScheme> wxgsProj{
        Eigen::Vector3d{m_focus[0], m_focus[1], m_focus[2]}, std::move(dc)};
    switch(m_prmode) {
    case ProjectionMode::Orthographic:
        wxgsProj.ortho(m_elev, m_azim);
        break;

    case ProjectionMode::Cabinet:
        wxgsProj.cabinet(m_azim, sin(M_PI*m_elev/180.0));
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Unknown projection mode: " << m_prmode << std::endl;
        return;
    }

    // If the computation is ready.
    if (m_bezseq_incomputation.valid()) {
        std::future_status stat
            = m_bezseq_incomputation.wait_for(std::chrono::seconds::zero());
        if(stat == std::future_status::ready) {
            m_bezseq = m_bezseq_incomputation.get();
        }
    }

    for(auto& bezseq : m_bezseq) {
        if(bezseq.empty())
           continue;

        //* Debug
        wxgsProj.setPen(5.0, bord2::PathColor::Blue);
        wxgsProj.moveTo(bezseq.front().source());
        wxgsProj.lineTo(bezseq.front().source());
        wxgsProj.moveTo(bezseq.back().target());
        wxgsProj.lineTo(bezseq.back().target());
        wxgsProj.stroke();
        // */

        wxgsProj.setPen(2.0, bord2::PathColor::Red);
        drawBezierSequence(bezseq, wxgsProj);
        wxgsProj.stroke();
    }
}
