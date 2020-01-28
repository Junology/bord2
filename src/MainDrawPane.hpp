/*!
 * \file MainDrawPane.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#endif

#include <memory>

#include "figures/figures.hpp"

class MainDrawPane : public wxPanel
{
private:
    std::unique_ptr<QBezierLine3D> mp_fig3d;
    std::vector<std::unique_ptr<PathFigure3D> > m_figs;
    double m_elev, m_azim;
    std::array<double,3> m_focus;

public:
    typedef enum _Error {
        FailedCreatingGC
    } Error;

    MainDrawPane(wxWindow *parent);

    inline void queuePaint() {
        MainDrawPane::render(wxClientDC(this));
    }

protected:
    void OnPaint(wxPaintEvent &event);
    void OnKeyDown(wxKeyEvent &event);
    void render(wxWindowDC &&dc);

    wxDECLARE_EVENT_TABLE();
};

