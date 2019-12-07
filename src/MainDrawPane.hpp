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

#include "WxGSScheme.hpp"
#include "figures/figures.hpp"

class MainDrawPane : public wxPanel
{
private:
    std::unique_ptr<PathFigure3D> mp_fig3d;
    double m_elev, m_azim;

public:
    typedef enum _Error {
        FailedCreatingGC
    } Error;

    MainDrawPane(wxFrame *parent);

    inline void queuePaint() {
        MainDrawPane::render(wxClientDC(this));
    }

protected:
    void OnPaint(wxPaintEvent &event);
    void OnKeyDown(wxKeyEvent &event);
    void render(wxWindowDC &&dc);

    DECLARE_EVENT_TABLE()
};

