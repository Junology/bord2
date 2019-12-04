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

class MainDrawPane : public wxPanel
{
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
    void render(wxWindowDC &&dc);

    DECLARE_EVENT_TABLE()
};

