/*!
 * \file MainFrame.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#    include <wx/dataview.h>
#endif

#include "MainDrawPane.hpp"
#include "PlTangView.hpp"

//! The class for the main window.
class MainFrame : public wxFrame
{
private:
    MainDrawPane *m_drawPane;
    PlTangView *m_pltangView;
    wxDataViewListCtrl *m_pltang_list;

public:
    MainFrame(const char* title);

private:
    void OnNew(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};
