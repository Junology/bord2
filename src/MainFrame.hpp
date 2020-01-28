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
#include "MoveListModel.hpp"

//! The class for the main window.
class MainFrame : public wxFrame
{
private:
    MainDrawPane *m_drawPane;
    PlTangView *m_pltangView;

    wxDataViewCtrl *m_pltang_list;
    wxObjectDataPtr<MoveListModel> m_list_model;

public:
    MainFrame(const char* title);

private:
    // Menu associated event handlers
    void OnNew(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnUndo(wxCommandEvent& event);
    void OnRedo(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

    // Handlers of other events
    void OnTangleMoved(PlTangEvent& event);

    DECLARE_EVENT_TABLE()
};
