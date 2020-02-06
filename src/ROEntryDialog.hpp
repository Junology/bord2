/**
 * \file ROEntryDialog.hpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 6, 2020: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#endif

class ROEntryDialog : public wxDialog
{
    wxDECLARE_DYNAMIC_CLASS(ROEntryDialog);
    wxDECLARE_EVENT_TABLE();

private:
    wxTextCtrl *m_textctrl = nullptr;

public:
    enum : long {
        ROED_DEFAULT_STYLE = wxCAPTION | wxCLOSE_BOX | wxSYSTEM_MENU | wxRESIZE_BORDER | wxTAB_TRAVERSAL | wxDIALOG_NO_PARENT
    };

    ROEntryDialog() : wxDialog() {}

    ROEntryDialog(wxWindow *parent,
                      wxWindowID id,
                      wxString const& title,
                      wxString const& text,
                      wxPoint const& pos = wxDefaultPosition,
                      wxSize const& size = wxDefaultSize,
                      long style = ROED_DEFAULT_STYLE )
        : wxDialog{parent, id, title, pos, size, style}
    {
        CreateControls(text);
    }

    virtual ~ROEntryDialog() = default;

    void Create(wxWindow *parent,
                wxWindowID id,
                wxString const& title,
                wxString const& text,
                wxPoint const& pos = wxDefaultPosition,
                wxSize const& size = wxDefaultSize,
                long style = ROED_DEFAULT_STYLE)
    {
        wxDialog::Create(parent, id, title, pos, size, style);
        CreateControls(text);
    }

protected:
    //! Called when wxDialog gets ready.
    void CreateControls(wxString const&);

    /** Event handlers **/
    void OnClose(wxCloseEvent &event);
};
