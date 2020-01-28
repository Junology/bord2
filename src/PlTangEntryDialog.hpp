/**
 * \file PlTangEntryDialog.hpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 26, 2020: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#endif

class PlTangEntryDialog : public wxDialog
{
    wxDECLARE_DYNAMIC_CLASS(PlTangEntryDialog);
    wxDECLARE_EVENT_TABLE();

private:
    wxTextCtrl *m_textctrl{nullptr};

public:
    enum : long {
        PTED_DEFAULT_STYLE = wxCAPTION | wxCLOSE_BOX | wxSYSTEM_MENU | wxRESIZE_BORDER | wxTAB_TRAVERSAL
    };
    //! Default constructor
    PlTangEntryDialog() : wxDialog{} {};

    //! Dialog constructor
    PlTangEntryDialog(wxWindow *parent,
                      wxWindowID id,
                      wxString const& message,
                      wxString const& title,
                      wxPoint const& pos = wxDefaultPosition,
                      wxSize const& size = wxDefaultSize,
                      long style = PTED_DEFAULT_STYLE )
        : wxDialog{parent, id, title, pos, size, style}
    {
        CreateControls(message);
    }

    ~PlTangEntryDialog() noexcept = default;

    void Create(wxWindow *parent,
                wxWindowID id,
                wxString const& message,
                wxString const& title,
                wxPoint const& pos = wxDefaultPosition,
                wxSize const& size = wxDefaultSize,
                long style = wxDEFAULT_DIALOG_STYLE)
    {
        wxDialog::Create(parent, id, title, pos, size, style);
        CreateControls(message);
    }

    wxString GetValue() const
    {
        return m_textctrl ? m_textctrl->GetValue() : wxString();
    }

protected:
    //! Called when wxDialog gets ready.
    void CreateControls(wxString const& msg);
    //void OnClose(wxCloseEvent&);
};
