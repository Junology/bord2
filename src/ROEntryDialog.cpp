/**
 * \file ROEntryDialog.cpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 6, 2020: created
 */

#include "ROEntryDialog.hpp"

wxIMPLEMENT_DYNAMIC_CLASS(ROEntryDialog, wxDialog);

wxBEGIN_EVENT_TABLE(ROEntryDialog, wxDialog)
wxEND_EVENT_TABLE()

//! Called when wxDialog gets ready.
void ROEntryDialog::CreateControls(wxString const& text)
{
    // Setup the text control.
    m_textctrl = new wxTextCtrl(
        this,
        wxID_ANY,
        text,
        wxDefaultPosition,
        wxSize(300,200),
        wxTE_MULTILINE | wxTE_DONTWRAP | wxTE_READONLY );

    // Place controls in the sizer.
    auto sizer = new wxBoxSizer(wxVERTICAL);
    wxDialog::SetSizer(sizer);
    wxDialog::SetAutoLayout(true);

    sizer->Add(m_textctrl, 1, wxEXPAND | wxALL, 5);
    sizer->Add(CreateButtonSizer(wxOK), 0, wxEXPAND | wxALL, 10);

    sizer->Fit(this);
    sizer->SetSizeHints(this);
}

