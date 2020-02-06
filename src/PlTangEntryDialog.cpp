/**
 * \file PlTangEntryDialog.cpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 26, 2020: created
 */

#include "PlTangEntryDialog.hpp"

wxIMPLEMENT_DYNAMIC_CLASS(PlTangEntryDialog, wxDialog);

wxBEGIN_EVENT_TABLE(PlTangEntryDialog, wxDialog)
wxEND_EVENT_TABLE()

template <class T>
struct ValidatorOf;

void PlTangEntryDialog::CreateControls(wxString const& msg)
{
    // Setup the text control.
    m_textctrl = new wxTextCtrl(
        this,
        wxID_ANY,
        wxEmptyString,
        wxDefaultPosition,
        wxSize{200,300},
        wxTE_MULTILINE | wxTE_DONTWRAP,
        EntryValidatorOf<decltype(m_pltang)>{&m_pltang});

    wxFont font{
        12,
        wxFONTFAMILY_TELETYPE,
        wxFONTSTYLE_NORMAL,
        wxFONTWEIGHT_NORMAL};

    m_textctrl->SetFont(font);

    // Place controls in the sizer.
    auto sizer = new wxBoxSizer(wxVERTICAL);
    wxDialog::SetSizer(sizer);
    wxDialog::SetAutoLayout(true);

    sizer->Add(new wxStaticText(this, wxID_ANY, msg), 0, wxEXPAND | wxALL, 10);
    sizer->Add(m_textctrl, 1, wxEXPAND | wxALL, 5);
    sizer->Add(CreateButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, 10);

    sizer->Fit(this);
    sizer->SetSizeHints(this);

}
