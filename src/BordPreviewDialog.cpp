/**
 * \file BordPreviewDialog.cpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 26, 2020: created
 */

#include <iostream>

#include "BordPreviewDialog.hpp"

wxIMPLEMENT_DYNAMIC_CLASS(BordPreviewDialog, wxDialog);

wxBEGIN_EVENT_TABLE(BordPreviewDialog, wxDialog)
EVT_CLOSE(BordPreviewDialog::OnClose)
wxEND_EVENT_TABLE()

void BordPreviewDialog::CreateControls()
{
    m_drawPane = new Figure3DView(this, wxID_ANY, wxDefaultPosition, wxSize{600,450});

    auto sizer = new wxBoxSizer(wxVERTICAL);
    wxDialog::SetSizer(sizer);
    wxDialog::SetAutoLayout(true);

    sizer->Add(m_drawPane, 0, wxEXPAND | wxALL, 3);

    sizer->Fit(this);
    sizer->SetSizeHints(this);
}

void BordPreviewDialog::OnClose(wxCloseEvent &event)
{
    if (event.CanVeto())
        Show(false);
    else
        Destroy();
}
