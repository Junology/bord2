/**
 * \file BordPreviewDialog.cpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 26, 2020: created
 */

#include "BordPreviewDialog.hpp"

#include <iostream>
#include <Eigen/Dense>

#include "TikzScheme.hpp"
#include "OrthoSpatialScheme.hpp"
#include "ROEntryDialog.hpp"

wxIMPLEMENT_DYNAMIC_CLASS(BordPreviewDialog, wxFrame);

wxBEGIN_EVENT_TABLE(BordPreviewDialog, wxFrame)
EVT_CLOSE(BordPreviewDialog::OnClose)
EVT_MENU(wxID_INFO, BordPreviewDialog::OnTikz)
wxEND_EVENT_TABLE()

void BordPreviewDialog::CreateControls()
{
    // Menubar
    wxMenu *menuView = new wxMenu;
    menuView->Append(wxID_INFO, "Show Tikz code");

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuView, "&View");

    wxFrame::SetMenuBar(menuBar);

    m_drawPane = new Figure3DView(this, wxID_ANY, wxDefaultPosition, wxSize{600,450});

    auto sizer = new wxBoxSizer(wxVERTICAL);
    wxFrame::SetSizer(sizer);
    wxFrame::SetAutoLayout(true);

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

void BordPreviewDialog::OnTikz(wxCommandEvent &event)
{
    double elev = m_drawPane->getElev();
    double azim = m_drawPane->getAzim();

    OrthoSpatialScheme<TikzScheme> scheme{
        Eigen::Vector3d{
            m_drawPane->getFocus()[0],
            m_drawPane->getFocus()[1],
            m_drawPane->getFocus()[2]
        },
        elev, azim
    };

    m_drawPane->getFigure()->draw(scheme);
    ROEntryDialog dlg(
        this, wxID_ANY,
        "Tikz Code",
        scheme.getBase()->getTikzSetCode()
        + scheme.getBase()->getTikzCode());
    dlg.ShowModal();
}
