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
#include "ProjSpatialScheme.hpp"
#include "ROEntryDialog.hpp"

enum {
    BPID_LOWEST = wxID_HIGHEST, // It is safe to use any id above this.
    BPID_ORTHO,
    BPID_CABINET
};

wxIMPLEMENT_DYNAMIC_CLASS(BordPreviewDialog, wxFrame);

wxBEGIN_EVENT_TABLE(BordPreviewDialog, wxFrame)
EVT_CLOSE(BordPreviewDialog::OnClose)
EVT_MENU(BPID_ORTHO, BordPreviewDialog::OnProjOrtho)
EVT_MENU(BPID_CABINET, BordPreviewDialog::OnProjCabinet)
EVT_MENU(wxID_INFO, BordPreviewDialog::OnTikz)
EVT_MENU(wxID_EXIT, BordPreviewDialog::OnExit)
wxEND_EVENT_TABLE()

void BordPreviewDialog::CreateControls()
{
    // Menubar
    wxMenu *menuView = new wxMenu;
    menuView->AppendRadioItem(BPID_ORTHO, "Orthographic Projection");
    menuView->AppendRadioItem(BPID_CABINET, "Cabinet Projection");
    menuView->AppendSeparator();
    menuView->Append(wxID_INFO, "Show Tikz code");
    menuView->AppendSeparator();
    menuView->Append(wxID_EXIT, "&Close");

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuView, "&View");

    wxFrame::SetMenuBar(menuBar);

    m_drawPane = new Figure3DView(this, wxID_ANY, wxDefaultPosition, wxSize{600,450});
    mp_tangleFig = new TangleMoveFigure<TangType>(
        TangType{},
        {},
        40*Eigen::Matrix3d::Identity() );
    // Figure3DView class automatically deletes the resource via std::unique_ptr.
    m_drawPane->setFigure(mp_tangleFig);

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

void BordPreviewDialog::OnProjOrtho(wxCommandEvent &event)
{
    m_drawPane->setProjMode(Figure3DView::ProjectionMode::Orthographic);
}

void BordPreviewDialog::OnProjCabinet(wxCommandEvent &event)
{
    m_drawPane->setProjMode(Figure3DView::ProjectionMode::Cabinet);
}

void BordPreviewDialog::OnTikz(wxCommandEvent &event)
{
    double elev = m_drawPane->getElev();
    double azim = m_drawPane->getAzim();

    ProjSpatialScheme<TikzScheme> scheme{
        Eigen::Vector3d{
            m_drawPane->getFocus()[0],
            m_drawPane->getFocus()[1],
            m_drawPane->getFocus()[2]
        }
    };

    /*
    switch(m_drawPane->getProjMode()) {
    case Figure3DView::ProjectionMode::Orthographic:
        scheme.ortho(elev, azim);
        break;

    case Figure3DView::ProjectionMode::Cabinet:
        scheme.cabinet(elev, azim);
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Unknown projection mode." << std::endl;
        break;
    }

    m_drawPane->getFigure()->draw(scheme);
    */
    m_drawPane->drawToScheme(scheme);

    ROEntryDialog dlg(
        this, wxID_ANY,
        "Tikz Code",
        scheme.getBase()->getTikzSetCode()
        + scheme.getBase()->getTikzCode());
    dlg.ShowModal();
}

void BordPreviewDialog::OnExit(wxCommandEvent& event)
{
    Close(false);
}

