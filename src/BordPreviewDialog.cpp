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
    BPID_TOP2BOTTOM,
    BPID_BOTTOM2TOP,
    BPID_ORTHO,
    BPID_CABINET
};

wxIMPLEMENT_DYNAMIC_CLASS(BordPreviewDialog, wxFrame);

wxBEGIN_EVENT_TABLE(BordPreviewDialog, wxFrame)
EVT_CLOSE(BordPreviewDialog::OnClose)
EVT_MENU(BPID_TOP2BOTTOM, BordPreviewDialog::OnVDirection)
EVT_MENU(BPID_BOTTOM2TOP, BordPreviewDialog::OnVDirection)
EVT_MENU(BPID_ORTHO, BordPreviewDialog::OnProjOrtho)
EVT_MENU(BPID_CABINET, BordPreviewDialog::OnProjCabinet)
EVT_MENU(wxID_INFO, BordPreviewDialog::OnTikz)
EVT_MENU(wxID_CLOSE, BordPreviewDialog::OnExit)
wxEND_EVENT_TABLE()

void BordPreviewDialog::CreateControls()
{
    // Menubar
    wxMenu *menuView = new wxMenu;
    menuView->AppendRadioItem(BPID_TOP2BOTTOM, "Top to Bottom");
    menuView->AppendRadioItem(BPID_BOTTOM2TOP, "Bottom to Top");
    menuView->Check(BPID_TOP2BOTTOM, m_is_top2bottom);
    menuView->AppendSeparator();
    menuView->AppendRadioItem(BPID_ORTHO, "Orthographic Projection");
    menuView->AppendRadioItem(BPID_CABINET, "Cabinet Projection");
    menuView->AppendSeparator();
    menuView->Append(wxID_INFO, "Show Tikz code");
    menuView->AppendSeparator();
    menuView->Append(wxID_CLOSE);

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

void BordPreviewDialog::OnVDirection(wxCommandEvent &event)
{
    switch(event.GetId()) {
    case BPID_TOP2BOTTOM:
        m_is_top2bottom = true;
        break;

    case BPID_BOTTOM2TOP:
        m_is_top2bottom = false;
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Unknown vertical mode: " << event.GetId() << std::endl;
        return;
    }
    setFigureBase();
    m_drawPane->updateBuffer();
    m_drawPane->Refresh();
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
    ProjSpatialScheme<TikzScheme> scheme = {
        m_drawPane->getPrMatrix(),
        Eigen::Vector3d{
            m_drawPane->getFocus()[0],
            m_drawPane->getFocus()[1],
            m_drawPane->getFocus()[2]
        }
    };

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

