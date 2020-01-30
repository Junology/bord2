/*!
 * \file MainFrame.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include <wx/sizer.h>

#include "config.hpp"
#include "MainFrame.hpp"
#include "PlTangEntryDialog.hpp"

wxBEGIN_EVENT_TABLE(MainFrame, wxFrame)
// Menu associated events
  EVT_MENU(wxID_NEW, MainFrame::OnNew)
  EVT_MENU(wxID_EXIT, MainFrame::OnExit)
  EVT_MENU(wxID_UNDO, MainFrame::OnUndo)
  EVT_MENU(wxID_REDO, MainFrame::OnRedo)
  EVT_MENU(wxID_PREVIEW, MainFrame::OnPreview)
  EVT_MENU(wxID_ABOUT, MainFrame::OnAbout)
// PlTangView associated events
  EVT_PLTANG_MOVED(wxID_ANY, MainFrame::OnTangleMoved)
wxEND_EVENT_TABLE()

constexpr auto pltangInit = PlTang<>{
    "||r7\n"
    "||LJ\n"
    "LJ  \n"
    "r7  \n"
};

//! Constructor.
MainFrame::MainFrame(const char* title)
  : wxFrame(nullptr, -1, title, wxDefaultPosition, wxSize(800,600)),
    m_pltangInit(pltangInit), m_pltangView(nullptr)
{
    // Setup Manus
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(wxID_NEW);
    menuFile->Append(wxID_EXIT);

    wxMenu *menuTool = new wxMenu;
    menuTool->Append(wxID_UNDO);
    menuTool->Append(wxID_REDO);
    menuTool->Append(wxID_SEPARATOR);
    menuTool->Append(wxID_PREVIEW);

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuTool, "&Tool");
    menuBar->Append(menuHelp, "&Help");

    wxFrame::SetMenuBar(menuBar);

    // Setup StatusBar
    wxFrame::CreateStatusBar();
    SetStatusText("Hello, wxWidgets!");

    // Setup the body of the frame
    wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);

    // List of planar tangles.
    m_pltang_list = new wxDataViewCtrl(this, wxID_ANY, wxDefaultPosition, wxSize(200,-1));
    m_list_model = new MoveListModel;
    m_pltang_list->AssociateModel(m_list_model.get());

    //auto renderer = new wxDataViewTextRenderer();
    //auto col = new wxDataViewColumn("Moves", renderer, 0);
    m_pltang_list->AppendTextColumn("Name", MoveListModel::Col_MoveName);
    m_pltang_list->AppendTextColumn("X", MoveListModel::Col_XCoord);
    m_pltang_list->AppendTextColumn("Y", MoveListModel::Col_YCoord);

    // Drawing Area
    //m_drawPane = new MainDrawPane(this);
    m_pltangView = new PlTangView(
        this, wxID_ANY, m_pltangInit
        );
    constexpr PlTangMove<2,2> moves[] = {
        PlTangMove<2,2>{
            "SaddleHV",
            PlTang<2,2>{"LJ\nr7\n"},
            PlTang<2,2>{"||\n||\n"}
        },
        PlTangMove<2,2>{
            "SaddleVH",
            PlTang<2,2>{"||\n||\n"},
            PlTang<2,2>{"LJ\nr7\n"}
        },
        PlTangMove<2,2>{
            "Cup",
            PlTang<2,2>{"  \n  \n"},
            PlTang<2,2>{"r7\nLJ\n"}
        },
        PlTangMove<2,2>{
            "Cap",
            PlTang<2,2>{"r7\nLJ\n"},
            PlTang<2,2>{"  \n  \n"}
        },
        PlTangMove<2,2>{
            "ExtendR",
            PlTang<2,2>{"| \n| \n"},
            PlTang<2,2>{"L7\nrJ\n"}
        },
        PlTangMove<2,2>{
            "ExtendL",
            PlTang<2,2>{" |\n |\n"},
            PlTang<2,2>{"rJ\nL7\n"}
        },
        PlTangMove<2,2>{
            "ExtendU",
            PlTang<2,2>{"  \nr7\n"},
            PlTang<2,2>{"r7\n||\n"}
        },
        PlTangMove<2,2>{
            "ExtendD",
            PlTang<2,2>{"LJ\n  \n"},
            PlTang<2,2>{"||\nLJ\n"}
        }
    };
    for(auto mv : moves) {
        m_pltangView->registerMove(mv);
    }

    // Register to a sizer
    sizer->Add(m_pltang_list, 0, wxEXPAND | wxALL);
    sizer->Add(m_pltangView, 1, wxEXPAND | wxALL);
    //sizer->Add(m_drawPane, 1, wxEXPAND);

    wxFrame::SetSizer(sizer);
    wxFrame::SetAutoLayout(true);
}

void MainFrame::OnNew(wxCommandEvent& event)
{
    PlTangEntryDialog entryDlg{this, wxID_ANY, "Enter ASCII-Art representation:", appconf::fullname};

    if (entryDlg.ShowModal() == wxID_OK) {
        m_pltangInit = entryDlg.GetPlTang();
        m_pltangView->SetPlTang(m_pltangInit);
        m_pltangView->Refresh();
        m_list_model->reset();
    }
}

void MainFrame::OnExit(wxCommandEvent& event)
{
    Close(true);
}

void MainFrame::OnUndo(wxCommandEvent &event)
{
    m_pltangView->undo();
    m_pltangView->Refresh();
}

void MainFrame::OnRedo(wxCommandEvent &event)
{
    m_pltangView->redo();
    m_pltangView->Refresh();
}

void MainFrame::OnPreview(wxCommandEvent &event)
{
    if(!m_pltangInit.isvalid())
        return;

    if(!m_prevDlg) {
        m_prevDlg = new BordPreviewDialog(this, wxID_ANY, "Preview Cobordism");
    }

    if(!m_prevDlg->IsShown()) {
        m_prevDlg->setPlTang(m_pltangInit);
        m_prevDlg->Show();
    }
}

void MainFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox(appconf::fullname, "About", wxOK | wxICON_INFORMATION, this);
}

void MainFrame::OnTangleMoved(PlTangEvent& event)
{
    auto const& mv = event.GetMove();

    if(event.IsRevert()) {
        m_list_model->pop_back();
    }
    else {
        m_list_model->emplace_back(event.GetMove(), event.GetX(), event.GetY());
    }
}
