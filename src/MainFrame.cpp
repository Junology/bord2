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
  EVT_MENU(wxID_NEW, MainFrame::OnNew)
  EVT_MENU(wxID_EXIT, MainFrame::OnExit)
  EVT_MENU(wxID_ABOUT, MainFrame::OnAbout)
wxEND_EVENT_TABLE()

//! Constructor.
MainFrame::MainFrame(const char* title)
  : wxFrame(nullptr, -1, title, wxDefaultPosition, wxSize(800,600)),
    m_drawPane(nullptr), m_pltangView(nullptr)
{
    // Setup Manus
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(wxID_NEW);
    menuFile->Append(wxID_EXIT);

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");

    wxFrame::SetMenuBar(menuBar);

    // Setup StatusBar
    wxFrame::CreateStatusBar();
    SetStatusText("Hello, wxWidgets!");

    // Setup the body of the frame
    wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);

    // List of planar tangles.
    m_pltang_list = new wxDataViewListCtrl(this, wxID_ANY, wxDefaultPosition, wxSize(200,-1));
    auto renderer = new wxDataViewTextRenderer();
    auto col = new wxDataViewColumn("Foooooo", renderer, 0);
    m_pltang_list->AppendColumn(col);
    //m_pltang_list->AppendTextColumn("Fooooooooooooooooo");

    for (size_t i = 0; i < 100; ++i) {
        wxVector<wxVariant> data;
        data.push_back(wxVariant(std::string("Foo \n") + std::to_string(i)));
        m_pltang_list->AppendItem(data);
    }

    // Drawing Area
    //m_drawPane = new MainDrawPane(this);
    m_pltangView = new PlTangView(this, wxID_ANY, PlTang<>());
    constexpr PlTangMove<2,2> moves[] = {
        PlTangMove<2,2>("SaddleHV", PlTang<2,2>{"LJ\nr7\n"}, PlTang<2,2>{"||\n||\n"}),
        PlTangMove<2,2>("SaddleVH", PlTang<2,2>{"||\n||\n"}, PlTang<2,2>{"LJ\nr7\n"}),
        PlTangMove<2,2>("Cup", PlTang<2,2>{"  \n  \n"}, PlTang<2,2>{"r7\nLJ\n"}),
        PlTangMove<2,2>("Cap", PlTang<2,2>{"r7\nLJ\n"}, PlTang<2,2>{"  \n  \n"})
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
//    wxTextEntryDialog entryDlg{this, "Enter ASCII-Art representation:", appconf::fullname, wxEmptyString, wxTextEntryDialogStyle | wxTE_MULTILINE};
    PlTangEntryDialog entryDlg{this, wxID_ANY, "Enter ASCII-Art representation:", appconf::fullname};

    if (entryDlg.ShowModal() == wxID_OK) {
        PlTang<> newtang{static_cast<char const*>(entryDlg.GetValue().c_str())};
        if(newtang.isvalid()) {
            m_pltangView->SetPlTang(newtang);
            m_pltangView->Refresh();
        }
        else {
            wxMessageBox("Invalid representation.", "Error", wxOK, this);
        }
    }
}

void MainFrame::OnExit(wxCommandEvent& event)
{
    Close(true);
}

void MainFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox(appconf::fullname, "About", wxOK | wxICON_INFORMATION, this);
}
