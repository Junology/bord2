/*!
 * \file MainFrame.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include "config.hpp"
#include "MainFrame.hpp"

wxBEGIN_EVENT_TABLE(MainFrame, wxFrame)
EVT_MENU(wxID_EXIT, MainFrame::OnExit)
EVT_MENU(wxID_ABOUT, MainFrame::OnAbout)
wxEND_EVENT_TABLE()

//! Constructor.
MainFrame::MainFrame(const char* title)
  : wxFrame(nullptr, -1, title, wxDefaultPosition, wxSize(800,600))
{
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(wxID_EXIT);

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);

    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");

    wxFrame::SetMenuBar(menuBar);

    wxFrame::CreateStatusBar();
    SetStatusText("Hello, wxWidgets!");
}

void MainFrame::OnExit(wxCommandEvent& event)
{
    Close(true);
}

void MainFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox(appconf::fullname, "About", wxOK | wxICON_INFORMATION, this);
}
