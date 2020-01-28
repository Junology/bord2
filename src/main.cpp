/*!
 * \file main.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include <iostream>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#endif

#include "config.hpp"
#include "MainFrame.hpp"

class MainApp : public wxApp
{
public:
    virtual bool OnInit() override;
    //virtual int FilterEvent(wxEvent& event) override;
};

DECLARE_APP(MainApp)
IMPLEMENT_APP(MainApp)

bool MainApp::OnInit()
{
    MainFrame *MainWin = new MainFrame(appconf::fullname);
    MainWin->Show(true);
    wxApp::SetTopWindow(MainWin);
    return true;
}

/*
int MainApp::FilterEvent(wxEvent& event)
{
    std::cout << event.GetId() << std::endl;
    return wxApp::FilterEvent(event);
}
*/
