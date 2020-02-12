/*!
 * \file MainFrame.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include <wx/sizer.h>
#include <wx/artprov.h>

#include "config.hpp"
#include "MainFrame.hpp"
#include "PlTangEntryDialog.hpp"

wxBEGIN_EVENT_TABLE(MainFrame, wxFrame)
// Menu associated events
  EVT_MENU(wxID_NEW, MainFrame::OnNew)
  EVT_MENU(wxID_EXIT, MainFrame::OnExit)
  EVT_MENU(wxID_UNDO, MainFrame::OnUndo)
  EVT_MENU(wxID_REDO, MainFrame::OnRedo)
  EVT_MENU(wxID_ADD, MainFrame::OnAdd)
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

enum class Move {
    SaddleHV,
    SaddleVH,
    Cup,
    Cap,
    ExtendR,
    ExtendL,
    ExtendU,
    ExtendD
};

constexpr PlTangMove<2,2> generateMove(Move mvkind)
{
    enum : size_t {
        BeforeUL = 0,
        BeforeUR = 1,
        BeforeDL = 2,
        BeforeDR = 3,
        AfterUL = 4,
        AfterUR = 5,
        AfterDL = 6,
        AfterDR = 7
    };
    Graph<8> graph(8);

    switch(mvkind) {
    case Move::SaddleHV:
        graph.connect(BeforeUL, BeforeDR);
        graph.connect(BeforeUR, BeforeDL);
        return PlTangMove<2,2>(
            "SaddleHV",
            PlTang<2,2>{"LJ\nr7\n"},
            PlTang<2,2>{"||\n||\n"},
            graph );

    case Move::SaddleVH:
        graph.connect(AfterUL, AfterDR);
        graph.connect(AfterUR, AfterDL);
        return PlTangMove<2,2>(
            "SaddleVH",
            PlTang<2,2>{"||\n||\n"},
            PlTang<2,2>{"LJ\nr7\n"},
            graph );

    case Move::Cup:
        graph.connect(AfterUL, AfterDR);
        graph.connect(AfterUR, AfterDL);
        return PlTangMove<2,2>(
            "Cup",
            PlTang<2,2>{"  \n  \n"},
            PlTang<2,2>{"r7\nLJ\n"},
            graph );

    case Move::Cap:
        graph.connect(BeforeUL, BeforeDR);
        graph.connect(BeforeUR, BeforeDL);
        return PlTangMove<2,2>(
            "Cap",
            PlTang<2,2>{"r7\nLJ\n"},
            PlTang<2,2>{"  \n  \n"},
            graph );

    case Move::ExtendR:
        graph.connect(BeforeUL, AfterUL);
        graph.connect(BeforeUL, AfterUR);
        graph.connect(BeforeDL, AfterDL);
        graph.connect(BeforeDL, AfterDR);
        return PlTangMove<2,2>(
            "ExtendR",
            PlTang<2,2>{"| \n| \n"},
            PlTang<2,2>{"L7\nrJ\n"},
            graph );

    case Move::ExtendL:
        graph.connect(BeforeUR, AfterUL);
        graph.connect(BeforeUR, AfterUR);
        graph.connect(BeforeDR, AfterDL);
        graph.connect(BeforeDR, AfterDR);
        return PlTangMove<2,2>(
            "ExtendL",
            PlTang<2,2>{" |\n |\n"},
            PlTang<2,2>{"rJ\nL7\n"},
            graph );

    case Move::ExtendU:
        graph.connect(BeforeDL, AfterUL);
        graph.connect(BeforeDR, AfterUR);
        return PlTangMove<2,2>(
            "ExtendU",
            PlTang<2,2>{"  \nr7\n"},
            PlTang<2,2>{"r7\n||\n"},
            graph );

    case Move::ExtendD:
        graph.connect(BeforeUL, AfterDL);
        graph.connect(BeforeUR, AfterDR);
        return PlTangMove<2,2>(
            "ExtendD",
            PlTang<2,2>{"LJ\n  \n"},
            PlTang<2,2>{"||\nLJ\n"},
            graph );

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Unknown move" << std::endl;
        return PlTangMove<2,2>{};
    }
}

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

    // Toolbar
    m_toolbar = wxFrame::CreateToolBar();
    m_toolbar->AddTool(
        wxID_NEW, "New",
        wxArtProvider::GetBitmap(wxART_NEW, wxART_TOOLBAR),
        "New");
    m_toolbar->AddSeparator();
    m_toolbar->AddTool(
        wxID_UNDO, "Undo",
        wxArtProvider::GetBitmap(wxART_UNDO, wxART_TOOLBAR),
        "Undo");
    m_toolbar->AddTool(
        wxID_REDO, "Redo",
        wxArtProvider::GetBitmap(wxART_REDO, wxART_TOOLBAR),
        "Redo");
    m_toolbar->AddSeparator();
    m_toolbar->AddTool(
        wxID_ADD, "Add new move",
        wxArtProvider::GetBitmap(wxART_PLUS, wxART_TOOLBAR),
        "Add a new move");

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
    m_pltangView->lock();
    constexpr PlTangMove<2,2> moves[] = {
        generateMove(Move::SaddleHV),
        generateMove(Move::SaddleVH),
        generateMove(Move::Cup),
        generateMove(Move::Cap),
        generateMove(Move::ExtendR),
        generateMove(Move::ExtendL),
        generateMove(Move::ExtendU),
        generateMove(Move::ExtendD)
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

    // Create an instance of preview dialog
    m_prevDlg = new BordPreviewDialog(this, wxID_ANY, "Preview Cobordism");
    if(m_pltangInit.isvalid())
        m_prevDlg->setPlTangMove(m_pltangInit, {});
}

void MainFrame::OnNew(wxCommandEvent& event)
{
    PlTangEntryDialog entryDlg{this, wxID_ANY, "Enter ASCII-Art representation:", appconf::fullname};

    if (entryDlg.ShowModal() != wxID_OK)
        return;

    // We can believe that the planar tangle is valid thanks to the validator associated to the entry dialog; \see PlTangEntrydialog::EntryValidatorOf.
    m_pltangInit = entryDlg.GetPlTang();

    m_pltangView->SetPlTang(m_pltangInit);
    m_pltangView->Refresh();
    m_list_model->reset();

    m_prevDlg->setPlTangMove(m_pltangInit, {});
    m_prevDlg->Refresh();
}

void MainFrame::OnExit(wxCommandEvent& event)
{
    Close(true);
}

void MainFrame::OnUndo(wxCommandEvent &event)
{

    switch(m_mode) {
    case FMODE_NORMAL:
        {
            auto pseq = m_list_model->getCurrent();
            if (!pseq)
                return;

            for(size_t i = 0; i < pseq->size(); ++i)
                m_pltangView->undo(false);
            m_list_model->roll_back();
        }
        break;

    case FMODE_RECORDMOVE:
        if (!m_mvseq_inrec.empty())
            m_pltangView->undo(true);
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Error: Unknown mode" << std::endl;
    }

    m_pltangView->Refresh();
}

void MainFrame::OnRedo(wxCommandEvent &event)
{
    auto pseq = m_list_model->getCurrent();

    switch(m_mode) {
    case FMODE_NORMAL:
        {
            m_list_model->advance();
            auto pseq = m_list_model->getCurrent();
            if (!pseq)
                return;

            for(size_t i = 0; i < pseq->size(); ++i)
                m_pltangView->redo(false);
        }
        break;

    case FMODE_RECORDMOVE:
        m_pltangView->redo(true);
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Error: Unknown mode" << std::endl;
    }
    m_pltangView->Refresh();
}

void MainFrame::OnAdd(wxCommandEvent &event)
{
    switch(m_mode) {
    case FMODE_NORMAL:
        m_mode = FMODE_RECORDMOVE;
        m_toolbar->SetToolNormalBitmap(
            wxID_ADD,
            wxArtProvider::GetBitmap(wxART_TICK_MARK, wxART_TOOLBAR));
        m_pltangView->unlock();
        break;

    case FMODE_RECORDMOVE:
        m_mode = FMODE_NORMAL;
        m_toolbar->SetToolNormalBitmap(
            wxID_ADD,
            wxArtProvider::GetBitmap(wxART_PLUS, wxART_TOOLBAR));
        m_pltangView->lock();
        m_list_model->push_back(m_mvseq_inrec);
        m_prevDlg->setPlTangMove(m_pltangInit, m_list_model->getMoveSeqs());
        m_mvseq_inrec.clear();
        break;

    default:
        std::cerr << __FILE__":" << __LINE__ << std::endl;
        std::cerr << "Error: Unknown mode" << std::endl;
    }
}

void MainFrame::OnPreview(wxCommandEvent &event)
{
    if(!m_prevDlg->IsShown()) {
        m_prevDlg->setPlTangMove(m_pltangInit, m_list_model->getMoveSeqs());
        m_prevDlg->Show();
    }
}

void MainFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox(
        appconf::fullname,
        "About",
        wxOK | wxICON_INFORMATION,
        this);
}

void MainFrame::OnTangleMoved(PlTangEvent& event)
{
    auto const& mv = event.GetMove();

    if(event.IsRevert()) {
        m_mvseq_inrec.pop_back();
    }
    else {
        m_mvseq_inrec.push_back(
            {event.GetMove(), event.GetX(), event.GetY()});
    }
}
