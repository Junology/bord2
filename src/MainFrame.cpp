/*!
 * \file MainFrame.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#include "MainFrame.hpp"

#include <fstream>
#include <wx/sizer.h>
#include <wx/artprov.h>

#include "config.hpp"
#include "appicon32x32.xpm"

#include "PlTangEntryDialog.hpp"
#include "AppData.hpp"

enum bord2ID {
    bord2ID_LOWEST = wxID_HIGHEST,
    bord2ID_NEWWITH
};

wxBEGIN_EVENT_TABLE(MainFrame, wxFrame)
// Menu associated events
  EVT_MENU(wxID_NEW, MainFrame::OnNew)
  EVT_MENU(bord2ID_NEWWITH, MainFrame::OnNewWith)
  EVT_MENU(wxID_OPEN, MainFrame::OnOpen)
  EVT_MENU(wxID_SAVE, MainFrame::OnSave)
  EVT_MENU(wxID_SAVEAS, MainFrame::OnSaveAs)
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
    ExtendD,
    ShrinkU,
    ShrinkD,
    RaiseL7,
    RaiserJ,
    LowerL7,
    LowerrJ
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

    case Move::ShrinkU:
        graph.connect(BeforeUL, AfterDL);
        graph.connect(BeforeUR, AfterDR);
        return PlTangMove<2,2>(
            "ShrinkU",
            PlTang<2,2>{"r7\n||\n"},
            PlTang<2,2>{"  \nr7\n"},
            graph );

    case Move::ShrinkD:
        graph.connect(BeforeDL, AfterUL);
        graph.connect(BeforeDR, AfterUR);
        return PlTangMove<2,2>(
            "ShrinkD",
            PlTang<2,2>{"||\nLJ\n"},
            PlTang<2,2>{"LJ\n  \n"},
            graph );

    case Move::RaiseL7:
        graph.connect(BeforeDL, AfterUL);
        graph.connect(BeforeDR, AfterUR);
        return PlTangMove<2,2>(
            "RaiseL7",
            PlTang<2,2>{"| \nL7\n"},
            PlTang<2,2>{"L7\n |\n"},
            graph );
        break;

    case Move::RaiserJ:
        graph.connect(BeforeDL, AfterUL);
        graph.connect(BeforeDR, AfterUR);
        return PlTangMove<2,2>(
            "RaiserJ",
            PlTang<2,2>{" |\nrJ\n"},
            PlTang<2,2>{"rJ\n| \n"},
            graph );

    case Move::LowerL7:
        graph.connect(BeforeUL, AfterDL);
        graph.connect(BeforeUR, AfterDR);
        return PlTangMove<2,2>(
            "LowerL7",
            PlTang<2,2>{"L7\n |\n"},
            PlTang<2,2>{"| \nL7\n"},
            graph );

    case Move::LowerrJ:
        graph.connect(BeforeUL, AfterDL);
        graph.connect(BeforeUR, AfterDR);
        return PlTangMove<2,2>(
            "LowerL7",
            PlTang<2,2>{"rJ\n| \n"},
            PlTang<2,2>{" |\nrJ\n"},
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
    // Setup Window Icon
    wxTopLevelWindow::SetIcon(wxIcon(appicon32x32_xpm));

    // Setup Manus
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(wxID_NEW);
    menuFile->Append(bord2ID_NEWWITH, "New With Current Tangle", "", false);
    menuFile->AppendSeparator();
    menuFile->Append(wxID_OPEN);
    menuFile->AppendSeparator();
    menuFile->Append(wxID_SAVE);
    menuFile->Append(wxID_SAVEAS);
    menuFile->AppendSeparator();
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
    m_toolbar->AddTool(
        wxID_OPEN, _("Open"),
        wxArtProvider::GetBitmap(wxART_FILE_OPEN, wxART_TOOLBAR),
        _("Open"));
    m_toolbar->AddTool(
        wxID_SAVE, _("Save"),
        wxArtProvider::GetBitmap(wxART_FILE_SAVE, wxART_TOOLBAR),
        _("Save"));
    m_toolbar->AddTool(
        wxID_SAVEAS, _("Save As"),
        wxArtProvider::GetBitmap(wxART_FILE_SAVE_AS, wxART_TOOLBAR),
        _("Save As"));
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
        generateMove(Move::ExtendD),
        generateMove(Move::ShrinkU),
        generateMove(Move::ShrinkD),
        generateMove(Move::RaiseL7),
        generateMove(Move::RaiserJ),
        generateMove(Move::LowerL7),
        generateMove(Move::LowerrJ)
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
    if(m_pltangInit.isvalid()) {
        m_prevDlg->setFigureBase();
        m_prevDlg->setPlTangMove(m_pltangInit, {});
        m_prevDlg->Refresh();
    }
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

    // Reset the recording state
    m_mode = FMODE_NORMAL;
    m_toolbar->SetToolNormalBitmap(
        wxID_ADD,
        wxArtProvider::GetBitmap(wxART_PLUS, wxART_TOOLBAR));
    m_pltangView->lock();
    m_mvseq_inrec.clear();
}

void MainFrame::OnNewWith(wxCommandEvent& event)
{
    m_pltangInit = m_pltangView->GetPlTang();

    m_pltangView->resetHistory();
    m_list_model->reset();
    m_prevDlg->setPlTangMove(m_pltangInit, {});
    m_prevDlg->Refresh();

    // Reset the recording state
    m_mode = FMODE_NORMAL;
    m_toolbar->SetToolNormalBitmap(
        wxID_ADD,
        wxArtProvider::GetBitmap(wxART_PLUS, wxART_TOOLBAR));
    m_pltangView->lock();
    m_mvseq_inrec.clear();
}

void MainFrame::OnOpen(wxCommandEvent& event)
{
    wxFileDialog openDlg(
        this, _("Open file"), wxEmptyString, wxEmptyString,
        _("bord2 file (*.bord2)|*.bord2|Any file (*.*)|*.*"),
        wxFD_OPEN, wxDefaultPosition);

	// Creates a "open file" dialog with 4 file types
    if (openDlg.ShowModal() != wxID_OK)
        return;

    std::string path = openDlg.GetPath().ToStdString();
    std::ifstream ifs(path);

    auto loaded = readTangleMove(ifs, m_pltangInit);

    // Load succeeded.
    if (loaded.first) {
        // Set the loaded tangle to the view.
        m_pltangView->SetPlTang(m_pltangInit);
        // Reset the move list.
        m_list_model->reset();

        // Apply loaded moves.
        for(auto& mvseq : loaded.second) {
            typename PlTangMove<2,2>::MoveSeq mvelemseq{};
            for(auto& mv : mvseq) {
                auto ptr = m_pltangView->applyMove(mv.name, mv.x, mv.y, false);
                if (ptr) {
                    mvelemseq.push_back({*ptr, mv.x, mv.y});
                    //* Debug
                    std::cout << __FILE__":" << __LINE__ << std::endl;
                    std::cout << mvelemseq.back().move.getName() << " "
                              << mvelemseq.back().x << " "
                              << mvelemseq.back().y << std::endl;
                    // */
                }
            }
            m_list_model->push_back(mvelemseq);
        }
        m_prevDlg->setPlTangMove(m_pltangInit, m_list_model->getMoveSeqs());
        m_pltangView->Refresh();

        // Reset the recording state
        m_mode = FMODE_NORMAL;
        m_toolbar->SetToolNormalBitmap(
            wxID_ADD,
            wxArtProvider::GetBitmap(wxART_PLUS, wxART_TOOLBAR));
        m_pltangView->lock();
        m_mvseq_inrec.clear();
    }
    // Load failed.
    else {
        wxMessageBox(std::string("Failed to load file ") + path);
    }
}

void MainFrame::OnSave(wxCommandEvent& event)
{
    if(m_filepath.empty()) {
        OnSaveAs(event);
        return;
    }

    std::ofstream ofs{m_filepath};
    writeTangleMove(ofs, m_pltangInit, m_list_model->getMoveSeqs());
}

void MainFrame::OnSaveAs(wxCommandEvent& event)
{
    wxFileDialog saveDlg(
        this, _("Open file"), wxEmptyString, wxEmptyString,
        _("bord2 file (*.bord2)|*.bord2|Any file (*.*)|*.*"),
        wxFD_SAVE, wxDefaultPosition);

    if (saveDlg.ShowModal() != wxID_OK)
        return;

    m_filepath = saveDlg.GetPath().ToStdString();
    std::ofstream ofs{m_filepath};
    writeTangleMove(ofs, m_pltangInit, m_list_model->getMoveSeqs());
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
