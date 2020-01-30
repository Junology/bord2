/**
 * \file BordPreviewDialog.hpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 26, 2020: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#endif

#include "PlTang.hpp"
#include "Figure3DView.hpp"
#include "figures/TangleMoveFigure.hpp"

class BordPreviewDialog : public wxDialog
{
    wxDECLARE_DYNAMIC_CLASS(BordPreviewDialog);
    wxDECLARE_EVENT_TABLE();

private:
    Figure3DView *m_drawPane{nullptr};

public:
    enum : long {
        BPD_DEFAULT_STYLE = wxCAPTION | wxCLOSE_BOX | wxSYSTEM_MENU | wxRESIZE_BORDER | wxTAB_TRAVERSAL
    };

    BordPreviewDialog() : wxDialog() {}

    BordPreviewDialog(wxWindow *parent,
                      wxWindowID id,
                      wxString const& title,
                      wxPoint const& pos = wxDefaultPosition,
                      wxSize const& size = wxDefaultSize,
                      long style = BPD_DEFAULT_STYLE )
        : wxDialog{parent, id, title, pos, size, style}
    {
        CreateControls();
    }

    void Create(wxWindow *parent,
                wxWindowID id,
                wxString const& title,
                wxPoint const& pos = wxDefaultPosition,
                wxSize const& size = wxDefaultSize,
                long style = wxDEFAULT_DIALOG_STYLE)
    {
        wxDialog::Create(parent, id, title, pos, size, style);
        CreateControls();
    }

    template <size_t R, size_t C>
    void setPlTang(PlTang<R,C> const& pltang) {
        m_drawPane->setFigure(new TangleMoveFigure<R,C>{
                pltang,
                Eigen::Vector2d(40.0, 0.0),
                Eigen::Vector2d(0.0, 40.0)
            });
    }

protected:
    //! Called when wxDialog gets ready.
    void CreateControls();

    /** Event handlers **/
    void OnClose(wxCloseEvent &event);
};

