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

#include <memory>

#include "PlTang.hpp"
#include "Figure3DView.hpp"
#include "figures/TangleMoveFigure.hpp"

class BordPreviewDialog : public wxFrame
{
    wxDECLARE_DYNAMIC_CLASS(BordPreviewDialog);
    wxDECLARE_EVENT_TABLE();

private:
    Figure3DView *m_drawPane{nullptr};
    TangleMoveFigure<PlTang<>> *mp_tangleFig{};

public:
    enum : long {
        BPD_DEFAULT_STYLE = wxCAPTION | wxCLOSE_BOX | wxSYSTEM_MENU | wxRESIZE_BORDER | wxTAB_TRAVERSAL | wxDIALOG_NO_PARENT
    };

    BordPreviewDialog() : wxFrame() {}

    BordPreviewDialog(wxWindow *parent,
                      wxWindowID id,
                      wxString const& title,
                      wxPoint const& pos = wxDefaultPosition,
                      wxSize const& size = wxDefaultSize,
                      long style = BPD_DEFAULT_STYLE )
        : wxFrame{parent, id, title, pos, size, style}
    {
        CreateControls();
    }

    virtual ~BordPreviewDialog() = default;

    void Create(wxWindow *parent,
                wxWindowID id,
                wxString const& title,
                wxPoint const& pos = wxDefaultPosition,
                wxSize const& size = wxDefaultSize,
                long style = BPD_DEFAULT_STYLE)
    {
        wxFrame::Create(parent, id, title, pos, size, style);
        CreateControls();
    }

    bool isvalid() const noexcept {
        return mp_tangleFig;
    }

    template <size_t R, size_t C>
    void setPlTang(PlTang<R,C> const& pltang) {
        mp_tangleFig = new TangleMoveFigure<PlTang<R,C>>{
            pltang,
            Eigen::Vector2d(40.0, 0.0),
            Eigen::Vector2d(0.0, 40.0)
        };
        // Figure3DView class will deletes the pointer in the destructor.
        m_drawPane->setFigure(mp_tangleFig);
    }

    //! Get the associated figure.
    //! \warning Make sure that there is a valid figure by \see isvalid().
    template <size_t R, size_t C>
    TangleMoveFigure<PlTang<R,C>>& getFigure() noexcept {
        return *mp_tangleFig;
    }

    template <size_t R, size_t C>
    TangleMoveFigure<PlTang<R,C>> const & getFigure() const noexcept {
        return *mp_tangleFig;
    }

protected:
    //! Called when wxDialog gets ready.
    void CreateControls();

    /** Event handlers **/
    void OnClose(wxCloseEvent &event);
    void OnTikz(wxCommandEvent &event);
};

