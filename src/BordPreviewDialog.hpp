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

public:
    enum : long {
        BPD_DEFAULT_STYLE = wxCAPTION | wxCLOSE_BOX | wxSYSTEM_MENU | wxRESIZE_BORDER | wxTAB_TRAVERSAL | wxDIALOG_NO_PARENT
    };

    using TangType = PlTang<>;

private:
    Figure3DView *m_drawPane = nullptr;
    TangleMoveFigure<PlTang<>> *mp_tangleFig = nullptr;

    //! Flag indicating the direction of bordisms.
    //! i.e. the top-to-bottom mode or the other
    bool m_is_top2bottom = true;

public:
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

    void setPlTangMove(TangType const& pltang,
                       std::vector<PlTangMove<2,2>::MoveSeq> const& mvseqs)
    {
        mp_tangleFig->setTangleMove(pltang, mvseqs);
        m_drawPane->updateBuffer();
        m_drawPane->Refresh();
    }

    void setFigureBase() noexcept {
        constexpr double unit = 40;
        double z = m_is_top2bottom ? unit : -unit;
        mp_tangleFig->baseMatrix() <<
            unit, 0, 0,
            0, unit, 0,
            0, 0,    z;
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
    void OnVDirection(wxCommandEvent&);
    void OnProjOrtho(wxCommandEvent&);
    void OnProjCabinet(wxCommandEvent&);
    void OnTikz(wxCommandEvent &event);
    void OnExit(wxCommandEvent &event);
};

