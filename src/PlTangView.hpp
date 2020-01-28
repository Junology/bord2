/*!
 * \file PlTangView.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 25, 2020: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#endif

#include <stack>
#include <vector>

#include "PlTang.hpp"
#include "PlTangMove.hpp"

/*!
 * Styles of the control.
 */
enum PlTangViewStyles : size_t {
    PTVS_HORIZONTAL,
    PTVS_VERTICAL,
    PTVS_DEFAULT = PTVS_VERTICAL
};

/*!
 * A control exhibits and manipulates planar tangles.
 */
class PlTangView : public wxControl
{
    wxDECLARE_DYNAMIC_CLASS(plTangView);
    wxDECLARE_EVENT_TABLE();

public:
    using MoveType = PlTangMove<2,2>;

private:
    enum : long {
        PTV_WINDOW_STYLE
            = wxBORDER_STATIC | wxVSCROLL | wxHSCROLL // | wxFULL_REPAINT_ON_RESIZE
    };

    enum : int {
        DefaultCellW = 48,
        DefaultCellH = 48
    };

    PlTangViewStyles m_style{PTVS_DEFAULT};
    PlTang<> m_pltang{};
    wxSize m_cellsz{DefaultCellW, DefaultCellH};
    wxPoint m_clientOrig{0,0};
    int m_curx, m_cury;

    //! Stack of moves that have been applied.
    std::stack<std::tuple<MoveType,size_t,size_t>> m_moveHistory{};

    //! Set of moves available.
    std::vector<MoveType> m_moveDict{};

public:
    PlTangView() = default;

    PlTangView(wxWindow *parent,
               wxWindowID winid,
               const PlTang<>& pltang,
               const wxPoint& pos = wxDefaultPosition,
               const wxSize& size = wxDefaultSize,
               PlTangViewStyles style = PTVS_VERTICAL,
               const wxValidator& val = wxDefaultValidator)
        : wxControl(parent, winid, pos, size, PTV_WINDOW_STYLE, val, wxControlNameStr),
          m_pltang(pltang)
    {}

    bool Create(wxWindow *parent,
               wxWindowID winid,
               const PlTang<>& pltang,
               const wxPoint& pos = wxDefaultPosition,
               const wxSize& size = wxDefaultSize,
               PlTangViewStyles style = PTVS_VERTICAL,
               const wxValidator& val = wxDefaultValidator)
    {
        m_pltang = pltang;
        return wxControl::Create(parent, winid, pos, size, PTV_WINDOW_STYLE, val, wxControlNameStr);
    }

    void SetPlTang(const PlTang<>& pltang) noexcept {
        m_pltang = pltang;
    }

    PlTang<> GetPlTang() const noexcept {
        return m_pltang;
    }

    void registerMove(MoveType &&move) noexcept {
        m_moveDict.push_back(std::forward<MoveType>(move));
    }

    void registerMove(MoveType const&move) noexcept {
        m_moveDict.push_back(move);
    }

protected:
    virtual wxSize DoGetBestClientSize() const override;
    virtual wxSize DoGetBestSize() const override;

    void OnSize(wxSizeEvent&) noexcept;
    void OnScroll(wxScrollWinEvent&) noexcept;
    void OnMouseMove(wxMouseEvent&) noexcept;
    void OnMouseLeft(wxMouseEvent&) noexcept;
    void OnPaint(wxPaintEvent&) noexcept;
};
