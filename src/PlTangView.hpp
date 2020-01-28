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
//#include "PlTangEvent.hpp"

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
    using TangleType = PlTang<>;
    using MoveType = PlTangMove<2,2>;

    struct MoveData {
        size_t index, x, y;

        constexpr MoveData(size_t i0, size_t x0, size_t y0) noexcept
          : index(i0), x(x0), y(y0) {}
        constexpr MoveData(MoveData const& src) noexcept = default;

        ~MoveData() noexcept = default;
    };

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
    TangleType m_pltang{};
    wxSize m_cellsz{DefaultCellW, DefaultCellH};
    wxPoint m_clientOrig{0,0};
    int m_curx, m_cury;

    //! Stack of moves that have been applied.
    std::stack<MoveData> m_moveHistory{};
    //! Stack of moves that are to be applied.
    std::stack<MoveData> m_moveRedoers{};

    //! Set of moves available.
    std::vector<MoveType> m_moveDict{};

public:
    PlTangView() = default;

    PlTangView(wxWindow *parent,
               wxWindowID winid,
               const TangleType& pltang,
               const wxPoint& pos = wxDefaultPosition,
               const wxSize& size = wxDefaultSize,
               PlTangViewStyles style = PTVS_VERTICAL,
               const wxValidator& val = wxDefaultValidator)
        : wxControl(parent, winid, pos, size, PTV_WINDOW_STYLE, val, wxControlNameStr),
          m_pltang(pltang)
    {}

    virtual ~PlTangView() noexcept = default;

    bool Create(wxWindow *parent,
               wxWindowID winid,
               const TangleType& pltang,
               const wxPoint& pos = wxDefaultPosition,
               const wxSize& size = wxDefaultSize,
               PlTangViewStyles style = PTVS_VERTICAL,
               const wxValidator& val = wxDefaultValidator)
    {
        m_pltang = pltang;
        return wxControl::Create(parent, winid, pos, size, PTV_WINDOW_STYLE, val, wxControlNameStr);
    }

    void SetPlTang(const TangleType& pltang) noexcept {
        m_pltang = pltang;
    }

    TangleType GetPlTang() const noexcept {
        return m_pltang;
    }

    //! Register a move so that it is available to apply.
    //! \param move The moves to be registered.
    //! \warning The duplicate of names will be not checked.
    void registerMove(MoveType &&move) noexcept {
        m_moveDict.push_back(std::forward<MoveType>(move));
    }

    //! Register a move so that it is available to apply.
    //! \param move The moves to be registered. If it is invalid, the function just ignores it.
    //! \warning The duplicate of names will be not checked.
    void registerMove(MoveType const&move) noexcept {
        if (move.isvalid())
            m_moveDict.push_back(move);
    }

    //! Apply the move on the top of the m_moveHistory stack.
    //! \retval true if undo is proceessed.
    //! \retval false if the stack is empty.
    bool undo() noexcept;

    //! Apply the move on the top of the m_moveRedoers stack.
    //! \retval true if redo was processed.
    //! \retval false if the stack is empty.
    bool redo() noexcept;

    //! Apply a move to the planar tangle.
    //! Compatibility check will be performed.
    //! It also pushes the move onto the stack m_moveHistory; if m_moveRedoers is non-empty, it will be cleared.
    //! \param name The name of the move applied, which have to be registered with \see registerMove.
    //! \param x The x-coordinate of the position where the move is applied.
    //! \param y The y-coordinate of the position where the move is applied.
    bool applyMove(std::string const& name, size_t x, size_t y) noexcept;

protected:
    //! Apply a move to the planar tangle; a version available only for derived classes.
    //! Compatibility check will not be performed.
    //! \param i The index of the move in the dictionary.
    //! \param x The x-coordinate of the position where the move is applied.
    //! \param y The y-coordinate of the position where the move is applied.
    //! \param revert If true, the move will be reverted in application.
    bool applyMove(size_t i, size_t x, size_t y, bool revert = false) noexcept;

    virtual wxSize DoGetBestClientSize() const override;
    virtual wxSize DoGetBestSize() const override;

    void OnSize(wxSizeEvent&) noexcept;
    void OnScroll(wxScrollWinEvent&) noexcept;
    void OnMouseMove(wxMouseEvent&) noexcept;
    void OnMouseLeft(wxMouseEvent&) noexcept;
    void OnPaint(wxPaintEvent&) noexcept;
};

//! Events invoked by PlTangView
//using PlTangViewEvent = PlTangEvent<2,2>;
struct PlTangEvent : public wxCommandEvent
{
    wxDECLARE_DYNAMIC_CLASS_NO_ASSIGN(PlTangEvent);
public:
    using MoveType = PlTangMove<2,2>;

private:
    MoveType m_move{};
    size_t m_x{}, m_y{};
    bool m_isrevert = false;

public:
    PlTangEvent() = default;

    PlTangEvent(MoveType const& move, size_t x, size_t y, bool revert, wxEventType eventType = wxEVT_NULL, int winid = wxID_ANY)
        : wxCommandEvent(eventType, winid),
          m_move(move), m_x(x), m_y(y), m_isrevert(revert)
    {
    }

    PlTangEvent(PlTangEvent const& src) = default;          

    virtual ~PlTangEvent() = default;

    MoveType GetMove() const noexcept { return m_move; }
    size_t GetX() const noexcept { return m_x; }
    size_t GetY() const noexcept { return m_y; }
    bool IsRevert() const noexcept { return m_isrevert; }

    virtual wxEvent *Clone() const override {return new PlTangEvent(*this);}
    virtual wxEventCategory GetEventCategory() const override { return wxEVT_CATEGORY_USER_INPUT; }
};

wxDECLARE_EVENT(PLTANG_MOVED, PlTangEvent);

#define EVT_PLTANG_MOVED(id, func)                \
    DECLARE_EVENT_TABLE_ENTRY(                    \
        PLTANG_MOVED, id, wxID_ANY,               \
        &func, nullptr                            \
        ),

