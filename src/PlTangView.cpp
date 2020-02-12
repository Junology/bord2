/*!
 * \file PlTangView.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 25, 2020: created
 */

#include <wx/graphics.h>

#include "WxGSScheme.hpp"
#include "PlTangView.hpp"
#include "figures/PlTangFigure.hpp"

wxIMPLEMENT_DYNAMIC_CLASS(PlTangView,wxControl);
wxIMPLEMENT_DYNAMIC_CLASS(PlTangEvent, wxCommandEvent);
wxDEFINE_EVENT(PLTANG_MOVED, PlTangEvent);

wxBEGIN_EVENT_TABLE(PlTangView, wxControl)
  EVT_SIZE(PlTangView::OnSize)
  EVT_SCROLLWIN(PlTangView::OnScroll)
  EVT_MOTION(PlTangView::OnMouseMove)
  EVT_LEFT_DOWN(PlTangView::OnMouseLeft)
  EVT_PAINT(PlTangView::OnPaint)
wxEND_EVENT_TABLE()

template<size_t MR, size_t MC>
static bool
renderPlTang(wxWindowDC const& dc, PlTang<MR,MC> tang, wxPoint orig, wxPoint baseX, wxPoint baseY)
{
    AdapterScheme<WxGSScheme, Eigen::Vector2d> scheme{
        new WxGSScheme(dc),
            [](Eigen::Vector2d vec) {
            return std::array<double,2>{vec(0), vec(1)};
        }
    };

    if(!scheme.isvalid())
        return false;

    PlTangFigure pltangfig{
        tang,
        Eigen::Vector2d(baseX.x, baseX.y),
        Eigen::Vector2d(baseY.x, baseY.y)
    };

    scheme.translate(Eigen::Vector2d(orig.x, orig.y));
    scheme.setPen(2, bord2::Red);
    pltangfig.draw(scheme);

    return true;
}

bool PlTangView::undo(bool invoke) noexcept
{
    if (m_cur == m_moveHistory.begin())
        return false;

    // Move the cursor back.
    --m_cur;

    // Reverse the last move.
    m_pltang.replace(m_cur->x, m_cur->y, m_moveDict[m_cur->index].getBefore());

    return true;
}

bool PlTangView::redo(bool invoke) noexcept
{
    if (m_cur == m_moveHistory.end())
        return false;

    // Apply the next move.
    m_pltang.replace(m_cur->x, m_cur->y, m_moveDict[m_cur->index].getAfter());

    // Move the cursor forward.
    ++m_cur;

    return true;
}

bool PlTangView::applyMove(const std::string &name, size_t x, size_t y, bool invoke) noexcept
{
    size_t i = 0;

    for(; i < m_moveDict.size(); ++i) {
        if(std::strcmp(name.c_str(), m_moveDict[i].getName()) == 0) {
            auto loctang = m_pltang.slice<MoveType::rows, MoveType::cols>(x, y);
            if(m_moveDict[i].getBefore() == loctang)
                break;
        }
    }

    return applyMove(i, x, y, invoke);
}

bool PlTangView::applyMove(size_t i, size_t x, size_t y, bool revert, bool invoke) noexcept
{
    if (i >= m_moveDict.size())
        return false;

    // Apply the move
    auto newloc
        = revert ? m_moveDict[i].getBefore() : m_moveDict[i].getAfter();

    m_pltang.replace(x, y, newloc);

    // Clear the old "future" in the history
    m_moveHistory.erase(m_cur, m_moveHistory.end());
    // Add new move to the history.
    m_moveHistory.emplace_back(i,x,y);
    m_cur = m_moveHistory.end();

    //! Invoke event
    if (invoke) {
        PlTangEvent event{m_moveDict[i], x, y, revert, PLTANG_MOVED, GetId()};
        event.SetEventObject(this);

        ProcessEvent(event);
    }

    return true;
}

wxSize PlTangView::DoGetBestClientSize() const
{
    if (!m_pltang.isvalid())
        return wxSize();

    int wid = m_style == PTVS_VERTICAL ? m_pltang.hlength() : m_pltang.vlength();
    int hei = m_style == PTVS_VERTICAL ? m_pltang.vlength() : m_pltang.hlength();
    return wxSize(m_cellsz.GetWidth()*wid, m_cellsz.GetHeight()*hei);
}

wxSize PlTangView::DoGetBestSize() const
{
    auto sz = DoGetBestClientSize();
    return {sz.GetWidth()+2, sz.GetHeight()+2};
}

void PlTangView::OnSize(wxSizeEvent& event) noexcept
{
    auto bestsz = DoGetBestClientSize();
    auto actsz = GetClientSize();

    if(actsz.GetWidth() < bestsz.GetWidth())
        m_clientOrig.x = 0;
    if(actsz.GetHeight() < bestsz.GetHeight())
        m_clientOrig.y = 0;

    SetScrollbar(wxHORIZONTAL, m_clientOrig.x, actsz.GetWidth(), bestsz.GetWidth());
    SetScrollbar(wxVERTICAL, m_clientOrig.y, actsz.GetHeight(), bestsz.GetHeight());
    event.Skip();
}

void PlTangView::OnScroll(wxScrollWinEvent& event) noexcept
{
    switch(event.GetOrientation()) {
    case wxHORIZONTAL:
        m_clientOrig.x = GetScrollPos(wxHORIZONTAL);
        break;

    case wxVERTICAL:
        m_clientOrig.y = GetScrollPos(wxVERTICAL);
        break;
    }

    Refresh();
    event.Skip();
}

void PlTangView::OnMouseMove(wxMouseEvent& event) noexcept
{
    if(m_is_locked) {
        event.Skip();
        return;
    }

    bool modified = false;

    auto vpos = event.GetPosition() + m_clientOrig + m_cellsz/2;
    int x = vpos.x / m_cellsz.GetWidth();
    int y = vpos.y / m_cellsz.GetHeight();
    auto xmax
        = m_style == PTVS_VERTICAL ? m_pltang.hlength() : m_pltang.vlength();
    auto ymax
        = m_style == PTVS_VERTICAL ? m_pltang.vlength() : m_pltang.hlength();

    if(1 <= x && x < xmax && 1 <= y && y < ymax) {
        if(x != m_curx) {
            m_curx = x;
            modified = true;
        }
        if (y != m_cury) {
            m_cury = y;
            modified = true;
        }
    }
    else {
        if (m_curx > 0 || m_cury > 0) {
            m_curx = -1;
            m_cury = -1;
            modified = true;
        }
    }

    if(modified)
        Refresh();

    event.Skip();
}

void PlTangView::OnMouseLeft(wxMouseEvent& event) noexcept
{
    if(m_is_locked) {
        event.Skip();
        return;
    }

    if (m_curx > 0 && m_cury > 0) {
        auto loctang = m_pltang.slice<MoveType::rows, MoveType::cols>(m_curx-1, m_cury-1);
        wxMenu *popup = new wxMenu;

        for(size_t i = 0; i < m_moveDict.size(); ++i) {
            if (m_moveDict[i].isvalid() && m_moveDict[i].getBefore() == loctang)
                popup->Append(i, m_moveDict[i].getName());
        }

        if(popup->GetMenuItemCount() > 0) {
            popup->Bind(wxEVT_COMMAND_MENU_SELECTED, [&popup,this](wxCommandEvent& ev) {
                    applyMove(ev.GetId(), m_curx-1, m_cury-1);
                    Refresh();
                });
            this->PopupMenu(popup);
        }

        delete popup;
    }
    event.Skip();
}

void PlTangView::OnPaint(wxPaintEvent& event) noexcept
{
    wxPaintDC dc{this};

    if (!m_pltang.isvalid()) {
        // Clear
        dc.SetBackground(*wxGREY_BRUSH);
        dc.Clear();
        return;
    }

    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();

    // Calculate basis vectors
    wxPoint baseX{
        m_style == PTVS_VERTICAL ? m_cellsz.GetWidth() : m_cellsz.GetHeight(),
        0 };
    wxPoint baseY{
        0,
        m_style == PTVS_VERTICAL ? m_cellsz.GetHeight() : m_cellsz.GetWidth() };

    // Draw grid
    dc.SetPen(*wxGREY_PEN);
    // Draw horizontal lines of grid
    for(size_t i = 0; i <= m_pltang.vlength(); ++i)
        dc.DrawLine(baseY*i - m_clientOrig,
                    baseY*i + baseX*m_pltang.hlength() - m_clientOrig);
    for(size_t i = 0; i <= m_pltang.hlength(); ++i )
        dc.DrawLine(baseX*i - m_clientOrig,
                    baseX*i + baseY*m_pltang.vlength() - m_clientOrig);

    // Draw planar tangle
    renderPlTang(dc, m_pltang, -m_clientOrig, baseX, baseY);

    // Mask selected cells
    if(m_curx > 0 && m_cury > 0) {
        auto orig = wxPoint((m_curx-1)*m_cellsz.x, (m_cury-1)*m_cellsz.y) - m_clientOrig;
        dc.SetPen(wxNullPen);
        dc.SetBrush(wxBrush(wxColour(0,0,0xFF,0x7F), wxBRUSHSTYLE_CROSS_HATCH));
        dc.DrawRectangle(orig, m_cellsz*2);
    }

    event.Skip();
}
