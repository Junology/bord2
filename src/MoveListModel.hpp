/*!
 * \file MoveListModel.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 28, 2020: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#    include <wx/dataview.h>
#endif

#include <vector>

#include "config.hpp"
#include "PlTangMove.hpp"

//! List model for tangle moves
class MoveListModel : public wxDataViewVirtualListModel
{
public:
    enum : unsigned int {
        Col_MoveName = 0,
        Col_XCoord,
        Col_YCoord,
        Col_Max
    };
protected:
    struct MoveListElem {
        PlTangMove<2,2> move;
        size_t x, y;

        MoveListElem() noexcept = default;
        constexpr MoveListElem(PlTangMove<2,2> const& move0, size_t x0, size_t y0) noexcept
        : move(move0), x(x0), y(y0)
        {}

        MoveListElem(MoveListElem const&) noexcept = default;
        MoveListElem(MoveListElem &&) noexcept = default;
        ~MoveListElem() noexcept = default;
    };

private:
    std::vector<MoveListElem> m_elems;

public:
    MoveListModel() noexcept
        : wxDataViewVirtualListModel()
    {}

    virtual ~MoveListModel() = default;

    void append(MoveListElem const& elem) noexcept {
        m_elems.push_back(elem);
        wxDataViewVirtualListModel::RowAppended();
    }

    void emplace_back(PlTangMove<2,2> const& move, size_t x, size_t y) noexcept {
        m_elems.emplace_back(move, x, y);
        wxDataViewVirtualListModel::RowAppended();
    }

    void pop_back() noexcept {
        if (m_elems.size() > 0) {
            m_elems.pop_back();
            wxDataViewVirtualListModel::RowDeleted(m_elems.size());
        }
    }

    void reset() noexcept {
        m_elems.clear();
        wxDataViewVirtualListModel::Reset(0);
    }

    virtual unsigned int GetColumnCount() const override { return Col_Max; }

    virtual wxString GetColumnType(unsigned int col) const override {
        switch(col) {
        case Col_MoveName:
        case Col_XCoord:
        case Col_YCoord:
            return wxVariant(wxString()).GetType();

        default:
            return wxString();
        }
    }

    virtual void GetValueByRow(wxVariant &variant, unsigned int row, unsigned int col) const override
    {
        if (row >= m_elems.size()) {
            wxFAIL_MSG("Invalid row");
            return;
        }

        switch(col) {
        case Col_MoveName:
            variant = wxString(m_elems[row].move.getName());
            break;

        case Col_XCoord:
            variant = (wxString() << m_elems[row].x);
            break;

        case Col_YCoord:
            variant = (wxString() << m_elems[row].y);
            break;

        default:
            wxFAIL_MSG("Invalid column");
        }
    }

    //! Changing value is prohibited.
    virtual bool SetValueByRow(wxVariant const& variant, unsigned int row, unsigned int col) override
    {
        return false;
    }
};
