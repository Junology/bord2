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
#include <sstream>

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
    struct MoveElem {
        PlTangMove<2,2> move;
        size_t x, y;
    };

    using MoveSeq = std::vector<MoveElem>;

private:
    std::vector<MoveSeq> m_mvseqs;

public:
    MoveListModel() noexcept
        : wxDataViewVirtualListModel()
    {}

    virtual ~MoveListModel() = default;

    void append(MoveSeq const& seq) noexcept {
        m_mvseqs.emplace_back(seq.begin(), seq.end());
        wxDataViewVirtualListModel::RowAppended();
    }

    void emplace_back(PlTangMove<2,2> const& move, size_t x, size_t y) noexcept {
        m_mvseqs.push_back({MoveElem{move, x, y}});
        wxDataViewVirtualListModel::RowAppended();
    }

    void pop_back() noexcept {
        if (m_mvseqs.size() > 0) {
            m_mvseqs.pop_back();
            wxDataViewVirtualListModel::RowDeleted(m_mvseqs.size());
        }
    }

    void reset() noexcept {
        m_mvseqs.clear();
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
        if (row >= m_mvseqs.size()) {
            wxFAIL_MSG("Invalid row");
            return;
        }

        std::vector<std::string> attrs{};
        for(auto& mvelem : m_mvseqs[row]) {
            switch(col) {
            case Col_MoveName:
                attrs.push_back(mvelem.move.getName());
                break;

            case Col_XCoord:
                attrs.push_back(std::to_string(mvelem.x));
                break;

            case Col_YCoord:
                attrs.push_back(std::to_string(mvelem.y));
                break;

            default:
                wxFAIL_MSG("Invalid column");
                return;
            }
        }
        std::ostringstream oss;
        std::copy(
            attrs.begin(), attrs.end(),
            std::ostream_iterator<std::string>(oss, "\n"));

        variant = wxString(oss.str());
    }

    //! Changing value is prohibited.
    virtual bool SetValueByRow(wxVariant const& variant, unsigned int row, unsigned int col) override
    {
        return false;
    }
};
