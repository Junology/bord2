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

    struct MoveElem {
        PlTangMove<2,2> move;
        size_t x, y;
    };

    using MoveSeq = std::vector<MoveElem>;

private:
    std::vector<MoveSeq> m_mvseqs;
    std::vector<MoveSeq>::iterator m_cur;

public:
    MoveListModel() noexcept
      : wxDataViewVirtualListModel(),
        m_mvseqs{}, m_cur{m_mvseqs.begin()}
    {}

    virtual ~MoveListModel() = default;

    MoveSeq const* getCurrent() const noexcept {
        if (m_cur != m_mvseqs.begin())
            return &(*std::prev(m_cur));
        else
            return nullptr;
    }

    void push_back(MoveSeq const& seq) noexcept {
        m_mvseqs.erase(m_cur, m_mvseqs.end());
        m_mvseqs.emplace_back(seq.begin(), seq.end());
        m_cur = m_mvseqs.end();
        wxDataViewVirtualListModel::RowAppended();
    }

    void push_back(std::initializer_list<MoveElem> &&seq) noexcept {
        m_mvseqs.erase(m_cur, m_mvseqs.end());
        m_mvseqs.emplace_back(seq.begin(), seq.end());
        m_cur = m_mvseqs.end();
        wxDataViewVirtualListModel::RowAppended();
    }

    void roll_back() noexcept {
        if (m_cur != m_mvseqs.begin()) {
            //m_mvseqs.pop_back();
            //wxDataViewVirtualListModel::RowDeleted(m_mvseqs.size());
            --m_cur;
            wxDataViewVirtualListModel::RowDeleted(
                std::distance(m_mvseqs.begin(), m_cur));
        }
    }

    void advance() noexcept {
        if (m_cur != m_mvseqs.end()) {
            ++m_cur;
            wxDataViewVirtualListModel::RowAppended();
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
