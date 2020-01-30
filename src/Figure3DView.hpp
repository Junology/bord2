/*!
 * \file Figure3DView.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 3, 2019: created
 */

#pragma once

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#    include <wx/wx.h>
#endif

#include <memory>

#include "figures/PathFigure3D.hpp"

class Figure3DView : public wxPanel
{
    wxDECLARE_DYNAMIC_CLASS(Figure3DView);
    wxDECLARE_EVENT_TABLE();

private:
    std::unique_ptr<PathFigure3D> mp_fig{};
    double m_elev{0.0}, m_azim{0.0};
    std::array<double,3> m_focus{0.0, 0.0, 0.0};

public:
    enum : long {
        MDP_DEFAULT_STYLE = wxTAB_TRAVERSAL
    };

    typedef enum _Error {
        FailedCreatingGC
    } Error;

    Figure3DView()
        : wxPanel{}
    {
        Init();
    }

    Figure3DView(wxWindow *parent,
                 wxWindowID id = wxID_ANY,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = MDP_DEFAULT_STYLE,
                 const wxString& name = wxPanelNameStr)
        : wxPanel(parent, id, pos, size, style, name)
    {
        Init();
    }

    //! Set figure.
    //! \param pfig A pointer to an instance of PathFigure3D which will be stored in std::unique_ptr; so one must not delete it unless \see release().
    //! This function just calls std::unique_ptr::reset(), so the old pointer, if any, will be deleted through it.
    void setFigure(PathFigure3D *pfig) {
        mp_fig.reset(pfig);
    }

    //! Release an instance of PathFigure3D stored in the view.
    //! This function not only releases std::unique_ptr but also set nullptr.
    PathFigure3D* release() {
        auto ret = mp_fig.get();
        mp_fig.release();
        mp_fig.reset(nullptr);
        return ret;
    }

    inline void queuePaint() {
        render(wxClientDC(this));
    }

protected:
    void Init() {}

    void OnPaint(wxPaintEvent &event);
    void OnKeyDown(wxKeyEvent &event);
    void render(wxWindowDC &&dc);
};

