/**
 * \file PlTangEntryDialog.hpp
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

#include <sstream>

#include "PlTang.hpp"

class PlTangEntryDialog : public wxDialog
{
    wxDECLARE_DYNAMIC_CLASS(PlTangEntryDialog);
    wxDECLARE_EVENT_TABLE();

protected:
    template <class T>
    struct EntryValidatorOf;

    template <size_t R, size_t C>
    class EntryValidatorOf< PlTang<R,C> > : public wxTextValidator
    {
    private:
        PlTang<R,C> *m_pltang;

    public:
        EntryValidatorOf(PlTang<R,C> *pltang) : m_pltang(pltang) {}

        bool verifyPlTang() const {
            if(m_pltang && m_pltang->isvalid()) {
                return true;
            }
            else {
                std::ostringstream oss;
                oss << "Invalid ASCII-Art representation:" << std::endl;
                oss << m_pltang->aarep();
                wxMessageBox(oss.str(), "Error", wxICON_WARNING);
                return false;
            }
        }

        /*** Inherited virtual functions ***/
        virtual wxObject* Clone() const override {
            return new EntryValidatorOf<PlTang<R,C>>(*this);
        }

        virtual bool Validate(wxWindow *parent) override {
            wxTextCtrl *txtentry = wxDynamicCast(GetWindow(), wxTextCtrl);

            if (txtentry && m_pltang) {
                (*m_pltang) = PlTang<R,C>{
                    static_cast<char const*>(txtentry->GetValue())
                };
                return verifyPlTang();
            }
            else
                return false;
        }

        virtual bool TransferToWindow() override {
            wxTextCtrl *txtentry = wxDynamicCast(GetWindow(), wxTextCtrl);

            if (m_pltang && txtentry) {
                txtentry->SetValue(m_pltang->aarep());
                return true;
            }
            else {
                return false;
            }
        }

        virtual bool TransferFromWindow() override {
            wxTextCtrl *txtentry = wxDynamicCast(GetWindow(), wxTextCtrl);
            if (m_pltang && txtentry) {
                (*m_pltang) = PlTang<R,C>{
                    static_cast<char const*>(txtentry->GetValue())
                };
                return true;
            }
            else {
                return false;
            }
        }
    };

private:
    wxTextCtrl *m_textctrl{nullptr};
    PlTang<> m_pltang{};

public:
    enum : long {
        PTED_DEFAULT_STYLE = wxCAPTION | wxCLOSE_BOX | wxSYSTEM_MENU | wxRESIZE_BORDER | wxTAB_TRAVERSAL
    };
    //! Default constructor
    PlTangEntryDialog() : wxDialog{} {};

    //! Dialog constructor
    PlTangEntryDialog(wxWindow *parent,
                      wxWindowID id,
                      wxString const& message,
                      wxString const& title,
                      wxPoint const& pos = wxDefaultPosition,
                      wxSize const& size = wxDefaultSize,
                      long style = PTED_DEFAULT_STYLE )
        : wxDialog{parent, id, title, pos, size, style}
    {
        CreateControls(message);
    }

    ~PlTangEntryDialog() noexcept = default;

    void Create(wxWindow *parent,
                wxWindowID id,
                wxString const& message,
                wxString const& title,
                wxPoint const& pos = wxDefaultPosition,
                wxSize const& size = wxDefaultSize,
                long style = wxDEFAULT_DIALOG_STYLE)
    {
        wxDialog::Create(parent, id, title, pos, size, style);
        CreateControls(message);
    }

    wxString GetValue() const {
        return m_textctrl ? m_textctrl->GetValue() : wxString();
    }

    auto GetPlTang() const {
        return m_pltang;
    }

protected:
    //! Called when wxDialog gets ready.
    void CreateControls(wxString const& msg);
    //void OnClose(wxCloseEvent&);
};
