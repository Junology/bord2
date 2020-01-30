/*!
 * \file WxGSScheme.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 6, 2019: created
 */

#define _USE_MATH_DEFINES

#include "WxGSScheme.hpp"

// #include <iostream>
// #include <cmath>
// #include <wx/gdicmn.h>

/******************************
 *** Utility functions.
 ******************************/
//! Convert Color to wxColour
wxColour getWxColor(const bord2::PathColor &col)
{
    switch (col) {
    case bord2::Black:
        return *wxBLACK;

    case bord2::White:
        return *wxWHITE;

    case bord2::Red:
        return *wxRED;

    case bord2::Green:
        return *wxGREEN;

    case bord2::Blue:
        return *wxBLUE;

    case bord2::Cyan:
        return *wxCYAN;

    case bord2::Magenta:
        return wxTheColourDatabase->Find("MAGENTA");

    case bord2::Yellow:
        return *wxYELLOW;

    case bord2::Gray:
        return wxTheColourDatabase->Find("GREY");

    case bord2::Brown:
        return wxTheColourDatabase->Find("BROWN");

    case bord2::Lime:
        return wxTheColourDatabase->Find("LIME GREEN");

    case bord2::Olive:
        return wxTheColourDatabase->Find("DARK OLIVE GREEN");

    case bord2::Orange:
        return wxTheColourDatabase->Find("ORANGE");

    case bord2::Pink:
        return wxTheColourDatabase->Find("PINK");

    case bord2::Purple:
        return wxTheColourDatabase->Find("PURPLE");

    case bord2::Teal:
        return wxColour(0, 0x80, 0x80);

    case bord2::Violet:
        return wxTheColourDatabase->Find("VIOLET");

    default:
        std::cerr << "Unknown color: " << col
                  << " (" << __FILE__ << "," << __LINE__ << ")"
                  << std::endl;
        return wxColour();
    }
}
