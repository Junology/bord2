/*!
 * \file TikzScheme.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 6, 2019: created
 */

#include "TikzScheme.hpp"

#include <iostream>
#include <iomanip>
#include <sstream>

std::string getTikzColor(const bord2::PathColor &col)
{
    switch (col) {
    case bord2::None:
        return "";

    case bord2::Black:
        return "black";

    case bord2::White:
        return "white";

    case bord2::Red:
        return "red";

    case bord2::Green:
        return "green";

    case bord2::Blue:
        return "blue";

    case bord2::Cyan:
        return "cyan";

    case bord2::Magenta:
        return "magenta";

    case bord2::Yellow:
        return "yellow";

    case bord2::Gray:
        return "gray";

    case bord2::Brown:
        return "brown";

    case bord2::Lime:
        return "lime";

    case bord2::Olive:
        return "olieve";

    case bord2::Orange:
        return "orange";

    case bord2::Pink:
        return "pink";

    case bord2::Purple:
        return "purple";

    case bord2::Teal:
        return "teal";

    case bord2::Violet:
        return "violet";

    default:
        std::cerr << "Unknown color: " << col
                  << " (" << __FILE__ << "," << __LINE__ << ")"
                  << std::endl;
        return "";
    }
}

std::string getTikzPattern(const bord2::StrokePattern &pat)
{
    switch(pat) {
    case bord2::Solid:
        return "solid";

    case bord2::Dotted:
        return "dotted";

    case bord2::Dashed:
        return "dashed";

    default:
        std::cerr << "Unknown pattern: " << pat
                  << " (" << __FILE__ << "," << __LINE__ << ")"
                  << std::endl;
        return "";
    }
}

std::string intercalate(std::string med, std::vector<std::string> const& strs)
{
    std::string result;

    for(size_t i = 0; i < strs.size(); ++i) {
        result += strs[i];
        if (i+1 < strs.size())
            result += med;
    }

    return result;
}

template <class C>
std::ostream& operator<<(std::basic_ostream<C>& out, std::array<double,2> const&arr)
{
    out.put('(');
    out << arr[0];
    out.put(',');
    out << arr[1];
    out.put(')');

    return out;
}

static std::string realizePath(std::vector<TikzScheme::PathElement> const& path)
{
    std::ostringstream oss;

    oss << std::setprecision(4) << std::defaultfloat;
    for(auto elem : path) {
        switch (elem.type) {
        case bord2::PathElemType::BeginPoint:
            oss << " " << elem.v[0];
            break;

        case bord2::PathElemType::Line:
            oss << " -- " << elem.v[0];
            break;

        case bord2::PathElemType::Bezier:
            oss << " .. controls "
                << elem.v[0] << "and" << elem.v[1]
                << " .. " << elem.v[2];
            break;

        case bord2::PathElemType::QBezier:
            oss << " to[quadratic={" << elem.v[0] << "}] " << elem.v[1];
            break;

        default:
            std::cerr << "Unknown path element: " << elem.type
                      << " (" << __FILE__ << "," << __LINE__ << ")"
                      << std::endl;
            return "";
        }
    }

    return oss.str();
}

void TikzScheme::strokePres()
{
    if (m_path.empty() || m_state.pencolor == bord2::None)
        return;

    std::vector<std::string> drawattr{};

    if (m_state.pencolor != bord2::Black) {
        drawattr.push_back(getTikzColor(m_state.pencolor));
    }
    if (m_state.penwidth != 1.0) {
        drawattr.push_back(
            std::string("linewidth=") + std::to_string(m_state.penwidth));
    }
    if (m_state.pattern != bord2::Solid) {
        drawattr.push_back(getTikzPattern(m_state.pattern));
    }

    auto attrstr = intercalate(",", drawattr);

    // Output the path
    std::ostringstream oss;
    oss << "\\draw";
    if (!attrstr.empty())
        oss << "[" << attrstr << "]";
    oss << " ";
    oss << realizePath(m_path);
    oss << ";\n";

    m_tikzcode += oss.str();
}

void TikzScheme::fillPres()
{
    if (m_path.empty() || m_state.brushcolor == bord2::None)
        return;

    // Output the path
    std::ostringstream oss;
    oss << "\\fill";
    if(m_state.brushcolor != bord2::Black)
        oss << "[" << getTikzColor(m_state.brushcolor) << "]";
    oss << " ";
    oss << realizePath(m_path);
    oss << ";\n";

    m_tikzcode += oss.str();
}
