/*!
 * \file TikzScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 6, 2019: created
 */

#pragma once

#include <vector>
#include "PathScheme.hpp"

std::string getTikzColor(const bord2::PathColor &col);
std::string getTikzPattern(const bord2::StrokePattern &pat);

class TikzScheme : public PathScheme<std::array<double,2>>
{
public:
    using vertex_type = PathScheme<std::array<double, 2>>::vertex_type;

    struct PathElement {
        enum PathElemType {
            BeginPoint,
            LineEnd,
            BezierEnd,
            QBezierEnd
        } type = BeginPoint;
        std::array<std::array<double,2>, 3> v{};
        constexpr PathElement() noexcept = default;
        constexpr PathElement(PathElement const&) noexcept = default;
    };

    struct PathState {
        bord2::PathColor pencolor = bord2::Black;
        double penwidth = 1.0;
        bord2::StrokePattern pattern = bord2::Solid;
        bord2::PathColor brushcolor = bord2::None;
        vertex_type origin{0.0, 0.0};
    };

private:
    std::string m_tikzcode{};
    std::vector<PathElement> m_path{};
    bord2::PathColor m_brush = bord2::Black;

    PathState m_state;
    std::vector<PathState> m_statestack;

public:
    TikzScheme() = default;

    std::string getTikzSetCode() const {
        return R"(\tikzset{
  quadratic/.style={
    to path={
      (\tikztostart) .. controls
      ($#1!1/3!(\tikztostart)$) and ($#1!1/3!(\tikztotarget)$)
      .. (\tikztotarget)
    }
  }
}
)";
    }

    std::string getTikzCode() const {
        return
            std::string("\\begin{tikzpicture}\n")
            + m_tikzcode
            + "\\end{tikzpicture}\n";
    }

    //* Overriding methods.
    //** Scheme data query
    BBoxT getBBox() const override {
        return {0.0, 0.0, 1.0, 1.0};
    }

    //! Check if the instance is valid or not.
    //! This is always true.
    bool isvalid() const noexcept override {
        return true;
    }

    //** Pens and Brushes
    void setPen(double wid, bord2::PathColor col = bord2::Black, bord2::StrokePattern pat = bord2::Solid) override {
        m_state.pencolor = col;
        m_state.pattern = pat;
    }

    void setBrush(bord2::PathColor col) override {
        m_state.brushcolor = col;
    }


    void translate(std::array<double,2> const& p) override {
        m_state.origin[0] += p[0];
        m_state.origin[1] += p[1];
    }

    void save() override {
        m_statestack.push_back(m_state);
    }

    void restore() override {
        m_state = m_statestack.back();
        m_statestack.pop_back();
    }

    //** Drawing paths.
    //! Stroke and flush.
    void stroke() override {
        strokePres();
        m_path.clear();
    }

    //! Stroke without flush
    void strokePres() override;

    //! Fill and flush.
    void fill() override {
        fillPres();
        m_path.clear();
    }

    //! Fill without flush
    void fillPres() override;

    //* Path elements.
    //! Move the current position.
    virtual void moveTo(vertex_type const &p) override {
        m_path.push_back(PathElement{PathElement::BeginPoint, {p}});
    }

    //! Move the current position drawing a line from the old.
    virtual void lineTo(vertex_type const &p) override {
        m_path.push_back(PathElement{PathElement::LineEnd, {p}});
    }

    //! Move the current position drawing a cubic Bezier curve.
    virtual void bezierTo(vertex_type const &c1, vertex_type const &c2, vertex_type const &p) override {
        m_path.push_back(
            PathElement{
                PathElement::BezierEnd,
                {c1, c2, p}
            });
    }

    //! Move the current position drawing a quadratic Bezier curve.
    virtual void qbezierTo(vertex_type const &c, vertex_type const &p) override {
        m_path.push_back(
            PathElement{
                PathElement::QBezierEnd,
                {c, p}
            });
    }

    //! Close path.
    virtual void closePath() override {
        m_path.push_back(
            PathElement{
                PathElement::LineEnd,
                {m_path.front().v[0]}
            });
    }
};
