/*!
 * \file PlTangFigure.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 29, 2020: created
 */

#pragma once

#include <Eigen/Dense>
#include <map>

#include "../PlTang.hpp"
#include "../PathScheme.hpp"
#include "../GenericGraph.hpp"

#include <iostream> // For Debug


class PlTangFigure : public PathFigure<Eigen::Vector2d>
{
private:
    struct StrandJoint {
        enum Position : size_t {
            None  = 0x0,
            Left  = 0x1,
            Right = 0x2,
            Up    = 0x4,
            Down  = 0x8
        } pos;
        size_t x, y;
        constexpr bool operator==(StrandJoint other) const noexcept {
            if(pos == other.pos)
                return x==other.x && y==other.y;
            else if(pos == Left && other.pos == Right)
                return x==other.x+1 && y==other.y;
            else if(pos == Right && other.pos == Left)
                return x+1==other.x && y==other.y;
            else if(pos == Up && other.pos == Down)
                return x==other.x && y==other.y+1;
            else if(pos == Down && other.pos == Up)
                return x==other.x && y+1==other.y;
            else
                return false;
        }
    };

    std::vector<std::vector<StrandJoint>> m_components{};
    size_t m_rows, m_cols;
    Eigen::Vector2d m_baseX, m_baseY;

public:
    template <size_t R, size_t C>
    PlTangFigure(PlTang<R,C> const& pltang, Eigen::Vector2d const& baseX, Eigen::Vector2d const& baseY) noexcept
        : m_rows(pltang.vlength()),
          m_cols(pltang.hlength()),
          m_baseX(baseX), m_baseY(baseY)
    {
        GenericGraph<std::map<size_t,StrandJoint>> graph;

        for(size_t j = 0; j < m_rows; ++j) {
            pltang.forElTang(
                j,
                [&graph, &j](size_t i, char c) {
                    auto pos = mkJointPos(c);

                    if (pos[0] == StrandJoint::None || pos[1] == StrandJoint::None)
                        return;

                    size_t key0 = appendGraphNotFound(
                        graph, StrandJoint{pos[0], i, j} );
                    size_t key1 = appendGraphNotFound(
                        graph, StrandJoint{pos[1], i, j} );
                    graph.connect(key0, key1);
                } );
        }

        while(graph.inhabited()) {
            auto gcomp = graph.trimComponent(graph.minkey());
            m_components.emplace_back();

            auto maybe_end = gcomp.findEnd();
            size_t key = maybe_end.first ? maybe_end.second : gcomp.minkey();
            auto last = gcomp.trackPath(
                key,
                [&](std::pair<size_t, StrandJoint> const& val) {
                    this->m_components.back().push_back(val.second);
                } );

            if (last.first) {
                m_components.back().push_back(gcomp.at(last.second));
            }
        }
    }

    ~PlTangFigure() noexcept = default;

    virtual void draw(SchemeType &scheme) const override
    {
        for(auto& path : m_components) {
            if (path.empty()) {
                // Error
                continue;
            }

            scheme.moveTo(joint2Vect(path[0]));

            // Check if path[0].pos is Left or Right.
            bool is_hor_prev = static_cast<bool>(path[0].pos & 0x3);
            for(size_t i = 1; i < path.size(); ++i) {
                // Check if path[i].pos is Left or Right.
                bool is_hor = static_cast<bool>(path[i].pos & 0x3);
                if (is_hor == is_hor_prev)
                    scheme.lineTo(joint2Vect(path[i]));
                else if (is_hor) /* => is_hor_prev == false */ {
                    scheme.qbezierTo(
                        Eigen::Vector2d(
                            joint2Vect(path[i-1])(0),
                            joint2Vect(path[i])(1) ),
                        joint2Vect(path[i]) );
                }
                else /* <=> is_hor_prev == true */ {
                    scheme.qbezierTo(
                        Eigen::Vector2d(
                            joint2Vect(path[i])(0),
                            joint2Vect(path[i-1])(1) ),
                        joint2Vect(path[i]) );
                }
                is_hor_prev = is_hor;
            }
            scheme.stroke();
        }
    }

protected:
    template <class K, class T>
    static K appendGraphNotFound(GenericGraph<std::map<K,T>> &graph, T const& x)
    {
        auto maybeval = graph.findKey(
            [&x](std::pair<K,T> const& val) { return val.second == x; } );
        if (maybeval)
            return maybeval->first;
        else
            return graph.append(x);
    }

    static constexpr std::array<StrandJoint::Position, 2> mkJointPos(char c) noexcept
    {
        // Take care on the order; up-to-down, left-to-right, the former is of high priority.
        switch(c) {
        case '-':
            return {StrandJoint::Left, StrandJoint::Right};

        case '|':
            return {StrandJoint::Up, StrandJoint::Down};

        case '7':
            return {StrandJoint::Left, StrandJoint::Down};

        case 'r':
            return {StrandJoint::Right, StrandJoint::Down};

        case 'L':
            return {StrandJoint::Up, StrandJoint::Right};

        case 'J':
            return {StrandJoint::Up, StrandJoint::Left};

        default: // including ' '
            return {StrandJoint::None, StrandJoint::None};
        }
    }

    Eigen::Vector2d joint2Vect(StrandJoint const& joint) const noexcept
    {
        auto p = joint.x*m_baseX + joint.y*m_baseY;

        switch(joint.pos) {
        case StrandJoint::Left:
            return p + 0.5*m_baseY;

        case StrandJoint::Right:
            return p + m_baseX + 0.5*m_baseY;

        case StrandJoint::Up:
            return p + 0.5*m_baseX;

        case StrandJoint::Down:
            return p + 0.5*m_baseX + m_baseY;

        default:
            // Error
            return Eigen::Vector2d{};
        }
    }
};
