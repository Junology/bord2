/*!
 * \file TangleMoveFigure.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January 29, 2020: created
 */

#pragma once

#include <array>
#include <vector>
#include <functional>
#include <Eigen/Dense>

#include "../PlTang.hpp"
#include "../PlTangMove.hpp"
#include "../Graph.hpp"
#include "../GenericGraph.hpp"
#include "PlTangFigure.hpp"
#include "PathFigure3D.hpp"

//* Debug
#include <iostream>
#include <iomanip>
// */

template <class T>
class TangleMoveFigure;

template <size_t R, size_t C>
class TangleMoveFigure<PlTang<R,C>> : public PathFigure3D
{
private:
    //! Internal representation of drawn object.
    using PathGraph = GenericGraph<std::map<size_t,Eigen::Vector3d>>;

    //! Tangles in the domain and the codomain of the bordism.
    PlTang<R,C> m_tangDom{}, m_tangCod{};

    //! Sequence of grouped moves.
    std::vector<PlTangMove<2,2>::MoveSeq> m_mvseqs{};

    //! Basis matrix for the 3d-space in rendering.
    Eigen::Matrix3d m_base;

    //! Vector normal to ridge-lines.
    Eigen::Vector2d m_critV;

    struct PathElem {
        bord2::PathElemType type;
        Eigen::Vector3d v;
        double midheight[2];
    };

    //! Paths in drawing.
    std::vector<std::vector<PathElem>> m_paths{};

public:
    TangleMoveFigure(PlTang<R,C> const& tang,
                     std::vector<PlTangMove<2,2>::MoveSeq> const& mvseqs,
                     Eigen::Matrix3d const& base)
        : m_tangDom(tang), m_tangCod(tang),
          m_mvseqs(mvseqs),
          m_base(base),
          m_critV(1.0, 0.0)
    {
        for(auto& seq : mvseqs) {
            for(auto& mv : seq) {
                m_tangCod.replace(mv.x, mv.y, mv.move.getAfter());
            }
        }

        makePath();
    }

    void setTangleMove(PlTang<R,C> const& tang,
                       std::vector<PlTangMove<2,2>::MoveSeq> const& mvseqs,
                       Eigen::Matrix3d const& base)
    {
        m_tangDom = tang;
        m_tangCod = tang;
        m_base = base;
        m_mvseqs = mvseqs;

        for(auto& seq : mvseqs) {
            for(auto& mv : seq) {
                m_tangCod.replace(mv.x, mv.y, mv.move.getAfter());
            }
        }

        makePath();
    }

    virtual ~TangleMoveFigure() = default;

    /**!
     * \section Member functions derived from PathFigure
     **/
    virtual void draw(SchemeType &scheme) const override
    {
        Eigen::Vector3d orig
            = -m_base * Eigen::Vector3d(
                static_cast<double>(m_tangDom.hlength())/2.0,
                static_cast<double>(m_tangDom.vlength())/2.0,
                static_cast<double>(m_mvseqs.size())/2.0 );

        scheme.save();
        scheme.translate(orig);

        /* Draw the domain tangle */{
            AdapterScheme<PathScheme<Eigen::Vector3d>, Eigen::Vector2d> adpscheme(
                &scheme,
                [this, &orig](Eigen::Vector2d const&p) {
                    return m_base.block<3,2>(0,0)*p;
                } );

            PlTangFigure pltangfig{ m_tangDom,
                    Eigen::Vector2d(1.0, 0.0),
                    Eigen::Vector2d(0.0, 1.0) };

            pltangfig.draw(adpscheme);
            adpscheme.release();
        }

        // If no moves, stop here.
        if(m_paths.empty())
            return;

        /* Draw the cobordisms */
        for(auto& path : m_paths) {
            if (path.size() <= 1)
                continue;

            scheme.moveTo(m_base * path[0].v);

            for(size_t i = 1; i < path.size(); ++i) {
                switch(path[i].type) {
                case bord2::Line:
                    scheme.lineTo(m_base*path[i].v);
                    break;

                case bord2::Bezier:
                    {
                        auto c1 = Eigen::Vector3d(
                            path[i-1].v(0),
                            path[i-1].v(1),
                            path[i].midheight[0] );
                        auto c2 = Eigen::Vector3d(
                            path[i].v(0),
                            path[i].v(1),
                            path[i].midheight[1] );
                        scheme.bezierTo(m_base*c1, m_base*c2, m_base*path[i].v);
                    }
                    break;
                default:
                    break;
                }
            }
        }

        /* Draw the boundary */
        std::for_each(
            m_tangDom.domain().popBegin(),
            m_tangDom.domain().popEnd(),
            [this, &scheme](size_t x) {
                scheme.moveTo(
                    m_base*Eigen::Vector3d(x+0.5, 0.0, 0.0));
                scheme.lineTo(
                    m_base*Eigen::Vector3d(x+0.5, 0.0, m_mvseqs.size()));
            } );
        std::for_each(
            m_tangDom.codomain().popBegin(),
            m_tangDom.codomain().popEnd(),
            [this, &scheme](size_t x) {
                scheme.moveTo(
                    m_base*Eigen::Vector3d(
                        x+0.5, m_tangDom.vlength(), 0.0));
                scheme.lineTo(
                    m_base*Eigen::Vector3d(
                        x+0.5, m_tangDom.vlength(), m_mvseqs.size()));
            } );

        /* Draw the codomain tangle */{
            AdapterScheme<PathScheme<Eigen::Vector3d>, Eigen::Vector2d> adpscheme(
                &scheme,
                [this, &orig](Eigen::Vector2d const&p) {
                    return m_base.block<3,2>(0,0)*p
                        + m_mvseqs.size()*m_base.col(2);
                } );

            PlTangFigure pltangfig{ m_tangCod,
                    Eigen::Vector2d(1.0, 0.0),
                    Eigen::Vector2d(0.0, 1.0) };

            pltangfig.draw(adpscheme);
            adpscheme.release();
        }
        scheme.restore();
    }

    /**!
     * \section Member functions derived from PathFigure3d
     **/
    virtual void updateProjector(Eigen::Matrix<double,2,3> const &proj) override
    {
        m_critV = (proj*m_base.block<3,2>(0,0)).row(0).adjoint();
        makePath();
    }

protected:
    template <class F>
    static Eigen::Vector3d modifZ(Eigen::Vector3d v, F&& f) {
        static_assert(
            std::is_convertible<
                decltype(std::declval<F&&>()(std::declval<double>())),
                double>::value,
            "The function type should be double(double)");

            v(2) = f(v(2));
        return v;
    }

    std::pair<bool, Eigen::Vector2d> getCrit(char c) const {
        Eigen::Vector2d v0, v1{Eigen::Vector2d(0.5,0.5)}, v2;

        // Parametrize the curves in counter-clockwise direction.
        switch(c) {
        case 'L':
            v0 = Eigen::Vector2d(0.5, 0.0);
            v2 = Eigen::Vector2d(1.0, 0.5);
            break;

        case 'J':
            v0 = Eigen::Vector2d(0.0, 0.5);
            v2 = Eigen::Vector2d(0.5, 0.0);
            break;

        case 'r':
            v0 = Eigen::Vector2d(1.0, 0.5);
            v2 = Eigen::Vector2d(0.5, 1.0);
            break;

        case '7':
            v0 = Eigen::Vector2d(0.5, 1.0);
            v2 = Eigen::Vector2d(0.0, 0.5);
            break;

        default:
            return std::make_pair(false, Eigen::Vector2d{0.0, 0.0});
        }

        double numer = m_critV.dot(v2 - v1);
        double denom = m_critV.dot(v0 - 2.0*v1 + v2);
        double t = numer / denom;

        if (0.0 < t && t <= 1.0)
            return std::make_pair(
                true,
                (t*t*v0 + 2*t*(1-t)*v1 + (1-t)*(1-t)*v2) );
        else
            return std::make_pair(false, Eigen::Vector2d());
    }

    //! Append the identity on the pathgraph.
    //! \param f The predicator which returns true on the cells where the identity are put.
    template <class F>
    inline void putIdentity(PlTang<R,C> const& tang, size_t level, PathGraph& graph, F const& f) const noexcept {
        tang.traverse(
            [&](size_t i, size_t j, char c) {
                if(!f(i,j))
                    return;
                auto crv = getCrit(c);
                if (!crv.first)
                    return;
                auto cellorig = Eigen::Vector3d(i,j,level);
                Eigen::Vector3d beg = Eigen::Matrix<double,3,2>::Identity()*crv.second + cellorig;
                auto end = beg + Eigen::Vector3d(0,0,1);
                graph.connect(
                    findAppend(graph, beg),
                    findAppend(graph, end) );
            } );
    }

    void makePath() {
        // Clear the old paths
        m_paths.clear();

        PlTang<R,C> tang_cur = m_tangDom;
        PathGraph graph;
        std::set<size_t> tmp_keys{};
        std::vector<size_t> levels(m_mvseqs.size(), 0);

        // Counter for the layers.
        size_t q = 0;

        // Traverse all the grouped moves
        for (size_t r = 0; r < m_mvseqs.size(); ++r) {
            auto commBeg = m_mvseqs[r].begin();
            auto commData = noncommHead(
                m_tangCod,
                commBeg,
                m_mvseqs[r].end());

            // Empty move yields the identity.
            if(m_mvseqs[r].empty()) {
                putIdentity(
                    tang_cur, q, graph,
                    [](size_t, size_t) { return true; });
                ++q;
                levels[r] = 1;
                continue;
            }

            // Non-empty move.
            while (commBeg != m_mvseqs[r].end()) {
                PlTangMove<2,2>::MoveElem mv = *commBeg;

                applyMove(tang_cur, mv);

                auto critmap = mv.move.mapVertexIf(
                    [&mv, this](size_t i, size_t j, char c) noexcept
                    -> std::pair<bool,Eigen::Vector2d> {
                        auto crv = getCrit(c);
                        return std::pair<bool,Eigen::Vector2d>(
                            crv.first,
                            crv.second + Eigen::Vector2d(mv.x+i, mv.y+j));
                    } );

                // Connect critical points following the connection graph.
                mv.move.getGraph().forEachEdge(
                    [&](size_t m, size_t n) {
                        auto itrm = critmap.find(m);
                        auto itrn = critmap.find(n);

                        if (itrm == critmap.end() && itrn == critmap.end())
                            return;

                        auto vm = itrm == critmap.end()
                            ? Eigen::Vector3d(mv.x + m%2 + 0.5,
                                              mv.y + m/2 + 0.5,
                                              m >= 4 ? q+levels[r]+1 : q+levels[r] )
                            : Eigen::Vector3d(itrm->second(0),
                                              itrm->second(1),
                                              m >= 4 ? q+levels[r]+1 : q+levels[r] );
                        auto vn = itrn == critmap.end()
                            ? Eigen::Vector3d(mv.x + n%2 + 0.5,
                                              mv.y + n/2 + 0.5,
                                              n >= 4 ? q+levels[r]+1 : q+levels[r] )
                            : Eigen::Vector3d(itrn->second(0),
                                              itrn->second(1),
                                n >= 4 ? q+levels[r]+1 : q+levels[r] );

                        auto keym = findAppend(graph, vm);
                        auto keyn = findAppend(graph, vn);

                        // Connect two vertices via a mediate vertex.
                        auto keymid = graph.append(
                            Eigen::Vector3d(mv.x+1.0, mv.y+1.0, q+levels[r]+0.5));
                        graph.connect(keym, keymid);
                        graph.connect(keyn, keymid);
                        tmp_keys.insert(keymid);
                    } );
                // Advance the cursor
                ++commBeg;

                // If the cursor reaches the non-commutative move;
                if(commBeg == commData.first) {
                    // Draw the identities.
                    putIdentity(
                        tang_cur, q+levels[r], graph,
                        [&commData, &tang_cur](size_t i, size_t j) -> bool {
                            return !commData.second[i+j*tang_cur.hlength()];
                        } );

                    // Advance the count of levels.
                    ++levels[r];
                    // Compute the next cluster of mutually commutative moves.
                    commData = noncommHead(m_tangDom,
                                           commBeg,
                                           m_mvseqs[r].end());
                }
            } // end of while(...)

            // Now the variable levels[r] equals the number of levels of the bordism constructed from the grouped moves.

            // Advance the counter of total levels.
            q += levels[r];

        } // end of traverse of the array m_mvseqs.

        // Hight modificator.
        auto normh = [&levels] (double hei) -> double {
            size_t r = 0, q = 0;
            while(hei > q+levels[r] && r < levels.size()) {
                q+=levels[r];
                ++r;
            }
            //* Debug
            if ((hei-q)/levels[r] + r >= levels.size()) {
                std::cout << __FILE__":" << __LINE__ << std::endl;
                std::cout << std::setprecision(16)
                          << hei << std::endl
                          << (hei-q)/levels[r] + r << std::endl;
            }
            // */

            return (hei-q)/levels[r] + r;
        };

        while(graph.inhabited()) {
            auto comp = graph.trimComponent(graph.minkey());
            auto mayend = comp.findEnd();
            auto maynode = comp.findKey(
                [end = tmp_keys.end(), &tmp_keys](std::pair<size_t, Eigen::Vector3d> const &v)
                -> bool
                {
                    return tmp_keys.find(v.first) == end;
                } );
            m_paths.emplace_back();

            double prevheight = 0.0;
            bool prev_is_node = true;
            auto key = mayend.first
                ? mayend.second
                : (maynode ? maynode->first : comp.minkey());

            auto flag = comp.trackPathCyc(
                key,
                [&](std::pair<size_t, Eigen::Vector3d> const& v) {
                    bool is_node = tmp_keys.find(v.first) == tmp_keys.end();
                    if(m_paths.back().empty()) {
                        m_paths.back().push_back(
                            {bord2::BeginPoint, modifZ(v.second, normh), 0.0});
                        prevheight = normh(v.second(2));
                    }
                    else if(is_node) {
                        double height = normh(v.second(2));
                        m_paths.back().push_back({
                            prev_is_node ? bord2::Line : bord2::Bezier,
                            modifZ(v.second, normh),
                            prevheight,
                            height < prevheight
                                     ? normh(v.second(2) + 2.0/3)
                                     : normh(v.second(2) - 2.0/3) });
                        prevheight = height;
                    }
                    else {
                        if (normh(v.second(2)) > prevheight)
                            prevheight = normh(v.second(2)+1.0/6);
                        else
                            prevheight = normh(v.second(2)-1.0/6);
                    }
                    prev_is_node = is_node;
                });
        }
    }
};

