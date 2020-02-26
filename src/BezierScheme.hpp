/*!
 * \file BezierBox.hpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Januray 16, 2020: created
 */

#pragma once

#include <vector>
#include <future>

#include <Eigen/Dense>

#include "math/Bezier.hpp"
#include "math/Bezier2D.hpp"
#include "PathScheme.hpp"

class BezierScheme : public PathScheme<Eigen::Vector3d>
{
public:
    using vertex_type = typename PathScheme<Eigen::Vector3d>::vertex_type;
    using BBoxT = typename PathScheme<Eigen::Vector3d>::BBoxT;
    using BezierVarType = BezierVariant<Eigen::Vector3d,0,1,2,3>;
    using BezierSequence = std::vector<BezierVarType>;
    enum : size_t { max_degree = 3 };

private:

    std::vector<BezierSequence> m_bezseqs{};
    vertex_type m_lastpt{0.0, 0.0, 0.0};
    vertex_type m_origin{0.0, 0.0, 0.0};
    std::vector<vertex_type> m_orig_stack{};

public:

    std::vector<BezierSequence> const& getRaw() const noexcept {
        return m_bezseqs;
    }

    //! Divide The sequences of Bezier curves so that their projections have no intersection in the plane.
    //! \param kernel A kernel vector of the projection. If two Bezier curves intersect with each other, only the lower curve is divided with respect to this vector.
    auto getProject(Eigen::Vector3d const& kernel) const
        -> std::future<std::vector<BezierSequence>>;

    /*!
     * \name Inherited virtual members.
     * They are mostly trivial.
     */
    //@{
    virtual BBoxT getBBox() const noexcept override {
        return {};
    }

    virtual bool isvalid() const noexcept override {
        return true;
    }

    virtual void setPen(double, bord2::PathColor, bord2::StrokePattern) noexcept override {}
    virtual void setBrush(bord2::PathColor) noexcept override {}

    virtual void translate(vertex_type const& p) noexcept override {
        m_origin += p;
    }

    virtual void save() noexcept override {
        m_orig_stack.push_back(m_origin);
    }

    virtual void restore() noexcept override {
        if (m_orig_stack.empty()) {
            m_origin = Eigen::Vector3d(0.0, 0.0, 0.0);
        }
        else {
            m_origin = m_orig_stack.back();
            m_orig_stack.pop_back();
        }
    }

    virtual void stroke() override {}
    virtual void strokePres() override {}
    virtual void fill() override {}
    virtual void fillPres() override {}

protected:
    virtual void putPathElement(typename PathScheme<vertex_type>::PathElement const& elem) override {
        switch(elem.type) {
        case bord2::PathElemType::BeginPoint:
            m_bezseqs.emplace_back();
            m_lastpt = m_origin + elem.v[0];
            break;

        case bord2::PathElemType::Line:
            if(!m_bezseqs.empty()) {
                m_bezseqs.back().emplace_back(
                    std::integral_constant<size_t,1>(),
                    m_lastpt,
                    m_origin + elem.v[0] );
            }
            m_lastpt = m_origin + elem.v[0];
            break;

        case bord2::PathElemType::QBezier:
            if(!m_bezseqs.empty()) {
                m_bezseqs.back().emplace_back(
                    std::integral_constant<size_t,2>(),
                    m_lastpt,
                    m_origin + elem.v[0],
                    m_origin + elem.v[1] );
            }
            m_lastpt = m_origin + elem.v[1];
            break;

        case bord2::PathElemType::Bezier:
            if(!m_bezseqs.empty()) {
                m_bezseqs.back().emplace_back(
                    std::integral_constant<size_t,3>(),
                    m_lastpt,
                    m_origin + elem.v[0],
                    m_origin + elem.v[1],
                    m_origin + elem.v[2] );
            }
            m_lastpt = m_origin + elem.v[2];
            break;

        case bord2::PathElemType::Closing:
            if(!m_bezseqs.empty() && !m_bezseqs.back().empty()) {
                m_bezseqs.back().emplace_back(
                    std::integral_constant<size_t,1>(),
                    m_bezseqs.back().back().target(),
                    m_bezseqs.back().front().source() );
                m_lastpt = m_bezseqs.back().front().source();
            }
            break;

        default:
            // Error
            break;
        }
    }
    //@}
};

template <class V>
void drawBezierSequence(
    std::vector<BezierVariant<V,0,1,2,3>> const& bezseq,
    PathScheme<V> &scheme)
{
    V lastpt{};

    for(auto& bezvar : bezseq) {
        switch(bezvar.getActive()) {
        case 0:
            scheme.moveTo(bezvar.source());
            break;

        case 1:
            if(lastpt != bezvar.source())
                scheme.moveTo(bezvar.source());
            scheme.lineTo(bezvar.target());
            break;

        case 2:
            if(lastpt != bezvar.source())
                scheme.moveTo(bezvar.source());
            scheme.qbezierTo(
                bezvar.get(
                    std::integral_constant<size_t,2>()
                    ).get(std::integral_constant<size_t,1>()),
                bezvar.get(
                    std::integral_constant<size_t,2>()
                    ).get(std::integral_constant<size_t,2>())
                );
            break;

        case 3:
            if(lastpt != bezvar.source())
                scheme.moveTo(bezvar.source());
            scheme.bezierTo(
                bezvar.get(
                    std::integral_constant<size_t,3>()
                    ).get(std::integral_constant<size_t,1>()),
                bezvar.get(
                    std::integral_constant<size_t,3>()
                    ).get(std::integral_constant<size_t,2>()),
                bezvar.get(
                    std::integral_constant<size_t,3>()
                    ).get(std::integral_constant<size_t,3>())
                );
            break;

        default:
            /** NEVER REACHED **/
            break;
        }
        lastpt = bezvar.target();
    }
}
