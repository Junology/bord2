/*!
 * \file OrthoSpatialScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 12, 2019: created
 */

#pragma once

#include <memory>
#include <Eigen/Dense>

#include "PathScheme.hpp"
#include "utils.hpp"

/*! An implementation of PathScheme for 3-dimensional paths based on 2-dimensional one.
 * \tparam T A base implementaion of PathScheme for 2-dimensional paths. Ensure that it is an implementation of
 */
template <
    class T,
    class V = std::enable_if_t<
        bord2::is_pubbase_of_template<PathScheme,T>::value,
        typename T::vertex_type
        >,
    class = decltype(V{std::declval<double>(),std::declval<double>()})
    >
class OrthoSpatialScheme : public PathScheme<Eigen::Vector3d>
{
public:
    using BaseScheme = T;
    using BaseVertexType = V;

private:
    std::unique_ptr<BaseScheme> mp_base;
    Eigen::Matrix<double,2,3> m_mat_proj;
    Eigen::Vector3d m_focus;

public:
    //! Construct from an instance of the base scheme
    OrthoSpatialScheme(Eigen::Vector3d const& focus, double elev, double azim, BaseScheme *base)
        : PathScheme<Eigen::Vector3d>{}, mp_base(base), m_focus(focus)
    {
        double t = M_PI*elev/180.0;
        double u = M_PI*azim/180.0;

        m_mat_proj <<
            cos(u),       -sin(u), 0,
         // cos(t)*sin(u), cos(t)*cos(u), -sin(t),
            sin(t)*sin(u), sin(t)*cos(u), cos(t);
    }

    //! Direct construction of an instance of the base scheme.
    template <class... Ts>
    OrthoSpatialScheme(Eigen::Vector3d const & focus, double elev, double azim, Ts&&... args)
        : OrthoSpatialScheme(focus, elev, azim, new BaseScheme(std::forward<Ts>(args)...))
    {}

    virtual ~OrthoSpatialScheme() = default;

    //! Release the responsibility for deleting the pointer of a base scheme.
    //! \return The pointer to a base scheme, which have to be deleted by hand.
    BaseScheme* release() {
        auto ret = mp_base.get();
        if (mp_base) {
            mp_base->release();
            mp_base.reset(nullptr);
        }
        return ret;
    }

    Eigen::Vector2d project(Eigen::Vector3d const &p)
    {
        std::array<double,2> center = PathScheme<Eigen::Vector3d>::getCenter();
        Eigen::Vector2d orig2d(center[0], center[1]);

        return m_mat_proj * (p-m_focus) + orig2d;
    }

    //* Overriding methods.
    //** Scheme data query
    virtual BBoxT getBBox() const override {
        return mp_base->getBBox();
    }

    virtual bool isvalid() const noexcept {
        return mp_base && mp_base->isvalid();
    }

    virtual void translate(vertex_type const &p) noexcept {
        auto p2d = project(p);
        mp_base->translate(BaseVertexType{p2d(0), p2d(1)});
    }

    //* Pens and Brushes
    virtual void setPen(double wid, bord2::PathColor col = bord2::Black, bord2::StrokePattern pat = bord2::Solid) override {
        mp_base->setPen(wid, col, pat);
    }

    virtual void setBrush(bord2::PathColor col) override {
        mp_base->setBrush(col);
    }

    //* Drawing paths.
    //! Stroke and flush.
    virtual void stroke() override {
        mp_base->stroke();
    }

    //! Stroke without flush
    virtual void strokePres() override {
        mp_base->stroke();
    }

    //! Fill and flush.
    virtual void fill() override {
        mp_base->fill();
    }

    //! Fill without flush
    virtual void fillPres() override {
        mp_base->fillPres();
    }

    //* Path elements.
    //! Move the current position.
    virtual void moveTo(vertex_type const &p) override {
        auto p2d = project(p);
        mp_base->moveTo(BaseVertexType{p2d(0), p2d(1)});
    }

    //! Move the current position drawing a line from the old.
    virtual void lineTo(vertex_type const &p) override {
        auto p2d = project(p);
        mp_base->lineTo(BaseVertexType{p2d(0), p2d(1)});
    }

    //! Move the current position drawing a cubic Bezier curve.
    virtual void bezierTo(vertex_type const &c1, vertex_type const &c2, vertex_type const &p) override {
        auto c12d = project(c1);
        auto c22d = project(c2);
        auto p2d = project(p);
        mp_base->bezierTo(
            BaseVertexType{c12d(0), c12d(1)},
            BaseVertexType{c22d(0), c22d(1)},
            BaseVertexType{p2d(0), p2d(1)}
            );
    }

    //! Move the current position drawing a quadratic Bezier curve.
    virtual void qbezierTo(vertex_type const &c, vertex_type const &p) override {
        auto c2d = project(c);
        auto p2d = project(p);
        mp_base->qbezierTo(
            BaseVertexType{c2d(0), c2d(1)},
            BaseVertexType{p2d(0), p2d(1)}
            );
    }

    //! Close path.
    virtual void closePath() override {
        mp_base->closePath();
    }
};
