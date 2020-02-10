/*!
 * \file ProjSpatialScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 12, 2019: created
 */

#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "PathScheme.hpp"
#include "utils.hpp"

/*! An implementation of PathScheme for 3-dimensional paths based on 2-dimensional one.
 * \tparam T A base implementaion of PathScheme for 2-dimensional paths. Ensure that it is an implementation of
 */
template <
    class T,
    class = decltype(typename T::vertex_type{std::declval<double>(),std::declval<double>()})
    >
class ProjSpatialScheme : public AdapterScheme<T,Eigen::Vector3d>
{
    template <class, class>
    friend class ProjSpatialScheme;

public:
    using vertex_type = Eigen::Vector3d;
    using typename AdapterScheme<T,Eigen::Vector3d>::BaseScheme;
    using BaseVertexType = typename BaseScheme::vertex_type;

private:
    Eigen::Matrix<double,2,3> m_mat_proj;
    Eigen::Vector3d m_focus;

    // State stack for save/restore
    std::vector<std::pair<Eigen::Matrix<double,2,3>,Eigen::Vector3d>> m_ststack{};

public:
    //! Construct from an instance of the base scheme
    ProjSpatialScheme(Eigen::Vector3d const& focus, BaseScheme *base)
        : AdapterScheme<BaseScheme,Eigen::Vector3d>{base, std::bind(&ProjSpatialScheme::project, this, std::placeholders::_1)}, m_focus(focus)
    {}

    //! Direct construction of an instance of the base scheme.
    template <class... Ts>
    ProjSpatialScheme(Eigen::Vector3d const & focus, Ts&&... args)
        : ProjSpatialScheme(focus, new BaseScheme(std::forward<Ts>(args)...))
    {}

    virtual ~ProjSpatialScheme() = default;

    template<class U, class... Args>
    ProjSpatialScheme<U> mimic(Args&&... args) const {
        auto scheme = ProjSpatialScheme<U>(m_focus, new U(std::forward<Args>(args)...));
        scheme.m_mat_proj = m_mat_proj;
        scheme.m_ststack = m_ststack;
        return scheme;
    }

    BaseVertexType project(Eigen::Vector3d const &p)
    {
        std::array<double,2> center = PathScheme<Eigen::Vector3d>::getCenter();
        Eigen::Vector2d result
            = Eigen::Vector2d(center[0], center[1]) + m_mat_proj * (p-m_focus);

        return BaseVertexType{result(0), result(1)};
    }

    void ortho(double elev, double azim) {
        double t = M_PI*elev/180.0;
        double u = M_PI*azim/180.0;

        m_mat_proj <<
            cos(u),       -sin(u), 0,
         // cos(t)*sin(u), cos(t)*cos(u), -sin(t),
            sin(t)*sin(u), sin(t)*cos(u), cos(t);
    }

    void cabinet(double angle, double depth_factor) {
        double t = M_PI*angle/180.0;

        m_mat_proj <<
            1.0, depth_factor*cos(t), 0.0,
            0.0, depth_factor*sin(t), 1.0;
    }

    //* Overriding methods.
    //** Scheme data query
    virtual void translate(vertex_type const &p) noexcept override {
        //auto p2d = project(p);
        //mp_base->translate(BaseVertexType{p2d(0), p2d(1)});
        m_focus -= p;
    }

    virtual void save() override {
        m_ststack.emplace_back(m_mat_proj, m_focus);
        AdapterScheme<BaseScheme,Eigen::Vector3d>::save();
    }

    virtual void restore() override {
        m_mat_proj = m_ststack.back().first;
        m_focus = m_ststack.back().second;
        AdapterScheme<BaseScheme,Eigen::Vector3d>::restore();
    }
};
