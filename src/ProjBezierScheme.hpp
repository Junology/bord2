/*!
 * \file ProjBezierScheme.hpp
 * \author Jun Yoshida
 * \copyright (c) 2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date February 23, 2020: created
 */

#pragma once

#include <vector>

#include <Eigen/Dense>

#include "math/Bezier.hpp"
#include "math/BezierBox.hpp"
#include "PathScheme.hpp"
#include "ProjSpatialScheme.hpp"

template <class T>
class ProjBezierScheme
    : public ProjSpatialScheme<T>
{
    static_assert(std::is_base_of<T, PathScheme<Eigen::Vector2d>>::value,
                  "The base scheme must be derived from PathScheme<Eigen::Vector2d>.");

public:
    using typename PathScheme<Eigen::Vector3d>::PathElement;
    using typename ProjSpatialScheme<T>::BaseScheme;
    using typename ProjSpatialScheme<T>::vertex_type;
    enum : size_t { max_degree = 3 };

protected:
    template <class>
    struct BezVar_impl;

    template <size_t...Is>
    struct BezVar_impl<std::index_sequence<Is...>> {
        using type = BezierVariant<Eigen::Vector3d,Is...>;
    };

public:
    using BezierVarType
        = typename BezVar_impl<std::make_index_sequence<max_degree+1>>::type;

private:
    BezierBox m_bezv;
    Eigen::Vector3d m_lastpt;

public:
    using ProjSpatialScheme<T>::ProjSpatialScheme;

    virtual ~ProjBezierScheme() = default;

    //! Submit the buffered paths to the base scheme.
    //! The intersections of Bezier curves are considered.
    void submit() noexcept
    {
        auto divided_bez
            = m_bezv.project(
                ProjSpatialScheme<Eigen::Vector2d>::getDepthVec());
        for(auto& bez : divided_bez) {
            std::array<vertex_type,3> ctrlpt;
            if (bez.getActive() == 0) {
                ctrlpt[0] = bez.source();
            }
            else {
                std::copy(bez.begin()+1, bez.end(), std::begin(ctrlpt));
            }

            switch(bez.getActive()) {
            case 0:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::BeginPoint, ctrlpt);
                break;

            case 1:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::Line, ctrlpt);
                break;

            case 2:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::QBezier, ctrlpt);
                break;

            case 3:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::Bezier, ctrlpt);
                break;

            default:
                // Error
                break;
            }
        }
    }

    //! Submit the buffered paths to the base scheme.
    //! This draws the paths directly via ProjSpatialScheme.
    void submit_direct() noexcept
    {
        for(auto& bez : m_bezv) {
            std::array<vertex_type,3> ctrlpt;
            if (bez.getActive() == 0) {
                ctrlpt[0] = bez.source();
            }
            else {
                std::copy(bez.begin()+1, bez.end(), std::begin(ctrlpt));
            }

            switch(bez.getActive()) {
            case 0:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::BeginPoint, ctrlpt);
                break;

            case 1:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::Line, ctrlpt);
                break;

            case 2:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::QBezier, ctrlpt);
                break;

            case 3:
                ProjSpatialScheme<T>::putPathElement(
                    bord2::PathElemType::Bezier, ctrlpt);
                break;

            default:
                // Error
                break;
            }
        }

        ProjSpatialScheme<T>::stroke();
    }

protected:
    void putPathElement(PathElement const& elem) override
    {
        switch(elem.type) {
        case bord2::PathElemType::BeginPoint:
            m_bezv.append(
                Bezier<Eigen::Vector3d,0>(elem.v[0]));
            m_lastpt = elem.v[0];
            break;

        case bord2::PathElemType::Line:
            m_bezv.append(
                Bezier<Eigen::Vector3d,1>(m_lastpt, elem.v[0]));
            m_lastpt = elem.v[0];
            break;

        case bord2::PathElemType::QBezier:
            m_bezv.append(
                Bezier<Eigen::Vector3d,1>(
                    m_lastpt, elem.v[0], elem.v[1]));
            m_lastpt = elem.v[1];
            break;

        case bord2::PathElemType::Bezier:
            m_bezv.append(
                Bezier<Eigen::Vector3d,1>(
                    m_lastpt, elem.v[0], elem.v[1], elem.v[2]));
            m_lastpt = elem.v[2];
            break;

        default:
            // Error
            break;
        }
    }
};
