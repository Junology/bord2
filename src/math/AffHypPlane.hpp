/*!
 * \file AffHypPlane.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 9, 2019: created
 */

#include <utility>
#include <array>
#include <Eigen/Dense>

template<size_t n>
class AffHypPlane
{
public:
    static constexpr size_t dim = n;
    static constexpr double threshold = 1e-10;
    using VecT = typename Eigen::Matrix<double,dim,1>;

private:
    //! The defining function of the line is
    //!   <m_normal|x> + m_c = 0;
    VecT m_normal;
    double m_c;

public:
    AffHypPlane(std::array<double,dim> const & normal, double c)
        : m_normal(), m_c(c)
    {
        // We can use initializer_list instead of array since Eigen 3.3.10.
        for(size_t i=0; i<dim; ++i)
            m_normal(i) = normal[i];
    }

    AffHypPlane(VecT const &normal, VecT const &refpt)
        : m_normal(normal), m_c(-normal.adjoint()*refpt)
    {}

    ~AffHypPlane() = default;

    //! Compute an intersection with another hyper plane.
    std::pair<bool,VecT> intersect(AffHypPlane<dim> const &another) const {
        Eigen::Matrix<double,2,dim> A;
        A.row(0) = m_normal;
        A.row(1) = another.m_normal;
        Eigen::Vector2d b{m_c, another.m_c};

        Eigen::Matrix<double,dim,1> x = A.colPivHouseholderQr().solve(-b);

        return {(A*x+b).norm()/(b.norm()+m_normal.norm()) < threshold, x};
    }

    template <class T>
    auto height(T const &v) const
        -> std::enable_if_t<std::is_same<T,VecT>::value,double>
    {
     return m_normal.adjoint()*v+m_c;
    }

    double height(std::array<double,dim> const &v) const
    {
        return height(v, std::make_index_sequence<dim>{});
    }

private:
    template<size_t... I>
    double height(std::array<double,dim> const &v, std::index_sequence<I...>) const
    {
        return height(VecT(v[I]...));
    }
};

template<size_t n>
constexpr double AffHypPlane<n>::threshold;
