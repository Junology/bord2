/*!
 * \file Bezier.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 12, 2019: created
 */

#pragma once

#include <utility>
#include <type_traits>
#include <array>

#include "../utils.hpp"
#include "../utils/VarUnion.hpp"

//#include <iostream> // Debug

/*!
 * Basic traits for Bezier curve types
 */
template <class T>
struct BezierTraits {
    // The type of control points.
    using vertex_type = typename T::vertex_type;

    template <class V>
    using converted_type = decltype(std::declval<T>().convert(std::declval<V(vertex_type)>()));
};

/*! The class for Bezier curves.
 * \tparam T The type of vertices; e.g. Eigen::Vector2d, Eigen::Vector3d. It must be possible to
 * - add two vertices;
 * - multiply vertices by floating points, i.e. double.
 * \tparam n The degree of the Bezier curve. Hence, the number of control points would be n+1.
 */
template <class T, size_t n>
class Bezier
{
public:
    //! The type of control points.
    using vertex_type = T;
    enum : size_t {
        //! The degree of the Bezier curve.
        degree = n,
        //! The number of control points.
        num_pts = n+1
    };

protected:
    //std::array<vertex_type, num_pts> m_pts;
    vertex_type m_pts[num_pts];

public:
    template <
      class U,
      class... Ts,
      std::enable_if_t<
          std::is_constructible<vertex_type, U&&>::value
          && !std::is_same<Bezier<T,n>,std::decay_t<U> >::value,
          bool
          > = true
    >
    constexpr Bezier(U && pt0, Ts &&... pts) noexcept(std::is_nothrow_constructible<vertex_type, U&&>::value && bord2::allTrue({std::is_nothrow_constructible<vertex_type, Ts&&>::value...}))
      : m_pts{std::forward<U>(pt0), std::forward<Ts>(pts)...}
    {}

    Bezier() noexcept(std::is_nothrow_default_constructible<vertex_type>::value) = default;
    constexpr Bezier(Bezier const &) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value) = default;
    constexpr Bezier(Bezier &&) noexcept(std::is_nothrow_move_constructible<vertex_type>::value) = default;

    constexpr Bezier& operator=(Bezier const&) noexcept(std::is_nothrow_copy_assignable<vertex_type>::value) = default;
    constexpr Bezier& operator=(Bezier &&) noexcept(std::is_nothrow_move_assignable<vertex_type>::value) = default;

    //! Get a control point.
    template<size_t i>
    constexpr vertex_type const& get() const noexcept {
        static_assert(i<num_pts, "Index ouf of range.");
        return m_pts[i];
    }

    constexpr vertex_type const& source() const noexcept {
        return m_pts[0];
    }

    constexpr vertex_type const& target() const noexcept {
        return m_pts[degree];
    }

    /*! \name Range access interface */
    //@{
    constexpr vertex_type* begin() noexcept { return std::begin(m_pts); }
    constexpr vertex_type const* begin() const noexcept { return std::begin(m_pts); }
    constexpr vertex_type* end() noexcept { return std::end(m_pts); }
    constexpr vertex_type const* end() const noexcept { return std::end(m_pts); }
    constexpr vertex_type const* cbegin() const noexcept { return std::cbegin(m_pts); }
    constexpr vertex_type const* cend() const noexcept { return std::cend(m_pts); }
    //@}

    //! Evaluate the point at parameter t.
    constexpr vertex_type eval(double t) const noexcept {
        return eval_part<0,degree>(t);
    }

    //! Check if all the control points satisfy a given condition
    template <class F>
    constexpr bool allSatisfy(F const& f) const noexcept {
        static_assert(
            std::is_convertible<
                decltype(f(std::declval<vertex_type const&>())),
                bool
            >::value, "The condition has an illegal signature.");
        for(size_t i = 0; i < num_pts; ++i) {
            if (!f(m_pts[i])) return false;
        }
        return true;
    }

    //! Divide the Bezier curve using De Casteljau's algorithm.
    constexpr auto divide(double t = 0.5, double eps = 0.0) const noexcept
        -> std::pair< Bezier<vertex_type,degree>,Bezier<vertex_type,degree> >
    {
        return divide_impl(std::make_index_sequence<num_pts>(), t, eps);
    }

    //! Clip the Bezier curve to the one with the parameter on a given interval.
    //! \warning May cause 0 division when t0 = t1.
    constexpr auto clip(double t0, double t1) const noexcept
        -> Bezier<vertex_type, degree>
    {
        return clip_impl(std::make_index_sequence<num_pts>(), t0, t1);
    }

    //! Convert into Bezier curve of the same degree while with another vertex_type.
    template <class F>
    constexpr auto convert(F&& f) const
        -> Bezier<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),degree>
    {
        return convert_impl(std::make_index_sequence<num_pts>(), std::forward<F>(f));
    }

protected:
    template <size_t MIN, size_t MAX, size_t DEG=MAX-MIN>
    constexpr vertex_type eval_part(double t) const noexcept {
        static_assert(MAX<=degree && MIN <= MAX, "Index out of range");

        constexpr auto binomArr = bord2::binom<>::getArray<double,DEG>();
        vertex_type result = bord2::cipow(1.0-t,DEG)* m_pts[MIN];
        for(size_t i = 1; i <= DEG; ++i)
            result = std::move(result)
                + bord2::cipow(1.0-t,DEG-i)*bord2::cipow(t,i)*binomArr[i]*m_pts[MIN+i];
        return result;
    }

    template<size_t... Is>
    constexpr auto divide_impl(std::index_sequence<Is...>, double t, double eps) const noexcept
        -> std::pair< Bezier<vertex_type,degree>,Bezier<vertex_type,degree> >
    {
        static_assert(sizeof...(Is)==num_pts, "Wrong number of arguments");

        return std::make_pair(
            Bezier<vertex_type,degree>(eval_part<0,Is>(t+eps)...),
            Bezier<vertex_type,degree>(eval_part<Is,degree>(t-eps)...) );
    }

    template<size_t... Is>
    constexpr auto clip_impl(std::index_sequence<Is...>, double t0, double t1) const noexcept {
        auto aux = Bezier<vertex_type, degree>{eval_part<0,Is>(t1)...};
        double t = t0/t1;
        return Bezier<vertex_type, degree>(
            aux.template eval_part<Is,degree>(t)...
            );
    }

    template <class F, size_t... Is>
    constexpr auto convert_impl(std::index_sequence<Is...>, F&& f) const
        -> Bezier<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),degree>
    {
        return Bezier<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),degree>{
            f(m_pts[Is])...
        };
    }
};

/*!
 * Bezier curve of varying degrees.
 */
template<class T, size_t N, size_t...Ns>
class BezierVariant
{
public:
    using vertex_type = T;
    enum : size_t { var_num = 1+sizeof...(Ns) };

private:
    VarUnion<Bezier<T,N>,Bezier<T,Ns>...> m_unibez;
    size_t m_actid;

    template <size_t n>
    using bezier_type = typename VarUnion<Bezier<T,N>,Bezier<T,Ns>...>::template elem_type<n>;

public:
    //! Constructor with tag.
    template <size_t n, class...Args>
    constexpr BezierVariant(std::integral_constant<size_t,n>, Args&&...args) noexcept
        : m_unibez(std::integral_constant<size_t,n>(), std::forward<Args>(args)...), m_actid(n) {}

    //! Copy constructor.
    constexpr BezierVariant(BezierVariant const& src) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value)
        : BezierVariant(std::is_trivially_copy_constructible<vertex_type>(), src)
    {}

    //! Copy-construct using union's default copy constructor.
    template <class Tag, std::enable_if_t<std::is_base_of<std::true_type,Tag>::value, int> = 0>
    constexpr BezierVariant(Tag, BezierVariant const& src) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value)
        : m_unibez(src.m_unibez), m_actid(src.m_actid)
    {}

    //! Copy-construct using dynamic copy constructor of VarUnion class.
    template <class Tag, std::enable_if_t<std::is_base_of<std::false_type,Tag>::value, int> = 0>
    BezierVariant(Tag, BezierVariant const& src) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value)
        : m_unibez(src.m_actid, src.m_unibez), m_actid(src.m_actid)
    {}

    //! Move constructor.
    constexpr BezierVariant(BezierVariant && src) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value)
        : BezierVariant(std::is_trivially_copy_constructible<vertex_type>(), std::move(src))
    {}

    //! Move-construct using union's default move constructor.
    template <class Tag, std::enable_if_t<std::is_base_of<std::true_type,Tag>::value, int> = 0>
    constexpr BezierVariant(Tag, BezierVariant && src) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value)
        : m_unibez(std::move(src.m_unibez)), m_actid(src.m_actid)
    {}

    //! Move-construct using dynamic move constructor of VarUnion class.
    template <class Tag, std::enable_if_t<std::is_base_of<std::false_type,Tag>::value, int> = 0>
    BezierVariant(Tag, BezierVariant && src) noexcept(std::is_nothrow_copy_constructible<vertex_type>::value)
        : m_unibez(src.m_actid, std::move(src.m_unibez)), m_actid(src.m_actid)
    {}

    //! Copy-assignment operator
    constexpr BezierVariant& operator=(BezierVariant const& src) noexcept(std::is_nothrow_copy_assignable<vertex_type>::value) {
        copyAssign_impl(std::is_trivially_copy_assignable<vertex_type>(), src);
        return *this;
    }

    //! Move assignment operator
    constexpr BezierVariant& operator=(BezierVariant&& src) noexcept(std::is_nothrow_move_assignable<vertex_type>::value) {
        moveAssign_impl(std::is_trivially_move_assignable<vertex_type>(), std::move(src));
        return *this;
    }

    //! Get the index of the current active member
    constexpr size_t getActive() const noexcept {
        return m_actid;
    }

    //! Get the underlying Bezier curve.
    //! \tparam n The index, **not necessarily the degree**, of the type of the current active Bezier curve.
    template <size_t n>
    auto const& get(std::integral_constant<size_t,n> = {}) const noexcept(false)
    {
        if(m_actid != n)
            throw(std::out_of_range("Index of an inactive member."));
        return m_unibez.get(std::integral_constant<size_t,n>());
    }

    //! Assignment
    template <size_t n>
    constexpr void assign(std::integral_constant<size_t,n> &&ic, bezier_type<n> const& src) noexcept {
        m_unibez.assign(std::move(ic), src);
        m_actid = n;
    }

    //! Access to control points is available only for the ends.
    constexpr vertex_type const& source() const noexcept {
        return source_impl<0>();
    }

    constexpr vertex_type const& target() const noexcept {
        return target_impl<0>();
    }

    /*! \name Range access interface */
    //@{
    constexpr vertex_type* begin() noexcept {
        return begin_impl<0>();
    }

    constexpr vertex_type const* begin() const noexcept {
        return const_cast<BezierVariant*>(this)->begin();
    }

    constexpr vertex_type* end() noexcept {
        return end_impl<0>();
    }

    constexpr vertex_type const* end() const noexcept {
        return const_cast<BezierVariant*>(this)->end();
    }

    constexpr vertex_type const* cbegin() noexcept {
        return const_cast<BezierVariant*>(this)->begin();
    }
    constexpr vertex_type const* cend() noexcept {
        return const_cast<BezierVariant*>(this)->end();
    }
    //@}

    /*! \name Bezier interface */
    //@{
    //! Evaluate the point at a given parameter.
    constexpr vertex_type eval(double t) const noexcept {
        return eval_impl<0>(t);
    }

    //! Check a given condition on all the vertices.
    template <class F>
    constexpr bool allSatisfy(F&& f) const noexcept {
        return allSatisfy_impl<0>(std::forward<F>(f));
    }

    //! Divide the Bezier curve.
    constexpr auto divide(double t = 0.5, double eps = 0.0) const noexcept
        -> std::pair<BezierVariant,BezierVariant>
    {
        return divide_impl<0>(t, eps);
    }

    //! Clip the Bezier curve.
    constexpr BezierVariant clip(double t0, double t1) const noexcept {
        return clip_impl<0>(t0, t1);
    }

    //! Conversion of vertices.
    template <class F>
    constexpr auto convert(F&& f) const
        -> BezierVariant<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),N,Ns...>
    {
        return convert_impl<0>(std::forward<F>(f));
    }
    //@}

protected:
    /*! \name Implementations */
    //@{
    template <class Tag, std::enable_if_t<std::is_base_of<std::true_type,Tag>::value, int> = 0>
    constexpr void copyAssign_impl(Tag, BezierVariant const& src) noexcept(std::is_nothrow_copy_assignable<vertex_type>::value)
    {
        m_unibez = src.m_unibez;
        m_actid = src.m_actid;
    }

    template <class Tag, std::enable_if_t<std::is_base_of<std::false_type,Tag>::value, int> = 0>
    void copyAssign_impl(Tag, BezierVariant const& src) noexcept(std::is_nothrow_copy_assignable<vertex_type>::value)
    {
        m_unibez.assign_dynamic(src.m_actid, src.m_unibez);
        m_actid = src.m_actid;
    }

    template <class Tag, std::enable_if_t<std::is_base_of<std::true_type,Tag>::value, int> = 0>
    constexpr void moveAssign_impl(Tag, BezierVariant && src) noexcept(std::is_nothrow_move_assignable<vertex_type>::value)
    {
        m_unibez = src.m_unibez;
        m_actid = src.m_actid;
    }

    template <class Tag, std::enable_if_t<std::is_base_of<std::false_type,Tag>::value, int> = 0>
    void moveAssign_impl(Tag, BezierVariant && src) noexcept(std::is_nothrow_move_assignable<vertex_type>::value)
    {
        m_unibez.assign_dynamic(src.m_actid, std::move(src.m_unibez));
        m_actid = src.m_actid;
    }

    template <size_t n, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr vertex_type const& source_impl() const noexcept
    {
        if (m_actid == n)
            return (m_unibez.template get<n>()).source();
        else
            return source_impl<n+1>();
    }

    template <size_t n, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr vertex_type const& source_impl() const noexcept
    {
        return (m_unibez.template get<n>()).source();
    }

    template <size_t n, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr vertex_type const& target_impl() const noexcept
    {
        if (m_actid == n)
            return (m_unibez.template get<n>()).target();

        return target_impl<n+1>();
    }

    template <size_t n, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr vertex_type const& target_impl() const noexcept
    {
        return (m_unibez.template get<n>()).target();
    }

    template <size_t n, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr vertex_type* begin_impl() noexcept
    {
        if (m_actid == n)
            return (m_unibez.template get<n>()).begin();

        return begin_impl<n+1>();
    }

    template <size_t n, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr vertex_type* begin_impl() noexcept {
        return (m_unibez.template get<n>()).begin();
    }

    template <size_t n, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr vertex_type* end_impl() noexcept
    {
        if (m_actid == n)
            return (m_unibez.template get<n>()).end();

        return end_impl<n+1>();
    }

    template <size_t n, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr vertex_type* end_impl() noexcept
    {
        return (m_unibez.template get<n>()).end();
    }

    template <size_t n, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr vertex_type eval_impl(double t) const noexcept {
        if (m_actid == n)
            return (m_unibez.template get<n>()).eval(t);

        return eval_impl<n+1>(t);
    }

    template <size_t n, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr vertex_type eval_impl (double t) const noexcept {
        return (m_unibez.template get<n>()).eval(t);
    }

    template <size_t n, class F, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr bool allSatisfy_impl(F&& f) const noexcept {
        if (m_actid == n)
            return (m_unibez.template get<n>()).allSatisfy(std::forward<F>(f));

        return allSatisfy_impl<n+1>(std::forward<F>(f) );
    }

    template <size_t n, class F, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr bool allSatisfy_impl(F&& f) const noexcept {
        return (m_unibez.template get<n>()).allSatisfy(std::forward<F>(f));
    }

    template <size_t n, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr auto divide_impl(double t, double eps) const noexcept
        -> std::pair<BezierVariant,BezierVariant>
    {
        if (m_actid == n) {
            auto result = (m_unibez.template get<n>()).divide(t, eps);
            return std::make_pair(
                BezierVariant(std::integral_constant<size_t,n>(),
                              result.first ),
                BezierVariant(std::integral_constant<size_t,n>(),
                              result.second )
                );
        }

        return divide_impl<n+1>(t, eps);
    }

    template <size_t n, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr auto divide_impl(double t, double eps) const noexcept
        -> std::pair<BezierVariant,BezierVariant>
    {
        auto result = (m_unibez.template get<n>()).divide(t, eps);
        return std::make_pair(
            BezierVariant(std::integral_constant<size_t,n>(),
                          result.first ),
            BezierVariant(std::integral_constant<size_t,n>(),
                          result.second )
            );
    }

    template <size_t n, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr auto clip_impl(double t0, double t1) const noexcept
        -> BezierVariant
    {
        if (m_actid == n) {
            return BezierVariant(
                std::integral_constant<size_t,n>(),
                (m_unibez.template get<n>()).clip(t0, t1) );
        }

        return clip_impl<n+1>(t0, t1);
    }

    template <size_t n, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr auto clip_impl(double t0, double t1) const noexcept
        -> BezierVariant
    {
        return BezierVariant(
            std::integral_constant<size_t,n>(),
            (m_unibez.template get<n>()).clip(t0, t1) );
    }

    template <size_t n, class F, std::enable_if_t<(n+1<var_num),int> = 0>
    constexpr auto convert_impl(F&& f) const
        -> BezierVariant<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),N,Ns...>
    {
        using target_VT = decltype(std::declval<F&&>()(std::declval<vertex_type&&>()));
        if (m_actid == n) {
            return BezierVariant<target_VT,N,Ns...>(
                std::integral_constant<size_t,n>(),
                (m_unibez.template get<n>()).convert(std::forward<F>(f)));
        }

        return convert_impl<n+1>(std::forward<F>(f) );
    }

    template <size_t n, class F, std::enable_if_t<(n+1==var_num),int> = 0>
    constexpr auto convert_impl(F&& f) const
        -> BezierVariant<decltype(std::declval<F&&>()(std::declval<vertex_type&&>())),N,Ns...>
    {
        using target_VT = decltype(std::declval<F&&>()(std::declval<vertex_type&&>()));
        return BezierVariant<target_VT,N,Ns...>(
            std::integral_constant<size_t,n>(),
            (m_unibez.template get<n>()).convert(std::forward<F>(f)));
    }
};
