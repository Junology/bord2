/**
 * \file PlTang.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date Descember 29, 2019: created
 */

#pragma once

#include <utility>
//#include <bitset>
#include <tuple>
#include <array>
#include <string>

#include "utils.hpp"
#include "utils/BitArray.hpp"
#include "utils/StrLnView.hpp"

// #include <iostream> // Debug

//! The class representing a planar tangle; i.e. a 1-dimensional manifold properly embedded in the Euclidean plane.
/*!
 * Recall that a planar tangle decomposes into the following elementary parts:
 *   - a vertical strand;
 *   - a cup, or the graph of y=x^2;
 *   - a cap, or the graph of y=-x^2.
 * We say that a planar tangle is *semi-elementary* if it is a horizontal disjoint sum of elementary tangles.
 * Generic planar tangles, which form an open dense subset in the moduli, are vertical compositions of semi-elementary planar tangles.
 *
 * The class represents a planar tangle as an array of semi-elementary planar tangles.
 * In the internal class, namely *SemiElem*, we use two types of representations for semi-elementary ones.
 * The first one is "AA (ascii-art) representation"; i.e. a string consisting of the following characters:
 *   - '|' for a vertical strand;
 *   - '-' for a horizontal strand;
 *   - 'L' for the left end of a cup;
 *   - 'J' for the right end of a cup;
 *   - 'r' for the left end of a cap;
 *   - '7' for the right end of a cap;
 *   - a white space for the blank.
 * For example, the triangle identity for an adjunction is represented as follows:
 *   > r7|
 *   > |LJ == |
 * The second is a binary representation.
 * Note that a semi-elementary planar tangle is determined by its domain and codomain if we remember the *metric*.
 * Indeed, we represent a 0-dimensional submanifold in the real line as a sequence of 0 (absence) and 1 (presence).
 * An AA-representation is hence encoded into a pair of binary numbers which represents the domain and the codomain respectively.
 * More explicitly,
 *   > '|' -> (1,1);
 *   > ' ', '-' -> (0,0);
 *   > 'L', 'J' -> (0,1);
 *   > 'r', '7' -> (1,0)
 * The binary representation of a whole semi-elementary tangle is obtained by component-wise concatenation.
 */
template<size_t MR = sizeof(unsigned int)*8, size_t MC = sizeof(unsigned int)*8>
class PlTang
{
    template<size_t R, size_t C>
    friend class PlTang;

public:
    enum : size_t {
        max_rows = MR,
        max_cols = MC
     };

    using binrep_type = BitArray<max_cols>;//std::bitset<max_cols>;
    using length_type = std::size_t;

protected:
    //! A sequence of binary representations of 0-manifolds.
    //! Each consecutive pair represents a semi-elementary-planar tangle.
    binrep_type m_bincob[max_rows+1];
    //! The horizontal size of the planar tangle.
    size_t m_hlength;
    //! The vertical size of the planar tangle.
    size_t m_vlength;
    //! The flag indicating the representation is valid..
    bool m_isvalid;

public:
    /** Special member functions **/
    //! Construct an invalid representation
    constexpr PlTang() noexcept
      : m_bincob{}, m_hlength{0}, m_vlength{0}, m_isvalid(false)
    {}

    //! Construct from an AA-representation.
    template <
        class T,
        std::enable_if_t<
            std::is_same<std::decay_t<T>, char const*>::value,
            int
            > = 0
        >
    constexpr PlTang(T&& aarep) noexcept
      : m_bincob{},
        m_hlength{0},
        m_vlength{0},
        m_isvalid{true}
    {
        StrLnView aaview(aarep);//{/*std::forward<T>*/(aarep)};

        if (!aaview) {
            m_isvalid = false;
            return;
        }

        bord2::tupbind(encode(aaview.c_str(), aaview.remainLn()), m_isvalid, m_bincob[1], m_bincob[0]);

        if(!m_isvalid) {
            invalidate();
            return;
        }
        m_hlength = aaview.remainLn();
        m_vlength = 1;

        while(aaview.advLn()) {
            binrep_type dom{}, cod{};

            // empty line is ignored
            if (aaview.remainLn() == 0)
                continue;

            bord2::tupbind(encode(aaview.c_str(), aaview.remainLn()), m_isvalid, dom, cod);

            // Encoding failed.
            if (!m_isvalid) {
                invalidate();
                return;
            }

            // Composability check
            if (cod != m_bincob[m_vlength] || aaview.remainLn() != m_hlength) {
                invalidate();
                return;
            }

            // Advance the vertical length.
            ++m_vlength;
            if (m_vlength <= max_rows) {
                m_bincob[m_vlength] = dom;
            }
            // The capacity runs out.
            else {
                invalidate();
                return;
            }
        }
    }

    //! Default copy constructor.
    constexpr PlTang(PlTang<MR,MC> const &) noexcept = default;
    //! Default move constructor.
    constexpr PlTang(PlTang<MR,MC> &&) noexcept = default;

    //! Default virtual destructor.
    ~PlTang() = default;

    //! Copy assignment
    constexpr PlTang<MR,MC>& operator=(PlTang<MR,MC> const &) noexcept = default;

    //! Move assignment
    constexpr PlTang<MR,MC>& operator=(PlTang<MR,MC> &&) noexcept = default;

    //! Equality
    constexpr bool operator==(PlTang<MR,MC> const& other) const noexcept
    {
        // If both are invalid, they are equal.
        if(!isvalid() && !other.isvalid())
            return true;

        // Lengths must equal.
        if (m_vlength != other.m_vlength || m_hlength != other.m_hlength)
            return false;

        // Mask which seals bits outside of the tangle presentations.
        constexpr auto alltrue = ~(BitArray<max_cols>{});
        auto mask = alltrue.lowpass(m_hlength);

        for(size_t i = 0; i <= m_vlength; ++i) {
            if ((m_bincob[i] & mask) != (other.m_bincob[i] & mask))
                return false;
        }

        return true;
    }

    //! Inequality
    constexpr bool operator!=(PlTang<MR,MC> const& other) const noexcept
    {
        return !operator==(other);
    }

    /** Getters **/
    constexpr size_t vlength() const noexcept {
        return m_vlength;
    }

    constexpr size_t hlength() const noexcept {
        return m_hlength;
    }

    //! Check if the instance has a valid AA-representation of a planar tangle.
    constexpr bool isvalid() const noexcept {
        return m_vlength > 0;
    }

    //! Get the domain in its binary representation.
    constexpr binrep_type domain() const noexcept {
        return m_bincob[m_vlength];
    }

    //! Get the codomain in its binary representation.
    constexpr binrep_type codomain() const noexcept {
        return m_bincob[0];
    }

    //! Slice the planar tangle.
    //! \tparam R The number of rows of the slice.
    //! \tparam C The number of columns of the slice.
    //! \param left The index of the left-most cell to be sliced.
    //! \param top The index of the top-most cell to be sliced.
    //! \param wid The number of cells in a horizontal direction.
    //! \param hei The number of cells in a vertical direction.
    //! \return The slice of the planar tangle, which is invalid if the result is not a planar tangle, or if indices or sizes are invalid.
    template <size_t R = max_rows, size_t C = max_cols>
    constexpr PlTang<R,C> slice(size_t left, size_t top, size_t wid = C, size_t hei = R) const noexcept
    {
        // Range check.
        if (wid > C || hei > R || wid == 0 || hei == 0 || left+wid > m_hlength || top+hei > m_vlength)
            return PlTang<R,C>();

        PlTang<R,C> result;
        result.m_hlength = wid;
        result.m_vlength = hei;

        // Copy the slice of the codomain.
        result.m_bincob[0] = m_bincob[top].template slice<C>(left).lowpass(wid);
        size_t parityL
            = m_bincob[top].lowpass(left).popCount() & 0x1;
        size_t parityR
            = m_bincob[top].lowcut(left+wid).popCount() & 0x1;

        for(size_t j = 1; j <= hei; ++j) {
            size_t parityLD
                = m_bincob[top+j].lowpass(left).popCount() & 0x1;
            size_t parityRD
                = m_bincob[top+j].lowcut(left+wid).popCount() & 0x1;

            // It turns out that the slice is not a tangle.
            if(parityLD != parityL || parityRD != parityR) {
                return PlTang<R,C>();
            }

            result.m_bincob[j] = m_bincob[top+j].template slice<C>(left).lowpass(wid);

            parityL = parityLD;
            parityR = parityRD;
        }

        return result;
    }

    //! Replace local tangles.
    //! For the meaning of the arguments, \see slice.
    //! \retval true Replacement succeeded.
    //! \retval false Replacement failed. Note that, in case, the tangle will be kept unchanged.
    template <size_t R, size_t C>
    bool replace(size_t left, size_t top, PlTang<R,C> const& src) noexcept {
        // Validity check.
        if (!isvalid() || !src.isvalid())
            return false;

        // Range check.
        if (left + src.m_hlength > m_hlength || top + src.m_vlength > m_vlength)
            return false;

        // Compatibility of the codomains.
        if (m_bincob[top].template slice<C>(left).lowpass(src.m_hlength) != src.m_bincob[0])
            return false;

        // Compatibility of the domains.
        if (m_bincob[top+src.m_vlength].template slice<C>(left).lowpass(src.m_hlength) != src.m_bincob[src.m_vlength])
            return false;

        // Replacement
        // Note that codomains and domains are in the cases i==0 and i==src.m_vlength respectively.
        for(size_t i = 1; i < src.m_vlength; ++i)
            m_bincob[top+i].replace(left, src.m_bincob[i], src.m_hlength);

        return true;
    }

    //! Make the AA-representation.
    std::string aarep() const noexcept {
        if (m_vlength == 0)
            return std::string();

        size_t hlen = m_hlength + 1;
        std::string result(m_vlength * hlen, 0);

        for(size_t i = 0; i < m_vlength; ++i) {
            foreachAA(m_bincob[i+1], m_bincob[i], m_hlength,
                      [&hlen,&result, &i](int j, char c) -> void {
                          result[i*hlen+j] = c;
                      } );
            result[i*hlen + m_hlength] = '\n';
        }

        return result;
    }

    //! Operation on a specified elementary planar tangle.
    //! This is not really "constexpr" for lambda until C++14.
    //! \param fun The function so that the following line makes sense:
    //!  >> fun(i,c);
    //! where i is the index and c is the character at the index in the AA-rep.
    template <class F>
    constexpr void forElTang(size_t i, F && fun) noexcept
    {
        foreachAA(m_bincob[i+1], m_bincob[i], m_hlength, std::forward<F>(fun));
    }

    template <class F>
    constexpr void forElTang(size_t i, F && fun) const noexcept
    {
        foreachAA(m_bincob[i+1], m_bincob[i], m_hlength, std::forward<F>(fun));
    }

    /** Basic operations **/
    //! Invalidate the representation
    constexpr void invalidate() noexcept
    {
        m_hlength = 0;
        m_vlength = 0;
        m_isvalid = false;
    }

    //! Precomposition.
    //! \param lower The planar tangle to be attached to the bottom.
    //! \return *this.
    template <std::size_t C>
    constexpr PlTang<MR,MC> precomp(PlTang<MR,C> const &lower) noexcept
    {
        // If either of representations is invalid, or if the result is out-of-range, then return the invalid instance.
        if (!isvalid() || !lower.isvalid() || m_vlength + lower.m_vlength > max_rows)
            return PlTang<MR,MC>();

        // Check the composability
        if (m_hlength != lower.m_hlength || domain() != lower.codomain()) {
            return PlTang<MR,MC>();
        }

        for(size_t i = 1; i <= lower.m_vlength; ++i)
            m_bincob[m_vlength+i] = lower.m_bincob[i];

        m_vlength += lower.m_vlength;

        return *this;
    }

    //! Insert identities in the horizontal direction.
    //! \param pos The index of the position where the identities are to be inserted.
    //! \param n The number of the identities to be inserted.
    //! \return true if the operation succeeded.
    constexpr bool hstretch(std::size_t pos, std::size_t n)
    {
        if (!m_isvalid || m_hlength+n >= max_cols || pos > m_hlength) {
            invalidate();
            return false;
        }

        for(std::size_t i = 0; i <= m_vlength; ++i) {
            /** WIP **/
        }

        m_hlength += n;

        return true;
    }

    //! Insert identities in the vertical direction.
    //! \param pos The index of the position where the identities are to be inserted.
    //! \param n The number of the identities to be inserted.
    //! \return true if the operation succeeded.
    constexpr bool vstretch(std::size_t pos, std::size_t n)
    {
        if (!m_isvalid || m_vlength+n > max_rows || pos > m_vlength) {
            invalidate();
            return false;
        }
        /** WIP **/

        return true;
    }

protected:
    //! Encode an AA-representation into a binary representation.
    //! \param aastr An AA-representation.
    //! \return The tuple of
    //!   - get<0>: the flag indicating encoding succeeded or not;
    //!   - get<1>: the binary rep of the domain.
    //!   - get<2>: the binary rep of the codomain.
    template <
      class T,
      std::enable_if_t<
          std::is_same<std::decay_t<T>, char const*>::value,
          int
          > = 0
    >
    constexpr static auto encode(T&& aastr, std::size_t len) noexcept
        -> std::tuple<bool, binrep_type, binrep_type>
    {
        std::tuple<bool,binrep_type,binrep_type> result{true, 0, 0};
        bool hstate = false;

        for(size_t i = 0; i < len; ++i) {
            // Null-termination
            if (aastr[i] == 0) {
                break;
            }

            // In a horizontal strand.
            if(hstate) {
                if (aastr[i] == '7') {
                    std::get<1>(result).set(i);
                    hstate = false;
                }
                else if (aastr[i] == 'J') {
                    std::get<2>(result).set(i);
                    hstate = false;
                }
                // Illegal character found.
                else if (aastr[i] != '-') {
                    std::get<0>(result) = false;
                    break;
                }
            }
            // Outside of horizontal strands.
            else {
                if (aastr[i] == '|') {
                    std::get<1>(result).set(i);
                    std::get<2>(result).set(i);
                }
                else if (aastr[i] == 'r') {
                    std::get<1>(result).set(i);
                    hstate = true;
                }
                else if (aastr[i] == 'L') {
                    std::get<2>(result).set(i);
                    hstate = true;
                }
                // Illegal character found.
                else if (aastr[i] != ' ') {
                    std::get<0>(result) = false;
                    break;
                }
            }
        }

        return result;
    }

    //! Operation on AA-representations.
    //! \param dom The binary representation of the domain.
    //! \param cod The binary representation of the codomain.
    //! \param len The horizontal length of the representation.
    //! \param fun The function so that the following line makes sense:
    //!  >> fun(i,c);
    //! where i is the index and c is the character at the index in the AA-rep.
    //! \return true precisely if given binary representation is valid.
    template <class F>
    static constexpr bool foreachAA(binrep_type dom, binrep_type cod, std::size_t len, F const &fun) noexcept
    {
        bool hstate = false;

        for(size_t i = 0; i < len; ++i) {
            if(hstate) {
                if (!dom.test(i) && !cod.test(i)) {
                    fun(i,'-');
                }
                else if (!dom.test(i) && cod.test(i)) {
                    fun(i,'J');
                    hstate = false;
                }
                else if (dom.test(i) && !cod.test(i)) {
                    fun(i,'7');
                    hstate = false;
                }
                else {
                    return false;
                }
            }
            else {
                if (dom.test(i) && cod.test(i)) {
                    fun(i,'|');
                }
                else if (!dom.test(i) && cod.test(i)) {
                    fun(i,'L');
                    hstate = true;
                }
                else if (dom.test(i) && !cod.test(i)) {
                    fun(i,'r');
                    hstate = true;
                }
                else if (!dom.test(i) && !cod.test(i)) {
                    fun(i,' ');
                }
                else {
                    return false;
                }
            }
        }

        return true;
    }
};
