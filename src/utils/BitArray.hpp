/*!
 * \file BitArray.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January, 5 2020: created
 */

#pragma once

#include "../utils.hpp"

#include <algorithm>
#include <limits>

/*!
 * Personal re-implementation of std::bitset with constexpr supports even in C++14.
 * It is guaranteed that every bit out of range is 0.
 * \tparam N The number of bits
 * \tparam T The type carrying bits.
 */
template <std::size_t N, class T = uint_fast32_t>
class BitArray {
    static_assert(std::is_unsigned<T>::value, "The type T is not unsigned or integral.");
    static_assert( N > 0, "Zero-sized array is prohibited");

    //! Have access to BitArray of different lengths.
    template<std::size_t M, class U>
    friend class BitArray;

public:
    using chunk_type = T;

    enum : std::size_t {
        numbits = N,
        chunkbits = std::numeric_limits<chunk_type>::digits,
        length = (N + chunkbits - 1) / chunkbits,
        endbits = N % chunkbits
    };

    enum chunkval : chunk_type {
        zero = static_cast<chunk_type>(0u),
        one = static_cast<chunk_type>(1u),
        nzero = static_cast<chunk_type>(~zero)
    };

    template <size_t n>
    struct lowmask {
        enum : chunk_type {
            mask = (0 < n)
                ? (n < chunkbits)
                  ? static_cast<chunk_type>(chunkval::nzero >> (chunkbits - n))
                  : chunkval::nzero
                : 0
        };
    };

private:
    chunk_type m_arr[length];

    //! Initialize from an array of chunk_type
    template <std::size_t... is>
    constexpr BitArray(std::index_sequence<is...>, chunk_type const (&arr)[sizeof...(is)])
      : m_arr{arr[is]...}
    {}

public:
    /** Constructors **/
    //! Default constructor.
    //! Bits are cleared to 0 since array elements are initialized in the same way as objects with static durations.
    constexpr BitArray() noexcept
      : m_arr{}
    {}

    //! Can be constructed from chunk_type.
    constexpr BitArray(chunk_type x0) noexcept
      : m_arr{static_cast<chunk_type>(x0 & get_mask(0))}
    {}

    //! Can be constructed from smaller BitArray with same chunk_type.
    template<size_t M, std::enable_if_t<(M<N), int> = 0>
    constexpr BitArray(BitArray<M,T> const& src) noexcept
      : BitArray(std::make_index_sequence<BitArray<M,T>::length>(), src.m_arr)
    {}

    //! Construct from a sequence of chunk_type's.
    //! A version for the case where the last byte doesn't need rounding.
    template <
        class... Ts,
        std::enable_if_t<
            bord2::allTrue({
                    sizeof...(Ts)+2 <= length,
                    sizeof...(Ts)+2 < length || endbits == 0,
                    std::is_convertible<Ts,chunk_type>::value...}),
            int
            > = 0
        >
    constexpr BitArray(chunk_type x0, chunk_type x1, Ts... xs) noexcept
      : m_arr{x0, x1, xs...}
    {}

    //! Construct from a sequence of chunk_type's.
    //! A version for the case where the last byte needs rounding.
    template <
        class... Ts,
        std::enable_if_t<
            bord2::allTrue({
                    sizeof...(Ts)+2 <= length,
                    sizeof...(Ts)+2 == length && endbits > 0,
                    std::is_convertible<Ts,chunk_type>::value...}),
            int
            > = 0
        >
    constexpr BitArray(chunk_type x0, chunk_type x1, Ts... xs) noexcept
      : m_arr{x0, x1, xs...}
    {
        m_arr[length-1] &= lowmask<endbits>::mask;
    }

    //! Copy constructor is the default one.
    constexpr BitArray(BitArray<N,T> const&) noexcept = default;

    //! Move constructor is the default one.
    constexpr BitArray(BitArray<N,T>&&) noexcept = default;

    /** Basic operations **/
    //! Set a bit in the given position.
    constexpr void set(std::size_t pos, bool value = true) noexcept
    {
        // If the position is out-of-range, nothing happen.
        if (pos >= numbits)
            return;

        std::size_t gpos = pos / chunkbits;
        std::size_t lpos = pos % chunkbits;
        if(value)
            m_arr[gpos] |= (chunkval::one) << lpos;
        else
            m_arr[gpos] &= ~(chunkval::one << lpos);
    }

    //! Test if a bit in the given position is true.
    constexpr bool test(std::size_t pos) const noexcept
    {
        // If the position is out-of-range, the function always returns false.
        if (pos > numbits)
            return false;

        std::size_t gpos = pos / chunkbits;
        std::size_t lpos = pos % chunkbits;
        return m_arr[gpos] & (chunkval::one << lpos);
    }

    //! Population-count (aka. Hamming weight).
    //! The result may be incorrect when N > std::numeric_limits<std::size_t>::max.
    constexpr size_t popCount() const noexcept
    {
        size_t result = 0;

        for(size_t i = 0; i < length; ++i) {
            result += static_cast<size_t>(bord2::popcount(m_arr[i]));
        }

        return result;
    }

    //! Count trailing ones.
    constexpr size_t countTrail1() const noexcept {
        size_t result = 0;
        for(size_t i = 0; i < length; ++i) {
            size_t r = bord2::counttrail1(m_arr[i]);
            result += r;
            if (r < chunkbits)
                break;
        }
        return result;
    }

    //! Count trailing zeros.
    constexpr std::size_t countTrail0() const noexcept {
        std::size_t result = 0;
        for(std::size_t i = 0; i < length; ++i) {
            std::size_t r = m_arr[i]
                ? bord2::counttrail0<chunk_type>(m_arr[i])
                : chunkbits;

            // Not at the end byte.
            if (endbits == 0 || i+1 < length)
                result += r;
            // At the end byte.
            else {
                result += std::min(r, static_cast<std::size_t>(endbits));
                break;
            }

            if (r < chunkbits)
                break;
        }
        return result;
    }

    //! Slicing array; a version for slicing into a larger bit-array.
    //! Hence, it is actually just a cast with right shifts.
    template <size_t n>
    constexpr auto slice(std::size_t i) const noexcept
        -> std::enable_if_t<(n>=N),BitArray<n,T>>
    {
        return BitArray<n>(*this) >> i;
    }

    //! Slicing array; a version for slicing into a smaller bit-array.
    //! \param i The position of the lowest bit in the slice.
    template <size_t n>
    constexpr auto slice(std::size_t i) const noexcept
        -> std::enable_if_t<(n<N),BitArray<n,T>>
    {
        return slice_impl<n>(
            std::make_index_sequence<BitArray<n,T>::length>(), i);
    }

    //! Inactivate lower bits.
    //! \param n The number of bits ignored.
    constexpr BitArray<N,T> lowcut(std::size_t n) const noexcept
    {
        return n >= N ? BitArray<N,T>{} : lowcut_impl(std::make_index_sequence<length>(), n);
    }

    //! Inactivate higher bits.
    //! \param n The number of bits kept considered.
    constexpr BitArray<N,T> lowpass(std::size_t n) const noexcept
    {
        return n >= N ? *this : lowpass_impl(std::make_index_sequence<length>(), n);
    }

    //! Replace subarray with smaller array
    //! \param i The index of the lowest bit to be replaced.
    template <size_t M, std::enable_if_t<(M<=N),int> = 0 >
    constexpr void replace(size_t i, BitArray<M,T> const& src, size_t wid = M) noexcept
    {
        // Check if the operation is trivial.
        // Thanks to this, we can assume i < N in what follows.
        if (i >= N)
            return;

        std::size_t gpos = i / chunkbits;
        std::size_t lpos = i % chunkbits;

        BitArray<M+chunkbits,T> src_adj = BitArray<M+chunkbits,T>{src} << lpos;
        BitArray<M+chunkbits,T> mask
            = BitArray<M+chunkbits,T>{(~BitArray<M,T>{}).lowpass(wid)} << lpos;

        // The number of loops
        // Note that we here assume i < N so that gpos < length.
        std::size_t num
            = std::min(length-gpos, (M + lpos + chunkbits - 1) / chunkbits);
        for(size_t j = 0; j < num; ++j) {
            m_arr[gpos+j] &= ~(mask.m_arr[j]);
            m_arr[gpos+j] |= src_adj.m_arr[j];
        }
    }

    /** Operator overloads **/
    constexpr BitArray<N,T>& operator=(BitArray<N,T> const&) noexcept = default;
    constexpr BitArray<N,T>& operator=(BitArray<N,T>&&) noexcept = default;

    //! Copy from smaller BitArray (with the same chunk_type).
    template<size_t M, std::enable_if_t<(M<N),int> = 0>
    constexpr BitArray<N,T>& operator=(BitArray<M,T> const &src) noexcept
    {
        std::fill(
            std::copy(std::begin(src.m_arr),
                      std::end(src.m_arr),
                      std::begin(m_arr) ),
            std::end(m_arr), 0);

        return *this;
    }

    //! Copy from chunk-type
    template <
        class U,
        std::enable_if_t<
            std::is_same<U, chunk_type>::value,
            int
            > = 0
        >
    constexpr BitArray<N,T>& operator=(U x) noexcept
    {
        m_arr[0] = x;
        for(size_t i = 1; i < length; ++i) {
            m_arr[i] = 0;
        }

        return *this;
    }

    explicit constexpr operator bool() const noexcept {
        for(auto x : m_arr)
            if(x) return true;

        return false;
    }

    constexpr bool operator==(BitArray<N,T> const &other) const noexcept
    {
        for(std::size_t i = 0; i < length; ++i)
            if(m_arr[i] != other.m_arr[i]) return false;

        return true;
    }

    constexpr bool operator!=(BitArray<N,T> const &other) const noexcept
    {
        return !((*this)==other);
    }

    constexpr BitArray<N,T>& operator&=(BitArray<N,T> const &other) noexcept
    {
        for(size_t i = 0; i < length; ++i)
            m_arr[i] &= other.m_arr[i];
        return *this;
    }

    constexpr BitArray<N,T>& operator|=(BitArray<N,T> const &other) noexcept
    {
        for(size_t i = 0; i < length; ++i)
            m_arr[i] |= other.m_arr[i];
        return *this;
    }

    constexpr BitArray<N,T>& operator^=(BitArray<N,T> const &other) noexcept
    {
        for(size_t i = 0; i < length; ++i)
            m_arr[i] ^= other.m_arr[i];
        return *this;
    }

    constexpr BitArray<N,T> operator~() const noexcept
    {
        return not_impl(std::make_index_sequence<length>());
    }

    constexpr BitArray<N,T> operator<<(std::size_t n) const noexcept
    {
        return lshift_impl(std::make_index_sequence<length>(), n);
    }

    constexpr BitArray<N,T> operator>>(std::size_t n) const noexcept
    {
        return rshift_impl(std::make_index_sequence<length>(), n);
    }

protected:
    /** Some query on chunks **/
    static constexpr bool inrange(std::size_t i) noexcept {
        return i<length;
    }

    static constexpr chunk_type get_mask(std::size_t i) noexcept {
        return inrange(i)
            ? (endbits > 0 && i+1 == length // check if tail bit.
               ? static_cast<chunk_type>(lowmask<endbits>::mask)
               : static_cast<chunk_type>(chunkval::nzero))
            : static_cast<chunk_type>(chunkval::zero);
    }

    /** Implementations **/
    template <size_t n, size_t... is>
    constexpr auto slice_impl(std::index_sequence<is...>, std::size_t i) const noexcept
        -> std::enable_if_t<(n<N),BitArray<n,T>>
    {
        std::size_t gpos = i / chunkbits;
        std::size_t lpos = i % chunkbits;

        return BitArray<n,T>{static_cast<chunk_type>(
                gpos+is >= length
                ? 0u
                : ((m_arr[gpos+is] >> lpos) | (gpos+is+1>=length ? 0u : (m_arr[gpos+is+1] << (chunkbits-lpos))))
                )...};
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> lowcut_impl(std::index_sequence<is...>, std::size_t n) const noexcept
    {
        std::size_t gpos = n / chunkbits;
        std::size_t lpos = n % chunkbits;
        chunk_type mask{static_cast<chunk_type>(chunkval::nzero << lpos)};

        return BitArray<N,T>(
            static_cast<chunk_type>(
                is > gpos ? m_arr[is]
                : (is == gpos ? m_arr[is] & mask : 0u)
            )...
        );
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> lowpass_impl(std::index_sequence<is...>, std::size_t n) const noexcept
    {
        std::size_t gpos = n / chunkbits;
        std::size_t lpos = n % chunkbits;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
        chunk_type mask{lpos > 0u ? static_cast<chunk_type>(chunkval::nzero >> (chunkbits - lpos)) : chunkval::zero};
#pragma GCC diagnostic pop

        return BitArray<N,T>(
            static_cast<chunk_type>(
                is < gpos ? m_arr[is]
                : (is == gpos ? m_arr[is] & mask : 0)
            )...
        );
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> not_impl(std::index_sequence<is...>) const noexcept
    {
        return BitArray<N,T>(static_cast<chunk_type>(~m_arr[is])...);
    }

    template<std::size_t... is>
    constexpr BitArray<N,T> lshift_impl(std::index_sequence<is...>, std::size_t n) const noexcept
    {
        std::size_t gpos = n / chunkbits;
        std::size_t lpos = n % chunkbits;
        return BitArray<N,T>(
            static_cast<chunk_type>(
                (is > gpos)
                ? ((m_arr[is-gpos] << lpos) | (m_arr[is-gpos-1] >> (chunkbits-lpos))) & get_mask(is)
                : ((is==gpos) ? ((m_arr[0] << lpos) & get_mask(is)) : 0u)
            )...
        );
    }

    template<std::size_t... is>
    constexpr BitArray<N,T> rshift_impl(std::index_sequence<is...>, std::size_t n) const noexcept
    {
        std::size_t gpos = n / chunkbits;
        std::size_t lpos = n % chunkbits;
        return BitArray<N,T>(
            static_cast<chunk_type>(
                is+gpos < length-1
                ? (m_arr[is+gpos] >> lpos | m_arr[is+gpos+1] << (chunkbits-lpos))
                : ((is+gpos == length-1) ? (m_arr[length-1] >> lpos) : chunkval::zero )
            )...
        );
    }
};

template<size_t N, class T>
constexpr BitArray<N,T> operator&(BitArray<N,T> lhs, BitArray<N,T> const& rhs) noexcept
{
    lhs &= rhs;
    return lhs;
}

template<size_t N, class T>
constexpr BitArray<N,T> operator|(BitArray<N,T> lhs, BitArray<N,T> const& rhs) noexcept
{
    lhs |= rhs;
    return lhs;
}

template<size_t N, class T>
constexpr BitArray<N,T> operator^(BitArray<N,T> lhs, BitArray<N,T> const& rhs) noexcept
{
    lhs ^= rhs;
    return lhs;
}

template<class C, size_t N, class T>
std::ostream& operator<<(std::basic_ostream<C>& out, BitArray<N,T> const& bit)
{
    for(size_t i = 0; i < N; ++i)
        out.put(bit.test(N-i-1) ? '1' : '0');

    return out;
}
