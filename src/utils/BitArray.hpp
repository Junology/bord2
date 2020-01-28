/*!
 * \file BitArrY.hpp
 * \author Jun Yoshida
 * \copyright (c) 2019 Jun Yoshida.
 * The project is released under the MIT License.
 * \date January, 5 2020: created
 */

#pragma once

#include "../utils.hpp"

/*!
 * Personal re-implementation of std::bitset with constexpr supports even in C++14.
 * It is guaranteed that every bit out of range is 0.
 * \tparam N The number of bits
 * \tparam T The type carrying bits.
 */
template <std::size_t N, class T = unsigned int>
class BitArray {
    //! Have access to BitArray of different lengths.
    template<std::size_t M, class U>
    friend class BitArray;

public:
    using chunk_type = T;

    enum : std::size_t {
        chunkbits = 8 * sizeof(chunk_type),
        length = (N + chunkbits - 1) / chunkbits,
        endbits = N % chunkbits
    };

    enum chunkval : chunk_type {
        zero = static_cast<chunk_type>(0u),
        nzero = static_cast<chunk_type>(~zero)
    };

private:
    chunk_type m_arr[length];

    //! Implementation of 0 initialization
    template <std::size_t... is, class... Ts>
    constexpr BitArray(std::index_sequence<is...>, Ts&&... args) noexcept
      : m_arr{std::forward<Ts>(args)..., bord2::firstoftwo<0u,is>::value...}
    {}

    //! Initialize from an array of chunk_type
    template <std::size_t... is>
    constexpr BitArray(std::index_sequence<is...>, chunk_type const (&arr)[sizeof...(is)])
      : BitArray(arr[is]...)
    {}

public:
    /** Constructor and Destructor **/
    //! Default constructor.
    //! Bits are cleared to 0.
    constexpr BitArray() noexcept
      : BitArray(std::make_index_sequence<length>())
    {}

    //! Can be constructed from chunk_type.
    constexpr BitArray(chunk_type x0) noexcept
      : BitArray(std::make_index_sequence<(length>0 ? length-1 : 0)>(), x0)
    {}

    //! Can be constructed from smaller BitArray with same chunk_type.
    template<size_t M, std::enable_if_t<(M<N), int> = 0>
    constexpr BitArray(BitArray<M,T> const& src) noexcept
      : BitArray(std::make_index_sequence<BitArray<M,T>::length>(), src.m_arr)
    {}

    //! Construct from a sequence of chunk_type's.
    template <
        class... Ts,
        std::enable_if_t<
            (sizeof...(Ts)+2 <= length) &&
            bord2::all_of<std::is_same<std::decay_t<Ts>,chunk_type>::value...>::value,
            int
            > = 0
        >
    constexpr BitArray(chunk_type x0, chunk_type x1, Ts... xs) noexcept
    : BitArray(std::make_index_sequence<(length>=(sizeof...(Ts)+2) ? (length-sizeof...(Ts)-2) : 0)>(), x0, x1, xs...)
    {
    }

    //! Copy constructor is the default one.
    constexpr BitArray(BitArray<N,T> const&) noexcept = default;

    //! Move constructor is the default one.
    constexpr BitArray(BitArray<N,T>&&) noexcept = default;

    /** Basic operations **/
    //! Set a bit in the given position.
    constexpr void set(std::size_t pos, bool value = true) noexcept
    {
        std::size_t gpos = pos / chunkbits;
        std::size_t lpos = pos % chunkbits;
        if(value)
            m_arr[gpos] |= 0x1 << lpos;
        else
            m_arr[gpos] &= ~(0x1 << lpos);
    }

    //! Test if a bit in the given position is true.
    constexpr bool test(std::size_t pos) const noexcept
    {
        std::size_t gpos = pos / chunkbits;
        std::size_t lpos = pos % chunkbits;
        return m_arr[gpos] & (0x1 << lpos);
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

    //! Slicing array.
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
        return lowcut_impl(std::make_index_sequence<length>(), n);
    }

    //! Inactivate higher bits.
    //! \param n The number of bits kept considered.
    constexpr BitArray<N,T> lowpass(std::size_t n) const noexcept
    {
        return lowpass_impl(std::make_index_sequence<length>(), n);
    }

    //! Replace subarray with smaller array
    //! \param i The index of the lowest bit to be replaced.
    template <size_t M, std::enable_if_t<(M<=N),int> = 0 >
    constexpr void replace(size_t i, BitArray<M,T> const& src) noexcept
    {
        std::size_t gpos = i / chunkbits;
        std::size_t lpos = i % chunkbits;
        std::size_t num = (M + lpos + chunkbits - 1) / chunkbits;

        BitArray<M+chunkbits,T> src_adj = BitArray<M+chunkbits,T>{src} << lpos;
        BitArray<M+chunkbits,T> mask = BitArray<M+chunkbits,T>{~BitArray<M,T>{}} << lpos;

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
        for(size_t i=0; i < BitArray<M,T>::length; ++i)
            m_arr[i] = src.m_arr[i];

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

    constexpr bool operator==(BitArray<N,T> const &other) const noexcept
    {
        bool result = true;

        for(std::size_t i = 0; i < length; ++i)
            result &= (m_arr[i] == other.m_arr[i]);

        return result;
    }

    constexpr bool operator!=(BitArray<N,T> const &other) const noexcept
    {
        return !((*this)==other);
    }

    constexpr BitArray<N,T> operator&(BitArray<N,T> const &other) const noexcept
    {
        return and_impl(other, std::make_index_sequence<length>());
    }

    constexpr BitArray<N,T> operator|(BitArray<N,T> const &other) const noexcept
    {
        return or_impl(other, std::make_index_sequence<length>());
    }

    constexpr BitArray<N,T> operator^(BitArray<N,T> const &other) const noexcept
    {
        return xor_impl(other, std::make_index_sequence<length>());
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
               ? (chunkval::nzero >> (chunkbits-endbits))
               : chunkval::nzero)
            : chunkval::zero;
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
                ? 0
                : ((m_arr[gpos+is] >> lpos) | (gpos+is+1>=length ? 0u : (m_arr[gpos+is+1] << (chunkbits-lpos)))) & (BitArray<n,T>::get_mask(is))
                )...};
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> lowcut_impl(std::index_sequence<is...>, std::size_t n) const noexcept
    {
        std::size_t gpos = n / chunkbits;
        std::size_t lpos = n % chunkbits;
        chunk_type mask{chunkval::nzero << lpos};

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
        chunk_type mask{lpos > 0u ? static_cast<chunk_type>(chunkval::nzero >> (chunkbits - lpos)) : chunkval::zero};

        return BitArray<N,T>(
            static_cast<chunk_type>(
                is < gpos ? m_arr[is]
                : (is == gpos ? m_arr[is] & mask : 0)
            )...
        );
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> and_impl(BitArray<N,T> const &other, std::index_sequence<is...>) const noexcept
    {
        return BitArray<N,T>(static_cast<chunk_type>(m_arr[is] & other.m_arr[is])...);
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> or_impl(BitArray<N,T> const &other, std::index_sequence<is...>) const noexcept
    {
        return BitArray<N,T>(static_cast<chunk_type>(m_arr[is] | other.m_arr[is])...);
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> xor_impl(BitArray<N,T> const &other, std::index_sequence<is...>) const noexcept
    {
        return BitArray<N,T>(static_cast<chunk_type>((m_arr[is] ^ other.m_arr[is]) & get_mask(is)) ...);
    }

    template <std::size_t... is>
    constexpr BitArray<N,T> not_impl(std::index_sequence<is...>) const noexcept
    {
        return BitArray<N,T>(static_cast<chunk_type>((~m_arr[is]) & get_mask(is))...);
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

