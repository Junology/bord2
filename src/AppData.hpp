/*!
 * \file AppData.cpp
 * \author Jun Yoshida
 * \copyright (c) 2019-2020 Jun Yoshida.
 * The project is released under the MIT License.
 * \date March 7, 2020: created
 */

#include <ostream>
#include <istream>
#include <vector>
#include <string>

#include "PlTangMove.hpp"

/*!
 * \section Save data format.
 * The application saves its data into a text file, which consists of the following two parts.
 *   + The first part is the AA-representation of the tangle on the domain.
 *   + The second part is a sequence of moves of tangles. Each move is written in the following form:
 *     >> NameOfMove x-position y-position
 *     where (x,y) denotes the coordinate of the top-left cell where the move applies. Moreover, they are grouped; each group of moves must begin with the symbol '=' followed by arbitrary characters before EOL, which are just ignored and so can be used to attach comments.
 */

//! Write the data to a stream.
template <size_t R, size_t C>
void writeTangleMove(
    std::ostream& os,
    PlTang<R,C> const& domain,
    std::vector<typename PlTangMove<2,2>::MoveSeq> const& moveseqs
    ) noexcept
{
    // Write the AA representation of the domain tangle followed by an empty line.
    os << domain.aarep() << std::endl;

    for(auto const& movesq : moveseqs) {
        os << "=== " << movesq.size() << " moves ===" << std::endl;
        for(auto const& mv : movesq) {
            os << mv.move.getName() << " "
               << mv.x << " "
               << mv.y << std::endl;
        }
    }
}

struct LoadedMove
{
    std::string name;
    size_t x, y;
};

using LoadedMoveGroup = std::vector<LoadedMove>;

//! Write the data to a stream.
//! \param is The input stream from which the data are to be read.
//! \param domain_out A reference to a variable where the loaded domain tangle is stored.
//! \return The pair of
//!   - the flag indicating whether the loading suceeded or not. In case an error occured, every passed arguments are kept untouched;
//!   - the sequence of moves.
//! \note This function does not care about whether the names of moves are valid or not.
template <size_t R, size_t C>
auto readTangleMove(std::istream& is, PlTang<R,C> & domain_out) noexcept
    -> std::pair<bool, std::vector<LoadedMoveGroup>>
{
    std::string aarep{}, nextln{};

    while(is.peek() != '=' && std::getline(is, nextln)) {
        aarep += nextln + '\n';
    }

    auto domain = PlTang<R,C>(aarep.c_str());

    if (!domain.isvalid())
        return {false, {}};

    auto result = std::make_pair(
        true,
        std::vector<LoadedMoveGroup>{} );

    while((is >> std::ws) && !is.eof()) {
        // At the begining of a group.
        if(is.peek() == '=') {
            // Append a new group to the buffer.
            result.second.emplace_back();
        }
        else {
            // Load the move information.
            LoadedMove move;
            is >> move.name >> move.x >> move.y;
            result.second.back().push_back(move);
        }

        // Ignore the remaining characters in the current line.
        is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    domain_out = domain;

    return result;
}
