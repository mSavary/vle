/**
 * @file vle/vpz/Structures.hpp
 * @author The VLE Development Team
 * See the AUTHORS or Authors.txt file
 */

/*
 * VLE Environment - the multimodeling and simulation environment
 * This file is a part of the VLE environment
 * http://www.vle-project.org
 *
 * Copyright (C) 2003-2007 Gauthier Quesnel quesnel@users.sourceforge.net
 * Copyright (C) 2007-2010 INRA http://www.inra.fr
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef VLE_VPZ_STRUCTURES_HPP
#define VLE_VPZ_STRUCTURES_HPP

#include <vle/vpz/Base.hpp>
#include <vle/vpz/DllDefines.hpp>

namespace vle { namespace vpz {

    /**
     * @brief The Structure, only used by the SaxParser.
     */
    class VLE_VPZ_EXPORT Structures : public Base
    {
    public:
        /**
         * @brief Build a new Structures.
         */
        Structures()
        {}

        /**
         * @brief Nothing to output.
         */
        virtual void write(std::ostream& /* out */) const
        {}

        /**
         * @brief Nothing to delete.
         */
        virtual ~Structures()
        {}

        /**
         * @brief Get the type of this class.
         * @return STRUCTURES.
         */
        virtual Base::type getType() const
        { return STRUCTURES; }
    };

}} // namespace vle vpz

#endif
