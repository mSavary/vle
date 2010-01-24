/**
 * @file vle/value/Tuple.cpp
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
 * Copyright (C) 2003-2010 ULCO http://www.univ-littoral.fr
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


#include <vle/value/Tuple.hpp>
#include <vle/utils/Debug.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace vle { namespace value {

void Tuple::writeFile(std::ostream& out) const
{
    for (const_iterator it = m_value.begin(); it != m_value.end(); ++it) {
        if (it != m_value.begin()) {
            out << " ";
        }
        out << *it;
    }
}

void Tuple::writeString(std::ostream& out) const
{
    out << "(";
    for (const_iterator it = m_value.begin(); it != m_value.end(); ++it) {
        if (it != m_value.begin()) {
            out << ",";
        }
        out << *it;
    }
    out << ")";
}

void Tuple::writeXml(std::ostream& out) const
{
    out << "<tuple>";
    for (const_iterator it = m_value.begin(); it != m_value.end(); ++it) {
        if (it != m_value.begin()) {
            out << " ";
        }
        out << *it;
    }
    out << "</tuple>";
}

void Tuple::fill(const std::string& str)
{
    std::string cpy(str);
    boost::algorithm::trim(cpy);

    std::vector < std::string > result;
    boost::algorithm::split(result, cpy,
                            boost::algorithm::is_any_of(" \n\t\r"));

    for (std::vector < std::string >::iterator it = result.begin();
         it != result.end(); ++it) {
        boost::algorithm::trim(*it);
        if (not (*it).empty()) {
            try {
                m_value.push_back(boost::lexical_cast < double >(*it));
            } catch(const boost::bad_lexical_cast& e) {
                try {
                    m_value.push_back(boost::lexical_cast < long >(*it));
                } catch(const boost::bad_lexical_cast& e) {
                    throw utils::ArgError(fmt(
                                "Can not convert string '%1%' into"
                                " double or long") % (*it));
                }
            }
        }
    }
}

}} // namespace vle value

