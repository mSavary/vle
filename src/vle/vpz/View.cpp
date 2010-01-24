/**
 * @file vle/vpz/View.cpp
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


#include <vle/vpz/View.hpp>
#include <vle/utils/Debug.hpp>

namespace vle { namespace vpz {

View::View(const std::string& name,
           View::Type type,
           const std::string& output,
           double timestep) :
    m_name(name),
    m_type(type),
    m_output(output),
    m_timestep(timestep)
{
    if (m_type == View::TIMED) {
        if (m_timestep <= 0.0) {
            throw utils::ArgError(fmt(
                    _("Cannont define the View '%1%' with a timestep '%2%'")) %
                m_name % m_timestep);
        }
    }
}

void View::write(std::ostream& out) const
{
    out << "<view "
        << "name=\"" << m_name.c_str() << "\" "
        << "output=\"" << m_output.c_str() << "\" ";

    switch (m_type) {
    case View::EVENT:
        out << "type=\"event\"";
        break;
    case View::TIMED:
        out << "type=\"timed\" "
            << "timestep=\"" << m_timestep << "\"";
        break;
    case View::FINISH:
        out << "type=\"finish\"";
        break;
    }

    if (m_data.empty()) {
        out << " />\n";
    } else {
        out << ">\n"
            << "<![CDATA[\n"
            << m_data.c_str()
            << "]]>\n"
            << "</view>\n";
    }
}

void View::setTimestep(double time)
{
    if (time <= 0.0) {
        throw utils::ArgError(fmt(
                _("Bad time step %1% for view %2%")) % time % m_name);
    }

    m_timestep = time;
}

bool View::operator==(const View& view) const
{
    return m_name == view.name() and m_type == view.type()
	and m_output == view.output()
	and m_timestep == view.timestep() and m_data == view.data();
}


}} // namespace vle vpz
