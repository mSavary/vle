/**
 * @file examples/counter/RegularGenerator.hpp
 * @author The VLE Development Team
 * See the AUTHORS or Authors.txt file
 */

/*
 * VLE Environment - the multimodeling and simulation environment
 * This file is a part of the VLE environment
 * http://www.vle-project.org
 *
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


#ifndef EXAMPLES_RANDOM_REGULAR_GENERATOR_HPP
#define EXAMPLES_RANDOM_REGULAR_GENERATOR_HPP


namespace vle { namespace examples { namespace generator {

    class RegularGenerator : public Generator
    {
    public:
        RegularGenerator(utils::Rand& rnd, double data) :
            Generator(rnd),
            m_data(data)
        { }

        virtual ~RegularGenerator() { }

        virtual double generate() { return m_data; }

        void init(const devs::Time& /* time */) { }

    private:
        double m_data;

    };

}}} // namespace vle examples generator

#endif
