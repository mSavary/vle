/**
 * @file vle/manager/LinearExperimentGenerator.hpp
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


#ifndef VLE_MANAGER_LINEAREXPERIMENTGENERATOR_HPP
#define VLE_MANAGER_LINEAREXPERIMENTGENERATOR_HPP

#include <vle/manager/ExperimentGenerator.hpp>
#include <vle/manager/DllDefines.hpp>

namespace vle { namespace manager {

    /**
     * @brief A class to translate Experiment file into Instance of Experiment.
     */
    class VLE_MANAGER_EXPORT LinearExperimentGenerator :
        public ExperimentGenerator
    {
    public:
        /**
         * Just get a constant reference to VPZ. Use get_instances_files() to
         * generate all VPZ instance file.
         */
        LinearExperimentGenerator(const vpz::Vpz & file, std::ostream & out,
                                  bool storecomb, bool commonseed, RandPtr rnd)
            : ExperimentGenerator(file, out, storecomb, commonseed, rnd)
        {
        }

        virtual ~LinearExperimentGenerator()
        {
        }

        virtual void buildCombination(size_t& nb);

        /**
         * @brief Get the number of combination from vpz file.
         *
         * @return A value greater than 0.
         */
        virtual size_t getCombinationNumber() const;

    };

}} // namespace vle manager

#endif
