/**
 * @file src/vle/manager/LinearExperimentGenerator.hpp
 * @author The VLE Development Team
 */

/*
 * VLE Environment - the multimodeling and simulation environment
 * This file is a part of the VLE environment (http://vle.univ-littoral.fr)
 * Copyright (C) 2003 - 2008 The VLE Development Team
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



namespace vle { namespace manager {

    /** 
     * @brief A class to translate Experiment file into Instance of Experiment.
     */
    class LinearExperimentGenerator : public ExperimentGenerator
    {
    public:
        /**
         * Just get a constant reference to VPZ. Use get_instances_files() to
         * generate all VPZ instance file.
         *
         */
        LinearExperimentGenerator(const vpz::Vpz& file, std::ostream& out,
                                  RandPtr rnd) :
            ExperimentGenerator(file, out, rnd)
        { }

        virtual ~LinearExperimentGenerator()
        { }

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
