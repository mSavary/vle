/**
 * @file examples/dess/Counter.hpp
 * @author The VLE Development Team
 * See the AUTHORS or Authors.txt file
 */

/*
 * VLE Environment - the multimodeling and simulation environment
 * This file is a part of the VLE environment
 * http://www.vle-project.org
 *
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


#include <vle/devs.hpp>

namespace vle { namespace examples { namespace dess {

    /**
     * @brief A DEVS counter which store the date of the latest event.
     */
    class Counter : public devs::Dynamics
    {
    public:
        Counter(const devs::DynamicsInit& model,
		const devs::InitEventList& events) :
            devs::Dynamics(model, events),
            mDate(0),
            mNumber(0)
        { }

        virtual ~Counter()
        { }

        virtual void externalTransition(const devs::ExternalEventList& evts,
                                        const devs::Time& time);

        virtual value::Value* observation(
            const devs::ObservationEvent& e) const;

    private:
        devs::Time mDate;
        int mNumber;
    };

}}} // namespace vle examples dess

DECLARE_NAMED_DYNAMICS(Counter, vle::examples::dess::Counter)
