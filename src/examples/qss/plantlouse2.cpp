/**
 * @file examples/qss/plantlouse2.cpp
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


#include "plantlouse2.hpp"
#include <vle/utils/Debug.hpp>

namespace vle { namespace examples { namespace qss {

Plantlouse2::Plantlouse2(const devs::DynamicsInit& model,
                         const devs::InitEventList& events) :
    extension::DifferenceEquation::Simple(model, events)
{
    a = value::toDouble(events.get("a"));
    b = value::toDouble(events.get("b"));
    x = createVar("x");
    y = createSync("y");
}

Plantlouse2::~Plantlouse2()
{
}

double Plantlouse2::compute(const vle::devs::Time& time)
{
    return x(-1) + timeStep(time) * (a * x(-1) - b * y(0) * x(-1));
}

}}} // namespace vle examples qss

DECLARE_NAMED_DYNAMICS(plantlouse2, vle::examples::qss::Plantlouse2)
