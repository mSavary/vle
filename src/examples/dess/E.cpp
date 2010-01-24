/**
 * @file examples/dess/E.cpp
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


#include "E.hpp"
#include <cmath>

namespace vle { namespace examples { namespace dess {

E::E(const devs::DynamicsInit& model,
     const devs::InitEventList& evList):
    extension::DifferentialEquation::DESS(model, evList)
{
    // birth and death rate
    m = value::toDouble(evList.get("m"));
    // rate at which exposed individuals (E) become infected (I)
    a = value::toDouble(evList.get("a"));
    // contact rates
    b0 = value::toDouble(evList.get("b0"));
    b1 = value::toDouble(evList.get("b1"));

    _E = createVar("E");
    S = createExt("S");
    I = createExt("I");
}

double E::compute(const vle::devs::Time& time) const
{
    double b = b0 * (1 + b1 * cos(2 * M_PI * time.getValue()));

    return b * S() * I() - (m + a) * _E();
}

DECLARE_NAMED_DYNAMICS(E, E)

}}} // namespace vle examples dess
