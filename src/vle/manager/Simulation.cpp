/*
 * This file is part of VLE, a framework for multi-modeling, simulation
 * and analysis of complex dynamical systems.
 * http://www.vle-project.org
 *
 * Copyright (c) 2003-2013 Gauthier Quesnel <quesnel@users.sourceforge.net>
 * Copyright (c) 2003-2013 ULCO http://www.univ-littoral.fr
 * Copyright (c) 2007-2013 INRA http://www.inra.fr
 *
 * See the AUTHORS or Authors.txt file for copyright owners and
 * contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <vle/DllDefines.hpp>
#include <vle/utils/ModuleManager.hpp>
#include <vle/utils/Tools.hpp>
#include <vle/utils/Trace.hpp>
#include <vle/devs/RootCoordinator.hpp>
#include <vle/manager/Simulation.hpp>
#include <vle/vpz/Vpz.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

namespace vle { namespace manager {

template <typename T>
    static void write(const T& t,std::ostream *output)
    {
	if (output) {
	    (*output) << t;
	} else {
	    utils::Trace::send(t);
	}
    }

static value::Map * runVerboseRun(vpz::Vpz                   *vpz,
                                  const utils::ModuleManager &modulemgr,
                                  Error                      *error,
                                  std::ostream               *output)
{
    value::Map   *result = 0;
    boost::timer  timer;

    try {
        devs::RootCoordinator root(modulemgr);

        const double duration = vpz->project().experiment().duration();
        const double begin    = vpz->project().experiment().begin();

        write(fmt(_("[%1%]\n")) % vpz->filename(), output);
        write(_(" - Coordinator load models ......: "),output);

        root.load(*vpz);

        write(_("ok\n"), output);

        write(_(" - Clean project file ...........: "), output);
        vpz->clear();
        delete vpz;
        write(_("ok\n"), output);

        write(_(" - Coordinator initializing .....: "), output);
        root.init();
        write(_("ok\n"), output);

        write(_(" - Simulation run................: "), output);

        boost::progress_display display(100, *output, "\n   ", "   ", "   ");
        long previous = 0;

        while (root.run()) {
            long pc = std::floor(100. * (root.getCurrentTime() - begin) /
                                 duration);

            display  += pc - previous;
            previous  = pc;
        }

        display += 100 - previous;

        write(_(" - Coordinator cleaning .........: "), output);
        root.finish();
        write(_("ok\n"), output);

        result = root.outputs();

        write(fmt(_(" - Time spent in kernel .........: %1% s"))
              % timer.elapsed(), output);

        error->code    = 0;
    } catch(const std::exception& e) {
        error->message = (fmt(_("\n/!\\ vle error reported: %1%\n%2%"))
                          % utils::demangle(typeid(e))
                          % e.what()).str();
        error->code    = -1;
    }

    return result;
}

static value::Map * runVerboseSummary(vpz::Vpz                   *vpz,
                                      const utils::ModuleManager &modulemgr,
                                      Error                      *error,
                                      std::ostream               *output)
{
    value::Map   *result = 0;
    boost::timer  timer;

    try {
        devs::RootCoordinator root(modulemgr);

        write(fmt(_("[%1%]\n")) % vpz->filename(), output);
        write(_(" - Coordinator load models ......: "), output);

        root.load(*vpz);

        write(_("ok\n"), output);

        write(_(" - Clean project file ...........: "), output);
        vpz->clear();
        delete vpz;
        write(_("ok\n"), output);

        write(_(" - Coordinator initializing .....: "), output);
        root.init();
        write(_("ok\n"), output);

        write(_(" - Simulation run................: "), output);

        while (root.run());
        write(_("ok\n"), output);

        write(_(" - Coordinator cleaning .........: "), output);
        root.finish();
        write(_("ok\n"), output);

        result = root.outputs();

        write(fmt(_(" - Time spent in kernel .........: %1% s"))
              % timer.elapsed(), output);

        error->code    = 0;
    } catch(const std::exception& e) {
        error->message = (fmt(_("\n/!\\ vle error reported: %1%\n%2%"))
                          % utils::demangle(typeid(e))
                          % e.what()).str();
        error->code    = -1;
    }

    return result;
}

static value::Map * runQuiet(vpz::Vpz                   *vpz,
                             const utils::ModuleManager &modulemgr,
                             Error                      *error)
{
    value::Map *result = 0;

    try {
        devs::RootCoordinator root(modulemgr);
        root.load(*vpz);
        vpz->clear();
        delete vpz;

        root.init();
        while (root.run()) {}
        root.finish();

        error->code    = 0;
        result         = root.outputs();
    } catch(const std::exception& e) {
        error->message = (fmt(_("/!\\ vle error reported: %1%\n"))
                          % e.what()).str();
        error->code    = -1;
    }

    return result;
}

static value::Map * run(vpz::Vpz                   *vpz,
                        const utils::ModuleManager &modulemgr,
                        Error                      *error,
                        LogOptions                  logOptions,
                        SimulationOptions           simOptions,
                        std::ostream               *output)
{
    value::Map *result;
    if (logOptions != manager::LOG_NONE) {
        if (logOptions & manager::LOG_RUN and output) {
            result = runVerboseRun(vpz, modulemgr, error, output);
        } else {
            result = runVerboseSummary(vpz, modulemgr, error, output);
        }

    } else {
        result = runQuiet(vpz, modulemgr, error);
    }

    if (simOptions & manager::SIMULATION_NO_RETURN) {
        delete result;
        result = 0;
    }

    return result;
}

class Timeout
{
public:
    Timeout(int timeout,
	    vpz::Vpz *vpz,
	    const utils::ModuleManager &modulemgr,
	    Error *error,
	    LogOptions logOptions,
	    SimulationOptions simOptions,
	    std::ostream  *output)
	: m_success(false),
	m_timer(m_io_service, boost::posix_time::milliseconds(timeout)),
	m_vpz(vpz), m_modulemgr(modulemgr), m_error(error), m_result(0),
	m_logoptions(logOptions), m_simulationoptions(simOptions),
	m_out(output)
    {
        m_timer.async_wait(boost::bind(&Timeout::stop, this));
        boost::thread thrd(&Timeout::work, this);
        m_io_service.run();
    }

    value::Map * getResult()
    {
        return m_result;
    }

private:
    bool m_success;
    boost::asio::io_service m_io_service;
    boost::asio::deadline_timer m_timer;
    vpz::Vpz *m_vpz;
    const utils::ModuleManager &m_modulemgr;
    Error *m_error;
    value::Map *m_result;
    LogOptions         m_logoptions;
    SimulationOptions  m_simulationoptions;
    std::ostream      *m_out;

    void stop()
    {
        if (!m_success) {
            m_vpz->setFilename("Error");
	    m_result = 0;
	    m_io_service.stop();
        }
    }

    void work ()
    {
	vle::manager::run(m_vpz, m_modulemgr, m_error, m_logoptions,
			  m_simulationoptions, m_out);
        m_success = true;
        m_timer.cancel();
    }
};

struct Simulation::Pimpl
{
    std::ostream      *m_out;
    LogOptions         m_logoptions;
    SimulationOptions  m_simulationoptions;

public:
    Pimpl(LogOptions         logoptions,
          SimulationOptions  simulationoptionts,
          std::ostream      *output)
        : m_out(output),
        m_logoptions(logoptions),
        m_simulationoptions(simulationoptionts)
    {
        if (m_simulationoptions & manager::SIMULATION_SPAWN_PROCESS)
            TraceAlways(
                _("Simulation: SIMULATION_SPAWN_PROCESS is not yet"
                  " implemented"));
    }

    value::Map * runTimeout(vpz::Vpz                   *vpz,
                            const utils::ModuleManager &modulemgr,
                            Error                      *error,
                            int                         time,
                            LogOptions         logoptions,
                            SimulationOptions  simulationoptions,
                            std::ostream      *output)
    {
	Timeout timer(time, vpz, modulemgr, error, logoptions,
		      simulationoptions, output);

        return timer.getResult();
    }

    value::Map * run(vpz::Vpz                   *vpz,
                     const utils::ModuleManager &modulemgr,
                     Error                      *error,
                     LogOptions                  logoptions,
                     SimulationOptions           simulationoptions,
                     std::ostream               *output)
    {
	return vle::manager::run(vpz, modulemgr, error, logoptions,
				 simulationoptions, output);
    }
};

Simulation::Simulation(LogOptions         logoptions,
                       SimulationOptions  simulationoptionts,
                       std::ostream      *output) :
    mPimpl(new Simulation::Pimpl(logoptions, simulationoptionts, output))
{
}

Simulation::~Simulation()
{
    delete mPimpl;
}

value::Map * Simulation::run(vpz::Vpz                   *vpz,
                             const utils::ModuleManager &modulemgr,
                             Error                      *error)
{
    return Simulation::run(vpz, modulemgr, error, -1);
}

value::Map * Simulation::run(vpz::Vpz                   *vpz,
                             const utils::ModuleManager &modulemgr,
                             Error                      *error,
                             int 		  	 timeout)
{
    error->code 	   = 0;

    if (timeout >= 0) {
        return mPimpl->runTimeout(vpz, modulemgr, error, timeout,
                                  mPimpl->m_logoptions,
                                  mPimpl->m_simulationoptions, mPimpl->m_out);
    } else {
        return mPimpl->run(vpz, modulemgr, error,mPimpl->m_logoptions,
                           mPimpl->m_simulationoptions, mPimpl->m_out);
    }
}

}}
