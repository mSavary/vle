/** 
 * @file vpz/NoVLE.cpp
 * @brief 
 * @author The vle Development Team
 * @date mer, 22 fév 2006 16:29:01 +0100
 */

/*
 * Copyright (C) 2006 - The vle Development Team
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <vle/vpz/NoVLE.hpp>
#include <vle/vpz/Translator.hpp>
#include <vle/utils/XML.hpp>
#include <vle/utils/Debug.hpp>
#include <vle/utils/Path.hpp>

namespace vle { namespace vpz {
    
using namespace vle::utils;

void NoVLE::write(std::ostream& out) const
{
    out << "<translator name=\"" << m_name << "\" "
        << "library=\"" << m_library << "\">"
        << m_data
        << "</translator>";
}

void NoVLE::setNoVLE(const std::string& library,
                     const std::string& data)
{
    m_data.assign(data);
    m_library.assign(library);
}

void NoVLE::callTranslator(Model& model, Dynamics& dynamics,
                           Experiment& experiment)
{
    xmlpp::DomParser* dom = new xmlpp::DomParser;
    try {
        dom->parse_memory(m_data);
    } catch (const xmlpp::exception& e) {
        Throw(utils::SaxParserError,
              boost::format("Translator XML for '%1%' problem: %2%\n") %
              m_name % e.what());
    }

    Glib::Module* module = translator();
    void* makeNewTranslator;

    bool symb = module->get_symbol("makeNewTranslator", makeNewTranslator);
    Assert(utils::SaxParserError, symb,
           boost::format("Error in '%1%', function 'makeNewTranslator' not "
                         "found '%2%'") % m_name%
           Glib::Module::get_last_error());

    Translator* call;
    call = ((Translator*(*)(xmlpp::Element*))(makeNewTranslator))
        (dom->get_document()->get_root_node());

    Assert(utils::ParseError, call,
           boost::format("Error in '%1%', function 'makeNewTranslator': "
                           "problem allocation a new Translator '%2%'") %
                           m_name % Glib::Module::get_last_error());

    call->translate();
    model.initFromModel(call->getStructure()->get_root_node());
    dynamics.initFromModels(call->getDynamics()->get_root_node());

    if (call->getExperiment())
        experiment.initFromExperiment(call->getExperiment()->get_root_node());

    delete call;
    delete module;
    delete dom;
}

Glib::Module* NoVLE::translator()
{
    std::string file = Glib::Module::build_path(
        utils::Path::path().getDefaultTranslatorDir(), m_library);

    Glib::Module* module = new Glib::Module(file);
    if (not (*module)) {
        std::string err(Glib::Module::get_last_error());
        delete module;

        std::string ufile = Glib::Module::build_path(
                utils::Path::path().getUserTranslatorDir(), m_library);
        
        module = new Glib::Module(ufile);
        if (not (*module)) {
            std::string err2(Glib::Module::get_last_error());
            delete module;

            Glib::ustring error((boost::format(
                        "Error opening plugin %1% or %2% translator %3% "
                        "with error:") % file % ufile % m_library).str());

            error += "\n";
            error += err;
            error += "\n";
            error += err2;

            Throw(utils::FileError, error);
        }
    }
    return module;
}

}} // namespace vle vpz
