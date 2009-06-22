/**
 * @file vle/gvle/NewProjectBox.hpp
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

#ifndef GUI_NEWPROJECTBOX_HPP
#define GUI_NEWPROJECTBOX_HPP

#include <gtkmm.h>
#include <libglademm.h>
#include <vle/utils/Path.hpp>

namespace vle
{
namespace gvle {

class Modeling;

/**
 * @brief A window to create a package
 */
class NewProjectBox
{
public:

    NewProjectBox(Glib::RefPtr<Gnome::Glade::Xml> xml, Modeling* m);
    ~NewProjectBox();

    void show();

protected:

private:

    Glib::RefPtr<Gnome::Glade::Xml> mXml;

    Gtk::Dialog*                    mDialog;
    Modeling*                       mModeling;

    // Text Entry
    Gtk::Entry*                     mEntryName;

    // Buttons
    Gtk::Button*                    mButtonApply;
    Gtk::Button*                    mButtonCancel;

    void onApply();
    void onCancel();
    bool exist(std::string name);
};

}
}

#endif