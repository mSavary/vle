/**
 * @file vle/graph/ModelPortList.hpp
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


#ifndef VLE_GRAPH_MODELPORTLIST_HPP
#define VLE_GRAPH_MODELPORTLIST_HPP

#include <vle/graph/DllDefines.hpp>
#include <string>
#include <map>

namespace vle { namespace graph {

    class Model;

    class VLE_GRAPH_EXPORT ModelPortList
    {
    public:
        typedef std::multimap < Model*, std::string > value_type;
        typedef value_type::iterator iterator;
        typedef value_type::const_iterator const_iterator;
        typedef value_type::size_type size_type;

        ModelPortList()
        { }

        virtual ~ModelPortList();

        /**
         * @brief Add a new graph::ModelPort to the vector. No check is
         * performed is a connection already exist.
         *
         * @param model The model to add.
         * @param portname The port of the model to add.
         */
        void add(Model* model, const std::string& portname);

        /**
         * @brief Remove a graph::ModelPort from the vector. Linear complexity.
         *
         * @param model The model to delete.
         * @param portname The port of the model to delete.
         */
        void remove(Model* model, const std::string& portname);

        /**
         * @brief Remove the specified model.
         *
         * @param model Model to be removed.
         */
        void erase(Model* model) { m_lst.erase(model); }

        /**
         * @brief Remove all graph::ModelPort from the vector. Linear
         * complexity.
         */
        void clear() { m_lst.clear(); }

        /**
         * @brief Merge the vector from graph::ModelPort vector. Linera
         * complexity.
         *
         * @param lst The vector of graph::ModelPort to merger.
         */
        void merge(ModelPortList& lst);

        /**
         * @brief Check if a graph::ModelPort already exist in the vector.
         * Linear complexity.
         *
         * @param model The model to check.
         * @param portname The port of the model to check.
         *
         * @return True if the (model, portname) exist in vector, false
         * otherwise.
         */
        bool exist(Model* model, const std::string& portname) const;

        /**
         * @brief Check if a graph::ModelPort already exist in the vector.
         * Linear complexity.
         *
         * @param model The model to check.
         * @param portname The port of the model to check.
         *
         * @return True if the (model, portname) exist in vector, false
         * otherwise.
         */
        bool exist(const Model* model, const std::string& portname) const;

        inline iterator begin() { return m_lst.begin(); }
        inline const_iterator begin() const { return m_lst.begin(); }
        inline iterator end() { return m_lst.end(); }
        inline const_iterator end() const { return m_lst.end(); }
        inline size_type size() const { return m_lst.size(); }

    private:
        value_type m_lst;
    };

    std::ostream& operator<<(std::ostream& out, const ModelPortList& lst);

}} // namespace vle graph

#endif
