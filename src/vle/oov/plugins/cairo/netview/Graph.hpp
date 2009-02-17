/**
 * @file vle/oov/plugins/cairo/netview/Graph.hpp
 * @author The VLE Development Team
 */

/*
 * VLE Environment - the multimodeling and simulation environment
 * This file is a part of the VLE environment (http://vle.univ-littoral.fr)
 * Copyright (C) 2003 - 2009 The VLE Development Team
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


#ifndef VLE_OOV_PLUGINS_CAIRO_GRAPH_HPP
#define VLE_OOV_PLUGINS_CAIRO_GRAPH_HPP

#define BOOST_NO_HASH

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/gursoy_atun_layout.hpp>
#include <boost/random.hpp>
#include <boost/property_map.hpp>

#include <boost/algorithm/string/split.hpp>
#include <boost/tokenizer.hpp>
#include <vle/utils/Rand.hpp>

namespace vle { namespace oov { namespace plugin {


class Graph
{
private:
    /**
     * The strucutre for vertex properties
     */
    struct VertexProperty {
        std::string name;
        std::pair<double, double> position;
    };
    /**
    * Create a typedef for the directed
    * Graph type with std::vector as container
    * and the type of vertex properties
    */
    typedef boost::adjacency_list<boost::vecS, boost::vecS,
            boost::directedS, VertexProperty> G;

    /**
     * Typedef for the container of the positions of vertices
     * in the positioning algorithm
     */
    typedef std::map <unsigned int, boost::square_topology<
            boost::mt19937>::point_type> IndexMap;
    //
    ////
    //
    typedef std::map< std::string,
            std::pair<double,double> >::iterator pos_it;

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    typedef boost::graph_traits<G>::vertex_descriptor v_descriptor;

    typedef boost::graph_traits<G>::vertex_iterator vertex_it;

    typedef std::pair<vertex_it, vertex_it> p_vertex_it;

    typedef boost::graph_traits<G>::edge_descriptor e_descriptor;

    typedef std::pair<boost::graph_traits<G>::adjacency_iterator,
            boost::graph_traits<G>::adjacency_iterator> p_adjacency_it;

    typedef boost::graph_traits<G>::adjacency_iterator adjacency_it;

    typedef boost::graph_traits<G>::edge_iterator edge_it;

    typedef std::pair<edge_it, edge_it> p_edge_it;
    //
    ////
    //
    v_descriptor get_vertex_descriptor(const std::string &vertex);

    std::map< std::string, std::pair<double,double> > m_positions;

    IndexMap m_index_map;
    utils::Rand m_rand;

    G m_G;

public:
    Graph(const std::string &vertice_names,
          const std::string &matrix);

    ~Graph( ) { }

    inline unsigned int num_nodes(){ return num_vertices(m_G);  }

    void remove_all_edges(){
        p_edge_it p = edges(m_G);
        for (edge_it it = p.first; it != p.second; ++it)
            remove_edge(*it,m_G);
    }

    /**
     * @brief Add a new node to the graph
     * */
    inline void add_node(const std::string &name){
        v_descriptor vd = add_vertex(m_G);
        m_G[vd].name = name;
    }

    /**
     * @brief Remove a new node to the graph
     * */
    inline void remove_node(const std::string &name){
        v_descriptor vd = get_vertex_descriptor(name);
        remove_vertex(vd,m_G);
    }

    /**
     * @brief Add a new edge to the graph.
     * @param vertex the index of the vertex
     * @param out_edge the index of the in-vertex
     */
    void push_back_edge(const std::string &source, const std::string &out_edge);

    std::vector<std::string> get_all_vertice_names();

    /**
     * @brief use the Gursoy & Atun algorithm to compute vertices positions.
     * Vertices are postioned on a 2D surface.
     */
    void set_positions();

    void set_positions(const std::string &p);

    /**
     * @brief get index of out vertices of vertex i.
     * @return vector of index of vertices
     */
    std::vector<std::string> get_adjacent_vertices(const std::string &vertex);

    /**
     * @brief get all the positions.
     * @return the positions of vertices in a std::pair associated in a map with the
     * index of vertices as keys.
     */
    inline std::map< std::string, std::pair<double,double> >
    get_positions(){
        return m_positions;
    }
    /**
     * @brief get one position.
     * @param the vertex.
     * @return the positions of vertices in a std::pair associated in a map with the
     * vertices as keys.
     */
    inline std::pair<double,double>
    get_position(const std::string &vertex){
        return m_positions[vertex];
    }
    /**
     * @brief Set the vertex position.
     * @param vertex_indice, x position and y position on the surface
     */
    inline void set_position(const std::string &vertex,
                             const double x, const double y){
        m_positions[vertex].first  = x;
        m_positions[vertex].second = y;
    }

    inline void scale_positions(const double &scalex,
                                const double &scaley){
        for(pos_it it = m_positions.begin();
            it != m_positions.end(); ++it){
            it->second.first  *= scalex;
            it->second.second *= scaley;
        }
    }
};

}}} // vle::oov::plugin

#endif
