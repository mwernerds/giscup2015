/* Copyright 2015 Martin Werner - <martin.werner@ifi.lmu.de>
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
#ifndef ASTAR_HPP_INC
#define ASTAR_HPP_INC


/*! \brief search_explicit is a clean A* implementation 
 *         
 *  A trick is used in order not to update the elements of the queue. This pays
 *  off in road networks, where updates are seldom enough. Therefore, the last
 *  queue valuation of each vertex is recorded and a vertex from the priority
 *  queue is ignored, if it does not fit the stored last value...
 * 
 *  It uses the distance_heuristics from the boost_astar.hpp header for
 *  consistency.
 */

std::pair<bool, double> search_explicit(size_t s, size_t end)
		{
			lastStart = s; lastGoal = end;
			distance_heuristic<mygraph_t,float, std::vector<location>> 
				h(locations,end);
			
			//@TODO: Move somewhere else
			d.resize(num_vertices(g));
			colors.resize(num_vertices(g));
			p.resize(num_vertices(g));
			f.resize(num_vertices(g));		// The up to date priorities
						
			mygraph_t::vertex_iterator u, u_end;
			mygraph_t::out_edge_iterator oe, oe_end;

			// Initialize all data structures
			for (boost::tie(u, u_end) = vertices(g); u != u_end; ++u) 
			{
				d[*u] = std::numeric_limits<double>::infinity();
				colors[*u] = Color::white();
				p[*u] = *u;
			}
			// Start Search

			queue_t Q;
			colors[s] = Color::gray();
			d[s] = 0;
			f[s] = h(s);
			Q.push(make_pair(f[s],s));
			while(!Q.empty())
			{
				auto u = Q.top();
				Q.pop();
			/*
			 * Use the priority_queue and push element each time you 
			 * would like to update it. Accept the fact that you will
			 * have useless entries in the queue. When popping the 
			 * top value, check if it contains the up-to-date value. 
			 * If not, ignore it and pop the next.
			 * 
			 * This way you delay the removal of the updated element 
			 * until it comes to the top. 
			 * */

				if (u.first != f[u.second]) 
					continue;
				
				
				
				if (u.second == end)
				{
					shortest_path.clear();
					for(mygraph_t::vertex_descriptor v = end;; v = p[v]) {
						shortest_path.push_front(v);
						if(p[v] == v)
						break;
					}
#ifndef NO_STAT_OUTPUT
		DEFAULT_STATS("Basic A*");
#endif

					return make_pair(true,d[end]);
				}
				
				for (tie(oe, oe_end) = out_edges(u.second,g); 
						oe != oe_end; 
						++oe)
				{
					
					cost dst = weightmap[*oe]//dynamic_weightmap(*oe,g,weightmap, remove_map)
								+ d[u.second];
					auto v = target(*oe,g);
					STAT_EXAMINE();
					if ( dst < d[v])
					{
						d[v] = dst;
						p[v] = u.second;
						f[v] = dst+h(v);
						if (colors[v] == Color::white())
						{
							// discover
							colors[v] = Color::gray();
							Q.push(make_pair(f[v],v));
						}else 
						if(colors[v] == Color::black())
						{
							// reopen
							colors[v] = Color::gray();
							Q.push(make_pair(f[v],v));
						}else if(colors[v] == Color::gray())
						{
							// update in queue (we have found a shorter path)
							Q.push(make_pair(f[v],v));
														
						}
						
					}
				}// end for
				colors[u.second] = Color::black();
			}// while
#ifndef NO_STAT_OUTPUT
DEFAULT_STATS("Basic A*");
#endif
		
    	return make_pair(false,std::numeric_limits<double>::infinity());	
		};
  	
  #endif
