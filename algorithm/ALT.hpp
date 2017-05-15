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
#ifndef ALT_HPP_INC
#define ALT_HPP_INC

#ifdef ALT_DEBUG
#define LATE(fmt, ...) {if(numRuns > 1E7) printf(fmt, ##__VA_ARGS__);}
#else
#define LATE(fmt, ...) {}
#define ALT_USE_TIGHTEST_SUBSET
#endif


/*! \brief landmark_t is a container for the data representing a landmark.
 * 
 * It contains two vectors d_from and d_to, which store the distance from and to 
 * each vertex of the graph. Additionally, the location of the landmark is given.
 * Some global variables in this file will be class-global for VariousSearchEngines. 
 * They will become public members of VariousSearchEngines class
 * 
 */
class landmark_t{
		public:
			std::vector<cost> d_from;
			std::vector<cost> d_to;
			mygraph_t::vertex_descriptor l;
		
		landmark_t(mygraph_t::vertex_descriptor _l):l(_l){};
};
	
	std::vector<landmark_t> landmarks;  ///< Store landmark data

	/*! \brief WebMercatorEdge is a helper function returning the length of an edge given by vertex descriptors
	 */
	double WebMercatorEdge(mygraph_t::vertex_descriptor s, mygraph_t::vertex_descriptor e)
	{
		return WebMercatorDistance(locations[s].y,locations[s].x,locations[e].y,locations[e].x);
	}
	
	/*! \brief recreateLandmark recalculates landmark which at where
	 */ 
	bool recreateLandmark(size_t which, size_t where)
	{
		while (landmarks.size() <= which)
		  landmarks.push_back(landmark_t(where));
		return createLandmark(which,where);
	}

/*! \brief cretaeLandmark calculates landmark with id which from vertex where 
 * 
 * 
 */
	bool createLandmark(size_t which, size_t where)
	{
		reverse_graph<mygraph_t> g_dual(g);   ///< the reverse graph to get from and to distances.
		if (which >= landmarks.size())
			return false;
		landmarks[which].l = where;
	    
	    landmarks[which].d_from.resize(num_vertices(g));
		landmarks[which].d_to.resize(num_vertices(g));
		
		// distance from by searching on graph
		dijkstra_shortest_paths(g,landmarks[which].l,predecessor_map(&p[0]).
								distance_map(&(landmarks[which].d_from[0])));
		// distance to landmark by searching on reverse_graph
		dijkstra_shortest_paths(g_dual,landmarks[which].l,predecessor_map(&p[0]).
					distance_map(&(landmarks[which].d_to[0])));
        return true;
	}
	
	

	

	/*! \brief localCheckFeasibility checks feasibility of landmark-induced potential on complete graph for a single vertex
	 * 
	 * Feasibility of a potential is essentially the consistency of the distance estimation. With feasibility, the algorithm
	 * finds the shortest path, without feasibility, this is not clear. 
	 * 
	 */
	bool localCheckFeasibility(mygraph_t::vertex_descriptor t)
	{
		// feasible if reduced length l_pi non-negative
		mygraph_t::edge_iterator oe, oe_end;
		for (tie(oe, oe_end) = edges(g); 
						oe != oe_end; 
						++oe)
		{
			double l = weightmap[*oe];
			auto v = source(*oe,g);
			auto w = target(*oe,g);
			double l_pi=weightmap[*oe] - pi(source(*oe,g),t) + pi(target(*oe,g),t);
			
			if (l_pi < 0 && -l_pi > 10E-6)
				return false;
		}
		return true;
	}
	

/*! \brief pi calculates the potential 
 * 
 * This is the central trick for ALT: Distance estimation is given as the maximum
 * of several estimates coming from triangle inequality. In order to have
 * ALT not depend on the number of landmarks, we use some selected landmarks from
 * landmark_selection if ALT_USE_TIGHTEST_SUBSET is true. Otherwise, we use all landmarks 
 * (will be slow...)
 * 
 */
	template<class vd>
	double pi(vd v, vd w)
    {
		// d(v,w) <= d(v)  -d (w) for distance to L
		double pi = 0;
		
#ifndef ALT_USE_TIGHTEST_SUBSET
		size_t l = landmarks.size();
		for (size_t i=0; i < l; i++)
		
#else
		for (auto i:landmark_selection)
#endif
		{
			pi = std::max(pi, landmarks[i].d_to[v]-landmarks[i].d_to[w] );
			pi = std::max(pi, landmarks[i].d_from[w]-landmarks[i].d_from[v] );
		}
		return pi;
	}
	
	std::vector<size_t> landmark_selection; ///< store landmark selection.
	
	
	/*! \brief select_tightest creates a landmark selection.
	 * 
	 * Note that the ALT implementation will crash if there are too few landmarks. 
	 * Checking the availability of landmarks would have been reasonable practice,
	 * however, would also slow down the invocation of the algorithm... 
	 * 
	 */
	template<class vd>
	double select_tightest(vd v, vd w, size_t num)
    {
		// Calculate true distance
		double d = WebMercatorEdge(v,w);
		std::vector<std::pair<double, size_t>> tightness;
		
		double pi=0;
		size_t l = landmarks.size();
		tightness.resize(l);		
		// Calculate tightness value for each landmark
		for (size_t i=0; i < l; i++)
		{
			pi = std::max(pi, landmarks[i].d_to[v]-landmarks[i].d_to[w] );
			pi = std::max(pi, landmarks[i].d_from[w]-landmarks[i].d_from[v] );
			tightness[i] = make_pair(-(pi / d),i);
		}
		// Sort for tightest landmarks
		std::sort(tightness.begin(), tightness.end());
		// Get selection from sort ordering...
		landmark_selection.resize(num);
		for (size_t i=0; i < num; i++)
		   landmark_selection[i] = tightness[i].second;
		return pi;
	}

/*! \brief preprocess_ALT creates landmark set 
 * 
 * Can make use of OpenMP parallelism while creating landmarks. Just compile
 * with -fopenmp to exploit all cores.
 * 
 */
	bool preprocess_ALT(size_t numLandmarks = 25)
	{
		p.resize(num_vertices(g));

		landmarks.clear();
		cout << "Generating "<< numLandmarks << " landmarks" << endl;
		for (size_t i=0; i< numLandmarks; i++)
		{
			landmarks.push_back(landmark_t(random_v()));
		}
		
		// And now search
		cout << "Preparing " << landmarks.size() << " landmarks" << endl;
		#pragma omp parallel for
		for (size_t i=0;  i < landmarks.size(); i++)
		{
			
			createLandmark(i,landmarks[i].l);
		}
		cout << "Preparation complete" << endl;
		return true;
	}
	
	
/*! \brief search_ALT performs ALT search
 * 
 * It is a slightly extended / modified version of the explicit A* implementation in
 * astar.hpp
 */ 
	std::pair<bool, double> search_ALT(size_t s, size_t end)
		{
			lastStart = s; lastGoal = end;
#ifdef __WXWINDOWS__ 			
			if (landmarks.size() < 3)
			{
				wxMessageBox(_("You cannot use ALT with fewer than three landmarks in this implementation..."));
				return std::pair<bool,double>(false,std::numeric_limits<double>::infinity());
			}
#endif //__WXWINDOWS__  
			select_tightest(s,end,3);
			/*
			 * Use the priority_queue and push element each time you 
			 * would like to update it. Accept the fact that you will
			 *  have useless entries in the queue. When popping the 
			 * top value, check if it contains the up-to-date value. 
			 * If not, ignore it and pop the next.
			 * 
			 * This way you delay the removal of the updated element 
			 * until it comes to the top. I noticed this approach being 
			 * used by top programmers realizing Dijkstra algorithm.
			 * */
			
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
			f[s] = pi(s,end);
			Q.push(make_pair(f[s],s));
			
			while(!Q.empty())
			{
				auto u = Q.top();
				Q.pop();
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
		DEFAULT_STATS("My ALT");
#endif

					return make_pair(true,d[end]);
				}
				
				for (tie(oe, oe_end) = out_edges(u.second,g); 
						oe != oe_end; 
						++oe)
				{
					STAT_EXAMINE();	
					auto v = target(*oe,g);
					double pi_v_end = pi(v,end);
					
					if (d[u.second] + 
						weightmap[*oe]
						+ pi_v_end < d[v] + pi_v_end)
					{
						d[v] = d[u.second] + weightmap[*oe];
						p[v] = u.second;
						f[v] = d[v]+pi_v_end; 
						if (colors[v] == Color::white())
						{
							// discover
							colors[v] = Color::gray();
							Q.push(make_pair(f[v],v));
						}else 
						if(colors[v] == Color::black())
						{
							// reopen should not be needed
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
DEFAULT_STATS("ALT");
#endif
		
    	return make_pair(false,std::numeric_limits<double>::infinity());	
		};
  	
  #endif
