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
#include<dslab.h>
#include<iostream>
#include<stdexcept>
#include<vector>
#include<limits>

#include <stdlib.h>
#include <string.h>
#include "shapefil.h"
using namespace std;

//#define USE_ORIGINAL_DATA
//#define STAT_OUTPUT_VERTICES

#define DEFAULT_START_VERTEX wxT("99165")
#define DEFAULT_GOAL_VERTEX wxT("289689")
#define DEFAULT_LANDMARK_COUNT wxT("5")


#include "giscup.hpp"


// calculate MBR
// left right top bottom
template<class collection> 
void get2DMBR(collection &coll, double &l, double &r, double &b, double &t)
{
		l = b = std::numeric_limits<double>::infinity();
		r = t = -std::numeric_limits<double>::infinity();
	for (auto it = coll.begin(); it != coll.end(); it++)
	{
		// *it is a trajectory
		for(auto it_time = (*it).begin(); it_time != (*it).end(); it_time ++)
		{
			double x = (*it_time)[0];
			double y = (*it_time)[1];
			if (x < l) l = x;
			if (x > r) r = x;
			if (y < b) b = y;
			if (y > t) t = y;
		}
	}
}



enum {
	MENU_ZOOMFIT=500,
	MENU_ZOOMIN,
	MENU_ZOOMOUT,
	MENU_VIEW_ROADS,
	MENU_VIEW_JUNCTIONS,
	MENU_VIEW_BARRIERS,
	MENU_VIEW_TURNS,
	MENU_VIEW_GRAPH,
	MENU_VIEW_SHORTEST_PATH,
	MENU_VIEW_LANDMARK,
	
	MENU_MODE_DEMO,
	
	MENU_CLEAR_RESTRICTIONS,
	MENU_RESTRICTIONS_LOADFILE,
		
	
	MENU_RANDOMPATH,
	MENU_ADDPOLY,
	MENU_APPLYRESTRICTION,
	MENU_SEARCHBYINDEX,
	MENU_PREPARE_ALT,
	
	MENU_CREATELANDMARK,
	MENU_SEARCHMODE_TIME,
	MENU_SEARCHMODE_DISTANCE,
	MENU_SEARCHMODE_TIME_WITHOUT_RESTRICTIONS,
	MENU_SEARCHMODE_DISTANCE_WITHOUT_RESTRICTIONS,
	
	MENU_MAX

	
};

enum {
	MOUSE_MODE_IGNORE,
	MOUSE_MODE_ADDPOLYGON_POINTS,
	MOUSE_MODE_LEFTRIGHT_SEARCH,
	MOUSE_MODE_CREATE_LANDMARK,
	MOUSE_MODE_MAX
};


static inline double _d(std::vector<double> a, std::pair<double,double> b)
{
	return sqrt((a[0]-b.first)*(a[0]-b.first) + (a[1]-b.second)*(a[1]-b.second));
}



class MyDataEngine :public DataEngine
{
	private:
	double theta[3];
    
    std::vector<size_t> thick_roads;

#ifndef USE_ORIGINAL_DATA    
    NewShapeCollection ds; // short for dataset
#else
 
    ShapeCollection ds;
#endif
    //GridShape<10,10> ds;
    VariousSearchEngines se;
    
    polycollection::value_type edit_poly;    
    
    bool view_roads;
    bool view_junctions;
    bool view_barriers;
    bool view_turns;
    bool view_graph;
    bool view_shortestpath;
    bool view_landmark;
    
    bool mode_demo;
    
    size_t mouse_mode;
        
    long elapsed;
    
	size_t w,h;
	double mbr[4];
	dsOrthoZoomPan zoompan;
	
	size_t  start;
	size_t goal;
	
	public:
	virtual std::string getTitle()
	{
		return std::string("GISCUP 2015 - Routing with Polygonal Constraints - Demo GUI");
	}
	virtual void Init()
	{
		mode_demo = false;
		mouse_mode = MOUSE_MODE_LEFTRIGHT_SEARCH;
		view_shortestpath=true;
		
#ifdef USE_ORIGINAL_DATA
		ds.LoadDirectory("data/LA3-UTM-Shp");
#else
		ds.LoadDirectory("data/giscup15data-shape");
#endif
		ds.LoadRestrictions("data/sfo_poly/sfo_poly");
		// Then create a reference search engine
		cout << "Restrictions complete " << endl;
#ifdef USE_ORIGINAL_DATA		
		se.createFromShapeCollection(ds);
#else
		se.createFromSimplifiedShapeCollection(ds);
#endif
		cout << "Data import complete..." << endl;
		
		ds.getMBR(mbr[0],mbr[1],mbr[2], mbr[3]);
		cout << "MBR: " << mbr[0] << " " << mbr[1] << " " << mbr[2] << " " << mbr[3]  << endl;
		zoompan.setMBR(mbr[0],mbr[1],mbr[2],mbr[3]);
		zoompan.zoomFit();
				
		/*	    wxToolBar *toolbar;
		toolbar = getFrame()->GetToolBar(); 
		toolbar->AddSeparator();
		toolbar->AddTool(MENU_ZOOMIN,wxT("Zoom In"),wxBitmap((const char *const *) &xpm_viewmagp));
		toolbar->AddTool(MENU_ZOOMOUT,wxT("Zoom Out"),wxBitmap((const char *const *) &xpm_viewmagm));
		toolbar->AddTool(MENU_ZOOMFIT,wxT("Zoom Fit"),wxBitmap((const char *const *) &xpm_viewmag1));
		toolbar->Realize();
		getFrame()->SetToolBar(toolbar);
		*/
		view_roads = false;
		view_graph = true;
		view_landmark = false;
		start = se.random_v();
		goal = se.random_v();
		se.search(start,goal);
		glEnable(GL_DEPTH_TEST);
		
	}
	virtual void think(double iElapsed)
	{
		zoompan.animate(iElapsed);
		if (mode_demo)
		{
			if ( (elapsed += iElapsed) > 2000)
			{
				elapsed = 0;
				cout << "Performing a random search" << endl;
				
				start = se.random_v();
				goal = se.random_v();
				se.search(start,goal);
			}
			
		}
		
	}
	virtual void createMenu(wxMenu *menuDataEngine)
	{
		wxMenu *algos = new wxMenu();
		  
		  for (size_t i=SEARCH_ALGORITHM_REFERENCE_AStar;
				i < SEARCH_ALGORITHM_MAX; i++)
			{
				wxString item(SearchAlgorithmNames[i].c_str(),wxConvUTF8);
				algos->Append(MENU_MAX+i,item);
		   
		}
		menuDataEngine->AppendSubMenu(algos, wxT("&Algorithm"));

		wxMenu *valuations = new wxMenu();
		valuations->Append(MENU_SEARCHMODE_DISTANCE,_("Distance"));
		valuations->Append(MENU_SEARCHMODE_TIME,_("Time"));
		valuations->Append(MENU_SEARCHMODE_DISTANCE_WITHOUT_RESTRICTIONS,_("Distance (without Constraints)"));
		valuations->Append(MENU_SEARCHMODE_TIME_WITHOUT_RESTRICTIONS,_("Time (without Constraints)"));
	  
		menuDataEngine->AppendSubMenu(valuations, wxT("&Search Mode"));
		
		wxMenu *restrictions = new wxMenu();
		restrictions->Append(MENU_ADDPOLY, _("Add a polygonal constraint"));
		restrictions->Append(MENU_RESTRICTIONS_LOADFILE,wxT("Load Restriction File"),wxT("Loads and applies restrictions from user file"));
		restrictions->Append(MENU_CLEAR_RESTRICTIONS,_("Clear all polygonal restrictions."));
		
		  
		menuDataEngine->AppendSubMenu(restrictions, wxT("&Constraints"));
		
		menuDataEngine->Append(MENU_RANDOMPATH,wxT("Random Path [A*]"));
		menuDataEngine->Append(MENU_MODE_DEMO,wxT("Perform Demo"));

		//menuDataEngine->Append(MENU_SEARCHBYINDEX,wxT("Search By Index"),wxT("Asks for vertex indizes to recreate a specific search"));
		menuDataEngine->Append(MENU_PREPARE_ALT,wxT("Prepare ALT landmarks"),wxT("Calculate Preprocessing Data for ALT"));
		menuDataEngine->Append(MENU_CREATELANDMARK,wxT("Create Landmark 0 by Click"),wxT("Create and Show Landmark 0"));
		
		
		
	}
	virtual void extendViewMenu(wxMenu *menuDataEngine)
	{
		menuDataEngine->Append(MENU_VIEW_ROADS,wxT("Roads"),wxT(""));
		menuDataEngine->Append(MENU_VIEW_JUNCTIONS,wxT("Junctions"),wxT(""));
		menuDataEngine->Append(MENU_VIEW_GRAPH,wxT("Graph"),wxT(""));
		menuDataEngine->Append(MENU_VIEW_SHORTEST_PATH,wxT("Shortest Path"),wxT(""));
		menuDataEngine->Append(MENU_VIEW_LANDMARK,wxT("Landmark Table 0"),wxT(""));
		
		DS_start_rendering();
	}
		
		
		virtual void handleMenu(int id)
		{
			wxString s_start,s_end,s_landmarks;
			long tmp_start, tmp_goal, tmp;
			wxFileDialog	openRestrictionsDialog(NULL, _("Open SHP file"), _(""), _(""),
											_("Shape files (*.SHP)|*.shp"), wxFD_OPEN|wxFD_FILE_MUST_EXIST);
			switch (id)
			{
				case MENU_ZOOMFIT:
					zoompan.zoomFitAnimated(); break;
				case MENU_ZOOMIN:
					zoompan.zoom(1); break;
				case MENU_ZOOMOUT:
					zoompan.zoom(-1); break;
				case MENU_VIEW_ROADS:
					view_roads = !view_roads; break;
				case MENU_VIEW_JUNCTIONS:
					view_junctions = !view_junctions; break;
				case MENU_VIEW_BARRIERS:
					view_barriers = !view_barriers; break;
				case MENU_VIEW_TURNS:
					view_turns = !view_turns; break;
				case MENU_VIEW_GRAPH:
					view_graph = !view_graph; break;
				case MENU_VIEW_SHORTEST_PATH:
					view_shortestpath = !view_shortestpath; break;
					case MENU_VIEW_LANDMARK:
						view_landmark = !view_landmark; break;
				case MENU_CREATELANDMARK:
					mouse_mode = MOUSE_MODE_CREATE_LANDMARK;
					break;
					
				case MENU_MODE_DEMO:
					mode_demo = !mode_demo; 
					break;
				case MENU_RANDOMPATH:
					cout << "Performing a random search" << endl;
				
					start = se.random_v();
					goal = se.random_v();
					se.search(start,goal);
				
					
					 break;
				case MENU_ADDPOLY:
						edit_poly.clear();
						mouse_mode = MOUSE_MODE_ADDPOLYGON_POINTS;
					break;
				case MENU_APPLYRESTRICTION:
						se.applyPolygonalRestrictions(ds.obstacles);
					break;
					
				case MENU_RESTRICTIONS_LOADFILE:
					
					if (openRestrictionsDialog.ShowModal() == wxID_CANCEL)
						return; 
					ds.LoadRestrictions(std::string(openRestrictionsDialog.GetPath().mb_str()));
					se.applyPolygonalRestrictions(ds.obstacles);
					se.search(se.lastStart,se.lastGoal);
					break;
					
				case MENU_CLEAR_RESTRICTIONS:
					cout << "Clearing Constraints resets metric to distance..." << endl;
					ds.obstacles.clear();
					se.activateDistance();
					se.applyPolygonalRestrictions(ds.obstacles);
					break;
				case MENU_SEARCHBYINDEX:
					
					s_start = wxGetTextFromUser(_("Start Vertex Index"),_("Search Parameter"),DEFAULT_START_VERTEX);
					s_end   = wxGetTextFromUser(_("End Vertex Index"),_("Search Parameter"),DEFAULT_GOAL_VERTEX);
					
					s_start.ToLong(&tmp_start);
					s_end.ToLong(&tmp_goal);
					cout << tmp_start << tmp_goal << "[" <<s_start << s_end << "]"<< endl;
					start = (mygraph_t::vertex_descriptor)tmp_start;
					goal = (mygraph_t::vertex_descriptor)tmp_goal;
					
					cout << "Searching " << start <<"==>" <<goal << endl;
					se.search(start,goal);
					break;
				case MENU_PREPARE_ALT:
					s_landmarks = wxGetTextFromUser(_("Landmark Count"),_("ALT Parameter"),DEFAULT_LANDMARK_COUNT);
					s_landmarks.ToLong(&tmp);
					se.preprocess_ALT((size_t) tmp);
					break;
					
				case MENU_SEARCHMODE_DISTANCE:
					se.activateDistance();						
					se.applyPolygonalRestrictions(ds.obstacles);
					se.search(se.lastStart,se.lastGoal);
					break;
				case MENU_SEARCHMODE_TIME:
					se.activateTime();
					se.applyPolygonalRestrictions(ds.obstacles);
					se.search(se.lastStart,se.lastGoal);
					break;
				case MENU_SEARCHMODE_DISTANCE_WITHOUT_RESTRICTIONS:
					se.activateDistance();						
					se.search(se.lastStart,se.lastGoal);
					break;
				case MENU_SEARCHMODE_TIME_WITHOUT_RESTRICTIONS:
					se.activateTime();						
					se.search(se.lastStart,se.lastGoal);
					break;
							
				
				default:
					if (id >=MENU_MAX && id < MENU_MAX + SEARCH_ALGORITHM_MAX)
					{
						auto a = id - MENU_MAX;
						cout<<  "Search Algorithm: " << SearchAlgorithmNames[a] << endl;
						se.search_algorithm = a;
						se.search(start,goal);
						break;
					}
				
					cout << "Menu Item " << id << " ignored" << endl;
					break;
			}
		}

		virtual void beforeQuit(){
			// Do some data cleanup / file sync / close
			cout << "Preparing DataEngine for exit" << endl;
		};
		
				
		virtual void render(size_t width, size_t height)
		{
			w = width; h = height;
		  zoompan.setViewport(0,0,width,height);
		  zoompan.glViewport();
		  
		  glMatrixMode( GL_PROJECTION );
		  glLoadIdentity( );
		  zoompan.glProject();
			glMatrixMode( GL_MODELVIEW );
			
			
			GLdouble 	 size = 0.75;
			if (mode_demo){
				glClearColor(0.7,0.7,0.9,0);
			}else{					
				glClearColor( 1,1,1,0 );
			}

			glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ); 
			glLineWidth(3);
			// First render turns behind roads (larger)
#ifdef USE_ORIGINAL_DATA
 			if (view_turns)
			{
				glColor3f(0.2,0.2,0.2); 
					
				for (auto it = ds.turns.begin(), end=ds.turns.end(); it != end; it++)
				{
					glBegin(GL_LINE_STRIP);//GL_LINE_STRIP
					for(auto p:(*it))
					   glVertex2f(p[0],p[1]);
					
					glEnd();
				}	
			}
#endif
			glLineWidth(1);
			glColor3f(0,0,0);
			if (view_roads)
			for (auto it = ds.roads.begin(); it != ds.roads.end(); it++)
			{
//				cout << (*it).size() << endl;
				glBegin(GL_LINE_STRIP);//GL_LINE_STRIP
				for(auto it_time = (*it).begin(); it_time != (*it).end(); it_time ++)
				{
		/*			GLfloat r = (*it_time)[3];
					GLfloat g = (*it_time)[4];
					GLfloat b = (*it_time)[5];*/
					
					GLfloat x = (*it_time)[0];
					GLfloat y = (*it_time)[1];
					//glColor3f(r,g,b);
					glVertex2f(x,y);
//					cout << x << "," << y << endl;
				}
				glEnd();
			}		
			
			/*Render Junctions*/
			glPointSize(5);
			if (view_junctions)
			{
				glColor3f(0,0,1);
				for (size_t i=0; i < ds.junctions.size(); i++)
			{
				glColor3f(0,0,1);
				
				glBegin(GL_POINTS);//GL_LINE_STRIP
				for(auto it_time = ds.junctions[i].begin(); it_time != ds.junctions[i].end(); it_time ++)
				{
					GLfloat x = (*it_time)[0];
					GLfloat y = (*it_time)[1];
					glVertex2f(x,y);
				}
				glEnd();
			}		
			}
			
			/*Render barriers*/
#ifdef USE_ORIGINAL_DATA

	glPointSize(10);
			if (view_barriers)
			{
				glColor3f(1,0,0);
				glBegin(GL_POINTS);
				for (size_t i=0; i < ds.barriers.size(); i++)
			{
				glVertex2f(ds.barriers[i][0],ds.barriers[i][1]);
			}		
				glEnd();
			}

#endif
			/*Render thick roads			 */
			 
			 glLineWidth(5);
			 for (size_t i=0; i < thick_roads.size(); i++)
			{
				glBegin(GL_LINE_STRIP);
				if (thick_roads[i] < ds.roads.size())
				{
					for (auto rp: ds.roads[thick_roads[i]])
					{
					glColor3f(0,0,1);
					glVertex2f(rp[0],rp[1]);
					}
					
				}
				glEnd();
			}
			
			/*Render Graph*/		
			glPointSize(7);
			glLineWidth(1);
			glColor3f(1,0,1);
			if (view_graph)
			{
				se.renderGL();
			}
			if (view_shortestpath)
			{
				se.renderShortest();
			}
			
			
			
			
			/*Transparent obstructive poly*/
			
			glLineWidth(1);
			glColor4f(0,0,0,1);
			for (auto &p: ds.obstacles)
			{
				glBegin(GL_LINE_STRIP);
				for (auto &q: p.outer())
					glVertex2f(q.get<0>(),q.get<1>());
				glEnd();
			}
			glColor4f(0.9,0.7,0.9,0.2);

			
			for (auto &p: ds.obstacles)
			{
				glBegin(GL_POLYGON);
				for (auto &q: p.outer())
					glVertex2f(q.get<0>(),q.get<1>());
				glEnd();
			}

			
				
			
			glLineWidth(2);
			glColor4f(0,0,0,1);
			glBegin(GL_POINTS);
			   for (auto p:edit_poly)
			      glVertex2f(p[0],p[1]);
			glEnd();

			glBegin(GL_LINE_STRIP);
			   for (auto p:edit_poly)
			      glVertex2f(p[0],p[1]);
			   if (edit_poly.size() > 0)
				glVertex2f(edit_poly[0][0],edit_poly[0][1]);
			glEnd();
			
			glColor4f(0.8,0.6,0.8,0.3);
			glBegin(GL_POLYGON);
			   for (auto p:edit_poly)
			      glVertex2f(p[0],p[1]);
			   if (edit_poly.size() > 0)
				glVertex2f(edit_poly[0][0],edit_poly[0][1]);
			glEnd();
			
			if (view_landmark) 
				se.renderLandmark(0);
			
						
		}
		
		
		
		
		virtual void mouseClick(size_t x, size_t y)
		{
			auto l = zoompan.clientProject(x,y);
			cout << "Projected @" << l.first << "," << l.second << endl;
			auto wgs = WebMercator2WGS84(l.second, l.first);
			cout << "WGS: "<< wgs.first << "\t" << wgs.second << endl;
			
			size_t v;
			switch(mouse_mode)
			{
				case MOUSE_MODE_ADDPOLYGON_POINTS:
					cout << "Adding a polygon point" << endl;
					edit_poly.push_back({l.first,l.second});
					if (edit_poly.size() == 1)		// add a floating point
						edit_poly.push_back({l.first,l.second});
					break;
				case MOUSE_MODE_CREATE_LANDMARK:
					 v = se.nearestVertex(l.first,l.second);
					cout << "Landmark at vertex " << v << endl;
					se.recreateLandmark(0, v);
					mouse_mode = MOUSE_MODE_IGNORE;
					break;
				
				case MOUSE_MODE_LEFTRIGHT_SEARCH:
						start = se.nearestVertex(l.first,l.second);
						cout << "Starting at " << start << endl;
						se.search(start,goal);
					break;
				
				
			}
		}
		
		virtual void rightClick(size_t x, size_t y)
		{
			auto l = zoompan.clientProject(x,y);
			switch(mouse_mode)
			{
				case MOUSE_MODE_LEFTRIGHT_SEARCH:
						goal = se.nearestVertex(l.first,l.second);
						cout << "Ending at " << goal << endl;
						se.search(start,goal);
					break;
			}
			
			
			if (mouse_mode == MOUSE_MODE_ADDPOLYGON_POINTS)
			{
				if (edit_poly.size() > 0)
					edit_poly.resize(edit_poly.size() -1);
				ds.addPolygonalObstacle(edit_poly);
				edit_poly.clear();
				mouse_mode = MOUSE_MODE_LEFTRIGHT_SEARCH;
			}			
		}
		
		virtual void mouseMoved(size_t x, size_t y)
		{
			auto l = zoompan.clientProject(x,y);
			if (mouse_mode == MOUSE_MODE_ADDPOLYGON_POINTS)
			{
				if (edit_poly.size() > 0)
				{
					edit_poly[edit_poly.size()-1] = {l.first,l.second};
				}
			}
		}
		
		virtual void mouseDragging(size_t from_x,size_t from_y, size_t to_x,size_t to_y)
		{
			auto from = zoompan.clientProject(from_x,from_y);
			auto to = zoompan.clientProject(to_x,to_y);
			zoompan.x -=  (to.first - from.first);
			zoompan.y -=  (to.second - from.second);
		}
		virtual void mouseWheel(size_t x, size_t y, int steps) {
			zoompan.fixZoom(x,y,steps);
			};
};

/*Instantiate it and make it available to the library*/
MyDataEngine eng;
DataEngine *getDataEngineImplementation()
{
   return (DataEngine*) &eng;
}

