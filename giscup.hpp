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

#ifndef GISCUP_HPP_INC
#define GISCUP_HPP_INC

/*
 * This is a header-only library containing all algortihms
 * and mechanisms for my GISCup 2015 contribution
 * 
 * It is kept as a header-only library in order not to have problems
 * with different modules compiled using different compiler parameters
 * and similar problems. Furthermore, this opens up the algorithms
 * to quick integration in GUI and console versions
 * 
 * Dependencies:
 * I tried to have as few dependencies as possible. However, the following
 * libraries are being used:
 * 
 * shapelib-1.3 
 * 			used to load the dataset
 * 			download from http://shapelib.maptools.org/, my version: 1.3.0
 * 			unpack somewhere, ./configure, make, sudo make install...
 * Boost 1.58.0
 * 			used to model the graph and the geometry
 * 			download from http://www.boost.org/users/download/
 * 			unpack to your home (or somewhere else)
 * 			update Makefile -I../boost_1_58 to match your location
 * 			the used boost libraries need not be compiled nor installed...
 * 
 * If you want to compile the GUI, you also need a working:
 * 
 * - wxWidgets with OpenGL
 * - OpenGL
 * - dslab
 * 			get from https://github.com/mwernerds/dslab
 * 			contribute to it
 * 
 * Additionally, the compiler must support modern C++ (C++11). I am 
 * compiling always with g++ -std=std++11...
 * 
 * 
 * This source code is structured as follows
 * 
 * Preamble:     Includes and global types
 * Section 0:    Configuration and Utilities
 * Section 1:    DataHandling
 * Section 1.1:     Geometry
 * Section 1.2:     Shape Data Types
 * Section 1.3:     Data Loading
 * Section 1.4:     The Document (ShapeCollection)
 * Section 2:    Graph Search
 * Section 2.1:     Graph Types
 * Section 2.2:     Implementation and Evaluation of Searches
 * Section 2.2.1:      Reference A Search
 * Section 2.2.2:      Explicit A Search
 * Section 2.2.3:      Explicit Dijkstra
 * Section 2.2.4:      Boost Dijkstra
 * Section 2.2.5:      ALT
 * Section 2.3:  OpenGL Rendering (Optional)
 * 
 * The actual algorithms have been put in header files in the algorithm
 * subfolder. 
 *
 * If you have any questions, don't hesitate to contact me at
 * 		<martin.werner@ifi.lmu.de>
 * 
 * If you have any contributions, just mail me a patch or put a pull request
 * on GitHub. It would be great, if we can join forces to cleanly implement
 * various other search algorithms in this setting...
 */
 
 
 /*************************************************
  * Preamble: Includes and global types           *
  *************************************************/
  
#include<iostream>
#include<iomanip>
#include<sstream>
#include<stdexcept>
#include<limits>
#include<cmath>

#include<vector>
#include<queue>
#include<map>


#include <boost/pending/queue.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/graph_utility.hpp>

#include <boost/random.hpp>


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/algorithms/distance.hpp> 

#include <boost/foreach.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <shapefil.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef bg::model::polygon<point, false, false> polygon; // ccw, open polygon
typedef bg::model::linestring<point> linestring;
typedef std::pair<box, unsigned> value;

using namespace std;




 /**************************************************
  * Section 0: Configuration and Utilities         *
  *************************************************/

/*! \brief satof converts an std::string to a double irrespective of the current locale in "C"-locale
 *         
 *
 *  Converting string to double is non-trivial as several languages (especially german) use a "," instead
 *  of a "." and vice versa. If this source is pulled from a system that actually uses german locale (or system locale)
 *  classical atof will fail with respect to shapefiles. This satof is a replacement explicitly setting C-locale in which
 *  "." and "," have the meaning of the English language
 */

double satof(std::string s)
{
	std::istringstream istr(s);
	istr.imbue(std::locale("C"));
	double ret;
	istr >> ret;
	return ret;
}

/*! \brief _DBFReadDoubleAttribute reads a double attribute from Shapefiles in "C"-locale
 *         
 * 
 * Reading the Double values fails in most locales. This function reads the value as a string and converts it using
 * our safe implementation satof. This function is used in loading Shapefile attributes.
 */

double _DBFReadDoubleAttribute(DBFHandle hDBF, int id,  int iField)
{
	const char *s = DBFReadStringAttribute(hDBF,id,iField);
	return satof(std::string(s));
}

/*! \brief removeExt removes the extension from a filename. 
 *         
 * The shapefile library expects shapefiles given without extension and refuses to load files
 * given the SHP extension.
 */

std::string removeExt( std::string const& filename )
{
    std::string::const_reverse_iterator pivot
            = std::find( filename.rbegin(), filename.rend(), '.' );
    return pivot == filename.rend()
        ? filename
        : std::string( filename.begin(), pivot.base() - 1 );
}

/*! \brief compares pairs of value & on the second argument
 *         
 * 
 * Implements the less relation for pairs depending only on the second
 * part of a pair. The pairs are fixed types value
 */

bool less_second  (const value& lhs, const value& rhs)
{ 
	return lhs.second<rhs.second;
}

 
 /**************************************************
  * Section 1: DataHandling                        *
  *************************************************/
 
 /**************************************************
  * Section 1.1: Geometry                          *
  *************************************************/

/*! \brief WebMercator2WGS84 transforms WebMercator to WGS84. 
 *         
 * 
 * Returns a std::pair(lat,lon) out of two distinct parammeters in mercator.
 */
std::pair<double,double> WebMercator2WGS84(double mercatorY_lat, double mercatorX_lon)
{
	// hacky way of doing it: Convert to WGS84 using some constants
	// and use haversine after that. This can be made faster and 
	// much much more reliable ;-)
		
    if (abs(mercatorX_lon) < 180 && abs(mercatorY_lat) < 90)
        return make_pair(std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity());

    if ((abs(mercatorX_lon) > 20037508.3427892) || (abs(mercatorY_lat) > 20037508.3427892))
        return make_pair(std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity());

     double x = mercatorX_lon;
     double y = mercatorY_lat;
     double  num3 = x / 6378137.0;
     double num4 = num3 * 57.295779513082323;
     double num5 = floor((num4 + 180.0) / 360.0);
     double num6 = num4 - (num5 * 360.0);
     double num7 = 1.5707963267948966 - (2.0 * atan(exp((-1.0 * y) / 6378137.0)));
     mercatorX_lon = num6;
	 mercatorY_lat = num7 * 57.295779513082323;
//	 cout << "WGS84: "<< mercatorX_lon << ";" << mercatorY_lat << endl;
     return (make_pair(mercatorY_lat,mercatorX_lon));
}


/*! \brief WebMercatorDistance: Calculate Distance between WebMercator Points
 *         
 * 
 * Return the distance between two WebMercatorCoordinate pairs by first
 * transforming to WGS84 and using well-known Haversine formula.
 */

template<typename coord>
double WebMercatorDistance(coord p1, coord p2, coord p3, coord p4 )
{
	double lon1, lon2, lat1, lat2;
	tie(lat1,lon1) = WebMercator2WGS84(p1,p2);
	tie(lat2,lon2) = WebMercator2WGS84(p3,p4);
	// convert to radians
	lat1 *= (M_PI / 180.0);
	lon1 *= (M_PI / 180.0);
	lat2 *= (M_PI / 180.0);
	lon2 *= (M_PI / 180.0);
	
	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;
	double a = sin(dlat/2)*sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlon/2)*sin(dlon/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	return 6371000.0 * c;
}

 
 /*************************************************
  * Section 1.2: Shape Data Types                 *
  *************************************************/
 
typedef std::vector<std::vector<double>> polyline;
typedef std::vector<polyline> polycollection;
typedef std::vector<std::vector<double>> pointcollection;

#include "codegen/NewRoadAttrib.hpp"
#include "codegen/NewNodeAttrib.hpp"

typedef std::vector<NewRoadAttrib> newroadattribcollection;
typedef std::vector<NewNodeAttrib> newnodeattribcollection;


 /*************************************************
  * Section 1.3: Data Loading                     *
  *************************************************/
/*! \brief shapename returns a string name of the SHP type of geometry field
 *         
 * 
 * Given a SHP type, returns a string representation of the geometry type
 * of the a specific entity.
 */
std::string shapename(size_t type)
{
	switch(type){
		case 0: return std::string("NULL"); 
		case 1: return std::string("POINT"); 
		case 3: return std::string("ARC"); 
		case 5: return std::string("POLYGON"); 
		case 8: return std::string("MULTIPOINT"); 
		default: return std::string("undefined");
	}
}

/*! \brief dbftype returns a string representation of the data type of a DBF field
 *         
 * 
 * Given a DBF database, the type of the field is returned as a string from the
 * type of a field's integer description.
 */

std::string dbftype(size_t type)
{
	switch(type){
		case 0: return std::string("FTString");
		case 1: return std::string("FTInteger");
		case 2: return std::string("FTDouble");
		case 3: return std::string("FTLogical");
		default: return std::string("FTundefined");
	}
}

/*! \brief handlePolyline adds a polyline to a collection from a SHP shape
 *         
 * 
 * Whenever a shape of is read, this function can be used to create a polyline
 * from the geometry in the SHPObject parameter and push it onto the collection
 * given as a parameter
 */
inline void handlePolyline(polycollection &coll, SHPObject *shape)
{
    polycollection::value_type thispline;
	for (size_t i=0; i < shape->nVertices; i++)
	{
		double x=0,y=0,z=0;
		if(shape->padfX != NULL) x = shape->padfX[i];
		if(shape->padfY != NULL) y = shape->padfY[i];
		if(shape->padfZ != NULL) z = shape->padfZ[i];
		thispline.push_back({x,y,z});
    	}
     	coll.push_back(thispline);
}

/*! \brief handlePoint adds a point to a collection from a SHP shape
 *         
 * This function reads point data from SHP file objects given by the
 * shape parameter. The resulting point is pushed to a collection in 3D (x,y,z)
 *  
 */
inline void handlePoint(pointcollection &coll, SHPObject *shape)
{
    pointcollection::value_type thispoint;
    if (shape->nVertices != 1)
    {
		cout << "Point object with more than one vertex. Only adding first" << endl;
	}
		
    if (shape->nVertices == 0)
    {
		cout << "Barrier object without point skipped" << endl;
		return;
	}
     
    double x=0,y=0,z=0;
	if(shape->padfX != NULL) x = shape->padfX[0];
	if(shape->padfY != NULL) y = shape->padfY[0];
	if(shape->padfZ != NULL) z = shape->padfZ[0];
	coll.push_back({x,y,z});
}



/*! \brief importSHP reads a shapefile into collections
 *         
 * 
 * This function imports a shapefile. The filename is given without the SHP suffix,
 * the collections must support push_back and brace expressions, attributecollections
 * must provide the interface as generated in codegen (checking file format), shapehandler 
 * should be either handlePoint for point geometry or handlePolyline for other geometry.
 * 
 * if VERY_VERBOSE is defined to one, all shapes are output to the given stream
 */

template<typename polylinecollection, typename attribcollection, typename outstream,
		 typename shapehandler>
void importSHP(std::string filename, polylinecollection &coll, 
						attribcollection &attribs,
						shapehandler handler,
						outstream &out = std::cerr, bool verbose=true
						)
{
	SHPHandle	hSHP;
	DBFHandle   hDBF;
    int		nShapeType, i, nEntities, nShpInFile;
    SHPObject	*shape;
    
    hSHP = SHPOpen( (filename+std::string(".shp")).c_str(), "rb" );
	hDBF = DBFOpen ((filename+std::string(".dbf")).c_str(), "rb");
    if(hSHP == NULL) throw(std::runtime_error("Unable to open Shapefile"));
    if(hDBF == NULL) throw(std::runtime_error("Unable to open DBF "));
    
    SHPGetInfo( hSHP, &nEntities, &nShapeType, NULL, NULL );
    if (verbose)
    out << "Importing file with " << nEntities << " entities"<<endl;
    
    size_t nDBFRecords = DBFGetRecordCount(hDBF );
    if (verbose)
    out << "Found a DBF with " << nDBFRecords << " entries" << endl;
    
    if (nEntities != nDBFRecords)
    {
		out << "Using DBF and SHP pair of different size: " << filename << endl;
	}    

    /*And the associated DBF information*/
    size_t fields = DBFGetFieldCount(hDBF);
    if (verbose){
        for (size_t i=0; i < fields; i++)
        {
			char name[20];
			size_t type = DBFGetFieldInfo(hDBF,i,name, NULL, NULL);
			cout << dbftype(type) << ":" << name << "\n";
		}
		cout << endl << endl;
	}

    // File Type Checking
    if (!attribcollection::value_type::assertCorrectFormat(hDBF)){
		cout << "Error: Wrong DBF format" << endl;
	}
    
    // File Loading loop
    for( i = 0; i < nEntities; i++ )
    {
        shape = SHPReadObject( hSHP, i );
        if (shape == NULL) throw(std::runtime_error("unable to read some shape"));
#if VERY_VERBOSE
        cout << "Found a shape" << shapename(shape->nSHPType) << "(" << shape->nSHPType <<")"<< endl;
        cout << "ID:" << shape->nShapeId << endl;
        cout << "numParts: " << shape->nParts << endl;
        cout << "numVertices:" << shape->nVertices << endl;
#endif
		handler(coll,shape);
        SHPDestroyObject ( shape );
        /*Read all attributes*/
		typename attribcollection::value_type attrib;
		attrib.readFromID(hDBF, i);
		if (verbose)
			attrib.dump(cout);
		attribs.push_back(attrib);
    }
    SHPClose( hSHP );
    DBFClose( hDBF);
    out << "Completed " << nEntities << " entities."<<endl;
}

/*! \brief importSHPOnlyGeometry
 *         
 * 
 * Reads an SHP file similar to importSHP, but does not read or parse
 * the associated DBF file. Currently used for polygonal avoidances, as their
 * attributes are actually never used.
 */

template<typename polylinecollection, typename outstream,
		 typename shapehandler>
void importSHPOnlyGeometry(std::string filename, polylinecollection &coll, 
						shapehandler handler,
						outstream &out = std::cerr, bool verbose=false
						)
{
	SHPHandle	hSHP;
	DBFHandle   hDBF;
    int		nShapeType, i, nEntities, nShpInFile;
    SHPObject	*shape;
    
    hSHP = SHPOpen( (filename+std::string(".shp")).c_str(), "rb" );
    if(hSHP == NULL) throw(std::runtime_error("Unable to open Shapefile"));
    
    SHPGetInfo( hSHP, &nEntities, &nShapeType, NULL, NULL );
    if (verbose)
    out << "Importing file with " << nEntities << " entities"<<endl;
   
    
    // File Loading loop
    for( i = 0; i < nEntities; i++ )
    {
        shape = SHPReadObject( hSHP, i );
        if (shape == NULL) throw(std::runtime_error("unable to read some shape"));
#if VERY_VERBOSE
        cout << "Found a shape" << shapename(shape->nSHPType) << "(" << shape->nSHPType <<")"<< endl;
        cout << "ID:" << shape->nShapeId << endl;
        cout << "numParts: " << shape->nParts << endl;
        cout << "numVertices:" << shape->nVertices << endl;
#endif
		handler(coll,shape);
        SHPDestroyObject ( shape );
    }
    SHPClose( hSHP );
    out << "Completed " << nEntities << " entities."<<endl;
}

/*! \brief Interface for importSHP for the case of polylines
 *         
 * 
 * simplifies calling importSHP.
 */

template<typename polylinecollection, typename attribcollection, typename outstream>
void importSHPPolylines(std::string filename, polylinecollection &coll, 
						attribcollection &attribs,
						outstream &out = std::cerr, bool verbose=false
						)
{
	
	importSHP(filename,coll,attribs, handlePolyline ,out,verbose);
}

/*! \brief Interface for importSHPOnlyGeometry for polylines
 *         
 * 
 * simplifies calling importSHP.
 */
template<typename polylinecollection,  typename outstream>
void importSHPPolylinesOnlyGeometry(std::string filename, polylinecollection &coll, 
						outstream &out = std::cerr, bool verbose=false
						)
{
	
	importSHPOnlyGeometry(filename,coll, handlePolyline ,out,verbose);
}
 
/*! \brief Interface for importSHP for the case of points
 *         
 * 
 * simplifies calling importSHP.
 */
template<typename collection, typename attribcollection, typename outstream>
void importSHPPoints(std::string filename, collection &coll, 
						attribcollection &attribs,
						outstream &out = std::cerr, bool verbose=false
						)
{
	
	importSHP(filename,coll,attribs, handlePoint ,out,verbose);
}



/*************************************************
  * Section 1.4: The Document (ShapeCollection)  *
  *************************************************/
 class NewShapeCollection
 {
	 public:
	 
	 polycollection roads; 						///< The road geometry as an STL vector
	 newroadattribcollection aroads;		    ///< The junction attributes
	 bgi::rtree< value, bgi::rstar<16, 4> > 
			roads_rtree;						///< An R* tree indexing roads
	 
	 polycollection junctions;					///< The junction geometry as an STL vector
	 newnodeattribcollection ajunctions;		///< The junction attributes
	 bgi::rtree< value, bgi::rstar<16, 4> > 
			junction_rtree;						///< An A* tree indexing junctions
	 
	 polycollection polygons;					///< The polygons from the constraint file, as STL vectors
	std::vector<polygon> obstacles; 			///< The polygons from the constraint file as boost::geometry polygons
	  
	 
	 public:
	 
/*! \brief addPolygonalObstacle
 *         
 * 
 * Adds a polygonal obstacle from the polycollection's valuetype (e.g., an STL 
 * vector<vector<double>> to a boost::geometry polygon (outer, the real polygon,
 * counter-clockwise oriented, and pushes it to obstacles.
 * 
 * In short: converts SHAPE data polygon to boost::geometry objects.
 */

		void addPolygonalObstacle(polycollection::value_type &in)
		{
			polygon p;
			for (auto &x: in)
			{
				p.outer().push_back(point(x[0],x[1]));
			}
			obstacles.push_back(p);
	/*		BOOST_FOREACH(polygon const& x, obstacles)
				std::cout << bg::wkt<polygon>(x) << std::endl;*/
		}
		
/*! \brief getMBR returns the bounding box of the junctinos
 *         
 * 
 * This method returns the bounding rectangle of junctions into
 * referenced parameters left, right, bottom and up. It is mainly
 * used for OpenGL GUI zoomFit() functionality
 * 
 */
		
		void getMBR(double &l, double &r, double &b, double &t)
		{
			l = b = std::numeric_limits<double>::infinity();
			r = t = -std::numeric_limits<double>::infinity();
			for (auto &j:junctions)
			{
					double x = j[0][0];
					double y = j[0][1];
					if (x < l) l = x;
					if (x > r) r = x;
					if (y < b) b = y;
					if (y > t) t = y;
			}
		}
	
/*! \brief fillSpatialIndizes()
 *         
 * 
 * Takes all roads and junctions and fills the repspective R-trees.
 * The roads are indexed as points, that is nearest point from any road is
 * directly answered as nearest predicate on road R-tree resulting in the 
 * road index to which the point belongs. Note that this is not actually nearest 
 * on the road given as a linestring.
 * 
 * Junctions are points and therefore obviously indexed as points.
 */

		void fillSpatialIndizes()
		{
			/*Fill roads by creating temporary polygons*/
			for (size_t i=0; i < roads.size(); i++)
			{
				 polygon poly;
				 for (auto p:roads[i])						
				{
					box b(point(p[0],p[1]),point(p[0],p[1]));
					roads_rtree.insert(std::make_pair(b,i));
				}
			}
			
			for (size_t i=0; i < junctions.size(); i++)
			{
				box b(point(junctions[i][0][0],junctions[i][0][1]),
					  point(junctions[i][0][0],junctions[i][0][1]));
					  
				junction_rtree.insert(std::make_pair(b, i));
			}
		}
		
		
				
/*! \brief nearestJunction returns the index of the nearest junction given coordinates
 *         
 * 
 * The nearest junction is calculated using Euclidean geometry. Mainly used to support 
 * mouse clicking in the GUI, therefore Euclidean geometry is what users expect. This method
 * interrupts with SEGFAULT, if junction_rtree should be empty.
 */
		size_t nearestJunction(double x, double y)
		{
			std::vector<value> result_n;
			junction_rtree.query(bgi::nearest(point(x, y), 1), std::back_inserter(result_n));
			return result_n[0].second;
		}
		


/*! \brief nearestRoads returns the index of the k nearest road points
 *         
 * 
 * This method interrupts with SEGFAULT, if roads_rtree should be empty.
 */
		std::vector<size_t> nearestRoads(double x, double y, size_t k) 
		{
			std::vector<value> result_n;
			roads_rtree.query(bgi::nearest(point(x, y), k), std::back_inserter(result_n));
			std::vector<size_t> ret;
			for (auto a : result_n) {
				ret.push_back(a.second);
			}
			return ret;			
		}
		
		/*std::vector<size_t> junctionsOnRoad(size_t road)
		{
			std::vector<size_t> ret;
			for (size_t i=0; i < roads[road].size(); i++)
			{
				auto junction=nearestJunction(roads[road][i][0],roads[road][i][1]);
				if (d(roads[road][i], junctions[junction][0]) < 0.1)
				{
					ret.push_back(junction);
				}
			}
			return ret;
		}
		bool is_junction(size_t road, size_t idx)
		{
			auto junction=nearestJunction(roads[road][idx][0],roads[road][idx][1]);
			return (d(roads[road][idx], junctions[junction][0]) < 0.1);
		}
		std::pair<size_t,size_t> find_next_junction(size_t road, size_t idx)
		{
			for (size_t i=idx; i < roads[road].size(); i++)
			{
				auto junction=nearestJunction(roads[road][i][0],roads[road][i][1]);
				if (d(roads[road][i], junctions[junction][0]) < 0.1)
				{
					return make_pair(junction,i); // junction index and road index
				}
			}
			return make_pair(junctions.size(),roads[road].size());
		}
		
		
		double roadWeight(size_t road, size_t start, size_t end, size_t valuation=VALUATION_DISTANCE)
		{
			double ret = 0;
			for (size_t i=start; i < end; i++)
			{
				switch(valuation){
					case VALUATION_DISTANCE:
						ret += d(roads[road][i],roads[road][i+1]);
						
						break;
					case VALUATION_TIME:
						
						break;
					default: 
						cout << "Unknown valuation" << endl;
				}
				
			}
			//cout << "#?" << d(roads[road][start],roads[road][end]) << endl;
			//cout << road <<"," << start<< "," << end <<"=="  << ret << endl;
			return ret;
		}
		*/
		
/*! \brief LoadFiles loads road and node from file names
 *         
 * 
 * First, loads given roads file (removing extension, if present)
 * Second, loads given junctions file (removing extension, if present)
 * Finally, creates spatial indizes
 */
		void LoadFiles(std::string roadfile, std::string junctionfile)
		{
			
			cout << "Step 1: Load roads    \t";
			importSHPPolylines(removeExt(roadfile), roads,aroads,cout);
			
			cout << "Step 2: Load junctions\t";
			importSHPPolylines(removeExt(junctionfile), junctions,ajunctions,cout);
			
			fillSpatialIndizes();
		}



/*! \brief LoadDirectory loads road and node files from a directory assuming default names
 *         
 * 
 * First, loads sfo_roads.* SHP and DBF file into roads and aroads,
 * Second, loads sfo_nodes.* into junctions and ajunctions
 * Finally, creates spatial indizes
 */
		void LoadDirectory(std::string filebase)
		{
			cout << "Loading from "<<filebase << endl;
			cout << "Step 1: Load roads    \t";
			importSHPPolylines((filebase + "/sfo_roads"), roads,aroads,cout);
			
			cout << "Step 2: Load junctions\t";
			importSHPPolylines((filebase + "/sfo_nodes"), junctions,ajunctions,cout);
			//cout << "Step 5: Spatial Indizes"<< endl;
			fillSpatialIndizes();
		}
		
		
/*! \brief LoadRestrictions
 *         
 * 
 * Loads a file (extension is removed, if it was given) containing polygonal
 * avoidance data and pushes these polygons using addPolygonalObstacles.
 * Note that it clears polygons, the polygon set last read from a file.
 * It does not remove polygons possibly contained in the obstacles geometry
 * file. Therefore, several files can be loaded after each other. Polygons 
 * always contains the polygons from the file last loaded and obstacles contains
 * all polygons that have been loaded or otherwise added to the document.
 */
		
		void LoadRestrictions(std::string fname)
		{
			polygons.clear();
			importSHPPolylinesOnlyGeometry(removeExt(fname), polygons,cout);
			cout << "Found " << polygons.size() << "polygons" << endl;
			for (auto &p: polygons)
				addPolygonalObstacle(p);
		}
		
		
		
		
 };

/**************************************************
  * Section 2: Graph Search                       *
  *************************************************/
/**************************************************
  * Section 2.1: Graph Types                      *
  *************************************************/

using namespace boost;

struct location
{
  double x,y;
};
typedef double cost;

 typedef adjacency_list<vecS, vecS, bidirectionalS, property<vertex_color_t,default_color_type>,
    property<edge_weight_t, cost, property<edge_index_t,size_t>> > mygraph_t;

 typedef adjacency_list<vecS, vecS, undirectedS, property<vertex_color_t,default_color_type>,
    property<edge_weight_t, cost, property<edge_index_t,size_t>> > mygraph_t_undirected;
 
  typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
  typedef property_map<mygraph_t, vertex_color_t>::type ColorMap;
  typedef color_traits<property_traits<ColorMap>::value_type> Color;
  typedef property_map<mygraph_t, edge_index_t>::type IndexMap;
  typedef mygraph_t::vertex_descriptor vertex;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef mygraph_t::vertex_iterator vertex_iterator;
  
  
  
  
  /***********************************************************
  * Section 2.2: Implementation and Evaluation of Searches *
  **********************************************************/

/*! \brief stats(...) returns statistics from the last run of an algorithm
 *         
 * 
 * It uses the  search variables given as arguments to create a json string
 * containing graph search performance relevant data. Note that this might
 * be wrong / uncallable on twosided variants of searches, as it only supports
 * one colormap. Some statistics will, however, be vaild (such as examines, 
 * distance, ...)
 */

  template<class Graph, class Colors, class PathDescriptor, class Distance>
std::string stats(std::string name, Graph &g, Colors &colors, PathDescriptor &shortest_path, Distance &d, 
			size_t lastStart, size_t lastGoal, double _d, size_t examines)
	{
		size_t white=0, gray=0, black=0;
		for (auto vp = vertices(g); vp.first != vp.second; ++vp.first)
		{
			if(colors[*vp.first] == Color::white()) white++;
			if(colors[*vp.first] == Color::gray())  gray++;
			if(colors[*vp.first] == Color::black())  black++;
		}
		
		
		boost::property_tree::ptree pt;
		pt.put ("algorithm", name);
		pt.put("examines",examines);
		pt.put ("white", white);
		pt.put ("gray", gray);
		pt.put ("black", black);
		pt.put("path_vertices",shortest_path.size());
		pt.put("distance", _d);
		pt.put("from", lastStart);
		pt.put("to", lastGoal);
#ifdef STAT_OUTPUT_VERTICES
		boost::property_tree::ptree vertex_list;
		auto it = shortest_path.begin();
		for (size_t i=0; i < shortest_path.size(); i++){
			stringstream s(""); s << i;
		   vertex_list.put(s.str(),*it);
		   it = std::next(it);
	   }
		//pt.put("vertices",vertex_list);
		pt.push_back(std::make_pair("vertices",vertex_list));
#endif		
		
		std::ostringstream buf; 
		write_json (buf, pt, false);
		return buf.str();
}


/*! \brief Outpus (cout) statistics with the most common global parameters
 *         
 * 
 * This function define is called from search engines to output statistics
 * after searching. This is usually conditional on a preprocessor configuration
 * flag.
 */

#define DEFAULT_STATS(x) {cout << stats (x,g, colors, shortest_path, d, lastStart, lastGoal,getDistance(),stat_examines) << endl;}


/*! \brief SearchAlgorithms
 *         
 * 
 * Constants to give identifications to the various search algorithm
 * implementations in this file.
 */

enum SearchAlgorithms 
{
	SEARCH_ALGORITHM_REFERENCE_AStar,
	SEARCH_ALGORITHM_REFERENCE_Dijkstra,
	SEARCH_ALGORITHM_EXPLICIT_AStar,
	SEARCH_ALGORITHM_Dijkstra,
	SEARCH_ALGORITHM_ALT,
	SEARCH_ALGORITHM_MAX
};
/*! \brief Names of the search algorithms as strings
 *         
 * 
 * These names have been modified to be flushleft into equal-length strings
 * in order to have more readable output.
 */

const std::vector<std::string> SearchAlgorithmNames=
{ 
	"Boost A*  ",
	"Boost Dijkstra",
	"Basic A*   ",
	"Basic Dijkstra      ",
	"ALT           "
};

/*! \brief STAT_EXAMINE()
 *         
 * 
 * Helper macro called on an examination of an edge. Updates class-global variable
 * stat_examines accordingly. For speed, you can redefine this to do nothing...
 */
#define STAT_EXAMINE() {stat_examines ++;}


/*! \brief VariousSearchEngines
 *         
 * 
 * Class containing search engine data and implementations.
 */

class VariousSearchEngines
{
	public:
	/*Types*/
	typedef std::pair<double,mygraph_t::vertex_descriptor> queue_element; ///< The type of a queue element, essentially a pair of priority and vertex index.
	struct queue_compare{
		bool operator()( queue_element const& lhs, queue_element  const& rhs){
			// we want a smallest queue
			return lhs.first >= rhs.first;
	}
	}; ///< a comparation functional to sort with smallest priority first
	typedef std::priority_queue<queue_element, 
								std::vector<queue_element>,
								queue_compare> queue_t; ///< The priority queue
	size_t search_algorithm; ///< The search algorithm to be used in next calls to search
	
	
	mygraph_t g;	///< The graph g
	std::vector<double> f;	///< the priorities in the queue, stored at vertices.
	WeightMap weightmap;    ///< the edge weightmap
	vector<mygraph_t::vertex_descriptor> p; ///< the default predecessor map
	vector<cost> d;							///< the default distance map
	vector<default_color_type> colors;		///< the default color map

	// for twosided search
	vector<mygraph_t::vertex_descriptor> p1,p2;
	vector<cost> f1,f2,d1,d2;
	vector<default_color_type> colors1,colors2;
		

	std::vector<double> Time; ///< the original weights for time mapped to edges over edge_index property represented by edgeidmap member variable
	std::vector<double> Length; ///< the original weights for length, use edgeid member to map edges to ids
	
	IndexMap edgeidmap;			///< we manually assign indizes to edges in order to use vectors sometimes. This transforms from edge_descriptors to indizes
	
	
	
	vector<unsigned long> vertex_ids; ///< Stores the vertex IDs as they had been modelled in the shapefiles. Might become useful when outputting shortest paths or initiating searches.
	
	std::vector<location> locations; ///< The vertex locations

	std::map<mygraph_t::edge_descriptor,linestring> ls_roads; ///< The road geometry (if needed) as a boost::geometry::linestring. 
	
	list<mygraph_t::vertex_descriptor> shortest_path; ///< the shortest path of the last search, empty if no path found
	mygraph_t::vertex_descriptor lastStart, lastGoal; ///< the last start vertex and end vertex

	 bgi::rtree< value, bgi::rstar<16, 4> > vertex_rtree; ///< an R*-tree on vertices. Used to transform quickly and to cleanup error in the data.
	
	size_t stat_examines;	///< a class-global counter for edge examinations, reset in search()
	
	 
	
/*! \brief returns the nearest vertex, if any
 *         
 * 
 * Uses R*-tree, crashes, if rtree is empty by acessing unavailable result
 */

		size_t nearestVertex(double x, double y) 
		{
			std::vector<value> result_n;
			vertex_rtree.query(bgi::nearest(point(x, y), 1), std::back_inserter(result_n));
			return result_n[0].second;
		}
	
/*! \brief returns the default stat string using class-global variables
 *         
 * 
 * Therefore calls stat and returns the JSON string containing all information relevant for the last search. Some statistics
 * might be outdated, if the algorithm does not employ the global variables colors, shortest_path, examines
 */
std::string default_stats()
{
	return stats (SearchAlgorithmNames[search_algorithm],g, colors, shortest_path, d, lastStart, lastGoal,getDistance(),stat_examines);
}

/*! \brief Constructor setting the default search algorithm to REFERENCE_AStar
 *         
 * 
 */
		
		VariousSearchEngines():search_algorithm(SEARCH_ALGORITHM_REFERENCE_AStar){};
		

/*! \brief calculates distance of shortest_path global variable and asserst, that all edges exist
 *         
 * 
 * Some searches don't return the distance, e.g., two-sided. So we recalculate
 * in this case
 */

		double getDistance()
		{
			// Some searches don't return the distance, e.g., two-sided. So we recalculate
			// in this case
			double d = 0;
				list<mygraph_t::vertex_descriptor>::iterator it,nx, it_end;
				it_end = shortest_path.end();
				for (it = shortest_path.begin(); it != it_end; it++)
				{
					nx = std::next(it);
					if (nx == it_end) break;
					edge_descriptor e; bool found;
					tie(e,found) = boost::edge( *it,*nx,g );
					if (!found){
						cout << "Edge of SP not found!" << *it <<"=>" << *nx << endl;
						return 0;
					}
					d += weightmap[e];
				}
				return d;
		}
		
/*! \brief calculates Time of shortest_path global variable and asserst, that all edges exist
 *         
 * 
 * Some searches don't return the distance, e.g., two-sided. So we recalculate
 * in this case
 */
		double getTime()
		{
			double d = 0;
				list<mygraph_t::vertex_descriptor>::iterator it,nx, it_end;
				it_end = shortest_path.end();
				for (it = shortest_path.begin(); it != it_end; it++)
				{
					nx = std::next(it);
					if (nx == it_end) break;
					edge_descriptor e; bool found;
					tie(e,found) = boost::edge( *it,*nx,g );
					if (!found){
						cout << "Edge of SP not found!" << *it <<"=>" << *nx << endl;
						return 0;
					}
					d += Time[edgeidmap[e]];//weightmap[e];
				}
				return d;
		}


	size_t g_edge_index;	///< global counter for the edge index
	
/*! \brief myadd_edge adds an edge to the graph and assigns all needed properties
 *         
 * 
 * Creates a new edge in the graph, sets edge_id and weights in edgeidmap and weightmap.
 */

	edge_descriptor myadd_edge(mygraph_t::vertex_descriptor s,mygraph_t::vertex_descriptor e,double w )
    {
			edge_descriptor ed;
			 
			bool inserted;
			tie(ed, inserted) = add_edge(s,e, g);
			if (!inserted)
			{
				throw(std::runtime_error("unable to insert edge"));
			}
			edgeidmap[ed] = g_edge_index ++;
			weightmap[ed] = w;
			return ed;
	}
	
	
/*! \brief Return vertex index from location
 *         
 * 
 * If there is already a vertex at given location (in Euclidean distance smaller epsilon)
 * return this index. Otherwise create a new vertex, create location data and add it to
 * the vertex rtree. This method speeds up reading graph data from the prescribed edge format
 * considerably and suppresses errors due to multiple nodes at the same location.
 */

	
	mygraph_t::vertex_descriptor getVertexIndexFromLocation(cost x, cost y, double eps=0.01)
	{
		// check, if the vertex is already modelled, otherwise add a vertex
		mygraph_t::vertex_iterator v,ve;
		// Query R tree for nearest 
		std::vector<value> result_n;
		vertex_rtree.query(bgi::nearest(point(x, y), 1), std::back_inserter(result_n));
		if (result_n.size() != 0)
		if (bg::distance(result_n[0].first, point(x,y)) < eps)
		{
			return result_n[0].second;
		}
		
		
		auto w = add_vertex(g);
		locations.push_back({x,y});
		box b(point(x,y),
			  point(x,y));
	    vertex_rtree.insert(std::make_pair(b,w));
		return w;
	}
	
	
/*! \brief Creates all data in the graph from a NewShapeCollection document
 *         
 * 
 * Therefore fills vertices, edges, property maps, Time and Weight maps 
 * ls_roads and spatial index on vertices. Furthermore allocates global
 * search variables (p,f,d,etc.) to the size of the novel graph.
 * Also records vertex_ids, which represent the SHP attribute ID field for 
 * each vertex. 
 */
	
	void createFromSimplifiedShapeCollection (NewShapeCollection &shapes)
	{
		g.clear();

		
		std::vector<location> tmp_locations;
	
		vertex_ids.resize(shapes.ajunctions.size());
		for (size_t i=0; i < shapes.ajunctions.size(); i++)
		{
			vertex_ids[i] = shapes.ajunctions[i].ID;
		}
		
		// Add the location
		tmp_locations.resize(shapes.junctions.size());
		for (size_t i=0; i < shapes.junctions.size(); i++)
		  tmp_locations[i] = {shapes.junctions[i][0][0],shapes.junctions[i][0][1]};
		
		// now vertex_ids can be used to find the location in tmp_locations
		
		
		weightmap = get(edge_weight, g);
		edgeidmap = get(edge_index, g);

		Time.resize(shapes.aroads.size());
		Length.resize(shapes.aroads.size());
		
		g_edge_index = 0;		
		for (size_t i=0; i < shapes.aroads.size(); i++)
		{
			auto &r = shapes.aroads[i];
			//cout << r.EDGEID << ":" << r.STARTID << "=>" << r.ENDID << " (" << r.LENGTH  <<";" << r.SPD << ")"<< endl;
			auto start = std::find(vertex_ids.begin(),vertex_ids.end(),r.STARTID);
			auto end = std::find(vertex_ids.begin(),vertex_ids.end(),r.ENDID);
			
			if (start == vertex_ids.end() || end == vertex_ids.end())
			{
				cerr << "Dataset contains edge without a vertex" << endl;
				 throw(std::runtime_error("Dataset contains edge without a vertex\n"));
		    }
		  
		    location l1,l2;
		    l1 = tmp_locations[std::distance(vertex_ids.begin(),start)];
		    l2 = tmp_locations[std::distance(vertex_ids.begin(),end)];
		    auto v1 = getVertexIndexFromLocation(l1.x,l1.y);
		    auto v2 = getVertexIndexFromLocation(l2.x,l2.y);
		    
		    auto id = g_edge_index;
			auto e = myadd_edge(v1,v2,r.LENGTH); 
			
			// store the complete road
			linestring road;
			for (auto it = shapes.roads[i].begin(), it_end = shapes.roads[i].end(); 
				  it != it_end; ++it )
			{
				road.push_back(point((*it)[0],(*it)[1]));
			}
			ls_roads[e] = road;
			
			
			//auto id = num_edges(g)-1;
			// check that this was unique
			start ++;
			end ++;

			Length[id] = r.LENGTH;
			Time[id] = r.LENGTH / r.SPD;
		}
		
		
		
		/*removed_edges.resize(num_edges(g));
		for (size_t i=0; i < removed_edges.size(); i++) 
			remove_map[i] = false;*/
			
		// add all vertices
		cout << "Built a graph." << endl;
		cout << "Vertices: " << num_vertices(g) <<", Edges: " << num_edges(g) << endl;
		/*Allocate all properties*/
		f.resize(num_vertices(g));
		p.resize(num_vertices(g));
		d.resize(num_vertices(g));
		colors.resize(num_vertices(g));;
		
	}
	
/*! \brief activateTime pulls time information into the graph weightmap
 *         
 * 
 * After that, each search uses clean Time information from the Shapefile, without
 * polygonal restrictions in place. See and use applyPolygonalRestrictions for that
 * after calling this function.
 */
	void activateTime()
	{
		mygraph_t::edge_iterator oe, oe_end;
		for(tie(oe,oe_end) = edges(g); oe != oe_end; ++oe)
		  weightmap[*oe] = Time[edgeidmap[*oe]];
	}
	
	
	
/*! \brief activateDistance pulls distance information into the graph weightmap
 *         
 * 
 * After that, each search uses clean Length information from the Shapefile, without
 * polygonal restrictions in place. See and use applyPolygonalRestrictions for that
 * after calling this function.
 */
	void activateDistance()
	{
		mygraph_t::edge_iterator oe, oe_end;
		for(tie(oe,oe_end) = edges(g); oe != oe_end; ++oe)
		  weightmap[*oe] = Length[edgeidmap[*oe]];
	}

/*! \brief applyPolygonalRestrictions enforces restrictions from polygon set
 *         
 * 
 * This function checks each edge road linestring
 * and restriction polygon for an intersection and sets weightmap to 
 * infinity for those cases. Should be called after activateDistance or activateTime
 * Should only be used occasionally as it iterates over ALL edges and ALL polygons.
 * But, obstructions should change seldom enough to do it this way.
 */
		
	void applyPolygonalRestrictions(std::vector<polygon> &restrictions)
	{
		
		mygraph_t::edge_iterator oe, oe_end;
		for(tie(oe,oe_end) = edges(g); oe != oe_end; oe ++)
		{
			for (auto &r: restrictions)
			{
				if (bg::intersects(r,ls_roads[*oe]))
				{
					weightmap [*oe] = std::numeric_limits<cost>::infinity();
				}
			}
			
		}
	}
	 
	
/*! \brief Utility to return a random vertex
 *         
 * 
 * Returns a random vertex using boost generator mt19937. This function would
 * be problematic with Microsoft compilers, which do not fully support local
 * statics. However, you could remove the static declaration for the generator.
 */
	mygraph_t::vertex_descriptor random_v()
	{
		static boost::mt19937 gen(time(0)); 
		return random_vertex(g, gen);
	}
	
/*! \brief search using the given algorithm
 *         
 * 
 * cleans stat_examines and shortest_path and then calls the appropriate search
 * algorithm in the algorithms/* header.
 * 
 */
	
	
	std::pair<bool, double> search(size_t start, size_t end)
	{
		std::pair<bool,double> ret;
		stat_examines = 0;
		shortest_path.clear();	
		
		switch(search_algorithm)
		{
			case SEARCH_ALGORITHM_REFERENCE_AStar:
				return search_reference(start,end); 
			case SEARCH_ALGORITHM_REFERENCE_Dijkstra:
				return search_boostdijkstra(start,end);
			case SEARCH_ALGORITHM_EXPLICIT_AStar:
				return search_explicit(start,end);
			case SEARCH_ALGORITHM_Dijkstra:
				return search_dijkstra(start,end);
			case SEARCH_ALGORITHM_ALT:
				return search_ALT(start,end);
			
		};
		cout << "Invalid Search Algorithm" << endl;
		return make_pair(false,std::numeric_limits<double>::infinity());
	}
	
/*! \brief search using given algorithm and last parameters to search
 *         
 * 
 * Shorthand for search(lastStart,lastGoal)
 */

  std::pair<bool,double> search()
  {
	  return search(lastStart,lastGoal);
  }
  
  /**************************************************
  * Section 2.2.1: Reference A* Search               *
  *************************************************/
	
#include "algorithm/boostastar.hpp"
  	
  /**************************************************
  * Section 2.2.2: Explicit A* Search               *
  *************************************************/
#include "algorithm/astar.hpp"
	
  /**************************************************
  * Section 2.2.3: Explicit Dijkstra               *
  *************************************************/
#include "algorithm/dijkstra.hpp"
  
  /**************************************************
  * Section 2.2.4: Boost Dijkstra                    *
  *************************************************/
#include "algorithm/boostdijkstra.hpp"
	
  /**************************************************
  * Section 2.2.5: ALT 							*
  *************************************************/
#include "algorithm/ALT.hpp"



  /**************************************************
  * Section 2.3: OpenGL Rendering (Optional)     	*
  *************************************************/



/*! \brief renderGL renders the graph
 *         
 * 
 * Renders the graph (directed in form of central arrows) using OpenGL
 * If you don't have / need openGL, define GISCUP_NO_OPENGL
 */

	void renderGL(bool showDirected=true)
	{
#ifndef GISCUP_NO_OPENGL	
		//colors = get(vertex_color,g);
		glPointSize(2);
		glBegin(GL_POINTS);
		for (auto vp = vertices(g); vp.first != vp.second; ++vp.first)
		{
			glColor3f(0.5,0.5,0.5);
			
			//if(colors[*vp.first] == Color::white()) glColor3f(0.2,0.8,0.2);
			if(colors[*vp.first] == Color::white()) continue;
			if(colors[*vp.first] == Color::gray()) glColor3f(0.8,0.8,0.8);
			if(colors[*vp.first] == Color::black()) glColor3f(0,0,0);
			
			
			double x = locations[*vp.first].x;
			double y = locations[*vp.first].y;
			glVertex2f(x,y);
			//cout << "Putting a point" << x << "/" << y  << endl;
		}
		glEnd();
		
		
		
		glColor4f(0,0,1,0.2);
		glLineWidth(2);
		glBegin(GL_LINES);
		for (auto ed = edges(g); ed.first != ed.second; ++ed.first)
		{
			edge_descriptor e = *ed.first;
			double x =  locations[source(e,g)].x;
			double y =  locations[source(e,g)].y;
			double x2 = locations[target(e,g)].x;
			double y2 = locations[target(e,g)].y;
			glVertex2f(x,y);
			glVertex2f(x2,y2);
			if (showDirected){
					double dx = (x2-x);
					double dy = (y2-y);
					double len = sqrt (dx*dx+dy*dy);
					double glmpx = (x+x2) / 2;
					double glmpy = (y+y2) / 2;
		
					double glrpx = x + 0.4 *  dx;
					double glrpy = y + 0.4 *  dy;
					double glpfeil1_x = glrpx + 0.1 *(-dy);
					double glpfeil1_y = glrpy + 0.1 *(dx);
					double glpfeil2_x = glrpx - 0.1 *(-dy);
					double glpfeil2_y = glrpy - 0.1 *(dx);
					//	cout << glrpx << "," << glrpy<<"--"<<glpfeil1_x << "," << glpfeil1_y << endl;
					glVertex2f(glmpx,glmpy); glVertex2f(glpfeil1_x,glpfeil1_y);
					glVertex2f(glmpx,glmpy); glVertex2f(glpfeil2_x,glpfeil2_y);
					glVertex2f(glpfeil1_x,glpfeil1_y); glVertex2f(glpfeil2_x,glpfeil2_y);
		}
			
			//cout << "Putting a point" << x << "/" << y  << endl;
		}
		glEnd();
				glPointSize(2);
		glBegin(GL_POINTS);
		for (auto vp = vertices(g); vp.first != vp.second; ++vp.first)
		{
			glColor3f(0.5,0.5,0.5);
			
			//if(colors[*vp.first] == Color::white()) glColor3f(0.2,0.8,0.2);
			if(colors[*vp.first] == Color::white()) continue;
			if(colors[*vp.first] == Color::gray()) glColor3f(0.8,0.8,0.8);
			if(colors[*vp.first] == Color::black()) glColor3f(0,0,0);
			
			
			double x = locations[*vp.first].x;
			double y = locations[*vp.first].y;
			glVertex2f(x,y);
			//cout << "Putting a point" << x << "/" << y  << endl;
		}
		glEnd();

#endif
	}
	
/*! \brief renderShortest renders shortest path
 *         
 * 
 * This renders the shortest path in the graph 
 */

	
	void renderShortest()
	{
#ifndef GISCUP_NO_OPENGL
		glDisable(GL_BLEND);
		glLineWidth(50);
		glColor4f(1,0.2,0.2,1);
		glBegin(GL_LINE_STRIP);
		
		for (auto &l : shortest_path)
		{
			double x = locations[l].x;
			double y = locations[l].y;
			glVertex2f(x,y);
		}
		glEnd();
		glEnable(GL_POINT_SMOOTH);
		glPointSize(16);
		glColor3f(0,0,1);
		glBegin(GL_POINTS);
		glVertex2f(locations[lastStart].x,locations[lastStart].y);
		glEnd();
		glColor3f(0,1,0);
		glBegin(GL_POINTS);
		glVertex2f(locations[lastGoal].x,locations[lastGoal].y);
		glEnd();
		
		glDisable(GL_POINT_SMOOTH);
#endif
	}
	
#ifndef GISCUP_NO_OPENGL
/*! \brief glJet sets color according to value v in [v_min, v_max] using Jet colormap (blue ==> red)
 *         
 * 
 * 
 */

void glJet(double v, double vmin, double vmax, double alpha=1)
{
	double r=1, g =1, b = 1.0;
   double dv;
   if (v < 0)
   {
	  r = g = b = 1;
	  alpha = 0.1;
   }else{

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      r = 0;
      g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      r = 0;
      b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      r = 4 * (v - vmin - 0.5 * dv) / dv;
      b = 0;
   } else {
      g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      b = 0;
   }
	}
   glColor4f(r,g,b,alpha);
}


/*! \brief renderLandmark
 *         
 * 
 * renders a landmark distance map as a Jet colored set of points.
 */
	void renderLandmark(size_t which)
	{
		// first pass: max of landmark distance
		if (which >= landmarks.size() ) 
			return;
		
		double m = 0;
		for (size_t i=0; i < num_vertices(g); i++)
			if (landmarks[which].d_from[i] > m && landmarks[which].d_from[i] < 10E9)
				m = landmarks[which].d_from[i];
		glPointSize(4);
		glBegin(GL_POINTS);
		for (auto vp = vertices(g); vp.first != vp.second; ++vp.first)
		{
			glColor3f(0.5,0.5,0.5);
			
			if (landmarks[which].d_from[*vp.first] < std::numeric_limits<cost>::infinity())
			{
				glJet(landmarks[which].d_from[*vp.first],0,m);
			}else 
				continue;
			
			
			double x = locations[*vp.first].x;
			double y = locations[*vp.first].y;
			glVertex2f(x,y);
			
		}
		glEnd();
		
		
	}


#endif // OpenGL
};

#endif // header
