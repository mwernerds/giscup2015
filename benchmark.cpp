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
#define GISCUP_NO_OPENGL
#define NO_STAT_OUTPUT
#include "giscup.hpp"
#include<time.h>
#include <getopt.h>
#include<algorithm>
/*Some timing*/

clock_t gTime;
#define TIC() {gTime = clock();}
#define TOC() ((clock() -gTime) / (double) 1000000)


/*Data*/
NewShapeCollection ds; // short for dataset
/*Search Engine*/
VariousSearchEngines se;

/*Options*/

class tconfig
{
public:
	std::string filebase;
	std::string constraint;
	int numProblems;	
	bool store_path;
	tconfig():	filebase("./data/giscup15data-shape"),
				constraint(""),
				numProblems(10),
				store_path(false)
		
	{};
	void dump()
	{
		cout << "Benchmark settings:" << endl;
		cout << setw(20) << "Directory: " << setw(0) << filebase << endl;
		cout << setw(20) << "Constraint: " << setw(0) << constraint << endl;
		cout << setw(20) << "numProblems: " << setw(0) << numProblems << endl;
		cout << setw(20) << "storePath: " << setw(0) << store_path << endl;
		
		cout << endl;
	}
};


    //Specifying the expected options
    //The two options l and b expect numbers as argument
const char *opt_string="d:c:p:";
const struct option long_options[] = {
        {"data",      required_argument,       0,  'd' },
        {"constraint",      required_argument,       0,  'c' },
        {"num",      optional_argument,       0,  'p' },
        {"store-path", no_argument,0,'s'},
        {0,           0,                 0,  0   }
    };

   void print_usage()
   {
	   cout << "Usage"<< endl;
   }

int main(int argc, char **argv)
{
   /*The performance benchmark system*/
   tconfig cfg;
   int long_index =0;
   int opt;
    while ((opt = getopt_long(argc, argv,opt_string, 
                   long_options, &long_index )) != -1) {
        switch (opt) {
             case 'd' : cfg.filebase = std::string(optarg);
                 break;
             case 'c' : cfg.constraint = std::string(optarg);;
                 break;
             case 'p' : cfg.numProblems = atoi(optarg); 
                 break;
             case 's':
						cfg.store_path = true;
						break;
             default: print_usage(); 
                 exit(EXIT_FAILURE);
        }
     }
     cfg.dump();
    
     TIC()
     ds.LoadDirectory(cfg.filebase);
     if (cfg.constraint != "")
		ds.LoadRestrictions(cfg.constraint);
	se.createFromSimplifiedShapeCollection(ds);
	cout << "Done init in " << TOC() <<"s"<< endl;
	cout << "Now creating problem set" << endl;
    
    std::vector<std::pair<size_t,size_t>> problem_set;
	problem_set.clear();
	for (size_t i=0; i < cfg.numProblems; i++)
		problem_set.push_back(make_pair(se.random_v(),se.random_v()));
   
   se.preprocess_ALT(25);
   cout << "Feasible: " << se.localCheckFeasibility(se.random_v()) << endl;
      
   std::map<std::pair<int,std::pair<size_t,size_t>>, std::string> searches;
   double ela;
   
   cout << "Performance: " << endl;
   se.search_algorithm = SEARCH_ALGORITHM_Dijkstra;
   cout << SearchAlgorithmNames[se.search_algorithm] << endl;
   TIC();
   for (auto &p: problem_set)
   {
	   se.search(p.first,p.second);
	   if (cfg.store_path)
	      searches[make_pair(se.search_algorithm, p)] = se.default_stats();
		   //map.insert(make_pair(se.search_algorithm,se.shortest_path));
   }
   ela = TOC();
   cout << "Elapsed " << ela << "s" << endl;
   cout << "Approx. " << (double) cfg.numProblems/ela << " operations per second" << endl;
   
   
   cout << "Performance: " << endl;
   se.search_algorithm = SEARCH_ALGORITHM_EXPLICIT_AStar;
   cout << SearchAlgorithmNames[se.search_algorithm] << endl;
   TIC();
   for (auto &p: problem_set)
   {
	   se.search(p.first,p.second);
	   if (cfg.store_path)
	      searches[make_pair(se.search_algorithm, p)] = se.default_stats();
		   //map.insert(make_pair(se.search_algorithm,se.shortest_path));
   }
   
   ela = TOC();
   cout << "Elapsed " << ela << "s" << endl;
   cout << "Approx. " << (double) cfg.numProblems/ela << " operations per second" << endl;
   
   se.search_algorithm = SEARCH_ALGORITHM_ALT;
   cout << SearchAlgorithmNames[se.search_algorithm] << endl;
   TIC();
   for (auto &p: problem_set)
   {
	   se.search(p.first,p.second);
	   if (cfg.store_path)
	      searches[make_pair(se.search_algorithm, p)] = se.default_stats();
   }
   ela = TOC();
   cout << "Elapsed " << ela << "s" << endl;
   cout << "Approx. " << (double) cfg.numProblems/ela << " operations per second" << endl;

   if (cfg.store_path)
   {
			
		std::vector<std::string> output;
		for (auto result : searches)
		{
			std::stringstream ss("");
			ss	<< "[" << result.first.second.first <<","<< result.first.second.second 	<< "]" 
			<< result.first.first 
			<< "--" << result.second << endl;
			output.push_back(ss.str());
		}
		std::sort(output.begin(), output.end());
		for (auto &s:output)
			cout << s ;
		
   }
   
   
   return 0;
   
   
   /*

    TIC();
	ds.LoadDirectory("data/LA3-UTM-Shp");
	ref_search.createFromShapeCollection(ds);
	twoside.createFromShapeCollection(ds);
	
	
	// Create Problem
	
	
	
	
	
	
	
	
	while (true)
	{
		// First path Quality
		std::pair<bool,double> res;
		for (auto &p: problem_set)
		{
			SearchEngineHeader();
			TIC();
			res = ref_search.search(p.first,p.second);
			SearchEngineStats(ref_search);
			//cout << res.second << "\t" << TOC() << "\t";
			TIC();
			res = twoside.search(p.first,p.second);
			SearchEngineStats(ref_search);
			
			SearchEngineDelim();//cout << res.second << "\t" << TOC() << "\n";
		}

	}
	
	
	
	/*
	while(true)
	{
		cout << "Searching " << endl;
		auto s = ref_search.random_v();
		auto e = ref_search.random_v();
		ref_search.search(s,e);
		twoside.search(s,e);
	}*/
	
	
	/*
	while (true)
	{
		// new problem set
		TIC();
		for (auto &p: problem_set)
		{
			auto res = ref_search.search(p.first,p.second);
			cout << (res.first?"true":"false") << "\t" << res.second<<"\t";
		}
		cout << TOC() << "\n";
		TIC();
		for (auto &p: problem_set)
		{
			auto res = twoside.search(p.first,p.second);
			cout << (res.first?"true":"false") << "\t" << res.second<<"\t";
		}
		cout << TOC() << "\n\n";
	}*/
	
	
	
	
	
	
	return 0;
	
	
	
}


void SearchEngineHeader()
{
	cout << "start\tgoal\twhite\tgray\tblack\tN\td\n" << endl;
}
void SearchEngineDelim()
{
	cout << "-------------" << endl;
}
template<class SearchEngine>
void SearchEngineStats(SearchEngine &se)
{
		size_t white=0, gray=0, black=0;
		for (auto vp = vertices(se.g); vp.first != vp.second; ++vp.first)
		{
			if(se.colors[*vp.first] == Color::white()) white++;
			if(se.colors[*vp.first] == Color::gray())  gray++;
			if(se.colors[*vp.first] == Color::black())  black++;
		}
		cout  << se.lastStart <<"\t" << se.lastGoal <<"\t"
			  << white <<"\t" << gray << "\t" << black <<"\t" 
			  << se.shortest_path.size() << "\t" << se.d[se.lastGoal] <<endl;
}

