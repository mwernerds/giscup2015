/*
(c) 2017 M. Werner

Minimal python library for fast shortest path calculations using GISCUP2015 C++ 
code.

Look at the array GiscupMethods in this file to see the exported functions.

*/


#include<Python.h>
#include<iostream>
#define GISCUP_NO_OPENGL
#define NO_STAT_OUTPUT
#include "../giscup.hpp"


using namespace std;

NewShapeCollection ds; // short for dataset
VariousSearchEngines se; // search engine


// Load a dataset from a Shape File.
static PyObject *
giscup15_LoadDirectory(PyObject *self, PyObject *args)
{
    const char *directory;
    int sts;

    if (!PyArg_ParseTuple(args, "s", &directory))
        return NULL;
    ds.LoadDirectory(directory);
    se.createFromSimplifiedShapeCollection(ds);
    se.search_algorithm = SEARCH_ALGORITHM_EXPLICIT_AStar;
    cout << "Set default search engine: "<< SearchAlgorithmNames[se.search_algorithm] << endl;
  
    return Py_BuildValue("i", 0);
}


static PyObject *
giscup15_preprocess_ALT(PyObject *self, PyObject *args)
{
    int sts;

    if (!PyArg_ParseTuple(args, "i", &sts))
        return NULL;
    se.preprocess_ALT(sts);
    cout << "Feasible: " << se.localCheckFeasibility(se.random_v()) << endl;
    se.search_algorithm = SEARCH_ALGORITHM_ALT;
    cout << "Set search engine: "<< SearchAlgorithmNames[se.search_algorithm] << endl;
    return Py_BuildValue("i", 0);
}






static PyObject *
giscup15_random(PyObject *self, PyObject *args)
{
    int count;

    if (!PyArg_ParseTuple(args, "i", &count))
        return NULL;
    for (int i=0; i < count; i++)
    {
	auto v1 = se.random_v(); 
        auto v2 = se.random_v();
        bool found; double d;
        tie(found,d) = se.search(v1,v2);
        cout << "Found: " << found << " Distance: " << d << endl;   
    }
    return Py_BuildValue("i", 0);
}


static PyObject *
giscup15_NearestVertex(PyObject *self, PyObject *args)
{
   double x,y;
   if (!PyArg_ParseTuple(args, "dd", &x,&y))
        return NULL;
   
   auto junction = ds.nearestJunction(x,y);
   return Py_BuildValue("i",junction);
}

static PyObject *
giscup15_VertexCoordinates(PyObject *self, PyObject *args)
{
   int v;
   if (!PyArg_ParseTuple(args, "i", &v))
        return NULL;
   double x,y;
   if (v < 0 | v >= ds.junctions.size())
   {
      cout << "Invalid vertex" << endl;
      return Py_BuildValue("i",0);
   }
   x = ds.junctions[v][0][0];
   y = ds.junctions[v][0][1];
   
   return Py_BuildValue("dd",x,y);
}

static PyObject *
giscup15_Distance(PyObject *self, PyObject *args)
{
   int v1,v2;
   if (!PyArg_ParseTuple(args, "ii", &v1,&v2))
        return NULL;
   bool found; double d;
   tie(found,d) = se.search(v1,v2);
   return Py_BuildValue("d",d);
}


static PyObject *
giscup15_Shortest(PyObject *self, PyObject *args)
{
   int v1,v2;
   if (!PyArg_ParseTuple(args, "ii", &v1,&v2))
        return NULL;
   bool found; double d;
   tie(found,d) = se.search(v1,v2);
   if (found)
   {
     PyObject *list = PyList_New(0);
     for (auto &l : se.shortest_path)
     {
	  double x = se.locations[l].x;
	  double y = se.locations[l].y;
	  cout << l <<";" << x << "/" << y << endl;
	  PyList_Append(list,PyInt_FromSize_t(l));
	  
     }

    return list;
   }

   
   return Py_BuildValue("i",-1);
}



static PyMethodDef Giscup15Methods[] = {
 
    {"LoadDirectory",  giscup15_LoadDirectory, METH_VARARGS, "LoadDirectory loads road and node files from a directory assuming default names"},
    {"preprocess_ALT",  giscup15_preprocess_ALT, METH_VARARGS, "preprocess_ALT creates landmark set"},
    {"NearestVertex",  giscup15_NearestVertex, METH_VARARGS, "Returns the (Euclidean) nearest vertex for point X / Y"},
    {"VertexCoordinates",  giscup15_VertexCoordinates, METH_VARARGS, "Returns coordinates for vertex"},
    {"Distance",  giscup15_Distance, METH_VARARGS, "Returns coordinates for vertex"},
    {"Shortest",  giscup15_Shortest, METH_VARARGS, "Returns coordinates for vertex"},
    {"demo",  giscup15_random, METH_VARARGS, "Calculates some random shortest paths"}, 
    {NULL, NULL, 0, NULL}        /* Sentinel */
};


PyMODINIT_FUNC
initgiscup15(void)
{
    (void) Py_InitModule("giscup15", Giscup15Methods);
}
