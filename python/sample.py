#
# Sample file for illustrating the python interface. 
# (c) 2017 M. Werner
#


import giscup15;
#
# Load a Directory, should contain two files with roads and junctions.
# Note that roads are directed...
#
giscup15.LoadDirectory("../data/giscup15data-shape/");

# 15 random distance queries (using A*)
giscup15.demo(15);

# Now, prepare ALT
giscup15.preprocess_ALT(25);
# 15 random distance queries, now faster as ALT removes distance calculation for table lookups 
# while keeping acceptable bound tightness
giscup15.demo(15)
#
# Note that the algorithms are intended for city-scale routing without much preprocessing time
# Otherwise, consider contraction hierarchies, which can be faster, but typically need more preprocessing...
#

x1 = -13627225.89691191;
y1 = 4539686.3750703428;
x2 = -13622691.809278855;
y2 = 4558742.5358461468;

#
# Using WebMercator coordinates (the Shapefiles are in WebMercator, right...)
#

v1 = giscup15.NearestVertex(x1,y1)
print(v1);
p = giscup15.VertexCoordinates(v1);
print(p);
print "Abs Error X/Y: %f %f "% ( (x1-p[0]),y1-p[1]);

#
# Distance and Shortest Queries
#
v2 = giscup15.NearestVertex(x2,y2);
# Distance is slightly faster than Shortest, as it does not transform the result for Python.
print("Distance: %f" % giscup15.Distance(v1,v2));
# Shortest returns a list of vertex indices
print ("Shortest:");
sp = giscup15.Shortest(v1,v2)
# A sample list expression to get the coordinates of shortest path vertices
sp_coords = [giscup15.VertexCoordinates(v) for v in sp];
print(sp);
print(sp_coords);

