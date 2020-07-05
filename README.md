# mesh-plane-intersection
Header-only C++ class for intersecting a triangulated mesh with a plane. It returnes the intersecting polylines (or closed polygons), avoiding the need to manually assemble each intersecting segment into polyline loops.

The class is templated to suit your required floating point coordinate type and integer index type. For example here we will use a typedef for easy reference:
```cpp
#include "MeshPlaneIntersect.hpp"
typedef MeshPlaneIntersect<double, int> Intersector;
```
We then create a mesh object object for the intersection. The mesh consists of vertices (points) in 3D space and a list of triangles (three point indices) that make up the faces.
```cpp
// create some example vertices
std::vector<Intersector::Vec3D> vertices{
    {1,0,-1},
    {3,0,1},
    {7,0,-1}
};

// create a face that links the first three vertices
std::vector<Intersector::Face> faces{
    {0,1,2}
};

// create the mesh object referencing the vertices and faces
Intersector::Mesh mesh(vertices, faces);
```
Finally, we need a plane to intersect with our mesh
```cpp
// by default the plane has an origin at (0,0,0) and normal (0,0,1)
// these can be modified, but we can also just use the default
Intersector::Plane plane;
```
To carry out the intersection we simply call:
```cpp
std::vector<Path3D> result = Intersector::Intersect(mesh, plane);
// the result is a vector of Path3D objects, which are planar polylines
// in 3D space. They have an additional attribute 'isClosed' to determine
// if the path forms a closed loop. if false, the path is open
// in this case we expect one, open Path3D with one segment
```
There is a further method "Clip" which, as well as returning the intersection polygons, also returns the polygons around any free edges on the positive side of the intersection plane.
```cpp
std::vector<Path3D> result = Intersector::Clip(mesh, plane);
// the result type is the same as above, but the polylines
// are not neccessarily planar.
// in this case we expect one, closed Path3D with three segments
```
There is a test project that verifies the result for a number of cases. If you come across an unexpected result then please let me know.