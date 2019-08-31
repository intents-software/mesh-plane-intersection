#include "pch.h"
#include "CppUnitTest.h"
#include "../../../src/MeshPlaneIntersect.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
typedef MeshPlaneIntersect<double> Intersector;

namespace IntersectionTests
{
	TEST_CLASS(OpenMeshTests)
	{
	public:
		
		TEST_METHOD(SingleFaceCrossingPlane)
		{
			std::vector<Intersector::Vec3D> vertices{
				{1,0,-1},
				{3,0,1},
				{7,0,-1}
			};

			std::vector<Intersector::Face> faces{
				{0,1,2}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(1, (int)intersections.size(), L"There should be one polygon on the intersection");
			Assert::AreEqual(2, (int)intersections[0].points.size(), L"The intersection polygon should contain one segment");
			Assert::IsFalse(intersections[0].isClosed, L"The intersection polygon should be open");
		}

		TEST_METHOD(SingleFaceAbovePlane)
		{
			std::vector<Intersector::Vec3D> vertices{
				{1,0,-1},
				{3,0,1},
				{7,0,-1}
			};

			std::vector<Intersector::Face> faces{
				{0,1,2}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;
			plane.origin[2] = -2;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(0, (int)intersections.size(), L"There should be no intersection");
		}

		TEST_METHOD(SingleFaceBelowPlane)
		{
			std::vector<Intersector::Vec3D> vertices{
				{1,0,-1},
				{3,0,1},
				{7,0,-1}
			};

			std::vector<Intersector::Face> faces{
				{0,1,2}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;
			plane.origin[2] = 2;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(0, (int)intersections.size(), L"There should be no intersection");
		}

		TEST_METHOD(PyramidMesh)
		{
			//  3     2
			//     4
			//  0     1

			std::vector<Intersector::Vec3D> vertices{
				{-1,-1,-1},
				{1,-1,-1},
				{1,1,-1},
				{-1,1,-1},
				{0,0,1},
			};

			std::vector<Intersector::Face> faces{
				{0,1,4},
				{1,2,4},
				{2,3,4},
				{3,0,4}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(1, (int)intersections.size(), L"There should be one polygon on the intersection");
			Assert::AreEqual(4, (int)intersections[0].points.size(), L"The intersection polygon should contain four points");
			Assert::IsTrue(intersections[0].isClosed, L"The intersection polygon should be closed");
		}

		TEST_METHOD(DoublePyramidMesh)
		{
			//  1     3     5
			//     6     7
			//  0     2     4

			std::vector<Intersector::Vec3D> vertices{
				{-2,-1,-1},
				{-2,1,-1},
				{0,-1,-1},
				{0,1,-1},
				{2,-1,-1},
				{2,1,-1},
				{-1,0,1},
				{1,0,1}
			};

			std::vector<Intersector::Face> faces{
				{0,1,6},
				{1,3,6},
				{3,2,6},
				{2,0,6},
				{2,3,7},
				{3,5,7},
				{7,5,4},
				{2,7,4}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(2, (int)intersections.size(), L"There should be two polygons on the intersection");
			Assert::AreEqual(4, (int)intersections[0].points.size(), L"The first polygon should contain four points");
			Assert::IsTrue(intersections[0].isClosed, L"The first polygon should be closed");
			Assert::AreEqual(4, (int)intersections[1].points.size(), L"The second polygon should contain four points");
			Assert::IsTrue(intersections[1].isClosed, L"The second polygon should be closed");
		}

		TEST_METHOD(DoublePyramidMeshIncomplete1)
		{
			//  1     3     5
			//     6     7
			//  0     2     4

			std::vector<Intersector::Vec3D> vertices{
				{-2,-1,-1},
				{-2,1,-1},
				{0,-1,-1},
				{0,1,-1},
				{2,-1,-1},
				{2,1,-1},
				{-1,0,1},
				{1,0,1}
			};

			std::vector<Intersector::Face> faces{
				{0,1,6},
				{1,3,6},
				{3,2,6},
				{2,0,6},
				{2,3,7},
				{3,5,7},
				//{7,5,4}, missing face
				{2,7,4}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(2, (int)intersections.size(), L"There should be two polygons on the intersection");
			Assert::AreEqual(4, (int)intersections[0].points.size(), L"The first polygon should contain four points");
			Assert::IsTrue(intersections[0].isClosed, L"The first polygon should be closed");
			Assert::AreEqual(4, (int)intersections[1].points.size(), L"The second polygon should contain four points");
			Assert::IsFalse(intersections[1].isClosed, L"The second polygon should be open");
		}

		TEST_METHOD(DoublePyramidMeshIncomplete2)
		{
			//  1     3     5
			//     6     7
			//  0     2     4

			std::vector<Intersector::Vec3D> vertices{
				{-2,-1,-1},
				{-2,1,-1},
				{0,-1,-1},
				{0,1,-1},
				{2,-1,-1},
				{2,1,-1},
				{-1,0,1},
				{1,0,1}
			};

			std::vector<Intersector::Face> faces{
				{0,1,6},
				{1,3,6},
				//{3,2,6}, missing face
				{2,0,6},
				{2,3,7},
				{3,5,7},
				//{7,5,4}, missing face
				{2,7,4}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(2, (int)intersections.size(), L"There should be two polygons on the intersection");
			Assert::AreEqual(4, (int)intersections[0].points.size(), L"The first polygon should contain four points");
			Assert::IsFalse(intersections[0].isClosed, L"The first polygon should be open");
			Assert::AreEqual(4, (int)intersections[1].points.size(), L"The second polygon should contain four points");
			Assert::IsFalse(intersections[1].isClosed, L"The second polygon should be open");
		}

		TEST_METHOD(DoublePyramidMeshIncomplete3)
		{
			//  1     3     5
			//     6     7
			//  0     2     4

			std::vector<Intersector::Vec3D> vertices{
				{-2,-1,-1},
				{-2,1,-1},
				{0,-1,-1},
				{0,1,-1},
				{2,-1,-1},
				{2,1,-1},
				{-1,0,1},
				{1,0,1}
			};

			std::vector<Intersector::Face> faces{
				{0,1,6},
				{1,3,6},
				//{3,2,6}, missing face
				{2,0,6},
				//{2,3,7}, missing face
				{3,5,7},
				//{7,5,4}, missing face
				{2,7,4}
			};

			Intersector::Mesh mesh;
			mesh.vertices = &vertices;
			mesh.faces = &faces;

			Intersector::Plane plane;

			auto intersections = Intersector::Intersect(mesh, plane);

			Assert::AreEqual(3, (int)intersections.size(), L"There should be two polygons on the intersection");
			Assert::AreEqual(4, (int)intersections[0].points.size(), L"The first polygon should contain four points");
			Assert::IsFalse(intersections[0].isClosed, L"The first polygon should be open");
			Assert::AreEqual(2, (int)intersections[1].points.size(), L"The second polygon should contain two points");
			Assert::IsFalse(intersections[1].isClosed, L"The second polygon should be open");
			Assert::AreEqual(2, (int)intersections[2].points.size(), L"The third polygon should contain two points");
			Assert::IsFalse(intersections[2].isClosed, L"The third polygon should be open");
		}
	};
}
