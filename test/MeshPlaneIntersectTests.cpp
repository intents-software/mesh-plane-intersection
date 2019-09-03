#include "CppUnitTest.h"
#include "MeshPlaneIntersect.hpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
typedef MeshPlaneIntersect<double, int> Intersector;

namespace IntersectionTests
{
	TEST_CLASS(OpenMeshTests)
	{
		// Each of these tests run four intersections for each geometry:
		//   1) Intersect
		//   2) Clip
		//   3) Intersect with inverted plane
		//   4) Clip with inverted plane
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"1a There should be one polygon");
			Assert::AreEqual(2, (int)result[0].points.size(), L"1b The polygon should contain two points");
			Assert::IsFalse(result[0].isClosed, L"1c The polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"2a There should be one polygon");
			Assert::AreEqual(3, (int)result[0].points.size(), L"2b The polygon should contain three points");
			Assert::IsTrue(result[0].isClosed, L"2c The intersection polygon should be closed");

			plane.normal[2] = -1; // flip the intersection plane

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"3a There should be one polygon");
			Assert::AreEqual(2, (int)result[0].points.size(), L"3b The polygon should contain two points");
			Assert::IsFalse(result[0].isClosed, L"3c The polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"4a There should be one polygon");
			Assert::AreEqual(4, (int)result[0].points.size(), L"4b The polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"4c The intersection polygon should be closed");
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(0, (int)result.size(), L"1a There should be no intersection");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"2a There should be one polygon");
			Assert::AreEqual(3, (int)result[0].points.size(), L"2b The polygon should contain three points");
			Assert::IsTrue(result[0].isClosed, L"2c The polygon should be closed");

			plane.normal[2] = -1;

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(0, (int)result.size(), L"3a There should be no intersection");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(0, (int)result.size(), L"4a There should be no intersection");
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(0, (int)result.size(), L"1a There should be no intersection");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(0, (int)result.size(), L"2a There should be no intersection");
			
			plane.normal[2] = -1;

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(0, (int)result.size(), L"3a There should be no intersection");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"4a There should be one polygon");
			Assert::AreEqual(3, (int)result[0].points.size(), L"4b The polygon should contain three points");
			Assert::IsTrue(result[0].isClosed, L"4c The polygon should be closed");
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"1a There should be one polygon");
			Assert::AreEqual(4, (int)result[0].points.size(), L"1b The polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"1c The polygon should be closed");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"2a There should be one polygon");
			Assert::AreEqual(4, (int)result[0].points.size(), L"2b The polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"2c The polygon should be closed");

			plane.normal[2] = -1;

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"3a There should be one polygon");
			Assert::AreEqual(4, (int)result[0].points.size(), L"3b The polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"3c The polygon should be closed");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"4a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"4b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"4c The first polygon should be closed");
			Assert::AreEqual(4, (int)result[1].points.size(), L"4d The second polygon should contain four points");
			Assert::IsTrue(result[1].isClosed, L"4e The second polygon should be closed");
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"1a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"1b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"1c The first polygon should be closed");
			Assert::AreEqual(4, (int)result[1].points.size(), L"1d The second polygon should contain four points");
			Assert::IsTrue(result[1].isClosed, L"1e The second polygon should be closed");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"2a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"2b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"2c The first polygon should be closed");
			Assert::AreEqual(4, (int)result[1].points.size(), L"2d The second polygon should contain four points");
			Assert::IsTrue(result[1].isClosed, L"2e The second polygon should be closed");

			plane.normal[2] = -1;

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"3a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"3b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"3c The first polygon should be closed");
			Assert::AreEqual(4, (int)result[1].points.size(), L"3d The second polygon should contain four points");
			Assert::IsTrue(result[1].isClosed, L"3e The second polygon should be closed");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(3, (int)result.size(), L"4a There should be three polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"4b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"4c The first polygon should be closed");
			Assert::AreEqual(4, (int)result[1].points.size(), L"4d The second polygon should contain four points");
			Assert::IsTrue(result[1].isClosed, L"4e The second polygon should be closed");
			Assert::AreEqual(6, (int)result[2].points.size(), L"4d The third polygon should contain six points");
			Assert::IsTrue(result[2].isClosed, L"4e The third polygon should be closed");

		}

		TEST_METHOD(DoublePyramidMeshIncomplete1)
		{
			//  1     3     5
			//     6     7 x
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"1a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"1b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"1c The first polygon should be closed");
			Assert::AreEqual(4, (int)result[1].points.size(), L"1d The second polygon should contain four points");
			Assert::IsFalse(result[1].isClosed, L"1e The second polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"2a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"2b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"2c The first polygon should be closed");
			Assert::AreEqual(5, (int)result[1].points.size(), L"2d The second polygon should contain five points");
			Assert::IsTrue(result[1].isClosed, L"2e The second polygon should be closed");

			plane.normal[2] = -1;

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"3a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"3b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"3c The first polygon should be closed");
			Assert::AreEqual(4, (int)result[1].points.size(), L"3d The second polygon should contain four points");
			Assert::IsFalse(result[1].isClosed, L"3e The second polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"4a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"4b The first polygon should contain four points");
			Assert::IsTrue(result[0].isClosed, L"4c The first polygon should be closed");
			Assert::AreEqual(10, (int)result[1].points.size(), L"4b The second polygon should contain ten points");
			Assert::IsTrue(result[1].isClosed, L"4c The second polygon should be closed");
		}

		TEST_METHOD(DoublePyramidMeshIncomplete2)
		{
			//  1     3     5
			//     6 x   7 x
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"1a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"1b The first polygon should contain four points");
			Assert::IsFalse(result[0].isClosed, L"1c The first polygon should be open");
			Assert::AreEqual(4, (int)result[1].points.size(), L"1d The second polygon should contain four points");
			Assert::IsFalse(result[1].isClosed, L"1e The second polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"2a There should be two polygons");
			Assert::AreEqual(5, (int)result[0].points.size(), L"2b The first polygon should contain five points");
			Assert::IsTrue(result[0].isClosed, L"2c The first polygon should be closed");
			Assert::AreEqual(5, (int)result[1].points.size(), L"2d The second polygon should contain five points");
			Assert::IsTrue(result[1].isClosed, L"2e The second polygon should be closed");

			plane.normal[2] = -1;

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"3a There should be two polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"3b The first polygon should contain four points");
			Assert::IsFalse(result[0].isClosed, L"3c The first polygon should be open");
			Assert::AreEqual(4, (int)result[1].points.size(), L"3d The second polygon should contain four points");
			Assert::IsFalse(result[1].isClosed, L"3e The second polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"4a There should be one polygon");
			Assert::AreEqual(16, (int)result[0].points.size(), L"4b The first polygon should contain sixteen points");
		}

		TEST_METHOD(DoublePyramidMeshIncomplete3)
		{
			//  1     3     5
			//     6 x x 7 x
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

			auto result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(3, (int)result.size(), L"1a There should be three polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"1b The first polygon should contain four points");
			Assert::IsFalse(result[0].isClosed, L"1c The first polygon should be open");
			Assert::AreEqual(2, (int)result[1].points.size(), L"1d The second polygon should contain two points");
			Assert::IsFalse(result[1].isClosed, L"1e The second polygon should be open");
			Assert::AreEqual(2, (int)result[2].points.size(), L"1f The third polygon should contain two points");
			Assert::IsFalse(result[2].isClosed, L"1g The third polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(2, (int)result.size(), L"2a There should be two polygons");
			Assert::AreEqual(5, (int)result[0].points.size(), L"2b The first polygon should contain five points");
			Assert::IsTrue(result[0].isClosed, L"2c The first polygon should be closed");
			Assert::AreEqual(6, (int)result[1].points.size(), L"2d The second polygon should contain six points");
			Assert::IsTrue(result[1].isClosed, L"2e The second polygon should be closed");

			plane.normal[2] = -1;

			result = Intersector::Intersect(mesh, plane);
			Assert::AreEqual(3, (int)result.size(), L"3a There should be three polygons");
			Assert::AreEqual(4, (int)result[0].points.size(), L"3b The first polygon should contain four points");
			Assert::IsFalse(result[0].isClosed, L"3c The first polygon should be open");
			Assert::AreEqual(2, (int)result[1].points.size(), L"3d The second polygon should contain two points");
			Assert::IsFalse(result[1].isClosed, L"3e The second polygon should be open");
			Assert::AreEqual(2, (int)result[2].points.size(), L"3f The third polygon should contain two points");
			Assert::IsFalse(result[2].isClosed, L"3g The third polygon should be open");

			result = Intersector::Clip(mesh, plane);
			Assert::AreEqual(1, (int)result.size(), L"4a There should be one polygon");
			Assert::AreEqual(16, (int)result[0].points.size(), L"4b The first polygon should contain sixteen points");
		}
	};
}
