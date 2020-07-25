#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"
#include "MeshPlaneIntersect.hpp"

typedef MeshPlaneIntersect<double, int> Intersector;


SCENARIO("Open meshes intersections work as expected") {

	GIVEN("A mesh with a single face that crosses the intersection plane") {

		std::vector<Intersector::Vec3D> vertices{
			{1,0,-1},
			{3,0,1},
			{7,0,-1}
		};

		std::vector<Intersector::Face> faces{
			{0,1,2}
		};

		Intersector::Mesh mesh(vertices, faces);

		WHEN("the intersection plane normal is positive z") {
			Intersector::Plane plane;
			plane.normal = { 0, 0, 1 };

			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 2);
				REQUIRE(!result[0].isClosed);
			}

			WHEN("We clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 3);
				REQUIRE(result[0].isClosed);
			}
		}

		WHEN("the intersection plane normal is negative z") {
			Intersector::Plane plane;
			plane.normal = { 0, 0, -1 };

			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 2);
				REQUIRE(!result[0].isClosed);
			}

			WHEN("We clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
			}
		}
	}

	GIVEN("A mesh with a single face that doesn't intersect the plane")
	{
		std::vector<Intersector::Vec3D> vertices{
			{1,0,-1},
			{3,0,1},
			{7,0,-1}
		};

		std::vector<Intersector::Face> faces{
			{0,1,2}
		};

		Intersector::Mesh mesh(vertices, faces);

		WHEN("the face is above the intersection plane") {
			Intersector::Plane plane;
			plane.normal = { 0,0,1 };
			plane.origin = { 0,0,-2 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 0);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 3);
				REQUIRE(result[0].isClosed);
			}
		}

		WHEN("the face is below the intersection plane") {
			Intersector::Plane plane;
			plane.normal[2] = -1;
			plane.origin[2] = -2;
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 0);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 0);
			}
		}
	}

	GIVEN("a pyramid shaped mesh") {
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

		Intersector::Mesh mesh(vertices, faces);

		WHEN("the intersection plane normal is positive z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
			}
		}

		WHEN("the intersection plane normal is negative z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,-1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(result[1].isClosed);
			}
		}
	}

	GIVEN("a double pyramid mesh")
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

		Intersector::Mesh mesh(vertices, faces);

		WHEN("the intersection plane normal is positive z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(result[1].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(result[1].isClosed);
			}
		}
		WHEN("the intersection plane normal is negative z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,-1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(result[1].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 3);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(result[1].isClosed);
				REQUIRE(result[2].points.size() == 6);
				REQUIRE(result[2].isClosed);
			}
		}
	}

	GIVEN("a double pyramid mesh with one external face missing")
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

		Intersector::Mesh mesh(vertices, faces);

		WHEN("the intersection plane normal is positive z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(!result[1].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 5);
				REQUIRE(result[1].isClosed);
			}
		}
		WHEN("the intersection plane normal is negative z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,-1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(!result[1].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 10);
				REQUIRE(result[1].isClosed);
			}
		}
	}

	GIVEN("a double pyramid mesh with one external and one internal face missing")
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

		Intersector::Mesh mesh(vertices, faces);

		WHEN("the intersection plane normal is positive z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(!result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(!result[1].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 5);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 5);
				REQUIRE(result[1].isClosed);
			}
		}
		WHEN("the intersection plane normal is negative z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,-1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(!result[0].isClosed);
				REQUIRE(result[1].points.size() == 4);
				REQUIRE(!result[1].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 16);
			}
		}
	}

	GIVEN("a double pyramid mesh with three missing faces")
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

		Intersector::Mesh mesh(vertices, faces);

		WHEN("the intersection plane normal is positive z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 3);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(!result[0].isClosed);
				REQUIRE(result[1].points.size() == 2);
				REQUIRE(!result[1].isClosed);
				REQUIRE(result[2].points.size() == 2);
				REQUIRE(!result[2].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 2);
				REQUIRE(result[0].points.size() == 5);
				REQUIRE(result[0].isClosed);
				REQUIRE(result[1].points.size() == 6);
				REQUIRE(result[1].isClosed);
			}
		}
		WHEN("the intersection plane normal is negative z") {
			Intersector::Plane plane;
			plane.normal = { 0,0,-1 };
			WHEN("we intersect the mesh with the plane") {
				auto result = mesh.Intersect(plane);
				REQUIRE(result.size() == 3);
				REQUIRE(result[0].points.size() == 4);
				REQUIRE(!result[0].isClosed);
				REQUIRE(result[1].points.size() == 2);
				REQUIRE(!result[1].isClosed);
				REQUIRE(result[2].points.size() == 2);
				REQUIRE(!result[2].isClosed);
			}
			WHEN("we clip the mesh with the plane") {
				auto result = mesh.Clip(plane);
				REQUIRE(result.size() == 1);
				REQUIRE(result[0].points.size() == 16);
			}
		}
	}
};
