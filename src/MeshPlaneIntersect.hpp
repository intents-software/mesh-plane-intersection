// (c) 2019 Tim Lyon
// This code is licensed under MIT license
// https://github.com/tim-lyon/mesh-plane-intersection

#include <vector>
#include <array>
#include <map>

template <class T>
class MeshPlaneIntersect {

public:
	typedef std::array<T, 3> Vec3D;
	typedef std::array<int, 3> Face;

	struct Mesh {
		const std::vector<Vec3D>* vertices;
		const std::vector<Face>* faces;
	};

	struct Plane {
		Vec3D origin = { 0,0,0 };
		Vec3D normal = { 0,0,1 };
	};

	struct Path3D {
		std::vector<Vec3D> points;
		bool isClosed = false;
	};

	static std::vector<Path3D> Intersect(const Mesh& mesh, const Plane& plane) {
		return _Intersect(mesh, plane);
	}

private:

	// constructor is private, use the static method
	// MeshPlaneIntersect<T>::Intersect(mesh, plane)
	MeshPlaneIntersect() {};

	static Vec3D add(const Vec3D& a, const Vec3D& b) {
		return Vec3D{ b[0] + a[0],b[1] + a[1],b[2] + a[2] };
	}

	static Vec3D difference(const Vec3D& a, const Vec3D& b) {
		return Vec3D{ b[0] - a[0],b[1] - a[1],b[2] - a[2] };
	}

	static void factorBy(Vec3D& a, const T& factor) {
		for (int i(0); i < 3; ++i) {
			a[i] *= factor;
		}
	}

	typedef std::pair<int, int> Edge;
	typedef std::vector<Edge> EdgePath;
	static std::vector<Path3D> _Intersect(const Mesh& mesh, const Plane& plane) {
		const auto vertexOffsets = VertexOffsets(*mesh.vertices, plane);
		auto crossingFaces = CrossingFaces(*mesh.faces, vertexOffsets);
		std::vector<EdgePath> edgePaths;
		while (crossingFaces.size() > 0) {
			edgePaths.push_back(GetEdgePath(crossingFaces));
		}
		ChainEdgePaths(edgePaths);
		return ConstructGeometricPaths(mesh, edgePaths, vertexOffsets);
	}

	static std::vector<Path3D> ConstructGeometricPaths(const Mesh& mesh,
		const std::vector<EdgePath>& edgePaths, const std::vector<T> vertexOffsets) {
		std::vector<Path3D> paths;
		for (const auto& edgePath : edgePaths) {
			Path3D path;
			for (const auto& edge : edgePath) {
				const auto& edgeStart(mesh.vertices->at(edge.first));
				auto vector(difference(edgeStart, mesh.vertices->at(edge.second)));
				const auto& offset1(vertexOffsets[edge.first]);
				const auto& offset2(vertexOffsets[edge.second]);
				factorBy(vector, offset1 / (offset1 - offset2));
				path.points.push_back(add(edgeStart, vector));
			}
			path.isClosed = edgePath.front() == edgePath.back();
			if (path.isClosed) {
				path.points.pop_back();
			}
			paths.push_back(path);
		}
		return paths;
	}

	static const std::vector<T> VertexOffsets(const std::vector<Vec3D>& vertices, const Plane& plane) {
		std::vector<T> offsets(vertices.size());
		int iVertex(0);
		for (const auto& vertex : vertices) {
			offsets[iVertex++] = VertexOffset(vertex, plane);
		}
		return offsets;
	}

	static T VertexOffset(const Vec3D& vertex, const Plane& plane) {
		return plane.normal[0] * (vertex[0] - plane.origin[0]) +
			plane.normal[1] * (vertex[1] - plane.origin[1]) +
			plane.normal[2] * (vertex[2] - plane.origin[2]);
	}

	enum CrossingPlane {
		DOWNWARDS = -1,
		NONE = 0,
		UPWARDS = 1
	};

	static CrossingPlane DoesEdgeCrossPlane(const int& v0, const int& v1, const std::vector<T>& vertexOffsets) {
		return vertexOffsets[v1] > 0 && vertexOffsets[v0] <= 0 ? UPWARDS :
			vertexOffsets[v0] > 0 && vertexOffsets[v1] <= 0 ? DOWNWARDS : NONE;
	}

	typedef std::map<Edge, int> CrossingFaceMap;
	static CrossingFaceMap CrossingFaces(const std::vector<Face>& faces, const std::vector<T>& vertexOffsets) {
		CrossingFaceMap crossingFaces;
		for (const auto& face : faces) {
			const auto e1 = DoesEdgeCrossPlane(face[0], face[1], vertexOffsets);
			const auto e2 = DoesEdgeCrossPlane(face[1], face[2], vertexOffsets);
			if (e1 == NONE && e2 == NONE) {
				continue;
			}
			int oddVertex = e1 == NONE ? 2 : e2 == NONE ? 0 : 1;
			const bool oddIsHigher = vertexOffsets[face[oddVertex]] > 0;
			crossingFaces[{face[(oddVertex + 1 + oddIsHigher) % 3], face[oddVertex]}] =
				face[(oddVertex + 2 - oddIsHigher) % 3];
		}
		return crossingFaces;
	}

	static EdgePath GetEdgePath(CrossingFaceMap& crossingFaces) {
		auto currentFace = crossingFaces.begin();
		EdgePath edgePath({ currentFace->first });
		int closingVertex(currentFace->second);
		while (GetNextPoint(currentFace, crossingFaces)) {
			edgePath.push_back(currentFace->first);
			closingVertex = currentFace->second;
		}
		edgePath.push_back({ edgePath.back().second, closingVertex });
		for (auto& edge : edgePath) {
			if (edge.first > edge.second) {
				std::swap(edge.first, edge.second);
			}
		}
		return edgePath;
	}

	static bool GetNextPoint(CrossingFaceMap::const_iterator& currentFace, CrossingFaceMap& crossingFaces) {
		auto nextKey(std::make_pair(currentFace->first.second, currentFace->second));
		crossingFaces.erase(currentFace);
		currentFace = crossingFaces.find(nextKey);
		if (currentFace == crossingFaces.end()) {
			std::swap(nextKey.first, nextKey.second);
			currentFace = crossingFaces.find(nextKey);
		}
		return currentFace != crossingFaces.end();
	}

	static bool GetStartingEdgePath(const std::vector<EdgePath>& edgePaths,
		std::vector<bool>& usedPaths, EdgePath& startPath) {
		for (auto iPath(0); iPath < edgePaths.size(); ++iPath) {
			if (usedPaths[iPath]) {
				continue;
			}
			startPath = edgePaths[iPath];
			usedPaths[iPath] = true;
			return true;
		}
		return false;
	}

	static bool InsertConnectingEdgePath(const std::vector<EdgePath>& edgePaths,
		std::vector<bool>& usedPaths, EdgePath& currentChain) {
		int iPath(-1);
		for (auto& path : edgePaths) {
			++iPath;
			if (usedPaths[iPath]) {
				continue;
			}
			if (path.front() == currentChain.back())
				currentChain.insert(currentChain.end(), path.begin() + 1, path.end());
			else if (path.back() == currentChain.back())
				currentChain.insert(currentChain.end(), path.rend(), path.rbegin() - 1);
			else if (path.back() == currentChain.front())
				currentChain.insert(currentChain.begin(), path.begin(), path.end() - 1);
			else if (path.front() == currentChain.front())
				currentChain.insert(currentChain.begin(), path.rend() + 1, path.rbegin());
			else continue;

			usedPaths[iPath] = true;
			return true;
		}
		return false;
	}

	static void ChainEdgePaths(std::vector<EdgePath>& edgePaths) {
		if (edgePaths.size() < 1) {
			return;
		}
		std::vector<bool> usedPaths(edgePaths.size());
		std::vector<EdgePath> chainedPaths;
		EdgePath chain;
		while (GetStartingEdgePath(edgePaths, usedPaths, chain)) {
			while (InsertConnectingEdgePath(edgePaths, usedPaths, chain)) {}
			chainedPaths.push_back(chain);
		}
		edgePaths = chainedPaths;
	}
};