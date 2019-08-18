
// (c) 2019 Tim Lyon
// This code is licensed under MIT license

#include <vector>
#include <array>
#include <map>
#include <algorithm>

template <class T>
struct MeshPlaneIntersect {

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

	static std::vector<Path3D> Intersect(const Mesh & mesh, const Plane & plane) {
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

	static Vec3D factor(const Vec3D& a, const T& factor) {
		return Vec3D{ a[0] * factor,a[1] * factor,a[2] * factor };
	}

	static std::vector<Path3D> _Intersect(const Mesh & mesh, const Plane & plane) {
		
		const auto vertexOffsets = VertexOffsets(*mesh.vertices, plane);
		auto crossingFaces = CrossingFaces(*mesh.faces, vertexOffsets);

		std::vector<std::vector<std::pair<int, int>>> edgePaths;
		while (crossingFaces.size() > 0) {
			edgePaths.push_back(GetEdgePath(crossingFaces));
		}
		edgePaths = ChainEdgePaths(edgePaths);

		std::vector<Path3D> paths;
		for (const auto& edgePath : edgePaths) {
			Path3D path;
			for (const auto& edge : edgePath) {
				auto ratio = vertexOffsets[edge.first] /(vertexOffsets[edge.first] - vertexOffsets[edge.second]);
				auto vector = difference(mesh.vertices->at(edge.first), mesh.vertices->at(edge.second));
				auto point = add(mesh.vertices->at(edge.first), factor(vector, ratio));
				path.points.push_back(point);
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

	typedef std::map<std::pair<int, int>, int> CrossingFaceMap;
	static CrossingFaceMap CrossingFaces(const std::vector<Face>& faces, const std::vector<T>& vertexOffsets) {
		CrossingFaceMap crossingFaces;
		for (const auto& face : faces) {

			const auto e1 = DoesEdgeCrossPlane(face[0], face[1], vertexOffsets);
			const auto e2 = DoesEdgeCrossPlane(face[1], face[2], vertexOffsets);

			if (e1 == NONE && e2 == NONE)
				continue;

			int oddVertex = e1 == NONE ? 2 : e2 == NONE ? 0 : 1;
			const bool oddIsHigher = vertexOffsets[face[oddVertex]] > 0;
			crossingFaces[{face[(oddVertex + 1 + oddIsHigher) % 3], face[oddVertex]}] =
				face[(oddVertex + 2 - oddIsHigher) % 3];
		}
		return crossingFaces;
	}

	static std::vector<std::pair<int, int>> GetEdgePath(CrossingFaceMap& crossingFaces) {
		auto currentFace = crossingFaces.begin();
		std::vector<std::pair<int, int>> edgePath({ currentFace->first });
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

	static std::vector<std::vector<std::pair<int, int>>> ChainEdgePaths(
		const std::vector<std::vector<std::pair<int, int>>>& edgePaths) {

		std::map<std::tuple<int, int, bool>, int> ordered;

		int iPath(0);
		for (const auto& path : edgePaths) {
			ordered[{path.front().first, path.front().second, true}] = iPath;
			ordered[{path.back().first, path.back().second, false}] = iPath++;
		}
		std::vector<std::vector<int>> chains;
		std::vector<int> currentChain;
		for (auto thisEnd(ordered.begin()); thisEnd != ordered.end(); thisEnd++) {

			if (currentChain.size() == 0) {
				currentChain.push_back(thisEnd->second);
				thisEnd++;
			}
			if (std::get<2>(thisEnd->first)) {
				thisEnd++;
			}
			if (thisEnd == ordered.end()) {
				chains.push_back(currentChain);
				currentChain.clear();
				break;
			}
			auto nextEnd = thisEnd;
			nextEnd++;

			if (nextEnd != ordered.end() && 
				std::get<0>(thisEnd->first) == std::get<0>(nextEnd->first) &&
				std::get<1>(thisEnd->first) == std::get<1>(nextEnd->first)) {
				currentChain.push_back(nextEnd->second);
				thisEnd++;
			}
			else {
				chains.push_back(currentChain);
				currentChain.clear();
			}
		}

		std::vector<std::vector<std::pair<int, int>>> newPaths;
		for (const auto& chain : chains) {
			auto path(edgePaths[*chain.begin()]);
			for (int iTail(1); iTail < chain.size(); ++iTail) {
				const auto& tail(edgePaths[chain[iTail]]);
				path.insert(path.end(), tail.begin() + 1, tail.end());
			}
			newPaths.push_back(path);
		}
		return newPaths;
	}
};