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
		return _Execute(mesh, plane, false);
	}

	static std::vector<Path3D> Clip(const Mesh& mesh, const Plane& plane) {
		return _Execute(mesh, plane, true);
	}

private:
	// constructor is private, use the static methods
	// MeshPlaneIntersect<T>::Intersect(mesh, plane) or
	// MeshPlaneIntersect<T>::Clip(mesh, plane)
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
	struct VertexPath {
		Edge StartEdge, EndEdge;
		std::vector<int> vertices;
		bool isUsed = false;
	};

	static std::vector<Path3D> _Execute(const Mesh& mesh, const Plane& plane, const bool isClip) {
		const auto vertexOffsets(VertexOffsets(*mesh.vertices, plane));
		auto edgePaths(EdgePaths(*mesh.faces, vertexOffsets));
		ChainEdgePaths(edgePaths);
		if (!isClip) {
			return ConstructGeometricPaths(mesh, edgePaths, nullptr, vertexOffsets);
		}
		auto freeEdges = FreeEdges(*mesh.faces, vertexOffsets);
		auto vertexPaths = VertexPaths(freeEdges, vertexOffsets);
		return ConstructGeometricPaths(mesh, edgePaths, &vertexPaths, vertexOffsets);
	}

	static std::vector<EdgePath> EdgePaths(const std::vector<Face>& faces, const std::vector<T>& vertexOffsets) {
		auto crossingFaces = CrossingFaces(faces, vertexOffsets);
		std::vector<EdgePath> edgePaths;
		while (crossingFaces.size() > 0) {
			edgePaths.push_back(GetEdgePath(crossingFaces));
		}
		return edgePaths;
	}

	static std::vector<Path3D> ClosedVertexPaths(std::vector<VertexPath>* vertexPaths,
		const std::vector<Vec3D>& vectices) {

		std::vector<Path3D> closedVertexPaths;
		if (!vertexPaths) {
			return closedVertexPaths;
		}
		for (auto& vertexPath : *vertexPaths) {
			if (vertexPath.vertices.size() == 0 ||
				vertexPath.StartEdge != vertexPath.EndEdge) {
				continue;
			}
			Path3D path;
			for (const auto& iVertex : vertexPath.vertices) {
				path.points.push_back(vectices[iVertex]);
			}
			if (vertexPath.vertices.front() == vertexPath.vertices.back()) {
				path.points.pop_back();
			}
			path.isClosed = true;
			closedVertexPaths.push_back(path);
			vertexPath.isUsed = true;
		}
		return closedVertexPaths;
	}

	static void ClosePath(Path3D& path, const EdgePath& edgePath,
		std::vector<VertexPath>& vertexPaths, const std::vector<Vec3D>& vertices) {

		for (auto& vertexPath : vertexPaths) {
			if (vertexPath.isUsed) {
				continue;
			}
			const bool append(vertexPath.StartEdge == edgePath.back());
			const bool prepend(vertexPath.EndEdge == edgePath.front());
			const bool appendReverse(vertexPath.EndEdge == edgePath.back());
			const bool prependReverse(vertexPath.StartEdge == edgePath.front());
			if (!(append && prepend) && !(appendReverse && prependReverse)) {
				continue;
			}
			path.isClosed = true;
			std::vector<Vec3D> newPoints;
			for (const auto& iVertex : vertexPath.vertices) {
				newPoints.push_back(vertices[iVertex]);
			}
			if (append) {
				path.points.insert(path.points.end(), newPoints.begin(), newPoints.end());
			}
			else {
				path.points.insert(path.points.end(), newPoints.rbegin(), newPoints.rend());
			}
			vertexPath.isUsed = true;
			return;
		}
	}

	static std::vector<Path3D> ConstructGeometricPaths(const Mesh& mesh,
		const std::vector<EdgePath>& edgePaths, std::vector<VertexPath>* vertexPaths,
		const std::vector<T>& vertexOffsets) {
		std::vector<Path3D> paths = ClosedVertexPaths(vertexPaths, *mesh.vertices);
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
			else if(vertexPaths){
				ClosePath(path, edgePath, *vertexPaths, *mesh.vertices);
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

	static void AlignEdge(Edge& edge) {
		if (edge.first > edge.second) {
			std::swap(edge.first, edge.second);
		}
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
			AlignEdge(edge);
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

	template <typename Type>
	static bool GetStartingItem(const std::vector<Type>& items,
		std::vector<bool>& usedItems, Type& startItem) {
		for (auto i(0); i < items.size(); ++i) {
			if (usedItems[i]) {
				continue;
			}
			startItem = items[i];
			usedItems[i] = true;
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
				currentChain.insert(currentChain.end(), path.rend(), path.rbegin() + 1);
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
		while (GetStartingItem(edgePaths, usedPaths, chain)) {
			while (InsertConnectingEdgePath(edgePaths, usedPaths, chain)) {}
			chainedPaths.push_back(chain);
		}
		edgePaths = chainedPaths;
	}

	static std::vector<Edge> FreeEdges(const std::vector<Face>& faces, const std::vector<T>& vertexOffsets) {
		std::map<Edge, int> edgeFaceCount;
		for (const auto& face : faces) {
			for (int iEdge(0); iEdge < 3; ++iEdge) {
				const auto& v0(face[iEdge]);
				const auto& v1(face[(iEdge + 1) % 3]);
				if (vertexOffsets[v0] < 0 && vertexOffsets[v1] < 0) {
					continue;
				}
				Edge thisEdge(v0, v1);
				AlignEdge(thisEdge);
				edgeFaceCount[thisEdge]++;
			}
		}
		std::vector<Edge> freeEdges;
		for (const auto& edge : edgeFaceCount) {
			if (edge.second == 1) {
				freeEdges.push_back(edge.first);
			}
		}
		return freeEdges;
	}

	static std::vector<VertexPath> VertexPaths(const std::vector<Edge>& freeEdges, const std::vector<T> vertexOffsets) {
		std::vector<VertexPath> vertexPaths;
		std::vector<bool> usedEdges(freeEdges.size());
		Edge edge;
		while (GetStartingItem(freeEdges, usedEdges, edge)) {
			VertexPath path;
			path.StartEdge = path.EndEdge = Edge(-1, -1);

			if (vertexOffsets[edge.first] > 0) {
				path.vertices.push_back(edge.first);
				if (vertexOffsets[edge.second] < 0) {
					path.StartEdge = edge;
				}
			}
			if (vertexOffsets[edge.second] > 0) {
				path.vertices.push_back(edge.second);
				if (vertexOffsets[edge.first] < 0) {
					path.EndEdge = edge;
				}
			}
			while (ExtendVertexPathUsing(path, freeEdges, usedEdges, vertexOffsets)) {
				// chain getting longer, available edges getting smaller
			}
			vertexPaths.push_back(path);
		}
		return vertexPaths;
	}

	static bool ExtendVertexPathUsing(VertexPath& vertexPath, const std::vector<Edge>& freeEdges,
		std::vector<bool>& usedFreeEdges, const std::vector<T> vertexOffsets) {

		for (auto iEdge(0); iEdge < freeEdges.size(); iEdge++) {
			if (usedFreeEdges[iEdge]) {
				continue;
			}
			const auto& edge(freeEdges[iEdge]);
			const bool edgeCrosses = DoesEdgeCrossPlane(edge.first,
				edge.second, vertexOffsets) != CrossingPlane::NONE;

			// try adding to the back of the chain
			if (vertexPath.EndEdge.first == -1) {
				const bool append(vertexPath.vertices.back() == edge.first);
				const bool appendReverse(vertexPath.vertices.back() == edge.second);
				if (append || appendReverse) {
					if (edgeCrosses) {
						vertexPath.EndEdge = edge;
					}
					else {
						vertexPath.vertices.push_back(append ? edge.second : edge.first);
					}
					usedFreeEdges[iEdge] = true;
					return true;
				}
			}
			// try adding to the front of the chain
			if (vertexPath.StartEdge.first == -1) {
				const bool prepend(vertexPath.vertices.front() == edge.first);
				const bool prependReverse(vertexPath.vertices.front() == edge.second);
				if (prepend || prependReverse) {
					if (edgeCrosses) {
						vertexPath.StartEdge = edge;
					}
					else {
						vertexPath.vertices.insert(vertexPath.vertices.begin(),
							prepend ? edge.second : edge.first);
					}
					usedFreeEdges[iEdge] = true;
					return true;
				}
			}
		}
		return false;
	}
};