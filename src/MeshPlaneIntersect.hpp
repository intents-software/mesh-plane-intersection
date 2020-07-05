#pragma once
#include <vector>
#include <array>
#include <map>
#include <algorithm>

template <class FloatType, class IndexType>
class MeshPlaneIntersect {

public:
	typedef std::array<FloatType, 3> Vec3D;
	typedef std::array<IndexType, 3> Face;

	struct Mesh {
		const std::vector<Vec3D>& vertices;
		const std::vector<Face>& faces;
		Mesh(const std::vector<Vec3D>& vertices, const std::vector<Face>& faces) :
			vertices(vertices), faces(faces) {}
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
	// MeshPlaneIntersect<FloatType, IndexType>::Intersect(mesh, plane) or
	// MeshPlaneIntersect<FloatType, IndexType>::Clip(mesh, plane)
	MeshPlaneIntersect() {};

	static Vec3D add(const Vec3D& a, const Vec3D& b) {
		return Vec3D{ b[0] + a[0],b[1] + a[1],b[2] + a[2] };
	}

	static Vec3D difference(const Vec3D& a, const Vec3D& b) {
		return Vec3D{ b[0] - a[0],b[1] - a[1],b[2] - a[2] };
	}

	static void factorBy(Vec3D& a, const FloatType& factor) {
		for (int i(0); i < 3; ++i) {
			a[i] *= factor;
		}
	}

	typedef std::pair<int, int> Edge;
	typedef std::vector<Edge> EdgePath;
	static std::vector<Path3D> _Execute(const Mesh& mesh, const Plane& plane,
		const bool isClip) {
		const auto vertexOffsets(VertexOffsets(mesh.vertices, plane));
		auto edgePaths(EdgePaths(mesh.faces, vertexOffsets));
		if (isClip) {
			auto freeEdges = FreeEdges(mesh.faces, vertexOffsets);
			auto freeEdgePaths = FreeEdgePaths(freeEdges, vertexOffsets);
			edgePaths.insert(edgePaths.end(), freeEdgePaths.begin(), freeEdgePaths.end());
		}
		ChainEdgePaths(edgePaths);
		return ConstructGeometricPaths(mesh, edgePaths, vertexOffsets);
	}

	static std::vector<EdgePath> EdgePaths(const std::vector<Face>& faces,
		const std::vector<FloatType>& vertexOffsets) {
		auto crossingFaces = CrossingFaces(faces, vertexOffsets);
		std::vector<EdgePath> edgePaths;
		while (crossingFaces.size() > 0) {
			edgePaths.push_back(GetEdgePath(crossingFaces));
		}
		return edgePaths;
	}

	static std::vector<Path3D> ConstructGeometricPaths(const Mesh& mesh,
		const std::vector<EdgePath>& edgePaths,
		const std::vector<FloatType>& vertexOffsets) {
		std::vector<Path3D> paths;
		for (const auto& edgePath : edgePaths) {
			Path3D path;
			bool skipThisPoint = path.isClosed = edgePath.front() == edgePath.back();
			for (const auto& edge : edgePath) {
				if (skipThisPoint) {
					skipThisPoint = false;
				}
				else if (edge.first == edge.second) {
					path.points.push_back(mesh.vertices.at(edge.first));
				}
				else {
					const auto& edgeStart(mesh.vertices.at(edge.first));
					auto vector(difference(edgeStart, mesh.vertices.at(edge.second)));
					const auto& offset1(vertexOffsets[edge.first]);
					const auto& offset2(vertexOffsets[edge.second]);
					factorBy(vector, offset1 / (offset1 - offset2));
					path.points.push_back(add(edgeStart, vector));
				}
			}
			paths.push_back(path);
		}
		return paths;
	}

	static const std::vector<FloatType> VertexOffsets(const std::vector<Vec3D>& vertices,
		const Plane& plane) {
		std::vector<FloatType> offsets;
		offsets.reserve(vertices.size());
		std::transform(vertices.begin(), vertices.end(), std::back_inserter(offsets),
			[&plane](const auto& vertex) {
				return VertexOffset(vertex, plane);
			});
		return offsets;
	}

	static FloatType VertexOffset(const Vec3D& vertex, const Plane& plane) {
		FloatType offset(0);
		for (int i(0); i < 3; ++i) {
			offset += plane.normal[i] * (vertex[i] - plane.origin[i]);
		}
		return offset;
	}

	typedef std::map<Edge, int> CrossingFaceMap;
	static CrossingFaceMap CrossingFaces(const std::vector<Face>& faces,
		const std::vector<FloatType>& vertexOffsets) {
		std::vector<std::pair<Edge, int>> crossingFaces;
		for (const auto& face : faces) {
			const bool edge1crosses = vertexOffsets[face[0]] * vertexOffsets[face[1]] < 0;
			const bool edge2crosses = vertexOffsets[face[1]] * vertexOffsets[face[2]] < 0;
			if (edge1crosses || edge2crosses) {
				int oddVertex = edge2crosses - edge1crosses + 1;
				const bool oddIsHigher = vertexOffsets[face[oddVertex]] > 0;
				int v0 = oddVertex + 1 + oddIsHigher;
				if (v0 > 2) {
					v0 -= 3;
				}
				int v2 = oddVertex + 2 - oddIsHigher;
				if (v2 > 2) {
					v2 -= 3;
				}
				crossingFaces.push_back({ {face[v0], face[oddVertex]},face[v2] });
			}
		}
		return CrossingFaceMap(crossingFaces.begin(), crossingFaces.end());
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

	static bool GetNextPoint(CrossingFaceMap::const_iterator& currentFace,
		CrossingFaceMap& crossingFaces) {
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
		for (size_t i(0); i < items.size(); ++i) {
			if (!usedItems[i]) {
				startItem = items[i];
				usedItems[i] = true;
				return true;
			}
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
				currentChain.insert(currentChain.end(), path.rbegin() + 1, path.rend());
			else if (path.back() == currentChain.front())
				currentChain.insert(currentChain.begin(), path.begin(), path.end() - 1);
			else if (path.front() == currentChain.front())
				currentChain.insert(currentChain.begin(), path.rbegin(), path.rend() - 1);
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

	static std::vector<Edge> FreeEdges(const std::vector<Face>& faces,
		const std::vector<FloatType>& vertexOffsets) {
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

	struct FreeEdgePath {
		Edge StartEdge, EndEdge;
		std::vector<int> vertices;
		bool isUsed = false;
	};
	static std::vector<EdgePath> FreeEdgePaths(const std::vector<Edge>& freeEdges,
		const std::vector<FloatType> vertexOffsets) {
		std::vector<EdgePath> freeEdgePaths;
		std::vector<bool> usedEdges(freeEdges.size());
		Edge edge;
		while (GetStartingItem(freeEdges, usedEdges, edge)) {
			FreeEdgePath path;
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
			while (ExtendFreeEdgePath(path, freeEdges, usedEdges, vertexOffsets)) {
				// chain getting longer, available edges getting smaller
			}
			EdgePath edgePath;
			if (path.StartEdge.first >= 0) {
				edgePath.push_back(path.StartEdge);
			}
			for (const int iVert : path.vertices) {
				edgePath.push_back(std::make_pair(iVert, iVert));
			}
			if (path.EndEdge.first >= 0) {
				edgePath.push_back(path.EndEdge);
			}
			freeEdgePaths.push_back(edgePath);
		}
		return freeEdgePaths;
	}

	static bool ExtendFreeEdgePath(FreeEdgePath& path, const std::vector<Edge>& freeEdges,
		std::vector<bool>& usedFreeEdges, const std::vector<FloatType> vertexOffsets) {

		for (size_t iEdge(0); iEdge < freeEdges.size(); iEdge++) {
			if (usedFreeEdges[iEdge]) {
				continue;
			}
			const auto& edge(freeEdges[iEdge]);
			const bool edgeCrosses = vertexOffsets[edge.first] * vertexOffsets[edge.second] < 0;

			// try adding to the back of the chain
			if (path.EndEdge.first == -1) {
				const bool append(path.vertices.back() == edge.first);
				const bool appendReverse(path.vertices.back() == edge.second);
				if (append || appendReverse) {
					if (edgeCrosses) {
						path.EndEdge = edge;
					}
					else {
						path.vertices.push_back(append ? edge.second : edge.first);
					}
					usedFreeEdges[iEdge] = true;
					return true;
				}
			}
			// try adding to the front of the chain
			if (path.StartEdge.first == -1) {
				const bool prepend(path.vertices.front() == edge.first);
				const bool prependReverse(path.vertices.front() == edge.second);
				if (prepend || prependReverse) {
					if (edgeCrosses) {
						path.StartEdge = edge;
					}
					else {
						path.vertices.insert(path.vertices.begin(),
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