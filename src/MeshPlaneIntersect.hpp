
// (c) 2019 Tim Lyon
// This code is licensed under MIT license

#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>

template <class T>
struct MeshPlaneIntersect {

    struct Mesh {
        const std::vector<std::array<T, 3>>* vertices;
        const std::vector<std::array<int, 3>>* faces;
    };

    struct Plane {
        std::array<T, 3> origin;
        std::array<T, 3> normal;
    };

    std::vector<std::array<T, 3>> Intersect(const Mesh& mesh, const Plane& plane) {
        return _Intersect(mesh, plane);
    }

    private:

    struct Edge {
        std::array<int, 2> adjacent_faces;
        bool crossesPlane = false;
        bool visited = false;
        Edge(){
            adjacent_faces.fill(-1);
        }
    };

    const Mesh* m_mesh;
    const Plane* m_plane;
    std::vector<T> m_vertexOffsets;
    std::unordered_map<std::pair<int, int>, Edge> m_edges;
    auto m_currentEdge, m_startingEdge;
    bool m_isFirstPoint;
    std::array<T, 2> m_currentPoint;

    std::vector<std::array<T, 3>> _Intersect(const Mesh& mesh, const Plane& plane) {
        m_mesh = &_mesh;
        m_plane = &_plane;
        CalculateVertexOffsets();
        CreateEdgesMap();
        std::vector<std::vector<T>> intersections;
        while(FindStartingEdge()){
            intersections.push_back(GetIntersection());
        }
        return intersections;
    }
    
    void CalculateVertexOffsets(){
        vertexOffsets.resize(mesh->vertices.size());
        int iVertex(0);
        for(const auto& vertex : mesh->vertices){
            vertexOffsets[iVertex++] = VertexOffset(vertex);
        }
    }

    T VertexOffset(const std::array<T, 3>& vertex){
        // TODO do transform on plane and return z value
    }
    
    CreateEdgesMap(){
        edges.clear();
        int iFace(0);
        for(const auto& face : mesh->faces){
            for(int iEdge(0); iEdge<3; ++iEdge){
                auto key(std::make_pair(face[iEdge], face[(iEdge+1)%3]));
                const bool isReversed(key.second > key.first);
                if(isReversed){
                    std::swap(key.first, key.second);
                }
                const bool isNewKey(edges.find(key) == edges.end());
                auto& edge(edges[key]);
                edge.adjacent_faces[isReversed] = iFace++;
                if(isNewKey){
                    edge.crossesPlane =
                        (vertexOffsets[key.first] > 0 && vertexOffsets[key.second] <= 0) ||
                        (vertexOffsets[key.second] > 0 && vertexOffsets[key.first] <= 0);
                }
            }
        }
    }

    bool FindStartingEdge(){
        m_currentEdge = std::find_if(edges.begin(), edges.end(), [](auto& edge){
            return edge.second.crossesPlane && !edge.second.visited;
        });
        m_startingEdge = m_currentEdge;
        m_isFirstPoint = true;
        return m_currentEdge != edges.end();
    }

    std::vector<std::array<T, 3>> GetIntersection(){
        std::vector<std::array<T, 3>> intersection;
        while(MoveToNextPoint()){
            intersection.push_back(m_currentPoint);
        }
        return intersection;
    }

    bool MoveToNextPoint(){

        if(m_isFirstPoint){
            SetCurrentPointFromCurrentEdge();
            m_isFirstPoint = false;
            return true;
        }

        while MoveToNextEdge();
        SetCurrentPointFromCurrentEdge();
        return m_currentEdge != m_startingEdge;
    }

    bool MoveToNextEdge(){
        // loop through the faces of iFace and return the edge that 
        // is not currentEdge but crosses the plane

        // if none cross, return the first free edge
        return true;

        // if no free edges, return the last edge
        return false;
    }

    void SetCurrentPointFromCurrentEdge(){
        // if edge crosses, interpolate, otherwise return the
        // point at the other end of the edge
        // always transformed projected onto the plane
        currentPoint = ...
    }


}