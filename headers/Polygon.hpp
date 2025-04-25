#pragma once
#include "Vector.hpp"
#include "Matrix4x4.hpp"
#include <vector>
#include <stdexcept>


class Polygon {
private:
    std::vector<Vector3D> vertices;
    Vector3D normal;

public:
    Polygon(const std::vector<Vector3D>& vec) : vertices(vec) {
        if (vec.size() < 3) throw std::runtime_error("Polygon must have at least 3 vertices!");
        calculateNormal();
    }

    void addVertex(const Vector3D& vertex) {
        vertices.push_back(vertex);
        if (vertices.size() >= 3) calculateNormal();
    }

    void calculateNormal() {
        if (vertices.size() < 3) return;

        Vector3D edge1 = vertices[1] - vertices[0];
        Vector3D edge2 = vertices[2] - vertices[0];
        normal = edge1 % edge2;
        normal = normal.normalize();

        /*std::cout << edge1.x << ' ' << edge1.y << ' ' << edge1.z << std::endl;
        std::cout << edge2.x << ' ' << edge2.y << ' ' << edge2.z << std::endl;
        std::cout << normal.x << ' ' << normal.y << ' ' << normal.z << std::endl;
        std::cout << "..." << std::endl;*/
    }

    const std::vector<Vector3D>& getVertices() const { return vertices; }

    const Vector3D& getNormal() const { return normal; }

    Polygon applyTransform(const Matrix4x4& mat) const {
        std::vector<Vector3D> transformedVertices;
        for (const Vector3D& vertex : vertices) {
            transformedVertices.push_back(mat * vertex);
        }

        Polygon transformedPoly(transformedVertices);

        return Polygon(transformedVertices);
    }

    Vector3D calculateCenter() const {
        Vector3D centre(0, 0, 0);
        for (const auto& v : vertices) {
            centre = centre + v;
        }
        centre = centre / vertices.size();
        return centre;
    }
};