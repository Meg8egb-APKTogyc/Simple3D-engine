#pragma once
#include "Vector.hpp"
#include "Polygon.hpp"
#include <vector>
#include <cmath>


class Mesh {
protected:
    std::vector<Polygon> polygons;

public:
    Mesh() = default;

    void addPolygon(const Polygon& polygon) {
        polygons.push_back(polygon);
    }

    void clear() {
        polygons.clear();
    }

    std::vector<Polygon>& getPolygons() {
        return polygons;
    }

    const std::vector<Polygon>& getPolygons() const { 
        return polygons;
    }

    void applyTransform(const Matrix4x4& mat) {
        for (auto& polygon : polygons) {
            polygon = polygon.applyTransform(mat);
        }
    }
};


class ParametricTorus : public Mesh {
private:
    double angle_step = 2 * M_PI / 30;
    double majorRadius = 1.0;
    double tubeRadius = 0.3;

    double calcX(double a, double b, double u, double v) {
        return (a + b * std::cos(v)) * std::cos(u);
    }

    double calcY(double a, double b, double u, double v) {
        return (a + b * std::cos(v)) * std::sin(u);
    }

    double calcZ(double b, double v) {
        return b * std::sin(v);
    }

public:
    ParametricTorus() = default;

    void setMajorRadius(double radius) {
        majorRadius = radius;
        generatePolygons(majorRadius, tubeRadius);
    }
    
    void setTubeRadius(double radius) {
        tubeRadius = radius;
        generatePolygons(majorRadius, tubeRadius);
    }

    double getMajorRadius() const { return majorRadius; }
    double getTubeRadius() const { return tubeRadius; }

    void generatePolygons(double R, double r) {
        majorRadius = R;
        tubeRadius = r;
        polygons.clear();

        for (double u = 0; u < 2 * M_PI; u += angle_step) {
            for (double v = 0; v < 2 * M_PI; v += angle_step) {
                Vector3D v1(calcX(R, r, u, v), calcY(R, r, u, v), calcZ(r, v));
                Vector3D v2(calcX(R, r, u + angle_step, v), calcY(R, r, u + angle_step, v), calcZ(r, v));
                Vector3D v3(calcX(R, r, u, v + angle_step), calcY(R, r, u, v + angle_step), calcZ(r, v + angle_step));
                Vector3D v4(calcX(R, r, u + angle_step, v + angle_step), calcY(R, r, u + angle_step, v + angle_step), calcZ(r, v + angle_step));

                std::vector<Vector3D> triangleVertices1 = {v1, v2, v4, v3};
                //std::vector<Vector3D> triangleVertices2 = {v1, v4, v3};
                polygons.push_back(Polygon(triangleVertices1));
                //polygons.push_back(Polygon(triangleVertices2));
            }
        }
    }
};