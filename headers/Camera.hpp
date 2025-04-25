#pragma once
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <cstdint>
#include <vector>
#include <iostream>
#include "Vector.hpp"
#include "Matrix4x4.hpp"
#include "Polygon.hpp"
#include "Mesh.hpp"
#include <algorithm>


class Camera {
private:
    Vector3D position;
    Vector3D forward;
    Vector3D up;
    Vector3D right;

    double l, r, b, t; // границы near-плоскости
    double near, far;
    int width, height;
    Matrix4x4 projectionMatrix;

    double moveSpeed = 0.1;
    double rotateSpeed = 0.005;
    double sensivity = 0.2;

    void updateProjectionMatrix() {
        projectionMatrix = Matrix4x4::perspectiveProjection(l, r, b, t, near, far);
    }

    void updateVectors() {
        forward = forward.normalize();
        right = (forward % up).normalize();
        up = (right % forward).normalize();
    }

public:
    Camera(const Vector3D& pos_, const Vector3D& target, const Vector3D& up_, double l_, double r_, double b_, double t_, double near_, double far_) 
        : position(pos_), l(l_), r(r_), b(b_), t(t_), near(near_), far(far_) {
        forward = (target - position).normalize();
        right = (forward % up_).normalize();
        up = (right % forward).normalize();
        updateProjectionMatrix();
    }

    void lookAt(const Vector3D& target) {
        forward = (target - position).normalize(); 
        right = (up % forward).normalize();
        up = right % forward;
    }

    std::pair<int, int> projectPoint(const Vector3D& point) const {
        Vector3D viewPoint = getViewMatrix() * point;
        Vector3D projectedPoint = projectionMatrix * viewPoint;
        
        if (fabs(projectedPoint.z) < 1e-6) {
            projectedPoint.z = 1e-6;
        }

        projectedPoint.x /= projectedPoint.z;
        projectedPoint.y /= projectedPoint.z;
        
        int screenX = static_cast<int>((projectedPoint.x + 1.0) * 0.5 * width);
        int screenY = static_cast<int>((1.0 - (projectedPoint.y + 1.0) * 0.5) * height);
        
        return std::make_pair(screenX, screenY);
    }

    void setViewport(int w, int h) {
        width = w;
        height = h;
    }

    Matrix4x4 getViewMatrix() const {
        return Matrix4x4::lookAt(position, position + forward, up);
    }

    Vector3D getPosition() const {
        return position;
    }

    bool isPointInFrustrum(const Vector3D& viewPoint) const {
        if (viewPoint.z > -near || viewPoint.z < -far) return false;

        float scale = viewPoint.z / -near;  // near > 0, viewPoint.z < 0 => scale > 0
        float leftBound   = l * scale;
        float rightBound  = r * scale;
        float bottomBound = b * scale;
        float topBound    = t * scale;

        if (viewPoint.x < leftBound || viewPoint.x > rightBound) return false;
        if (viewPoint.y < bottomBound || viewPoint.y > topBound) return false;

        return true;
    }

    bool isPolyInFrustrum(const Polygon& poly) const {
        const auto& vertices = poly.getVertices();

        for (const auto& v : vertices) {
            Vector3D viewPoint = getViewMatrix() * v;
            if (!isPointInFrustrum(viewPoint)) {
                return false;
            }
        }

        return true;
    }

    void processMovement(int keycode, double deltaTime) {
        float velocity = moveSpeed * deltaTime;
    
        switch(keycode) {
            case XK_w:
            case XK_W: position += forward * velocity; break;
            case XK_s:
            case XK_S: position -= forward * velocity; break;
            case XK_a:
            case XK_A: position -= right * velocity; break;
            case XK_d:
            case XK_D: position += right * velocity; break;
            case XK_q:
            case XK_Q: position += up * velocity; break;
            case XK_e:
            case XK_E: position -= up * velocity; break;
        }
        updateVectors();
    }

    void processMouseMovement(double xoffset, double yoffset) {
        const double smoothingFactor = 0.15;
        static double smoothedX = 0.001;
        static double smoothedY = 0.001;
        
        smoothedX = smoothedX * (1.0 - smoothingFactor) + xoffset * smoothingFactor;
        smoothedY = smoothedY * (1.0 - smoothingFactor) + yoffset * smoothingFactor;

        double sensitivityFactor = 0.0005;
        
        Matrix4x4 rotY = Matrix4x4::rotateY(-smoothedX * sensitivityFactor);
        forward = rotY * forward;
        up = rotY * up;

        right = (forward % up).normalize();
        Matrix4x4 rotX = Matrix4x4::rotate(right, -smoothedY * sensitivityFactor);
        Vector3D newForward = rotX * forward;
        
        if (fabs(dot(newForward, up)) < 0.985) {
            forward = newForward.normalize();
        }

        right = (forward % up).normalize();
        up = (right % forward).normalize();
    }

    void processMouseScroll(double yoffset) {
        const double zoomSpeed = 0.1;
        static double accumulatedOffset = 0.0;
        const double smoothingFactor = 0.2;
        const double minDistance = 0.5; // Минимальное расстояние до объекта
        const double maxDistance = 50.0; // Максимальное расстояние
        
        accumulatedOffset = accumulatedOffset * (1.0 - smoothingFactor) + yoffset * smoothingFactor;
        
        Vector3D newPosition = position + forward * accumulatedOffset * moveSpeed * zoomSpeed;
        double newDistance = (newPosition - (position + forward * 10.0)).magnitude(); // Примерная точка фокуса
        
        if (newDistance > minDistance && newDistance < maxDistance) {
            position = newPosition;
        }
        
        accumulatedOffset *= 0.9;
        updateVectors();
    }

    void setSensivity(double newSense) {
        sensivity = std::clamp(newSense, 0.01, 1.0);
    }

    void setMoveSpeed(double speed) {
        moveSpeed = std::max(0.01, speed);
    }
};

class GraphicsContext {
private:
    Display* display_;
    Window window_;
    int width_, height_;
    GC gc_;
    std::vector<uint32_t> framebuffer_;

public:
    GraphicsContext(int width, int height, const char* title) 
        : width_(width), height_(height), framebuffer_(width * height, 0xFFFFFFFF) {
        
        display_ = XOpenDisplay(nullptr);
        if (!display_) {
            throw std::runtime_error("Cannot open X display");
        }

        int screen = DefaultScreen(display_);

        XVisualInfo vinfo;
        if (!XMatchVisualInfo(display_, screen, 32, TrueColor, &vinfo)) {
            XCloseDisplay(display_);
            throw std::runtime_error("32-bit TrueColor visual not available");
        }

        if (vinfo.red_mask != 0xFF0000 || 
            vinfo.green_mask != 0x00FF00 || 
            vinfo.blue_mask != 0x0000FF) {
            XCloseDisplay(display_);
            throw std::runtime_error("Unexpected color masks");
        }

        Colormap colormap = XCreateColormap(display_, RootWindow(display_, screen), 
                vinfo.visual, AllocNone);

        XSetWindowAttributes swa;
        swa.colormap = colormap;
        swa.background_pixel = 0;
        swa.border_pixel = 0;

        window_ = XCreateWindow(
            display_, RootWindow(display_, screen),
            0, 0, width, height, 0,
            vinfo.depth,
            InputOutput,
            vinfo.visual,
            CWColormap | CWBackPixel | CWBorderPixel,
            &swa
        );

        XStoreName(display_, window_, title);

        // Выбираем события
        XSelectInput(display_, window_, ExposureMask | KeyPressMask | KeyReleaseMask | PointerMotionMask | ButtonPressMask | ButtonReleaseMask);

        XMapWindow(display_, window_);

        gc_ = XCreateGC(display_, window_, 0, nullptr);
        XSetForeground(display_, gc_, WhitePixel(display_, screen));

        XEvent event;
        do {
            XNextEvent(display_, &event);
        } while (event.type != Expose);
    }

    ~GraphicsContext() {
        if (gc_) XFreeGC(display_, gc_);
        if (window_) XDestroyWindow(display_, window_);
        if (display_) XCloseDisplay(display_);
    }

    void setPixel(int x, int y, uint32_t color) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            framebuffer_[y * width_ + x] = color;
        }
    }

    void clear(uint32_t color = 0xFF000000) {
        std::fill(framebuffer_.begin(), framebuffer_.end(), color);
    }

    void flush() {
        XImage* image = XCreateImage(
            display_,
            DefaultVisual(display_, DefaultScreen(display_)),
            32,
            ZPixmap,
            0,
            reinterpret_cast<char*>(framebuffer_.data()),
            width_,
            height_,
            32,
            width_ * 4
        );

        if (!image) {
            std::cerr << "Failed to create XImage" << std::endl;
            return;
        }

        XPutImage(
            display_,
            window_,
            gc_,
            image,
            0, 0,  // Начало в источнике
            0, 0,  // Начало в назначении
            width_,
            height_
        );

        image->data = nullptr;
        XDestroyImage(image);

        XFlush(display_);
    }

    Display* getDisplay() const {
        return display_;
    }

    Window getWindow() const { 
        return window_;
    }

    void drawLine(int x1, int y1, int x2, int y2, uint32_t color) {
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = x1 < x2 ? 1 : -1;
        int sy = y1 < y2 ? 1 : -1;
        int err = dx - dy;

        while (true) {
            setPixel(x1, y1, color);

            if (x1 == x2 && y1 == y2) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
    }
};


class Light {
public:
    Vector3D position;
    Vector3D color;
    double intensity;
    bool enabled;

    Light(Vector3D position_, Vector3D color_, double intensity_, bool enabled_) : position(position_), color(color_), intensity(intensity_), enabled(enabled_) {}
};


class Render {
private:
    Camera& camera_;
    GraphicsContext& gc_;
    std::vector<Light> lights_;
    Vector3D ambientColor_;

    void findPolygonBoundsY(const std::vector<std::pair<int, int>>& points, int& minY, int & maxY) {
        minY = points[0].second;
        maxY = points[0].second;

        for (const auto& p : points) {
            if (p.second < minY) minY = p.second;
            if (p.second > maxY) maxY = p.second;
        }
    }

    void fillPoly3D(const Polygon& poly, uint32_t color) {        
        const auto& vertices = poly.getVertices();
        std::vector<std::pair<int, int>> points2D;
        for (const auto& v : vertices) {
            points2D.push_back(camera_.projectPoint(v));
        }
    
        int minY, maxY;
        findPolygonBoundsY(points2D, minY, maxY);
    
        for (int y = minY; y <= maxY; y++) {
            std::vector<int> intersections;
    
            for (int i = 0; i < (int)points2D.size(); i++) {
                int j = (i + 1) % points2D.size();
    
                int y1 = points2D[i].second;
                int y2 = points2D[j].second;
    
                if ((y1 <= y && y < y2) || (y2 <= y && y < y1)) {
                    double x = points2D[i].first + (double)(y - y1) / (y2 - y1) * (points2D[j].first - points2D[i].first);
                    intersections.push_back(static_cast<int>(x));
                }
            }
    
            std::sort(intersections.begin(), intersections.end());
    
            for (int k = 0; k + 1 < (int)intersections.size(); k += 2) {
                gc_.drawLine(intersections[k], y, intersections[k + 1], y, color);
            } 
        }
    }


    Vector3D colorToVec(uint32_t color) const {
        return Vector3D(((color >> 16) & 0xFF) / 255.0, ((color >> 8) & 0xFF) / 255.0, (color & 0xFF) / 255.0);
    }

    uint32_t vecToColor(const Vector3D& vec) const {
        return 0xFF000000 | ((uint32_t)(vec.x * 255.0) << 16) | ((uint32_t)(vec.y * 255.0) << 8) | (uint32_t)(vec.z * 255.0);
    }

    double calculateDepth(const Vector3D& point) const {
        Vector3D viewPoint = camera_.getViewMatrix() * point;
        return -viewPoint.z;
    }

public:
    Render(Camera& camera, GraphicsContext& gc, Light light) : camera_(camera), gc_(gc) {
        lights_.push_back(light);
    }

    void addLight(const Light& light) {
        lights_.push_back(light);
    }

    void setAmbientColor(const Vector3D color) {
        ambientColor_ = color;
    }

    uint32_t calculateShadedColor(const Polygon& poly, uint32_t baseColor) {
        const Vector3D& norm = poly.getNormal();
        Vector3D resultColor = ambientColor_ * colorToVec(baseColor);

        for (const auto& light : lights_) {
            if (!light.enabled) continue;

            Vector3D lightDir = (light.position - poly.calculateCenter()).normalize();
            double diffuse = std::max(0.0, dot(norm, lightDir)) * light.intensity;
            resultColor = resultColor + (diffuse * light.color) * colorToVec(baseColor);
        }

        return vecToColor(resultColor.clamp(0.0, 1.0));
    }

    std::vector<const Polygon*> sortPolygonByDepth(const std::vector<Polygon>& polygons) const {
        std::vector<std::pair<const Polygon*, double>> depthInfo;
        depthInfo.reserve(polygons.size());
    
        for (const auto& poly : polygons) {
            depthInfo.emplace_back(&poly, calculateDepth(poly.calculateCenter()));
        }
    
        std::sort(depthInfo.begin(), depthInfo.end(), 
            [](const auto& a, const auto& b) { return a.second > b.second; });
    
        std::vector<const Polygon*> sorted;
        sorted.reserve(polygons.size());
        for (const auto& info : depthInfo) {
            sorted.push_back(info.first);
        }
    
        return sorted;
    }

    void drawLine3D(const Vector3D& v1, const Vector3D& v2, uint32_t color);
    void drawPoly3D(const Polygon& poly, uint32_t color);
    void drawPolygonWithEdges(const Polygon& poly, uint32_t color);
    void drawPolyNormals(const Polygon& poly, uint32_t color);
    void drawMesh3D(const Mesh& mesh, uint32_t baseColor, bool drawEdges = false, bool drawNormals = false);
};

void Render::drawLine3D(const Vector3D& v1, const Vector3D& v2, uint32_t color) {
    Vector3D view1 = camera_.getViewMatrix() * v1;
    Vector3D view2 = camera_.getViewMatrix() * v2;

    if (!camera_.isPointInFrustrum(view1) || !camera_.isPointInFrustrum(view2)) {
        return;
    }

    std::pair<int, int> v1p = camera_.projectPoint(v1);
    std::pair<int, int> v2p = camera_.projectPoint(v2);

    gc_.drawLine(v1p.first, v1p.second, v2p.first, v2p.second, color);
}

void Render::drawPoly3D(const Polygon& poly, uint32_t color) {
    if (!camera_.isPolyInFrustrum(poly)) return;

    const auto& vertices = poly.getVertices();
    for (size_t i = 0; i < vertices.size() - 1; i++) {
        drawLine3D(vertices[i], vertices[i + 1], color);
    }
    drawLine3D(vertices.back(), vertices.front(), color);
}


void Render::drawPolyNormals(const Polygon& poly, uint32_t color=0xFFFF0000) {
    if (!camera_.isPolyInFrustrum(poly)) return;

    Vector3D center = poly.calculateCenter();
    Vector3D normalEnd = center + poly.getNormal() * 0.5f;
    drawLine3D(center, normalEnd, color);
}


void Render::drawPolygonWithEdges(const Polygon& poly, uint32_t color) {
    if (!camera_.isPolyInFrustrum(poly)) return;

    const Vector3D& normal = poly.getNormal();
    Vector3D viewDir = (poly.calculateCenter() - camera_.getPosition()).normalize(); // От камеры к вершине
    
    if (dot(normal, viewDir) >= 0) return; // Полигон спереди камеры

    uint32_t shadedColor = calculateShadedColor(poly, color);
    fillPoly3D(poly, shadedColor);
    drawPoly3D(poly, 0xFFFFFFFF);
}


void Render::drawMesh3D(const Mesh& mesh, uint32_t baseColor, bool drawEdges, bool drawNormals) {
    auto sorted = sortPolygonByDepth(mesh.getPolygons());

    for (const auto* polygonPtr : sorted) {
        const auto& polygon = *polygonPtr;
        if (!camera_.isPolyInFrustrum(polygon)) return;

        Vector3D normal = polygon.getNormal();
        Vector3D viewDir = (polygon.calculateCenter() - camera_.getPosition()).normalize();
        if (dot(normal, viewDir) >= 0) continue;

        uint32_t shadedColor = calculateShadedColor(polygon, baseColor);
        fillPoly3D(polygon, shadedColor);

        if (drawEdges) {
            drawPoly3D(polygon, 0xFFFFFFFF);
        } else {
            drawPoly3D(polygon, shadedColor);
        }

        if (drawNormals) {
            drawPolyNormals(polygon);
        }
    }
}