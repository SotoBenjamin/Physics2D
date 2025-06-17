#pragma once
#include "Math.h"
#include <bits/stdc++.h>

using Vec2 = Math::Vec2;
using Vec3 = Math::Vec3;
using Vec4 = Math::Vec4;

constexpr int PH2D_MAX_CONVEX_SHAPE_POINTS = 12;
//----------------Utilities--------------------//
inline Vec2 rotateAroundCenter(const Vec2& in, float r) {
    float c = std::cos(-r);
    float s = std::sin(-r);
    return {
        in.x * c - in.y * s,
        in.x * s + in.y * c
    };
}

inline Vec2 rotateAroundPoint(const Vec2& in, const Vec2& centerReff, float r) {
    Vec2 tmp = in - centerReff;
    tmp = rotateAroundCenter(tmp, r);
    return tmp + centerReff;
}

inline Vec2 rotationToVector(float angle) {
    float s = std::sin(angle);
    float c = std::cos(angle);
    Vec2 d(-s,c);
    return Math::normalize(d);
}

inline float vectorToRotation(const Vec2& v) {
    if (Math::length(v) == 0.0f) {
        return 0.0f;
    }
    return std::atan2(-v.x, v.y);
}

//---------------AABB--------------------------//
struct AABB {
    Vec2 pos{0,0};
    Vec2 size{0,0};

    AABB() = default;

    explicit AABB(const Vec4& a)
      : pos(a.x, a.y)
      , size(a.z, a.w)
    {}

    [[nodiscard]] Vec4 asVec4() const {
        return { pos.x, pos.y, size.x, size.y };
    }

    [[nodiscard]] Vec2 min()    const { return pos; }
    [[nodiscard]] Vec2 max()    const { return pos + size; }
    [[nodiscard]] Vec2 center() const { return pos + (size / 2.0f); }

    void getMinMaxPointsRotated(Vec2& outMin, Vec2& outMax, float r) const;

    void getCornersRotated(Vec2 corners[4], float r) const;

    void rotateAroundCenter(float r);

    [[nodiscard]] float down()  const { return pos.y + size.y; }
    [[nodiscard]] float top()   const { return pos.y; }
    [[nodiscard]] float left()  const { return pos.x; }
    [[nodiscard]] float right() const { return pos.x + size.x; }
};



inline void AABB::rotateAroundCenter(float r) {
    Vec2 newC = ::rotateAroundCenter(center(), r);
    pos = newC - (size / 2.0f);
}

inline void AABB::getMinMaxPointsRotated(Vec2& outMin, Vec2& outMax, float r) const {
    Vec2 p0 = min();
    Vec2 p1 = max();
    Vec2 p2{ p0.x, p1.y };
    Vec2 p3{ p1.x, p0.y };
    Vec2 c = center();

    p0 = rotateAroundPoint(p0, c, r);
    p1 = rotateAroundPoint(p1, c, r);
    p2 = rotateAroundPoint(p2, c, r);
    p3 = rotateAroundPoint(p3, c, r);

    outMin = p0;
    if (p1.x < outMin.x) outMin.x = p1.x;
    if (p2.x < outMin.x) outMin.x = p2.x;
    if (p3.x < outMin.x) outMin.x = p3.x;
    if (p1.y < outMin.y) outMin.y = p1.y;
    if (p2.y < outMin.y) outMin.y = p2.y;
    if (p3.y < outMin.y) outMin.y = p3.y;


    outMax = p0;
    if (p1.x > outMax.x) outMax.x = p1.x;
    if (p2.x > outMax.x) outMax.x = p2.x;
    if (p3.x > outMax.x) outMax.x = p3.x;
    if (p1.y > outMax.y) outMax.y = p1.y;
    if (p2.y > outMax.y) outMax.y = p2.y;
    if (p3.y > outMax.y) outMax.y = p3.y;
}

inline void AABB::getCornersRotated(Vec2 corners[4], float r) const {
    corners[0] = min();
    corners[1] = { min().x, max().y };
    corners[2] = max();
    corners[3] = { max().x, min().y };
    if (r != 0.0f) {
        Vec2 c = center();
        for (int i = 0; i < 4; ++i) {
            corners[i] = ::rotateAroundPoint(corners[i], c, r);
        }
    }
}

//--------------Circle--------------//
struct Circle {
    Vec2  center{};
    float r{0};

    Circle() = default;

    explicit Circle(const Vec3& a)
      : center(a.x, a.y)
      , r(a.z)
    {}

    Vec2 getTopLeftCorner() {
        return center += Vec2{-r, r};
    }

    AABB getAABB() {
        Vec2 tl = getTopLeftCorner();
        return AABB{ Vec4{ tl.x, tl.y, r * 2.0f, r * 2.0f } };
    }
};

//------------------LineEquation-------------------//

struct LineEquation {
    Vec3 lineEquation{};

    LineEquation() = default;

    [[nodiscard]] Vec2 getNormal() const {
        return { lineEquation.x, lineEquation.y };
    }

    void normalize() {
        Vec2 n{ lineEquation.x, lineEquation.y };
        float l = Math::length(n);
        if (l <= 1e-8f) {
            n = { 1, 0 };
        } else {
            n /= l;
        }
        lineEquation.x = n.x;
        lineEquation.y = n.y;
    }

    [[nodiscard]] float getDistanceFromCenter() const {
        return lineEquation.z;
    }

    void createFromNormalAndPoint(const Vec2& normal, const Vec2& point) {
        float c = -Math::dot(normal, point); // c = - (a*x + b*y)
        lineEquation = { normal.x, normal.y, c };
    }

    [[nodiscard]] float getDistanceToPoint(const Vec2& point) const {
        float num   = lineEquation.x * point.x
                    + lineEquation.y * point.y
                    + lineEquation.z;
        float den = std::sqrt(lineEquation.x * lineEquation.x
                              + lineEquation.y * lineEquation.y);
        return num / den;
    }

    void createFromRotationAndPoint(float rotation, const Vec2& point) {
        Vec2 n = rotationToVector(rotation);
        createFromNormalAndPoint(n, point);
    }

    [[nodiscard]] float computeEquation(const Vec2& p) const {
        return lineEquation.x * p.x
             + lineEquation.y * p.y
             + lineEquation.z;
    }

    [[nodiscard]] bool intersectPoint(const Vec2& p) const {
        return computeEquation(p) >= 0;
    }

    [[nodiscard]] Vec2 getClosestPointToOrigin() const {
        const Vec2 n = getNormal();
        return { -lineEquation.z * n.x,
                 -lineEquation.z * n.y };
    }

    [[nodiscard]] Vec2 getLineVector() const {
        return { -lineEquation.y,
                  lineEquation.x };
    }
};


struct ConvexPolygon {
    Vec2 vertexesObjectSpace[PH2D_MAX_CONVEX_SHAPE_POINTS];
    unsigned char vertexCount = 0;

    void getCornersRotated(Vec2 corners[PH2D_MAX_CONVEX_SHAPE_POINTS],
                           float angle) const;

    void getCornersRotatedInWorldSpace(Vec2 corners[PH2D_MAX_CONVEX_SHAPE_POINTS],
                                       float angle,
                                       const Vec2& centerPos) const;
};

inline void ConvexPolygon::getCornersRotated(Vec2 corners[],
                                             float angle) const
{
    int c = std::min<int>(vertexCount, PH2D_MAX_CONVEX_SHAPE_POINTS);
    for (int i = 0; i < c; ++i) {
        corners[i] = rotateAroundCenter(vertexesObjectSpace[i], angle);
    }
}

inline void ConvexPolygon::getCornersRotatedInWorldSpace(Vec2 corners[],
                                                         float angle,
                                                         const Vec2& centerPos) const
{
    int c = std::min<int>(vertexCount, PH2D_MAX_CONVEX_SHAPE_POINTS);
    for (int i = 0; i < c; ++i) {
        corners[i] = rotateAroundCenter(vertexesObjectSpace[i], angle)
                   + centerPos;
    }
}

//----------------  Collisions -----------//

inline bool AABBvsPoint(const AABB& a, const Vec2& b, float delta) {
    Vec2 aMin = a.min() - Vec2{ delta, delta };
    Vec2 aMax = a.max() + Vec2{ delta, delta };

    return (aMin.x < b.x && b.x < aMax.x &&
            aMin.y < b.y && b.y < aMax.y);
}


inline bool AABBvsAABB(const AABB& a, const AABB& b, float delta = 0) {
    Vec2 aMax = a.max() + Vec2{ delta, delta };
    Vec2 bMax = b.max() + Vec2{ delta, delta };
    Vec2 aMin = a.min() - Vec2{ delta, delta };
    Vec2 bMin = b.min() - Vec2{ delta, delta };

    if (aMax.x < bMin.x || aMin.x > bMax.x)
        return false;

    if (aMax.y < bMin.y || aMin.y > bMax.y)
        return false;

    return true;
}

/**
 * SAT test entre dos AABB para determinar colisión,
 * profundidad de penetración y normal de colisión.
 */
inline bool AABBvsAABB(const AABB& abox,
                       const AABB& bbox,
                       float& penetration,
                       Vec2& normal)
{
    // Vector de A hacia B
    Vec2 n = bbox.center() - abox.center();

    // Mitades de los ejes X
    float aExtentX = (abox.max().x - abox.min().x) * 0.5f;
    float bExtentX = (bbox.max().x - bbox.min().x) * 0.5f;

    // Solapamiento en X
    float xOverlap = aExtentX + bExtentX - std::fabs(n.x);
    if (xOverlap > 0.0f) {
        // Mitades de los ejes Y
        float aExtentY = (abox.max().y - abox.min().y) * 0.5f;
        float bExtentY = (bbox.max().y - bbox.min().y) * 0.5f;

        // Solapamiento en Y
        float yOverlap = aExtentY + bExtentY - std::fabs(n.y);
        if (yOverlap > 0.0f) {
            // El eje de menor penetración es el eje de colisión
            if (xOverlap < yOverlap) {
                // Normal hacia B en X
                normal = (n.x < 0.0f) ? Vec2{-1, 0} : Vec2{1, 0};
                penetration = xOverlap;
            } else {
                // Normal hacia B en Y
                normal = (n.y < 0.0f) ? Vec2{0, -1} : Vec2{0, 1};
                penetration = yOverlap;
            }
            return true;
        }
    }

    // Sin solapamiento en alguno de los ejes → no colisión
    return false;
}

inline bool AABBvsOBB(AABB a, AABB b, float bRotation) {
    Vec2 aMin = a.min();
    Vec2 aMax = a.max();
    Vec2 bMin, bMax;
    b.getMinMaxPointsRotated(bMin, bMax, bRotation);

    if (aMax.x < bMin.x || aMin.x > bMax.x) return false;
    if (aMax.y < bMin.y || aMin.y > bMax.y) return false;

    {
        AABB newA = a;
        AABB newB = b;
        float rotationA = -bRotation;
        Vec2 bCenter = b.center();

        newA.pos = newA.pos - bCenter;
        newB.pos = newB.pos - bCenter;

        newA.rotateAroundCenter(rotationA);

        Vec2 aMin2, aMax2;
        Vec2 bMin2 = newB.min();
        Vec2 bMax2 = newB.max();
        newA.getMinMaxPointsRotated(aMin2, aMax2, rotationA);

        if (aMax2.x < bMin2.x || aMin2.x > bMax2.x) return false;
        if (aMax2.y < bMin2.y || aMin2.y > bMax2.y) return false;
    }

    return true;
}

inline bool OBBvsOBB(AABB a, float aRotation, AABB b, float bRotation) {
    Vec2 aCenter = a.center();
    a.pos = a.pos - aCenter;
    b.pos = b.pos - aCenter;

    b.rotateAroundCenter(-aRotation);

    float brAdjusted = bRotation - aRotation;

    return AABBvsOBB(a, b, brAdjusted);
}

inline float calculatePenetrationAlongOneAxe(
    const Vec2* aPoints, size_t aPointsCount,
    const Vec2* bPoints, size_t bPointsCount,
    Vec2 axeDirection,
    bool* flipSign)
{
    if (aPointsCount == 0 || bPointsCount == 0 || !aPoints || !bPoints) {
        return 0.0f;
    }

    float d0a = Math::dot(aPoints[0], axeDirection);
    float d0b = Math::dot(bPoints[0], axeDirection);
    float aMin = d0a, aMax = d0a;
    float bMin = d0b, bMax = d0b;

    for (size_t i = 1; i < aPointsCount; ++i) {
        float d = Math::dot(aPoints[i], axeDirection);
        aMin = std::min(aMin, d);
        aMax = std::max(aMax, d);
    }

    for (size_t i = 1; i < bPointsCount; ++i) {
        float d = Math::dot(bPoints[i], axeDirection);
        bMin = std::min(bMin, d);
        bMax = std::max(bMax, d);
    }

    float overlapA = aMax - bMin;
    float overlapB = bMax - aMin;

    if (overlapA < overlapB) {
        if (flipSign) *flipSign = false;
        return overlapA;
    } else {
        if (flipSign) *flipSign = true;
        return overlapB;
    }
}


inline void clipPolygon(
    const Vec2* cornersA, int countA,
    const Vec2& normal,
    const Vec2* cornersB, int countB,
    std::vector<Vec2>& intersectionPoints)
{
    std::vector<Vec2> inputPolygon(cornersA, cornersA + countA);

    for (int i = 0; i < countB; ++i) {
        Vec2 edgeStart = cornersB[i];
        Vec2 edgeEnd   = cornersB[(i + 1) % countB];
        Vec2 edgeNormal = Math::normalize({ edgeEnd.y - edgeStart.y,
                                           edgeStart.x - edgeEnd.x });

        std::vector<Vec2> outputPolygon;
        for (size_t j = 0; j < inputPolygon.size(); ++j) {
            Vec2 current = inputPolygon[j];
            Vec2 next    = inputPolygon[(j + 1) % inputPolygon.size()];

            float currentDist = Math::dot(current - edgeStart, edgeNormal);
            float nextDist    = Math::dot(next    - edgeStart, edgeNormal);

            if (currentDist >= 0.0f) {
                outputPolygon.push_back(current);
            }

            if (currentDist * nextDist < 0.0f) {
                float t = currentDist / (currentDist - nextDist);
                Vec2 intersection = current + (next - current) * t;
                outputPolygon.push_back(intersection);
            }
        }

        inputPolygon = std::move(outputPolygon);
        if (inputPolygon.empty()) break;
    }

    intersectionPoints = std::move(inputPolygon);
}


inline Vec2 findClosestEdge(const Vec2* corners, int count, const Vec2& point)
{
    float minDist = std::numeric_limits<float>::infinity();
    Vec2 closestEdge{0.0f, 0.0f};

    for (int i = 0; i < count; ++i) {
        Vec2 edgeStart = corners[i];
        Vec2 edgeEnd   = corners[(i + 1) % count];
        Vec2 edge      = edgeEnd - edgeStart;
        Vec2 edgeNormal = Math::normalize({ -edge.y, edge.x });
        float dist = std::fabs(Math::dot(point - edgeStart, edgeNormal));
        if (dist < minDist) {
            minDist    = dist;
            closestEdge = edge;
        }
    }

    return Math::normalize(closestEdge);
}


inline bool OBBvsOBB(
    AABB a, float ar,
    AABB b, float br,
    float &penetration,
    Vec2 &normal,
    Vec2 &contactPoint,
    Vec2 &tangentA,
    Vec2 &tangentB)
{
    penetration = 0.0f;
    normal = {0, 0};

    Vec2 cornersA[4], cornersB[4];
    a.getCornersRotated(cornersA, ar);
    b.getCornersRotated(cornersB, br);

    Vec2 axes[4] = {
        rotateAroundCenter({0, 1}, ar), // eje A–Y rotado por ar
        rotateAroundCenter({1, 0}, ar), // eje A–X rotado por ar
        rotateAroundCenter({0, 1}, br), // eje B–Y rotado por br
        rotateAroundCenter({1, 0}, br)  // eje B–X rotado por br
    };

    float minPen = std::numeric_limits<float>::infinity();
    Vec2 minAxis = {0,0};
    bool flipSignAccum = false;

    for (int i = 0; i < 4; ++i) {
        bool flip = false;
        float pd = calculatePenetrationAlongOneAxe(
            cornersA, 4,
            cornersB, 4,
            axes[i],
            &flip
        );
        if (pd < 0.0f) return false;
        if (pd < minPen) {
            minPen = pd;
            minAxis = axes[i];
            flipSignAccum = flip;
        }
    }

    penetration = minPen;
    normal = Math::normalize(minAxis);
    if (flipSignAccum) {
        normal = -normal;
    }

    std::vector<Vec2> intersectionPoints;
    clipPolygon(
        cornersA, 4,
        normal,
        cornersB, 4,
        intersectionPoints
    );
    if (intersectionPoints.empty()) {
        return false;
    }

    contactPoint = {0, 0};
    for (auto &p : intersectionPoints) {
        contactPoint += p;
    }
    contactPoint /= static_cast<float>(intersectionPoints.size());

    tangentA = findClosestEdge(cornersA, 4, contactPoint);
    tangentB = findClosestEdge(cornersB, 4, contactPoint);

    return true;
}

inline bool OBBvsPoint(AABB a, float rotation, Vec2 b, float delta) {
    if (rotation == 0.0f) {
        return AABBvsPoint(a, b, delta);
    }

    Vec2 center = a.center();
    a.pos = a.pos - center;
    b     = b     - center;

    b = rotateAroundCenter(b, -rotation);

    return AABBvsPoint(a, b, delta);
}

inline bool CircleVsPoint(const Vec2& pos, float r, const Vec2& p, float delta) {
    Vec2 dist = pos - p;
    float sumR = r + delta;
    float rSquared = sumR * sumR;
    float distSquared = Math::dot(dist, dist);
    return distSquared < rSquared;
}


inline bool CirclevsCircle(
    const Circle& a,
    const Circle& b,
    float& penetration,
    Vec2& normal,
    Vec2& contactPoint)
{
    float sumR = a.r + b.r;
    float rSquared = sumR * sumR;

    float dx = a.center.x - b.center.x;
    float dy = a.center.y - b.center.y;
    float distanceSquared = dx*dx + dy*dy;

    bool rez = (rSquared > distanceSquared);
    if (rez) {
        normal = b.center - a.center;
        normal = Math::normalize(normal);

        penetration = sumR - std::sqrt(distanceSquared);

        contactPoint = a.center + normal * (a.r - penetration * 0.5f);
    }
    return rez;
}


/**
 * Colisión Half-Space vs Círculo.
 * @param line        Ecuación de la línea (normal + distancia).
 * @param circle      AABB que representa el círculo (size.x == 2·radio).
 * @param penetration Salida: profundidad de penetración (>0 si colisiona).
 * @param normal      Salida: vector normalizado de la colisión.
 * @param contactPoint Salida: punto de contacto estimado.
 * @return true si hay colisión.
 */
inline bool HalfSpaceVSCircle(LineEquation line,
                              AABB circle,
                              float& penetration,
                              Vec2& normal,
                              Vec2& contactPoint)
{
    float r = circle.size.x * 0.5f;

    line.normalize();

    normal = line.getNormal();

    Vec2 center = circle.center();
    float distance = line.computeEquation(center);

    penetration = r + distance;

    if (penetration > 0.0f) {
        contactPoint = center + normal * (r - penetration * 0.5f);
        return true;
    }
    return false;
}

/**
 * Colisión AABB vs Círculo (tratado como AABB).
 * @param abox        AABB del objeto A.
 * @param bbox        AABB que encierra al círculo B (size.x == 2·radio).
 * @param penetration Salida: profundidad de penetración.
 * @param normal      Salida: normal de la colisión.
 * @param contactPoint Salida: punto de contacto.
 * @return true si hay colisión.
 */
inline bool AABBvsCircle(const AABB& abox,
                         const AABB& bbox,
                         float& penetration,
                         Vec2& normal,
                         Vec2& contactPoint)
{
    normal = {0, 0};

    Vec2 n = bbox.center() - abox.center();

    Vec2 closest = n;

    float x_extent = (abox.max().x - abox.min().x) * 0.5f;
    float y_extent = (abox.max().y - abox.min().y) * 0.5f;

    closest.x = Math::clamp(closest.x, -x_extent, x_extent);
    closest.y = Math::clamp(closest.y, -y_extent, y_extent);

    bool inside = false;
    if (n.x == closest.x && n.y == closest.y) {
        inside = true;
        if (std::fabs(n.x) > std::fabs(n.y)) {
            closest.x = (closest.x > 0.0f ? x_extent : -x_extent);
        } else {
            closest.y = (closest.y > 0.0f ? y_extent : -y_extent);
        }
    }

    Vec2 normal2 = n - closest;
    float dist2 = Math::dot(normal2, normal2);
    float r = bbox.size.x * 0.5f;

    if (dist2 > r * r && !inside) {
        return false;
    }

    float d = std::sqrt(dist2);

    if (inside) {
        normal = -normal2;
        penetration = r - d;
        normal = Math::normalize(normal);
        contactPoint = bbox.center()
                     + (-normal * (r - std::min(penetration, r) * 0.5f));
    } else {
        normal = normal2;
        penetration = r - d;
        normal = Math::normalize(normal);
        contactPoint = bbox.center()
                     + (-normal * (r - penetration * 0.5f));
    }

    return true;
}

/**
 * Colisión OBB vs Círculo (tratado como AABB contenido en bbox).
 * @param abox         AABB del objeto A (orientado por ar).
 * @param ar           Rotación de A en radianes.
 * @param bbox         AABB que encierra al círculo B (size.x = 2·radio).
 * @param penetration  Salida: profundidad de penetración.
 * @param normal       Salida: normal de colisión en espacio world.
 * @param contactPoint Salida: punto de contacto en espacio world.
 * @return true si hay colisión.
 */
inline bool OBBvsCircle(
    AABB abox, float ar,
    AABB bbox,
    float& penetration,
    Vec2& normal,
    Vec2& contactPoint)
{
    if (ar == 0.0f) {
        return AABBvsCircle(abox, bbox, penetration, normal, contactPoint);
    }

    Vec2 centerA = abox.center();
    abox.pos = abox.pos - centerA;
    bbox.pos = bbox.pos - centerA;

    bbox.rotateAroundCenter(-ar);

    bool rez = AABBvsCircle(abox, bbox, penetration, normal, contactPoint);

    normal = rotateAroundCenter(normal, ar);
    normal = Math::normalize(normal);

    contactPoint = rotateAroundCenter(contactPoint, ar);
    contactPoint = contactPoint + centerA;

    return rez;
}


inline float orientationTest(const Vec2& A, const Vec2& B, const Vec2& C) {
    return (B.x - A.x) * (C.y - A.y)
         - (B.y - A.y) * (C.x - A.x);
}

/**
 * Colisión Círculo vs Polígono Convexo.
 * @param circle                  AABB que encierra al círculo (circle.size.x = 2·radio)
 * @param convexPolygon           Polígono convexo en espacio local
 * @param convexPolygonCenter     Centro del polígono en world space
 * @param rotation                Rotación del polígono (radianes)
 * @param penetration (out)       Profundidad de penetración
 * @param normal (out)            Normal de colisión, de A→B
 * @param contactPoint (out)      Punto aproximado de contacto
 * @return true si colisionan
 */
inline bool CirclevsConvexPolygon(
    const AABB& circle,
    const ConvexPolygon& convexPolygon,
    const Vec2& convexPolygonCenter,
    float rotation,
    float& penetration,
    Vec2& normal,
    Vec2& contactPoint)
{
    penetration = 0.0f;
    normal = {0.0f, 0.0f};

    Vec2 corners[PH2D_MAX_CONVEX_SHAPE_POINTS];
    convexPolygon.getCornersRotated(corners, rotation);
    int vCount = std::min<int>(convexPolygon.vertexCount, PH2D_MAX_CONVEX_SHAPE_POINTS);

    Vec2 circleCenter = circle.center();
    float circleRadius = circle.size.x * 0.5f;

    Vec2 toCircle = circleCenter - convexPolygonCenter;
    toCircle = Math::normalize(toCircle); // safe normalize

    int support = 0;
    float bestDot = Math::dot(corners[0], toCircle);
    for (int i = 1; i < vCount; ++i) {
        float d = Math::dot(corners[i], toCircle);
        if (d > bestDot) {
            bestDot = d;
            support = i;
        }
    }
    int closestSupport = support;

    int leftIdx  = (support - 1 + vCount) % vCount;
    int rightIdx = (support + 1) % vCount;

    Vec2 supportWorld   = corners[support]   + convexPolygonCenter;
    Vec2 leftWorld      = corners[leftIdx]   + convexPolygonCenter;
    Vec2 rightWorld     = corners[rightIdx]  + convexPolygonCenter;

    contactPoint = supportWorld; // punto base

    Vec2 dirToCircle = circleCenter - supportWorld;
    Vec2 dirLeft     = leftWorld  - supportWorld;  dirLeft  = Math::normalize(dirLeft);
    Vec2 dirRight    = rightWorld - supportWorld;  dirRight = Math::normalize(dirRight);

    int secondSupport;
    if (Math::dot(dirLeft, dirToCircle) > Math::dot(dirRight, dirToCircle)) {
        secondSupport = leftIdx;
    } else {
        secondSupport = rightIdx;
    }

    if (orientationTest({0,0}, corners[support], corners[secondSupport]) < 0) {
        std::swap(support, secondSupport);
    }

    Vec2 edge = corners[support] - corners[secondSupport];
    edge = Math::normalize(edge);
    normal = { edge.y, -edge.x };

    LineEquation contactLine;
    contactLine.createFromNormalAndPoint(-normal, corners[support] + convexPolygonCenter);

    if (contactLine.getDistanceToPoint(circleCenter) > circleRadius) {
        return false;
    }

    Vec2 cornerInWorld = corners[closestSupport] + convexPolygonCenter;
    Vec2 toCorner      = cornerInWorld - circleCenter;
    if (Math::dot(corners[support] - corners[secondSupport], dirToCircle) < 0 ||
        Math::dot(corners[secondSupport] - corners[support], toCircle) < 0)
    {
        // Compruebo si está fuera del radio
        if (Math::distance(cornerInWorld, circleCenter) > circleRadius) {
            return false;
        }
        // Ajusto normal
        normal = toCorner;
        float len = Math::length(normal);
        if (len > 1e-8f) {
            normal /= len;
        } else {
            normal = { edge.y, -edge.x };
        }
        Vec2 circleEdgePt = circleCenter + normal * circleRadius;
        contactPoint = (circleEdgePt + cornerInWorld) * 0.5f;
        penetration = Math::distance(circleEdgePt, cornerInWorld);
    }
    else
    {
        Vec2 circleEdgePt = circleCenter + normal * circleRadius;
        penetration = contactLine.getDistanceToPoint(circleEdgePt);
        contactPoint = circleEdgePt + normal * (penetration * 0.5f);
    }

    normal = Math::normalize(normal);
    return true;
}


/**
 * Colisión medio-plano (half-space) vs OBB.
 * @param line           Ecuación del medio-plano (normal + c).
 * @param bbox           AABB que representa el OBB (tamaño, pos).
 * @param rotation       Rotación del OBB en radianes.
 * @param penetration    (out) Profundidad de penetración.
 * @param normal         (out) Normal de colisión.
 * @param contactPoint   (out) Punto de contacto.
 * @param tangentA       (out) Tangente respecto al plano.
 * @param tangentB       (out) Tangente de la intersección con OBB.
 * @return true si colisionan.
 */
inline bool HalfSpaceVsOBB(
    LineEquation line,
    const AABB& bbox,
    float rotation,
    float& penetration,
    Vec2& normal,
    Vec2& contactPoint,
    Vec2& tangentA,
    Vec2& tangentB)
{
    Vec2 corners[4];
    bbox.getCornersRotated(corners, rotation);

    line.normalize();
    normal = line.getNormal();

    std::vector<Vec2> intersectionPoints;
    intersectionPoints.reserve(6);

    for (int i = 0; i < 4; ++i) {
        Vec2 start = corners[i];
        Vec2 end   = corners[(i + 1) % 4];

        float d0 = line.computeEquation(start);
        float d1 = line.computeEquation(end);

        if (d0 >= 0.0f) {
            intersectionPoints.push_back(start);
        }
        if (d0 * d1 < 0.0f) {
            float t = d0 / (d0 - d1);
            Vec2 ip = start + (end - start) * t;
            intersectionPoints.push_back(ip);
        }
    }

    if (intersectionPoints.empty()) {
        return false;
    }

    // 4) Determino tangentB según nº de puntos
    tangentB = {0, 0};
    Vec2 bestStart{0,0}, bestEnd{0,0};
    size_t count = intersectionPoints.size();

    if (count == 2) {
        bestStart = intersectionPoints[0];
        bestEnd   = intersectionPoints[1];
        tangentB  = Math::normalize(bestEnd - bestStart);
    }
    else if (count == 3) {
        float bestPen = -std::numeric_limits<float>::infinity();
        int  idx      = 0;
        for (int i = 0; i < 3; ++i) {
            float pd = line.computeEquation(intersectionPoints[i]);
            if (pd > bestPen) {
                bestPen = pd;
                idx     = i;
            }
        }
        Vec2 e1 = intersectionPoints[idx]
                - intersectionPoints[(idx + 1) % 3];
        Vec2 e2 = intersectionPoints[idx]
                - intersectionPoints[(idx + 2) % 3];
        float l1 = Math::length(e1), l2 = Math::length(e2);
        if (l1 > l2 && l1 > 0.0f) {
            tangentB = e1 / l1;
        } else if (l2 > 0.0f) {
            tangentB = e2 / l2;
        }
    }
    else {
        float bestPen = -std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < count; ++i) {
            Vec2 s = intersectionPoints[i];
            Vec2 e = intersectionPoints[(i + 1) % count];
            float sum = line.computeEquation(s) + line.computeEquation(e);
            if (sum > bestPen) {
                bestPen   = sum;
                bestStart = s;
                bestEnd   = e;
            }
        }
        tangentB = Math::normalize(bestEnd - bestStart);
    }

    Vec2 centroid{0,0};
    for (auto& p : intersectionPoints) {
        centroid += p;
    }
    centroid /= static_cast<float>(intersectionPoints.size());
    contactPoint = centroid;

    penetration = 0.0f;
    for (auto& p : intersectionPoints) {
        float d = line.computeEquation(p);
        if (d > penetration) penetration = d;
    }

    tangentA = line.getLineVector();

    return true;
}
/**
 * Colisión medio-plano vs polígono convexo.
 *
 * @param line                    Ecuación del medio-plano.
 * @param convexPolygon           Polígono convexo (vértices en object space).
 * @param convexPolygonCenter     Centro del polígono en world space.
 * @param rotation                Rotación del polígono (radianes).
 * @param penetration (out)       Profundidad de penetración.
 * @param normal      (out)       Normal de colisión (apunta de A a B).
 * @param contactPoint (out)       Punto de contacto aproximado.
 * @param tangentA    (out)       Tangente del medio-plano.
 * @param tangentB    (out)       Tangente del polígono.
 * @return true si hay colisión.
 */
inline bool HalfSpaceVsConvexPolygon(
    LineEquation line,
    const ConvexPolygon& convexPolygon,
    const Vec2& convexPolygonCenter,
    float rotation,
    float& penetration,
    Vec2& normal,
    Vec2& contactPoint,
    Vec2& tangentA,
    Vec2& tangentB)
{
    line.normalize();
    normal = line.getNormal();

    Vec2 corners[PH2D_MAX_CONVEX_SHAPE_POINTS];
    convexPolygon.getCornersRotated(corners, rotation);
    int vCount = std::min<int>(convexPolygon.vertexCount, PH2D_MAX_CONVEX_SHAPE_POINTS);
    if (vCount < 3) return false;

    for (int i = 0; i < vCount; ++i) {
        corners[i] += convexPolygonCenter;
    }

    std::vector<Vec2> intersectionPoints;
    intersectionPoints.reserve(vCount + 2);

    for (int i = 0; i < vCount; ++i) {
        Vec2 start = corners[i];
        Vec2 end   = corners[(i + 1) % vCount];

        float d0 = line.computeEquation(start);
        float d1 = line.computeEquation(end);

        if (d0 >= 0.0f) {
            intersectionPoints.push_back(start);
        }
        if (d0 * d1 < 0.0f) {
            float t = d0 / (d0 - d1);
            intersectionPoints.push_back(start + (end - start) * t);
        }
    }

    if (intersectionPoints.empty()) {
        return false;
    }

    tangentB = {0,0};
    Vec2 bestS{0,0}, bestE{0,0};
    size_t ipCount = intersectionPoints.size();

    if (ipCount == 2) {
        bestS     = intersectionPoints[0];
        bestE     = intersectionPoints[1];
        tangentB  = Math::normalize(bestE - bestS);
    }
    else if (ipCount == 3) {
        float bestPen = -std::numeric_limits<float>::infinity();
        int   idx     = 0;
        for (int i = 0; i < 3; ++i) {
            float pd = line.computeEquation(intersectionPoints[i]);
            if (pd > bestPen) {
                bestPen = pd;
                idx     = i;
            }
        }
        Vec2 e1 = intersectionPoints[idx]
                - intersectionPoints[(idx + 1) % 3];
        Vec2 e2 = intersectionPoints[idx]
                - intersectionPoints[(idx + 2) % 3];
        float l1 = Math::length(e1), l2 = Math::length(e2);
        if (l1 > l2 && l1 > 0.0f) {
            tangentB = e1 / l1;
        } else if (l2 > 0.0f) {
            tangentB = e2 / l2;
        }
    }
    else {
        float bestSum = -std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < ipCount; ++i) {
            Vec2 s = intersectionPoints[i];
            Vec2 e = intersectionPoints[(i + 1) % ipCount];
            float sum = line.computeEquation(s) + line.computeEquation(e);
            if (sum > bestSum) {
                bestSum = sum;
                bestS   = s;
                bestE   = e;
            }
        }
        tangentB = Math::normalize(bestE - bestS);
    }

    contactPoint = {0,0};
    for (auto& p : intersectionPoints) {
        contactPoint += p;
    }
    contactPoint /= static_cast<float>(intersectionPoints.size());

    penetration = 0.0f;
    for (auto& p : intersectionPoints) {
        float d = line.computeEquation(p);
        if (d > penetration) penetration = d;
    }

    tangentA = line.getLineVector();

    return true;
}