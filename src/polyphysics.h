#pragma once
#include "Math.h"
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <bitset>
#include <algorithm>
#include <limits>
#include <cstring>

using ph2dUserData = std::uint64_t;
using ph2dBodyId = std::uint32_t;
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

    Vec2 min()    const { return pos; }
    Vec2 max()    const { return pos + size; }
    Vec2 center() const { return pos + (size / 2.0f); }

    void getMinMaxPointsRotated(Vec2& outMin, Vec2& outMax, float r) const;

    void getCornersRotated(Vec2 corners[4], float r) const;

    void rotateAroundCenter(float r);

    float down()  const { return pos.y + size.y; }
    float top()   const { return pos.y; }
    float left()  const { return pos.x; }
    float right() const { return pos.x + size.x; }
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

    Vec2 getNormal() const {
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

    float getDistanceFromCenter() const {
        return lineEquation.z;
    }

    void createFromNormalAndPoint(const Vec2& normal, const Vec2& point) {
        float c = -Math::dot(normal, point); // c = - (a*x + b*y)
        lineEquation = { normal.x, normal.y, c };
    }

    float getDistanceToPoint(const Vec2& point) const {
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

    float computeEquation(const Vec2& p) const {
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
    Vec2 n = bbox.center() - abox.center();

    float aExtentX = (abox.max().x - abox.min().x) * 0.5f;
    float bExtentX = (bbox.max().x - bbox.min().x) * 0.5f;

    float xOverlap = aExtentX + bExtentX - std::fabs(n.x);
    if (xOverlap > 0.0f) {
        // Mitades de los ejes Y
        float aExtentY = (abox.max().y - abox.min().y) * 0.5f;
        float bExtentY = (bbox.max().y - bbox.min().y) * 0.5f;

        float yOverlap = aExtentY + bExtentY - std::fabs(n.y);
        if (yOverlap > 0.0f) {
            if (xOverlap < yOverlap) {
                normal = (n.x < 0.0f) ? Vec2{-1, 0} : Vec2{1, 0};
                penetration = xOverlap;
            } else {
                normal = (n.y < 0.0f) ? Vec2{0, -1} : Vec2{0, 1};
                penetration = yOverlap;
            }
            return true;
        }
    }

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

    for (auto axe : axes) {
        bool flip = false;
        float pd = calculatePenetrationAlongOneAxe(
            cornersA, 4,
            cornersB, 4,
            axe,
            &flip
        );
        if (pd < 0.0f) return false;
        if (pd < minPen) {
            minPen = pd;
            minAxis = axe;
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
    normal = line.getNormal(); // La normal del plano (apunta hacia el espacio "vacío")

    Vec2 center = circle.center();
    // Esta es la distancia con signo del centro del círculo al plano.
    // Será negativa si el centro está "dentro" del material del suelo.
    float distance = line.computeEquation(center);

    // Si la distancia es mayor que el radio, no hay contacto.
    if (distance > r) {
        return false;
    }

    // La penetración es la distancia que el borde del círculo ha cruzado.
    penetration = r - distance;

    // El punto de contacto es el punto en la superficie del círculo más profundo.
    contactPoint = center - normal * r;

    return true;
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
    Vec2& tangentA,  // Nota: No usaremos las tangentes en esta versión simplificada
    Vec2& tangentB)  // ya que no son la causa del problema.
{
    // 1. Obtener la normal del plano (suelo)
    line.normalize();
    normal = line.getNormal();

    // 2. Obtener los 4 vértices de la caja rotada en el mundo
    Vec2 corners[4];
    bbox.getCornersRotated(corners, rotation);

    // 3. Encontrar los vértices que están penetrando y la máxima penetración
    float maxPenetration = -std::numeric_limits<float>::infinity();
    std::vector<Vec2> contactPoints;
    contactPoints.reserve(4);

    for (int i = 0; i < 4; ++i)
    {
        // Calcula la distancia con signo de cada vértice al plano
        float distance = line.computeEquation(corners[i]);

        // Si la distancia es negativa, el vértice ha penetrado el plano
        if (distance <= 0.0f)
        {
            // La profundidad de penetración de este vértice es -distancia
            maxPenetration = std::max(maxPenetration, -distance);
            contactPoints.push_back(corners[i]);
        }
    }

    // 4. Si ningún vértice ha penetrado, no hay colisión
    if (contactPoints.empty())
    {
        return false;
    }

    // 5. Hay colisión. Asignar los valores de salida.
    penetration = maxPenetration;

    // El punto de contacto es el promedio (centroide) de los puntos que penetraron.
    Vec2 centroid = {0,0};
    for(const auto& p : contactPoints)
    {
        centroid += p;
    }
    contactPoint = centroid / static_cast<float>(contactPoints.size());

    // Las tangentes se pueden calcular si son necesarias, pero por ahora las ignoramos.
    tangentA = line.getLineVector();
    tangentB = {0,0}; // No es crítico para la resolución de la colisión.

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

enum BodyFlag : int {
    BodyFlag_freezeX        = 0,
    BodyFlag_freezeY        = 1,
    BodyFlag_freezeRotation = 2,
    BodyFlag_Sensor         = 3
};


struct BodyFlags {
    std::bitset<8> flags{};

    bool isKinematic() const {
        return isFreezeX() && isFreezeY() && isFreezeRotation();
    }
    void setKinematic(bool k = true) {
        setFreezeX(k);
        setFreezeY(k);
        setFreezeRotation(k);
    }

    bool isFreezeX() const                 { return flags.test(BodyFlag_freezeX); }
    void setFreezeX(bool f = true)         { flags.set(BodyFlag_freezeX, f); }

    bool isFreezeY() const                 { return flags.test(BodyFlag_freezeY); }
    void setFreezeY(bool f = true)         { flags.set(BodyFlag_freezeY, f); }

    bool isFreezePosition() const          { return isFreezeX() && isFreezeY(); }
    void setFreezePosition(bool f = true)  { setFreezeX(f); setFreezeY(f); }

    bool isFreezeRotation() const          { return flags.test(BodyFlag_freezeRotation); }
    void setFreezeRotation(bool f = true)  { flags.set(BodyFlag_freezeRotation, f); }

    bool isSensor() const                  { return flags.test(BodyFlag_Sensor); }
    void setSensor(bool s = true)          { flags.set(BodyFlag_Sensor, s); }
};


struct MotionState {
    Vec2 pos{0,0};
    Vec2 lastPos{0,0};
    Vec2 velocity{0,0};
    Vec2 acceleration{0,0};

    float rotation{0.0f};          ///< en radianes
    float angularVelocity{0.0f};
    float torque{0.0f};
    float mass{1.0f};
    float momentOfInertia{1.0f};

    void setPos(const Vec2& p) {
        pos     = p;
        lastPos = p;
    }

    /**
     * Aplica un impulso en coordenadas del objeto:
     *   - impulse: vector impulso
     *   - contactVector: vector desde el centro de masa al punto de contacto
     *   - flags: estados de congelación
     */
    void applyImpulseObjectPosition(const Vec2& impulse,
                                    const Vec2& contactVector,
                                    const BodyFlags& flags);

    /**
     * Aplica un impulso dado en coordenadas globales:
     *   - impulse: vector impulso
     *   - contactVectorWorldPos: punto de aplicación (world)
     *   - flags: estados de congelación
     */
    void applyImpulseWorldPosition(const Vec2& impulse,
                                   const Vec2& contactVectorWorldPos,
                                   const BodyFlags& flags);
};



inline void MotionState::applyImpulseObjectPosition(
    const Vec2& impulse,
    const Vec2& contactVector,
    const BodyFlags& flags)
{
    if (mass != 0.0f && mass != std::numeric_limits<float>::infinity()) {
        Vec2 deltaV =  impulse*(1.0f / mass) ;
        if (flags.isFreezeX()) deltaV.x = 0.0f;
        if (flags.isFreezeY()) deltaV.y = 0.0f;
        velocity += deltaV;
    }

    if (momentOfInertia != 0.0f
        && momentOfInertia != std::numeric_limits<float>::infinity()
        && !flags.isFreezeRotation())
    {
        float angularImpulse = Math::cross(contactVector, impulse) * 0.95f;
        angularVelocity    -= angularImpulse / momentOfInertia;
    }
}

inline void MotionState::applyImpulseWorldPosition(
    const Vec2& impulse,
    const Vec2& contactVectorWorldPos,
    const BodyFlags& flags)
{
    Vec2 relVec = contactVectorWorldPos - pos;
    applyImpulseObjectPosition(impulse, relVec, flags);
}


enum ColliderType : unsigned char {
    ColliderNone = 0,
    ColliderCircle,
    ColliderBox,
    ColliderHalfSpace,
    ColliderConvexPolygon
};
struct Collider {
    union {
        struct { Vec2 size; }        box;
        struct { float radius; }     circle;
        ConvexPolygon               convexPolygon;
    } collider{};
    unsigned char type = ColliderNone;

    /** Calcula la masa según la forma */
    float computeMass() const {
        switch (type) {
        case ColliderCircle:
            return collider.circle.radius * collider.circle.radius * 3.1415926f;

        case ColliderBox:
            return collider.box.size.x * collider.box.size.y;

        case ColliderConvexPolygon: {
            float area = 0.0f;
            int c = collider.convexPolygon.vertexCount;
            for (int i = 0; i < c; ++i) {
                Vec2 a = collider.convexPolygon.vertexesObjectSpace[i];
                Vec2 b = collider.convexPolygon.vertexesObjectSpace[(i + 1) % c];
                area += Math::cross(a, b) * 0.5f;
            }
            return area;
        }

        default:
            return 0.0f;
        }
    }

    /**
     * @brief Calcula el momento de inercia para esta forma, dado su mass.
     * @param mass Masa ya calculada con computeMass().
     */
    float computeMomentOfInertia(float mass) const {
        switch (type) {
        case ColliderCircle:
            return mass * collider.circle.radius * collider.circle.radius * 0.5f;

        case ColliderBox:
            return mass * (collider.box.size.x * collider.box.size.x
                         + collider.box.size.y * collider.box.size.y)
                         * (1.0f / 12.0f);

        case ColliderConvexPolygon: {
            float moi = 0.0f;
            int c = collider.convexPolygon.vertexCount;
            for (int i = 0; i < c; ++i) {
                Vec2 a = collider.convexPolygon.vertexesObjectSpace[i];
                Vec2 b = collider.convexPolygon.vertexesObjectSpace[(i + 1) % c];
                // contribución del triángulo (0,a,b)
                float crossAB = Math::cross(a, b);
                float term = Math::dot(a, a)
                           + Math::dot(a, b)
                           + Math::dot(b, b);
                moi += crossAB * term / 2.0f;
            }
            return moi * (mass / 6.0f);
        }

        default:
            return 0.0f;
        }
    }
};

/** Crea un Collider tipo caja de tamaño `size` */
inline Collider createBoxCollider(const Vec2& size) {
    Collider c;
    c.type = ColliderBox;
    c.collider.box.size = size;
    return c;
}

/** Crea un Collider tipo círculo de radio `r` */
inline Collider createCircleCollider(float r) {
    Collider c;
    c.type = ColliderCircle;
    c.collider.circle.radius = r;
    return c;
}

/**
 * Crea un Collider tipo polígono convexo.
 * @param shape  Array de vértices en object space.
 * @param count  Número de vértices (<= PH2D_MAX_CONVEX_SHAPE_POINTS).
 */
inline Collider createConvexPolygonCollider(const Vec2* shape, unsigned char count) {
    Collider c;
    c.type = ColliderConvexPolygon;
    unsigned char cmax = std::min<unsigned char>(count, PH2D_MAX_CONVEX_SHAPE_POINTS);
    std::memcpy(c.collider.convexPolygon.vertexesObjectSpace,
                shape,
                static_cast<size_t>(cmax) * sizeof(Vec2));
    c.collider.convexPolygon.vertexCount = cmax;
    return c;
}

struct Body {
    Collider    collider{};
    ph2dUserData userData{};
    MotionState motionState{};
    Vec2        gravityScale{1,1};
    float       elasticity{0.2f};
    float       staticFriction{0.7f};
    float       dynamicFriction{0.6f};
    float       dragScale{1.0f};
    float       angularDragScale{1.0f};
    BodyFlags   flags{};

    /** Impulso en coordenadas locales */
    void applyImpulseObjectPosition(const Vec2& impulse,
                                           const Vec2& contactVector)
    {
        motionState.applyImpulseObjectPosition(impulse, contactVector, flags);
    }

    /** Impulso en coordenadas globales */
    void applyImpulseWorldPosition(const Vec2& impulse,
                                          const Vec2& contactPointWorld)
    {
        motionState.applyImpulseWorldPosition(impulse, contactPointWorld, flags);
    }

    /** AABB que encierra este body, según su collider y posición */
    AABB getAABB() const {
        switch (collider.type) {
            case ColliderCircle: {
                Vec2 c = motionState.pos;
                float r = collider.collider.circle.radius;
                Vec4 v{ c.x - r, c.y - r, 2*r, 2*r };
                return AABB{ v };
            }
            case ColliderBox: {
                Vec2 c = motionState.pos;
                Vec2 s = collider.collider.box.size;
                // AABB pos = center - size/2
                Vec4 v{ c.x - s.x*0.5f, c.y - s.y*0.5f, s.x, s.y };
                return AABB{ v };
            }
            case ColliderConvexPolygon: {
                const auto& poly = collider.collider.convexPolygon;
                int c = poly.vertexCount;
                if (c == 0) return AABB{};
                Vec2 mn = poly.vertexesObjectSpace[0];
                Vec2 mx = mn;
                for (int i = 1; i < c; ++i) {
                    Vec2 p = poly.vertexesObjectSpace[i];
                    mn.x = std::min(mn.x, p.x);
                    mn.y = std::min(mn.y, p.y);
                    mx.x = std::max(mx.x, p.x);
                    mx.y = std::max(mx.y, p.y);
                }
                Vec2 pos = mn + motionState.pos;
                Vec2 size = mx - mn;
                Vec4 v{ pos.x, pos.y, size.x, size.y };
                return AABB{ v };
            }
            default:
                return AABB{};
        }
    }

    bool isHalfPlane() const {
        return collider.type == ColliderHalfSpace;
    }

    /** Intersección punto vs collider de este body */
    bool intersectPoint(const Vec2& p, float delta = 0.0f) const {
        switch (collider.type) {
            case ColliderCircle:
                return CircleVsPoint(motionState.pos,
                                     collider.collider.circle.radius,
                                     p, delta);
            case ColliderBox:
                return OBBvsPoint(getAABB(),
                                  motionState.rotation,
                                  p, delta);
            case ColliderHalfSpace: {
                LineEquation eq;
                eq.createFromRotationAndPoint(motionState.rotation,
                                              motionState.pos);
                return (eq.computeEquation(p) + delta) >= 0.0f;
            }
            case ColliderConvexPolygon: {
                const auto& poly = collider.collider.convexPolygon;
                Vec2 center = motionState.pos;
                for (int i = 0; i < poly.vertexCount; ++i) {
                    Vec2 v1 = poly.vertexesObjectSpace[i] + center;
                    Vec2 v2 = poly.vertexesObjectSpace[(i+1)%poly.vertexCount] + center;
                    if (pointInTriangle(p, v1, v2, center))
                        return true;
                }
                return false;
            }
            default:
                return false;
        }
    }

    /** Línea que define el half-space, si este body es de tipo HalfSpace */
    LineEquation getLineEquationForHalfPlaneColliders() const {
        if (collider.type != ColliderHalfSpace) {
            return LineEquation{};
        }
        LineEquation eq;
        eq.createFromRotationAndPoint(motionState.rotation,
                                      motionState.pos);
        return eq;
    }

private:
    static bool pointInTriangle(const Vec2& pt,
                                const Vec2& v1,
                                const Vec2& v2,
                                const Vec2& v3)
    {
        auto sign = [](const Vec2& p1,
                       const Vec2& p2,
                       const Vec2& p3)
        {
            return (p1.x - p3.x)*(p2.y - p3.y)
                 - (p2.x - p3.x)*(p1.y - p3.y);
        };
        float d1 = sign(pt, v1, v2);
        float d2 = sign(pt, v2, v3);
        float d3 = sign(pt, v3, v1);
        bool hasNeg = (d1<0)||(d2<0)||(d3<0);
        bool hasPos = (d1>0)||(d2>0)||(d3>0);
        return !(hasNeg && hasPos);
    }
};


inline bool BodyvsBody(
    Body& A, Body& B,
    float& penetration,
    Vec2& normal,
    Vec2& contactPoint,
    Vec2& tangentA,
    Vec2& tangentB,
    bool* reverseOrder = nullptr)
{
    tangentA = {0,0};
    tangentB = {0,0};
    if (reverseOrder) *reverseOrder = false;

    if (A.collider.type == ColliderCircle && B.collider.type == ColliderCircle) {
        Circle ca{ Vec3{A.motionState.pos.x,
                        A.motionState.pos.y,
                        A.collider.collider.circle.radius} };
        Circle cb{ Vec3{B.motionState.pos.x,
                        B.motionState.pos.y,
                        B.collider.collider.circle.radius} };
        return CirclevsCircle(ca, cb,
                              penetration, normal, contactPoint);
    }

    if (A.collider.type == ColliderBox && B.collider.type == ColliderBox) {
        AABB aabbA = A.getAABB();
        AABB aabbB = B.getAABB();
        return OBBvsOBB(aabbA, A.motionState.rotation,
                        aabbB, B.motionState.rotation,
                        penetration, normal, contactPoint,
                        tangentA, tangentB);
    }

    if (A.collider.type == ColliderBox && B.collider.type == ColliderCircle) {
        AABB aabbA = A.getAABB();
        AABB aabbB = B.getAABB();
        return OBBvsCircle(aabbA, A.motionState.rotation,
                           aabbB,
                           penetration, normal, contactPoint);
    }

    if (A.collider.type == ColliderCircle && B.collider.type == ColliderBox) {
        AABB aabbA = A.getAABB();
        AABB aabbB = B.getAABB();
        bool rez = OBBvsCircle(aabbB, B.motionState.rotation,
                               aabbA,
                               penetration, normal, contactPoint);
        normal = -normal;
        return rez;
    }

    if (A.collider.type == ColliderHalfSpace && B.collider.type == ColliderCircle) {
        AABB aabbB = B.getAABB();
        LineEquation eq = A.getLineEquationForHalfPlaneColliders();
        bool rez = HalfSpaceVSCircle(eq,
                                     aabbB,
                                     penetration, normal, contactPoint);

        return rez;
    }

    if (A.collider.type == ColliderCircle && B.collider.type == ColliderHalfSpace) {
        AABB aabbA = A.getAABB();
        LineEquation eq = B.getLineEquationForHalfPlaneColliders();
        if (reverseOrder) *reverseOrder = true;
        return HalfSpaceVSCircle(eq,
                                 aabbA,
                                 penetration, normal, contactPoint);
    }

    if (A.collider.type == ColliderHalfSpace && B.collider.type == ColliderBox) {
        AABB aabbB = B.getAABB();
        LineEquation eq = A.getLineEquationForHalfPlaneColliders();
        bool rez = HalfSpaceVsOBB(eq,
                                 aabbB,
                                 B.motionState.rotation,
                                 penetration, normal, contactPoint,
                                 tangentA, tangentB);
        return rez;
    }

    if (A.collider.type == ColliderBox && B.collider.type == ColliderHalfSpace) {
        AABB aabbA = A.getAABB();
        LineEquation eq = B.getLineEquationForHalfPlaneColliders();
        if (reverseOrder) *reverseOrder = true;
        return HalfSpaceVsOBB(eq,
                             aabbA,
                             A.motionState.rotation,
                             penetration, normal, contactPoint,
                             tangentB, tangentA);
    }

    if (A.collider.type == ColliderHalfSpace &&
        B.collider.type == ColliderConvexPolygon)
    {
        LineEquation eq = A.getLineEquationForHalfPlaneColliders();
        bool rez = HalfSpaceVsConvexPolygon(
            eq,
            B.collider.collider.convexPolygon,
            B.motionState.pos,
            B.motionState.rotation,
            penetration, normal, contactPoint,
            tangentA, tangentB);
        normal = -normal;
        return rez;
    }

    if (A.collider.type == ColliderConvexPolygon &&
        B.collider.type == ColliderHalfSpace)
    {
        LineEquation eq = B.getLineEquationForHalfPlaneColliders();
        if (reverseOrder) *reverseOrder = true;
        return HalfSpaceVsConvexPolygon(
            eq,
            A.collider.collider.convexPolygon,
            A.motionState.pos,
            A.motionState.rotation,
            penetration, normal, contactPoint,
            tangentB, tangentA);
    }

    if (A.collider.type == ColliderCircle &&
        B.collider.type == ColliderConvexPolygon)
    {
        if (reverseOrder) *reverseOrder = true;
        return CirclevsConvexPolygon(
            A.getAABB(),
            B.collider.collider.convexPolygon,
            B.motionState.pos,
            B.motionState.rotation,
            penetration, normal, contactPoint);
    }

    if (A.collider.type == ColliderConvexPolygon &&
        B.collider.type == ColliderCircle)
    {
        bool rez = CirclevsConvexPolygon(
            B.getAABB(),
            A.collider.collider.convexPolygon,
            A.motionState.pos,
            A.motionState.rotation,
            penetration, normal, contactPoint);
        normal = -normal;
        return rez;
    }

    return false;
}


struct ManifoldIntersection
{
    Vec2        normal{};        // normal de colisión (de A a B)
    Vec2        contactPoint{};  // punto de contacto
    Vec2        tangentA{};      // tangente sobre A
    Vec2        tangentB{};      // tangente sobre B
    float       penetration{0};  // profundidad de penetración
    ph2dBodyId  A{0};            // id del body A
    ph2dBodyId  B{0};            // id del body B
    float       massA{0};        // masa del body A
    float       massB{0};        // masa del body B
};


struct SimulationPhysicsSettings
{
    Vec2  gravity{0.0f, 9.81f};
    float maxVelocity            {10000.0f};
    float minVelocity            {   0.1f};
    float minAngularVelocity     {   0.01f};
    float maxAcceleration        {10000.0f};
    float maxAirDrag             {  100.0f};
    float maxAngularDrag         {  100.0f};
    float airDragCoefficient     {    0.1f};
    float rotationalDragCoefficient{   0.01f};
    float restingVelocity        {    0.01f};
    // 0.01 grados → radianes = 0.01 * π/180
    float restingAngularVelocity { 0.01f * (3.14159265358979323846f / 180.0f) };
};

struct Constrain
{
    ph2dBodyId A{};              // id del primer body
    ph2dBodyId B{};              // id del segundo body

    float restingDistance{0.0f};  // distancia al reposo
    float elasticForce   {50000.0f}; // constante elástica
};

struct PhysicsEngine
{
    ph2dBodyId idCounter{0};

    std::unordered_map<ph2dBodyId, Body> bodies;
    std::unordered_map<ph2dBodyId, Constrain> constrains;

    std::vector<ManifoldIntersection> intersections;

    float setFixedTimeStamp{0.008f};
    float maxAccumulated    {0.32f};
    int   collisionChecksCount{8};

    float _fixedTimeAccumulated{0.0f};

    void runSimulation(float deltaTime);

    SimulationPhysicsSettings simulationPhysicsSettings;

    ph2dBodyId addBody(const Vec2& centerPos, const Collider& collider);
    ph2dBodyId addConstrain(const Constrain& c);
    ph2dBodyId addHalfSpaceStaticObject(const Vec2& position, const Vec2& normal);
};


inline ph2dBodyId PhysicsEngine::addBody(const Vec2& centerPos, const Collider& collider)
{
    Body body;
    body.motionState.setPos(centerPos);
    body.collider = collider;
    body.motionState.mass = collider.computeMass();
    body.motionState.momentOfInertia = collider.computeMomentOfInertia(body.motionState.mass);

    bodies.emplace(++idCounter, body);
    return idCounter;
}

inline ph2dBodyId PhysicsEngine::addConstrain(const Constrain& c)
{
    constrains.emplace(++idCounter, c);
    return idCounter;
}

inline ph2dBodyId PhysicsEngine::addHalfSpaceStaticObject(const Vec2& position, const Vec2& normal)
{
    Body body;
    body.motionState.setPos(position);
    body.collider.type = ColliderHalfSpace;
    body.motionState.mass = 0.0f;
    body.motionState.momentOfInertia = 0.0f;


    body.motionState.rotation = vectorToRotation(normal);

    bodies.emplace(++idCounter, body);
    return idCounter;
}


inline void applyDrag(MotionState &ms, float scale, const SimulationPhysicsSettings &s)
{
    Vec2 dragForce = ms.velocity;
    dragForce.x = -dragForce.x * std::abs(dragForce.x);
    dragForce.y = -dragForce.y * std::abs(dragForce.y);
    dragForce *= (scale * s.airDragCoefficient * 0.5f);

    float len = Math::length(dragForce);
    if (len > 0.0f) {
        if (len > s.maxAirDrag) {
            dragForce *= (s.maxAirDrag / len);
        }
        ms.acceleration += dragForce;
    }
}

inline void applyAngularDrag(MotionState &ms, float scale, const SimulationPhysicsSettings &s)
{
    float drag = scale * s.rotationalDragCoefficient * -ms.angularVelocity;
    drag = std::clamp(drag, -s.maxAngularDrag, s.maxAngularDrag);
    ms.angularVelocity += drag;
}

inline void integrateForces(
    MotionState &ms,
    float deltaTime,
    const SimulationPhysicsSettings &s,
    const BodyFlags &flags)
{
    bool frezeX = flags.isFreezeX();
    bool frezeY = flags.isFreezeY();

    if (ms.mass == 0.0f
        || ms.mass == std::numeric_limits<float>::infinity()
        || (frezeX && frezeY))
    {
        ms.acceleration = {0,0};
        ms.velocity     = {0,0};
    }
    else
    {
        ms.acceleration.x = std::clamp(ms.acceleration.x, -s.maxAcceleration, s.maxAcceleration);
        ms.acceleration.y = std::clamp(ms.acceleration.y, -s.maxAcceleration, s.maxAcceleration);

        ms.velocity += ms.acceleration * (deltaTime * 0.5f);
        ms.velocity.x = std::clamp(ms.velocity.x, -s.maxVelocity, s.maxVelocity);
        ms.velocity.y = std::clamp(ms.velocity.y, -s.maxVelocity, s.maxVelocity);

        Vec2 deltav = ms.velocity;
        if (frezeX) { deltav.x = 0.0f; ms.velocity.x = 0.0f; }
        if (frezeY) { deltav.y = 0.0f; ms.velocity.y = 0.0f; }

        if (Math::length(deltav) < s.restingVelocity) {
            deltav = {0,0};
            ms.velocity = {0,0};
        }

        ms.pos += deltav * deltaTime;

        ms.velocity += ms.acceleration * (deltaTime * 0.5f);
        ms.velocity.x = std::clamp(ms.velocity.x, -s.maxVelocity, s.maxVelocity);
        ms.velocity.y = std::clamp(ms.velocity.y, -s.maxVelocity, s.maxVelocity);

        if (std::fabs(ms.velocity.x) < 1e-5f) ms.velocity.x = 0.0f;
        if (std::fabs(ms.velocity.y) < 1e-5f) ms.velocity.y = 0.0f;

        ms.acceleration = {0,0};
    }

    if (ms.momentOfInertia == 0.0f
        || ms.momentOfInertia == std::numeric_limits<float>::infinity()
        || flags.isFreezeRotation())
    {
        ms.angularVelocity = 0.0f;
        ms.torque          = 0.0f;
    }
    else
    {
        ms.angularVelocity += (ms.torque / ms.momentOfInertia) * deltaTime;

        if (std::fabs(ms.angularVelocity) < s.restingAngularVelocity)
            ms.angularVelocity = 0.0f;

        ms.rotation += ms.angularVelocity * deltaTime;
        ms.torque    = 0.0f;
    }
}


inline void PhysicsEngine::runSimulation(float deltaTime)
{
    int counter = 0;
    if (setFixedTimeStamp <= 0.0f) {
        _fixedTimeAccumulated = 0.0f;
        counter = 1;
    } else {
        _fixedTimeAccumulated += deltaTime;
        _fixedTimeAccumulated = std::min(_fixedTimeAccumulated, maxAccumulated);
        while (_fixedTimeAccumulated > setFixedTimeStamp) {
            _fixedTimeAccumulated -= setFixedTimeStamp;
            ++counter;
        }
        deltaTime = setFixedTimeStamp;
    }

    for (int iter = 0; iter < counter; ++iter) {

        auto positionalCorrection = [&](Body &A, Body &B, Vec2 n, float pen) {
            bool freezeA = A.flags.isFreezeX() || A.flags.isFreezeY();
            bool freezeB = B.flags.isFreezeX() || B.flags.isFreezeY();
            float invMassA = (!freezeA && A.motionState.mass > 0.0f && A.motionState.mass < INFINITY)
                              ? 1.0f / A.motionState.mass : 0.0f;
            float invMassB = (!freezeB && B.motionState.mass > 0.0f && B.motionState.mass < INFINITY)
                              ? 1.0f / B.motionState.mass : 0.0f;
            if (invMassA + invMassB == 0.0f) return;

            const float percent = 0.20f;
            const float slop    = 0.01f;
            float k = std::max(pen - slop, 0.0f) / (invMassA + invMassB) * percent;
            Vec2 corr = n * k;
            A.motionState.pos -= corr * invMassA;
            B.motionState.pos += corr * invMassB;
        };

        auto rotationalCorrection = [&](Body &A, Body &B, Vec2 tA, Vec2 tB) {
            if ((tA.x==0 && tA.y==0) || (tB.x==0 && tB.y==0)) return;
            bool freezeA = A.flags.isFreezeRotation();
            bool freezeB = B.flags.isFreezeRotation();
            float invIA = (!freezeA && A.motionState.momentOfInertia>0
                           && A.motionState.momentOfInertia<INFINITY)
                           ? 1.0f/A.motionState.momentOfInertia : 0.0f;
            float invIB = (!freezeB && B.motionState.momentOfInertia>0
                           && B.motionState.momentOfInertia<INFINITY)
                           ? 1.0f/B.motionState.momentOfInertia : 0.0f;
            if (invIA+invIB==0.0f) return;

            const float PI        = 3.1415926f;
            const float percent   = 0.17f;
            const float slop      = 0.00f;
            const float threshold = Math::radians(5.0f);

            // alinear tangentes
            if (Math::dot(tA,tB) < 0.0f) tB = -tB;

            float angA = std::atan2(tA.y,tA.x);
            float angB = std::atan2(tB.y,tB.x);
            float dAng = angB - angA;
            if      (dAng >  PI) dAng -= 2*PI;
            else if (dAng < -PI) dAng += 2*PI;

            if (std::fabs(dAng) > slop && std::fabs(dAng) < threshold) {
                float corr = percent * dAng / (invIA + invIB);
                if (invIA>0) A.motionState.rotation -= corr * invIA;
                if (invIB>0) B.motionState.rotation += corr * invIB;
            }
        };

        auto applyFriction = [&](Body &A, Body &B, Vec2 tangent,
                                 Vec2 rv, float invMassA, float invMassB,
                                 float j, Vec2 rA, Vec2 rB, Vec2 contact) {
            float invIA = (A.motionState.momentOfInertia>0
                           && A.motionState.momentOfInertia<INFINITY)
                          ? 1.0f/A.motionState.momentOfInertia : 0.0f;
            float invIB = (B.motionState.momentOfInertia>0
                           && B.motionState.momentOfInertia<INFINITY)
                          ? 1.0f/B.motionState.momentOfInertia : 0.0f;

            float jt = -Math::dot(rv, tangent);
            float denom = invMassA + invMassB
                          + std::pow(Math::cross(rA, tangent), 2.0f) * invIA
                          + std::pow(Math::cross(rB, tangent), 2.0f) * invIB;
            if (denom == 0.0f) return;
            jt /= denom;

            // μ = sqrt(μ_sA² + μ_sB²)
            float mu_s = std::sqrt(A.staticFriction*A.staticFriction
                                 + B.staticFriction*B.staticFriction);

            Vec2 frictionImpulse;
            if (std::fabs(jt) < j * mu_s) {
                frictionImpulse = tangent * jt;
            } else {
                float mu_d = std::sqrt(A.dynamicFriction*A.dynamicFriction
                                     + B.dynamicFriction*B.dynamicFriction);
                frictionImpulse = tangent * (-j * mu_d);
            }

            A.motionState.applyImpulseWorldPosition(-frictionImpulse, contact, A.flags);
            B.motionState.applyImpulseWorldPosition( frictionImpulse, contact, B.flags);
        };

        auto impulseResolution = [&](Body &A, Body &B,
                                     Vec2 normal, float velAlongNormal,
                                     float pen, Vec2 contact) {
            float e = std::min(A.elasticity, B.elasticity);

            float invMassA = (A.motionState.mass>0
                              && A.motionState.mass<INFINITY)
                              ? 1.0f/A.motionState.mass : 0.0f;
            float invMassB = (B.motionState.mass>0
                              && B.motionState.mass<INFINITY)
                              ? 1.0f/B.motionState.mass : 0.0f;

            Vec2 rA = contact - A.motionState.pos;
            Vec2 rB = contact - B.motionState.pos;

            float invIA = (A.motionState.momentOfInertia>0
                           && A.motionState.momentOfInertia<INFINITY)
                          ? 1.0f/A.motionState.momentOfInertia : 0.0f;
            float invIB = (B.motionState.momentOfInertia>0
                           && B.motionState.momentOfInertia<INFINITY)
                          ? 1.0f/B.motionState.momentOfInertia : 0.0f;

            float denom = invMassA + invMassB
                        + std::pow(Math::cross(rA, normal),2.0f)*invIA
                        + std::pow(Math::cross(rB, normal),2.0f)*invIB;
            if (denom == 0.0f) return;

            float j = -(1.0f + e) * velAlongNormal;
            j /= denom;

            Vec2 impulse = normal * j;
            A.motionState.applyImpulseWorldPosition(-impulse, contact, A.flags);
            B.motionState.applyImpulseWorldPosition( impulse, contact, B.flags);

            Vec2 rv = (B.motionState.velocity
                      + Math::cross(-B.motionState.angularVelocity, rB))
                    - (A.motionState.velocity
                      + Math::cross(-A.motionState.angularVelocity, rA));

            Vec2 tangent = rv - normal * Math::dot(rv, normal);
            float tLen = Math::length(tangent);
            if (tLen > 1e-5f) {
                tangent = tangent / tLen;
                applyFriction(A, B, tangent, rv, invMassA, invMassB, j, rA, rB, contact);
            }
        };

        for (auto &entry : bodies) {
            Body &b = entry.second;
            if (b.motionState.mass>0
                && b.motionState.mass<INFINITY
                && !b.flags.isFreezePosition())
            {
                b.motionState.acceleration +=
                    simulationPhysicsSettings.gravity * b.gravityScale;
            }
            if (b.motionState.mass<=0) b.motionState.mass = INFINITY;
            if (b.motionState.momentOfInertia<=0)
                b.motionState.momentOfInertia = INFINITY;

            applyDrag(b.motionState, b.dragScale, simulationPhysicsSettings);
            applyAngularDrag(b.motionState, b.angularDragScale, simulationPhysicsSettings);
        }

        intersections.clear();
        for (auto &it1 : bodies) {
            for (auto &it2 : bodies) {
                if (it1.first >= it2.first) continue;
                Body &A = it1.second;
                Body &B = it2.second;

                Vec2 normal, contact;
                float pen = 0.0f;
                Vec2 tA, tB;
                bool rev = false;

                if (BodyvsBody(A,B,pen,normal,contact,tA,tB,&rev)) {
                    ManifoldIntersection m{};
                    if (rev) {
                        m.A = it2.first; m.B = it1.first;
                        m.normal   = -normal;
                        m.tangentA = tB;  m.tangentB = tA;
                        m.massA    = B.motionState.mass;
                        m.massB    = A.motionState.mass;
                    } else {
                        m.A = it1.first; m.B = it2.first;
                        m.normal   = normal;
                        m.tangentA = tA;       m.tangentB = tB;
                        m.massA    = A.motionState.mass;
                        m.massB    = B.motionState.mass;
                    }
                    m.contactPoint = contact;
                    m.penetration  = pen;
                    intersections.push_back(m);
                }
            }
        }

        std::sort(intersections.begin(), intersections.end(),
            [&](auto &a, auto &b){
                if (a.penetration != b.penetration)
                    return a.penetration > b.penetration;
                return a.massA < b.massA;
            });

        for (int ci=0; ci<collisionChecksCount; ++ci) {
            for (auto &m : intersections) {
                Body &A = bodies[m.A];
                Body &B = bodies[m.B];
                Vec2 rv = B.motionState.velocity - A.motionState.velocity;
                float velAlongNormal = Math::dot(rv, m.normal);
                if (velAlongNormal < 0.0f) {
                    impulseResolution(A,B,m.normal,velAlongNormal,
                                      m.penetration,m.contactPoint);
                }
                if (ci == collisionChecksCount-1 || velAlongNormal<0.0f) {
                    positionalCorrection(A,B,m.normal,m.penetration);
                    rotationalCorrection(A,B,m.tangentA,m.tangentB);
                }
            }
        }

        for (auto &cc : constrains) {
            auto ait = bodies.find(cc.second.A);
            auto bit = bodies.find(cc.second.B);
            if (ait==bodies.end() || bit==bodies.end()) continue;
            Vec2 pA = ait->second.motionState.pos;
            Vec2 pB = bit->second.motionState.pos;
            float dist = Math::distance(pA,pB);
            float deform = cc.second.restingDistance - dist;
            if (deform == 0.0f) continue;
            float f = deform * cc.second.elasticForce;
            Vec2 dir = pB - pA;
            dir = Math::normalize(dir) * f;
            ait->second.applyImpulseObjectPosition(-dir, {});
            bit->second.applyImpulseObjectPosition( dir, {});
        }

        for (auto &entry : bodies) {
            Body &b = entry.second;
            integrateForces(b.motionState, deltaTime,
                            simulationPhysicsSettings, b.flags);
            b.motionState.lastPos = b.motionState.pos;
        }
    }
}

