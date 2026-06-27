#include "PathGapBridge.h"

#include <cmath>

namespace {

QVector3D normalizedOrX(const QVector3D &v) {
    const float L = v.length();
    if (L < 1.0e-8f) {
        return QVector3D(1.f, 0.f, 0.f);
    }
    return v / L;
}

QVector3D bezierPt(const QVector3D &p0, const QVector3D &p1, const QVector3D &p2, const QVector3D &p3, double t) {
    const double u = 1.0 - t;
    const double uu = u * u;
    const double tt = t * t;
    const double b0 = uu * u;
    const double b1 = 3.0 * uu * t;
    const double b2 = 3.0 * u * tt;
    const double b3 = tt * t;
    return p0 * float(b0) + p1 * float(b1) + p2 * float(b2) + p3 * float(b3);
}

} // namespace

QVector<QVector3D> gapBezierBridgeInterior(const QVector3D &endPrev,
                                         const QVector3D &endP,
                                         const QVector3D &startP,
                                         const QVector3D &startNext,
                                         int segments) {
    QVector<QVector3D> out;
    const QVector3D td = endP - endPrev;
    const QVector3D ta = startNext - startP;
    if (td.length() < 1.0e-6f || ta.length() < 1.0e-6f) {
        return out;
    }
    const QVector3D Td = normalizedOrX(td);
    const QVector3D Ta = normalizedOrX(ta);
    const double d = double((startP - endP).length());
    if (d < 1.0e-6) {
        return out;
    }
    // Control arm lengths: scale with chord so corners stay round instead of a single long chord.
    const float arm = float(qBound(0.08 * d, 0.35 * d, d * 1.2));
    const QVector3D P0 = endP;
    const QVector3D P3 = startP;
    const QVector3D P1 = P0 + Td * (arm * (2.f / 3.f)); // B'(0)=3(P1-P0) ∥ Td
    const QVector3D P2 = P3 - Ta * (arm * (2.f / 3.f));  // B'(1)=3(P3-P2) ∥ Ta

    const int n = qBound(4, segments, 96);
    const float zf = 0.5f * (endP.z() + startP.z());
    for (int k = 1; k < n; ++k) {
        const double t = double(k) / double(n);
        QVector3D p = bezierPt(P0, P1, P2, P3, t);
        p.setZ(zf);
        out.append(p);
    }
    return out;
}
