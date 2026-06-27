#pragma once

#include <QVector>
#include <QVector3D>

/// Interior samples of a C¹ cubic Bézier from endP to startP, tangent to (endP−endPrev) at endP
/// and to (startNext−startP) at startP. Skips endpoints (caller already has endP; append then add samples then startP).
/// Returns empty if tangents are degenerate — caller should use a single straight segment.
QVector<QVector3D> gapBezierBridgeInterior(const QVector3D &endPrev,
                                         const QVector3D &endP,
                                         const QVector3D &startP,
                                         const QVector3D &startNext,
                                         int segments = 22);
