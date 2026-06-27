#pragma once

#include <QString>
#include <QVector>
#include <QVector3D>

/// Issue category for tooltips and optional DXF quick-fix actions (right-click in 3D view).
enum class PathIssueKind : int {
    None = 0,
    OpenProfileEnds,
    OpenInteriorShortEdge,
    OpenAcrossPenUp,
    InteriorJump,
    OverlapCollinear,
    OverlapCrossing,
    JunctionOverrun,
};

/// One highlighted issue in the 3D path preview (position + hover tooltip + fix metadata).
struct PathIssueMarker {
    QVector3D positionGui{};
    QString toolTip;
    PathIssueKind kind = PathIssueKind::None;
    /// Primary polyline index in imported cut strips (DXF mm strips order).
    int stripIndex = -1;
    /// For edge-based issues: start vertex index of edge (edge is vertex → vertex+1).
    int edgeStartVertex = -1;
    /// Pen-up pair index `p`: rapid segment is flat[2p]→flat[2p+1] between strip `p` and `p+1`.
    int rapidPairIndex = -1;
    int secondaryStripIndex = -1;
};

inline bool operator==(const PathIssueMarker &a, const PathIssueMarker &b) {
    return a.kind == b.kind && a.stripIndex == b.stripIndex && a.edgeStartVertex == b.edgeStartVertex
        && a.rapidPairIndex == b.rapidPairIndex && a.secondaryStripIndex == b.secondaryStripIndex
        && a.toolTip == b.toolTip && (a.positionGui - b.positionGui).lengthSquared() < 1.0e-12f;
}

inline bool operator!=(const PathIssueMarker &a, const PathIssueMarker &b) {
    return !(a == b);
}

// Detect DXF / program path issues for 3D overlay (laser-oriented copy in tooltips).
// Geometry is analyzed in millimeters (XY); positions match incoming strip coordinates (GUI units).
// rapidSegmentsGui: optional pen-up segments as flat start/end pairs (same units as strips). When
// present (e.g. DXF import), short rapids between consecutive cut strips are flagged as cross-gap opens.
void BuildPathIssueMarkers(const QVector<QVector<QVector3D>> &cutStripsGui,
                           bool dxfSource,
                           bool guiInches,
                           double junctionDVmaxStepsPerSec,
                           double spmmX,
                           double spmmY,
                           double feedMmPerMin,
                           const QVector<QVector3D> *rapidSegmentsGui,
                           QVector<PathIssueMarker> *outMarkers);
