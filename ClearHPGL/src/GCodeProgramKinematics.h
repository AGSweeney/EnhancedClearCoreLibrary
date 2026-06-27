#pragma once

#include <QVector3D>
#include <QStringList>

// Parses loaded G-code for visualization only (not the controller's motor mapping).
// Works in "ideal" X/Y/Z: G20/G21, G90/G91, G0/G1 linear, G2/G3 arcs in XY (I J or R) with
// Z interpolated along the move. All positions are converted to the same DRO / GUI units as the app.
struct ProgramKinematicsResult {
    /// Cutting moves (G1/G2/G3) broken into separate polylines — one strip per consecutive run
    /// between rapids. Rendered yellow so the actual tool path is clearly visible.
    QVector<QVector<QVector3D>> cutStrips;

    /// Rapid moves (G0) as flat start→end pairs (index 0,1 = segment 0; 2,3 = segment 1 …).
    /// Rendered gray so the viewer can distinguish repositioning from cutting.
    QVector<QVector3D> rapidSegments;

    /// Legacy flat path (all moves, rapids included) — kept for callers that haven't migrated.
    QVector<QVector3D> pathInGuiUnits;
};

// Returns false if nothing could be built (e.g. empty file); still may fill partial data.
bool BuildProgramKinematics(const QStringList &lines, bool guiInches, ProgramKinematicsResult *out);
