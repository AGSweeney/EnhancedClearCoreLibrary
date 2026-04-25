#pragma once

#include <QVector3D>
#include <QStringList>

// Parses loaded G-code for visualization only (not the controller's motor mapping).
// Works in "ideal" X/Y/Z: G20/G21, G90/G91, G0/G1 linear, G2/G3 arcs in XY (I J or R) with
// Z interpolated along the move. All positions are converted to the same DRO / GUI units as the app.
struct ProgramKinematicsResult {
    /// Polyline in GUI units (mm or in per main window setting), including start at the origin
    /// when the first move leaves (0,0,0) so the line strip is not empty.
    QVector<QVector3D> pathInGuiUnits;
};

// Returns false if nothing could be built (e.g. empty file); still may fill partial data.
bool BuildProgramKinematics(const QStringList &lines, bool guiInches, ProgramKinematicsResult *out);
