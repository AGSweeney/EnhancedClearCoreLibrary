#pragma once

#include <QVector3D>
#include <QString>
#include <QVector>

// Minimal ASCII DXF import (LINE, LWPOLYLINE, POLYLINE/VERTEX, ARC, CIRCLE).
// LWPOLYLINE / POLYLINE vertex bulge (group 42) is tessellated to arcs; straight if bulge is 0.
// Binary DXF is not supported. Geometry is returned in millimeters (Z often 0 for 2D).
// $INSUNITS: 0 (unitless) and missing variable both use inches for raw coordinates; 4 = mm, 1 = inches, etc.
struct DxfImportOutput {
    QString errorMessage;
    /// Each polyline is one continuous pen-down path (mm).
    QVector<QVector<QVector3D>> polylinesMm;
    /// Pen-up moves as flat start/end pairs in mm (between polylines).
    QVector<QVector3D> rapidSegmentsMm;
    /// HP/GL-2 style commands (plotter units: 40 units per mm); suitable for preview / export.
    QString hpgl;
    /// Raw DXF $INSUNITS (same convention as import).
    int insunits = 0;
    /// Multiply drawing-unit coordinates by this to get millimeters (round-trip for export).
    double drawingToMmScale = 25.4;
};

bool ImportDxfFile(const QString &filePath, DxfImportOutput *out);

/// Regenerate HPGL text from tessellated polylines (mm); same rules as import output.
QString BuildHpglFromPolylinesMm(const QVector<QVector<QVector3D>> &polylinesMm);

/// Write minimal ASCII DXF (LWPOLYLINE per strip, bulge 0) in drawing units from mm geometry.
bool ExportAsciiDxfMinimal(const QString &filePath,
                           const QVector<QVector<QVector3D>> &polylinesMm,
                           int insunits,
                           double drawingToMmScale,
                           QString *errorMessage);
