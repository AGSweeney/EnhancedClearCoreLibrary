#pragma once

#include <QMap>
#include <QString>
#include <QStringList>

struct GrblSnapshot {
    QString variant;
    QMap<int, double> settings;
    QString parserState;
    QStringList parameters;
    QStringList startupBlocks;
    QString buildInfo;
};

class GrblSnapshotStore {
public:
    static bool SaveToFile(const QString &path, const GrblSnapshot &snapshot, QString &error);
    static bool LoadFromFile(const QString &path, GrblSnapshot &snapshot, QString &error);
};
