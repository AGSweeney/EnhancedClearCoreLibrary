#include "GrblSnapshotStore.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

bool GrblSnapshotStore::SaveToFile(const QString &path, const GrblSnapshot &snapshot, QString &error) {
    QJsonObject root;
    root.insert("variant", snapshot.variant);
    root.insert("parserState", snapshot.parserState);
    root.insert("buildInfo", snapshot.buildInfo);

    QJsonObject settingsObj;
    for (auto it = snapshot.settings.cbegin(); it != snapshot.settings.cend(); ++it) {
        settingsObj.insert(QString::number(it.key()), it.value());
    }
    root.insert("settings", settingsObj);

    QJsonArray params;
    for (const QString &line : snapshot.parameters) {
        params.append(line);
    }
    root.insert("parameters", params);

    QJsonArray startup;
    for (const QString &line : snapshot.startupBlocks) {
        startup.append(line);
    }
    root.insert("startupBlocks", startup);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        error = file.errorString();
        return false;
    }
    const QJsonDocument doc(root);
    file.write(doc.toJson(QJsonDocument::Indented));
    return true;
}

bool GrblSnapshotStore::LoadFromFile(const QString &path, GrblSnapshot &snapshot, QString &error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        error = file.errorString();
        return false;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) {
        error = "Snapshot file is not a valid JSON object.";
        return false;
    }

    const QJsonObject root = doc.object();
    snapshot = {};
    snapshot.variant = root.value("variant").toString();
    snapshot.parserState = root.value("parserState").toString();
    snapshot.buildInfo = root.value("buildInfo").toString();

    const QJsonObject settingsObj = root.value("settings").toObject();
    for (auto it = settingsObj.begin(); it != settingsObj.end(); ++it) {
        bool ok = false;
        const int key = it.key().toInt(&ok);
        if (!ok) {
            continue;
        }
        snapshot.settings.insert(key, it.value().toDouble());
    }

    const QJsonArray params = root.value("parameters").toArray();
    for (const QJsonValue &value : params) {
        if (value.isString()) {
            snapshot.parameters.append(value.toString());
        }
    }

    const QJsonArray startup = root.value("startupBlocks").toArray();
    for (const QJsonValue &value : startup) {
        if (value.isString()) {
            snapshot.startupBlocks.append(value.toString());
        }
    }
    return true;
}
