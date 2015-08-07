#ifndef IDWIDGET_H
#define IDWIDGET_H

#include "TemplateWidget.h"
#include "AvatarviewMesh.h"
#include "FacePerformance/FacePerformance.h"
#include "Util/ConfigParameter.h"
#include "idmesh.h"
class idwidget : public QWidget
{
    Q_OBJECT
public:
    idwidget(QWidget *parent = 0);

signals:

public slots:

public:
    QLabel *id_image;
    idmesh *id_mesh;
    void changeid(int id);
    void changeidd(int id,double distance);

};

#endif // IDWIDGET_H
