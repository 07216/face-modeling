#ifndef RECOGNITIONWIDGET_H
#define RECOGNITIONWIDGET_H
#include "TemplateWidget.h"
#include "FacePerformance/FacePerformance.h"
#include "Util/SettingDialog.h"

class RecognitionWidget : public TemplateWidget
{
    Q_OBJECT
public:
    RecognitionWidget(FacePerformance* face, QWidget *parent = 0);
    virtual void initLeft();
    virtual void initRight();
protected:
    void keyReleaseEvent(QKeyEvent* event);
private:
    FacePerformance * face;
signals:

public slots:
    void ShowColorImage(uchar* data, int cols, int rows);
    void ChangeUser();

};

#endif // RECOGNITIONWIDGET_H
