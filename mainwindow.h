#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QMainWindow>
#include "FacePerformance/FacePerformance.h"
#include "UI/animationwidget.h"
#include "UI/recognitionwidget.h"
#include "Util/ConfigParameter.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
private:
    void init();
    FacePerformance* face;
    QTabWidget *tabWidget;
    AnimationWidget * tabAnimation;
    RecognitionWidget * tabRecognition;
    int animationIndex;
    int recognitionIndex;

};

#endif // MAINWINDOW_H
