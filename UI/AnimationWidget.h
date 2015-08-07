#ifndef ANIMATIONWIDGET_H
#define ANIMATIONWIDGET_H
#include "TemplateWidget.h"
#include "UserviewMesh.h"
#include "AvatarviewMesh.h"
#include "FacePerformance/FacePerformance.h"
#include "Util/ConfigParameter.h"
#include "idwidget.h"
#include <QTimer>
#include <QTime>
#include <vector>

using namespace std;

class AnimationWidget : public TemplateWidget
{
    Q_OBJECT
public:
    AnimationWidget(FacePerformance* face, QWidget *parent = 0);

    virtual void initLeft();
    virtual void initRight();
    virtual void initBottom();
    int recognitionSimilaryId(double& distance);
    void showRecognizedImage(int id, double distance);
    void showImagedis(int id, double distance,int index);
    Eigen::VectorXi ids;
    Eigen::VectorXd distances;
    void changeimage(int id, int index);
protected:
    void keyReleaseEvent(QKeyEvent* event);
private:
    FacePerformance * face;
    QPushButton * preButton;
    QPushButton * nextButton;
    QPushButton * startButton;
    QPushButton * configButton;
    QPushButton * changeButton;
    QPushButton * recognitionButton;
    QPushButton * saveButton;
    QLineEdit* userIdText;
    QLineEdit* userNameText;
    std::vector<int> recognitionIndex;
    idwidget * idget;
    idwidget * idget0;
    idwidget * idget1;
    idwidget * idget2;
    idwidget * idget3;
    idwidget * idget4;
    idwidget * idget5;
    idwidget * idget6;
    idwidget * idget7;
    QTimer   * timer;
    QSplitter * spl;
    QHBoxLayout * idsp;
    int itime;
    vector<idwidget*> idwidgets;
    vector<idwidget*> meshwidgets;
    QLabel  * image0;
    idwidget * meshwidget0;
    idwidget * meshwidget1;
    idwidget * meshwidget2;
    idwidget * meshwidget3;
    idwidget * meshwidget4;
    QVBoxLayout * vlayout;
    vector<QLabel*> images;
    int number_face;
    int ktimer;
    int old1;
    int old2;
    int old3;
    int old4;
    void showRecognizedImagescale(int id, double distance,double scale);

signals:
    
public slots:
    void ShowColorImage(uchar* data, int cols, int rows);
    void startSlot();
    void changeUserSlot();
    void recognitionSlot();
    void saveSlot();
    void ktimerreduce();
    void ktimeradd();
    //void onTimerOut();
};

#endif // ANIMATIONWIDGET_H
