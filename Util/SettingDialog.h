#ifndef SETTINGDIALOG_H
#define SETTINGDIALOG_H
#include "QtGui"
#include <iostream>
#include "ShowParameter.h"
#include "ConfigParameter.h"

typedef struct
{
    int maxInIters ;
    int icpIters ;
    double markerWeight_RT ;
    double markerWeight_yz0;
    double registerRate ;
    double beta1 ;
    double beta2 ;
    double beta3;
    int num_P;
    int num_E;
    int radius_bilateral;
    double max_facerecognition_error;
}initConfig;

typedef struct
{
    int maxInIters ;
    int icpIters ;
    double markerWeight ;
    double registerRate ;
    double max_nose_error;
}rigidConfig;

typedef struct
{
    int maxInIters ;
    int icpIters ;
    double colorWeight ;
    double markerWeight ;
    double registerRate ;
    double alpha ;
    double beta ;
}nonrigidConfig;

typedef struct
{
    int maxInIters;
    double gamma;
    double beta1 ;
    double beta2 ;
    double beta3;
    double sigma;
}refineConfig;



//SETTINGDIALOGSHARED_EXPORT
class  SettingDialog:public QDialog
{
    Q_OBJECT
signals:
    void update();

public slots:
    void writeConfig();
    void updateConfig();

public:
    SettingDialog();
    ~SettingDialog();

private:
    void readConfig();
    void initInitGroup();
    void initRigidGroup();
    void initNonrigidGroup();
    void initRefineGroup();
    void initShowGroup();

public:

private:
    QGridLayout *main_layout;
    QSpinBox *initIterSpin ;
    QSpinBox *initOutIterSpin ;
    QDoubleSpinBox *initRegRateSpin ;
    QDoubleSpinBox *initMarkerWeightSpin_RT ;
    QDoubleSpinBox *initMarkerWeightSpin_yz0 ;
    QDoubleSpinBox *initBetaOneSpin ;
    QDoubleSpinBox *initBeta2Spin ;
    QDoubleSpinBox *initBeta3Spin ;
    QSpinBox *initNumPSpin;
    QSpinBox *initNumESpin;
    QSpinBox *radiusBilateralSpin;
    QDoubleSpinBox *maxFacerecognitionErrorSpin;

    QSpinBox *rigidIterSpin ;
    QSpinBox *rigidICPIterSpin ;
    QDoubleSpinBox *rigidRegRateSpin ;
    QDoubleSpinBox *rigidMarkerWeightSpin ;
    QDoubleSpinBox *rigidMaxNoseErrorSpin;

    QSpinBox *nonrigidIterSpin ;
    QSpinBox *nonrigidOutIterSpin ;
    QDoubleSpinBox *nonrigidRegRateSpin ;
    QDoubleSpinBox *nonrigidColorWeightSpin;
    QDoubleSpinBox *nonrigidMarkerWeightSpin ;
    QDoubleSpinBox *nonrigidAlphaSpin ;
    QDoubleSpinBox *nonrigidBetaSpin ;

    QSpinBox *refineIterSpin ;
    QDoubleSpinBox *refineGammaSpin ;
    QDoubleSpinBox *refineBetaOneSpin ;
    QDoubleSpinBox *refineBeta2Spin ;
    QDoubleSpinBox *refineBeta3Spin ;
    QDoubleSpinBox *refineSigmaSpin ;

    //show
    QComboBox *pointCloud_ComboBox;
    QComboBox *faceMesh_ComboBox;

    initConfig initParas;
    rigidConfig rigidParas;
    nonrigidConfig nonrigidParas;
    refineConfig refineParas;
    ShowConfig showParas;

};

#endif // SETTINGDIALOG_H
