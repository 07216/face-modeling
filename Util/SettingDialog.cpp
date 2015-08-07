#include "SettingDialog.h"
#include "fstream"
#include <QSettings>
#include<qdebug.h>

SettingDialog::SettingDialog()
{
    readConfig();
    setWindowTitle(tr("Face Performance Config"));
    setWindowFlags( Qt::Tool);

    main_layout = new QGridLayout();

    initInitGroup();
    initRigidGroup();
    initNonrigidGroup();
    initRefineGroup();
    initShowGroup();

    QPushButton *update = new QPushButton(tr("Update"));
    QPushButton *save = new QPushButton(tr("Save"));
    QPushButton *cancel = new QPushButton(tr("Cancel"));

    connect(update, SIGNAL(clicked()), this, SLOT(updateConfig()));
    connect(save,SIGNAL(clicked()), this, SLOT(writeConfig()));
    connect(cancel,SIGNAL(clicked()), this, SLOT(close()));

    main_layout->addWidget(update,3,1);
    main_layout->addWidget(save,3,2);
    main_layout->addWidget(cancel,3,3);
    setLayout(main_layout);
}

SettingDialog::~SettingDialog()
{
}

void SettingDialog::readConfig()
{
    QString config_filename(ConfigParameter::config_filename.c_str());
    QSettings config(config_filename,QSettings::IniFormat);//"Config.ini"
    initParas.maxInIters = config.value("init/maxInIters").toInt();
    initParas.icpIters = config.value("init/outIters").toInt();
    initParas.registerRate = config.value("init/registerRate").toDouble();
    initParas.markerWeight_RT = config.value("init/markerWeight_RT").toDouble();
    initParas.markerWeight_yz0 = config.value("init/markerWeight_yz0").toDouble();
    initParas.beta1 = config.value("init/beta1").toDouble();
    initParas.beta2 = config.value("init/beta2").toDouble();
    initParas.beta3 = config.value("init/beta3").toDouble();
    initParas.num_P = config.value("init/numP").toInt();
    initParas.num_E = config.value("init/numE").toInt();
    initParas.radius_bilateral = config.value("init/radiusBilateral").toInt();
    initParas.max_facerecognition_error = config.value("init/maxFacerecognitionError").toDouble();

    rigidParas.maxInIters = config.value("rigid/maxInIters").toInt();
    rigidParas.icpIters = config.value("rigid/outIters").toInt();
    rigidParas.registerRate = config.value("rigid/registerRate").toDouble();
    rigidParas.markerWeight = config.value("rigid/markerWeight").toDouble();
    rigidParas.max_nose_error = config.value("rigid/max_nose_error").toDouble();

    nonrigidParas.icpIters = config.value("nonrigid/outIters").toInt();
    nonrigidParas.registerRate = config.value("nonrigid/registerRate").toDouble();
    nonrigidParas.colorWeight = config.value("nonrigid/colorWeight").toDouble();
    nonrigidParas.markerWeight = config.value("nonrigid/markerWeight").toDouble();
    nonrigidParas.alpha = config.value("nonrigid/alpha").toDouble();
    nonrigidParas.beta = config.value("nonrigid/beta").toDouble();

    refineParas.maxInIters = config.value("refine/maxInIters").toInt();
    refineParas.gamma = config.value("refine/gamma").toDouble();
    refineParas.beta1 = config.value("refine/beta1").toDouble();
    refineParas.beta2 = config.value("refine/beta2").toDouble();
    refineParas.beta3 = config.value("refine/beta3").toDouble();
    refineParas.sigma = config.value("refine/sigma").toDouble();

    showParas.pointCloudFormat = (enum ShowPointCloudFormat)config.value("show/pointCloudFormat").toInt();
    showParas.faceMeshFormat = (enum ShowFaceMeshFormat)config.value("show/faceMeshFormat").toInt();
}

void SettingDialog::writeConfig()
{
    QString config_filename(ConfigParameter::config_filename.c_str());
    QSettings config(config_filename,QSettings::IniFormat);//"Config.ini"
    config.setValue("init/maxInIters",initIterSpin->value());
    config.setValue("init/outIters", initOutIterSpin->value());
    config.setValue("init/registerRate", initRegRateSpin->value());
    config.setValue("init/markerWeight_RT",initMarkerWeightSpin_RT->value());
    config.setValue("init/markerWeight_yz0", initMarkerWeightSpin_yz0->value());
    config.setValue("init/beta1",initBetaOneSpin->value());
    config.setValue("init/beta2",initBeta2Spin->value());
    config.setValue("init/beta3",initBeta3Spin->value());
    config.setValue("init/numP", initNumPSpin->value());
    config.setValue("init/numE", initNumESpin->value());
    config.setValue("init/radiusBilateral", radiusBilateralSpin->value());
    config.setValue("init/maxFacerecognitionError", maxFacerecognitionErrorSpin->value());

    config.setValue("rigid/maxInIters",rigidIterSpin->value());
    config.setValue("rigid/outIters", rigidICPIterSpin->value());
    config.setValue("rigid/registerRate", rigidRegRateSpin->value());
    config.setValue("rigid/markerWeight",rigidMarkerWeightSpin->value());
    config.setValue("rigid/max_nose_error",rigidMaxNoseErrorSpin->value());

    config.setValue("nonrigid/outIters", nonrigidOutIterSpin->value());
    config.setValue("nonrigid/registerRate", nonrigidRegRateSpin->value());
    config.setValue("nonrigid/colorWeight", nonrigidColorWeightSpin->value());
    config.setValue("nonrigid/markerWeight", nonrigidMarkerWeightSpin->value());
    config.setValue("nonrigid/alpha", nonrigidAlphaSpin->value());
    config.setValue("nonrigid/beta", nonrigidBetaSpin->value());

    config.setValue("refine/maxInIters",refineIterSpin->value());
    config.setValue("refine/gamma",refineGammaSpin->value());
    config.setValue("refine/beta1",refineBetaOneSpin->value());
    config.setValue("refine/beta2",refineBeta2Spin->value());
    config.setValue("refine/beta3",refineBeta3Spin->value());
    config.setValue("refine/sigma", refineSigmaSpin->value());

    config.setValue("show/pointCloudFormat", pointCloud_ComboBox->currentIndex());
    config.setValue("show/faceMeshFormat", faceMesh_ComboBox->currentIndex());
}

void SettingDialog::initInitGroup()
{
    double max_value = 1000000;
    double max_precision = 4;
    QGroupBox *initGroup = new QGroupBox();
    initGroup->setTitle(tr("Initialization"));

    QGridLayout *groupLayout = new QGridLayout;

    QLabel *maxIterLabel = new QLabel;
    maxIterLabel->setText(tr("ICP Iter"));
    initIterSpin = new QSpinBox;
    initIterSpin->setValue(initParas.maxInIters);
    groupLayout->addWidget(maxIterLabel,0,0);
    groupLayout->addWidget(initIterSpin,0,1);

    QLabel *outIterLabel = new QLabel;
    outIterLabel->setText(tr("Nonrigid Iter"));
    initOutIterSpin = new QSpinBox;
    initOutIterSpin->setValue(initParas.icpIters);
    groupLayout->addWidget(outIterLabel,1,0);
    groupLayout->addWidget(initOutIterSpin,1,1);

    QLabel *regRateLabel = new QLabel;
    regRateLabel->setText(tr("Outlier Threshold"));
    initRegRateSpin = new QDoubleSpinBox;
    initRegRateSpin->setValue(initParas.registerRate);
    groupLayout->addWidget(regRateLabel,2,0);
    groupLayout->addWidget(initRegRateSpin,2,1);

    QLabel *markerWeightLabel_RT = new QLabel;
    markerWeightLabel_RT->setText(tr("Marker weight RT"));
    initMarkerWeightSpin_RT = new QDoubleSpinBox;
    initMarkerWeightSpin_RT->setMaximum(max_value);
    initMarkerWeightSpin_RT->setDecimals(max_precision);
    initMarkerWeightSpin_RT->setValue(initParas.markerWeight_RT);
    groupLayout->addWidget(markerWeightLabel_RT,3,0);
    groupLayout->addWidget(initMarkerWeightSpin_RT,3,1);

    QLabel *markerWeightLabel_yz0 = new QLabel;
    markerWeightLabel_yz0->setText(tr("Marker weight yz0"));
    initMarkerWeightSpin_yz0 = new QDoubleSpinBox;
    initMarkerWeightSpin_yz0->setMaximum(max_value);
    initMarkerWeightSpin_yz0->setDecimals(max_precision);
    initMarkerWeightSpin_yz0->setValue(initParas.markerWeight_yz0);
    groupLayout->addWidget(markerWeightLabel_yz0,4,0);
    groupLayout->addWidget(initMarkerWeightSpin_yz0,4,1);

    QLabel *betaOneLabel = new QLabel;
    betaOneLabel->setText(tr("Beta 1"));
    initBetaOneSpin = new QDoubleSpinBox;
    initBetaOneSpin->setMaximum(max_value);
    initBetaOneSpin->setDecimals(max_precision);
    initBetaOneSpin->setValue(initParas.beta1);
    groupLayout->addWidget(betaOneLabel,5,0);
    groupLayout->addWidget(initBetaOneSpin,5,1);

    QLabel *beta2Label = new QLabel;
    beta2Label->setText(tr("Beta 2"));
    initBeta2Spin = new QDoubleSpinBox;
    initBeta2Spin->setMaximum(max_value);
    initBeta2Spin->setDecimals(max_precision);
    initBeta2Spin->setValue(initParas.beta2);
    groupLayout->addWidget(beta2Label,6,0);
    groupLayout->addWidget(initBeta2Spin,6,1);

    QLabel *beta3Label = new QLabel;
    beta3Label->setText(tr("Beta 3"));
    initBeta3Spin = new QDoubleSpinBox;
    initBeta3Spin->setMaximum(max_value);
    initBeta3Spin->setDecimals(max_precision);
    initBeta3Spin->setValue(initParas.beta3);
    groupLayout->addWidget(beta3Label,7,0);
    groupLayout->addWidget(initBeta3Spin,7,1);

    QLabel *numPLabel = new QLabel;
    numPLabel->setText(tr("P num"));
    initNumPSpin = new QSpinBox;
    //initNumPSpin->setMaxinum(50);
    initNumPSpin->setValue(initParas.num_P);
    groupLayout->addWidget(numPLabel,8,0);
    groupLayout->addWidget(initNumPSpin,8,1);

    QLabel *numELabel = new QLabel;
    numELabel->setText(tr("E num"));
    initNumESpin = new QSpinBox;
    initNumESpin->setMaximum(1000);
    initNumESpin->setValue(initParas.num_E);
    groupLayout->addWidget(numELabel,9,0);
    groupLayout->addWidget(initNumESpin,9,1);

    QLabel *radiusBilateralLabel = new QLabel;
    radiusBilateralLabel->setText(tr("Bilateral Radius"));
    radiusBilateralSpin = new QSpinBox;
    radiusBilateralSpin->setValue(initParas.radius_bilateral);
    groupLayout->addWidget(radiusBilateralLabel, 10, 0);
    groupLayout->addWidget(radiusBilateralSpin, 10, 1);

    QLabel *maxFacerecognitionErrorLabel = new QLabel;
    maxFacerecognitionErrorLabel->setText(tr("Max Facerecognition Error"));
    maxFacerecognitionErrorSpin = new QDoubleSpinBox;
    maxFacerecognitionErrorSpin->setMaximum(max_value);
    maxFacerecognitionErrorSpin->setDecimals(max_precision);
    maxFacerecognitionErrorSpin->setValue(initParas.max_facerecognition_error);
    groupLayout->addWidget(maxFacerecognitionErrorLabel, 11, 0);
    groupLayout->addWidget(maxFacerecognitionErrorSpin, 11, 1);

    initGroup->setLayout(groupLayout);
    main_layout->addWidget(initGroup,0,0,1,2);
}

void SettingDialog::initRigidGroup()
{
    QGroupBox *rigidGroup = new QGroupBox;
    rigidGroup->setTitle(tr("Rigid Registration"));

    QGridLayout *groupLayout = new QGridLayout;

    QLabel *maxIterLabel = new QLabel;
    maxIterLabel->setText(tr("Tracking Iter"));
    rigidIterSpin = new QSpinBox;
    rigidIterSpin->setValue(rigidParas.maxInIters);
    groupLayout->addWidget(maxIterLabel,0,0);
    groupLayout->addWidget(rigidIterSpin,0,1);

    QLabel *outIterLabel = new QLabel;
    outIterLabel->setText(tr("ICP Iter"));
    rigidICPIterSpin = new QSpinBox;
    rigidICPIterSpin->setValue(rigidParas.icpIters);
    groupLayout->addWidget(outIterLabel,1,0);
    groupLayout->addWidget(rigidICPIterSpin,1,1);

    QLabel *regRateLabel = new QLabel;
    regRateLabel->setText(tr("Outlier Threshold"));
    rigidRegRateSpin = new QDoubleSpinBox;
    rigidRegRateSpin->setValue(rigidParas.registerRate);
    groupLayout->addWidget(regRateLabel,2,0);
    groupLayout->addWidget(rigidRegRateSpin,2,1);

    QLabel *markerWeightLabel = new QLabel;
    markerWeightLabel->setText(tr("Marker weight"));
    rigidMarkerWeightSpin = new QDoubleSpinBox;
    rigidMarkerWeightSpin->setValue(rigidParas.markerWeight);
    groupLayout->addWidget(markerWeightLabel,3,0);
    groupLayout->addWidget(rigidMarkerWeightSpin,3,1);

    QLabel *maxNoseErrorLabel = new QLabel;
    maxNoseErrorLabel->setText(tr("max_nose_error"));
    rigidMaxNoseErrorSpin = new QDoubleSpinBox;
    rigidMaxNoseErrorSpin->setMaximum(1000);
    rigidMaxNoseErrorSpin->setDecimals(2);
    rigidMaxNoseErrorSpin->setValue(rigidParas.max_nose_error);
    groupLayout->addWidget(maxNoseErrorLabel, 4, 0);
    groupLayout->addWidget(rigidMaxNoseErrorSpin, 4, 1);

    rigidGroup->setLayout(groupLayout);
    main_layout->addWidget(rigidGroup,0,2,1,2);
}

void SettingDialog::initNonrigidGroup()
{
    int max_value = 100000;
    int max_precision = 4;
    QGroupBox *nonrigidGroup = new QGroupBox;
    nonrigidGroup->setTitle(tr("Nonrigid Registration"));

    QGridLayout *groupLayout = new QGridLayout;

    QLabel *outIterLabel = new QLabel;
    outIterLabel->setText(tr("Nonrigid Iter"));
    nonrigidOutIterSpin = new QSpinBox;
    nonrigidOutIterSpin->setValue(nonrigidParas.icpIters);
    groupLayout->addWidget(outIterLabel,1,0);
    groupLayout->addWidget(nonrigidOutIterSpin,1,1);

    QLabel *regRateLabel = new QLabel;
    regRateLabel->setText(tr("Outlier Threshold"));
    nonrigidRegRateSpin = new QDoubleSpinBox;
    nonrigidRegRateSpin->setMaximum(max_value);
    nonrigidRegRateSpin->setDecimals(max_precision);
    nonrigidRegRateSpin->setValue(nonrigidParas.registerRate);
    groupLayout->addWidget(regRateLabel,2,0);
    groupLayout->addWidget(nonrigidRegRateSpin,2,1);

    QLabel *colorWeightLabel = new QLabel;
    colorWeightLabel->setText(tr("Color weight"));
    nonrigidColorWeightSpin = new QDoubleSpinBox;
    nonrigidColorWeightSpin->setMaximum(max_value);
    nonrigidColorWeightSpin->setDecimals(max_precision);
    nonrigidColorWeightSpin->setValue(nonrigidParas.colorWeight);
    groupLayout->addWidget(colorWeightLabel,3,0);
    groupLayout->addWidget(nonrigidColorWeightSpin,3,1);

    QLabel *markerWeightLabel = new QLabel;
    markerWeightLabel->setText(tr("Marker weight"));
    nonrigidMarkerWeightSpin = new QDoubleSpinBox;
    nonrigidMarkerWeightSpin->setMaximum(max_value);
    nonrigidMarkerWeightSpin->setDecimals(max_precision);
    nonrigidMarkerWeightSpin->setValue(nonrigidParas.markerWeight);
    groupLayout->addWidget(markerWeightLabel,4,0);
    groupLayout->addWidget(nonrigidMarkerWeightSpin,4,1);

    QLabel *alphaLabel = new QLabel;
    alphaLabel->setText(tr("Smooth"));
    nonrigidAlphaSpin = new QDoubleSpinBox;
    nonrigidAlphaSpin->setMaximum(max_value);
    nonrigidAlphaSpin->setDecimals(max_precision);
    nonrigidAlphaSpin->setValue(nonrigidParas.alpha);
    groupLayout->addWidget(alphaLabel,5,0);
    groupLayout->addWidget(nonrigidAlphaSpin,5,1);

    QLabel *betaLabel = new QLabel;
    betaLabel->setText(tr("Sparse"));
    nonrigidBetaSpin = new QDoubleSpinBox;
    nonrigidBetaSpin->setMaximum(max_value);
    nonrigidBetaSpin->setDecimals(max_precision);
    nonrigidBetaSpin->setValue(nonrigidParas.beta);
    groupLayout->addWidget(betaLabel,6,0);
    groupLayout->addWidget(nonrigidBetaSpin,6,1);

    nonrigidGroup->setLayout(groupLayout);
    main_layout->addWidget(nonrigidGroup,1,0,1,2);
}

void SettingDialog::initRefineGroup()
{
    int max_value = 100000;
    int max_precision = 4;
    QGroupBox *refineGroup = new QGroupBox;
    refineGroup->setTitle(tr("Refinement"));

    QGridLayout *groupLayout = new QGridLayout;

    QLabel *maxIterLabel = new QLabel;
    maxIterLabel->setText(tr("Gauss-Seidel Iter"));
    refineIterSpin = new QSpinBox;
    refineIterSpin->setValue(refineParas.maxInIters);
    refineIterSpin->setMaximum(10000);
    groupLayout->addWidget(maxIterLabel,0,0);
    groupLayout->addWidget(refineIterSpin,0,1);

    QLabel *gammaLabel = new QLabel;
    gammaLabel->setText(tr("Gamma"));
    refineGammaSpin = new QDoubleSpinBox;
    refineGammaSpin->setValue(refineParas.gamma);
    groupLayout->addWidget(gammaLabel,1,0);
    groupLayout->addWidget(refineGammaSpin,1,1);

    QLabel *betaOneLabel = new QLabel;
    betaOneLabel->setText(tr("Beta 1"));
    refineBetaOneSpin = new QDoubleSpinBox;
    refineBetaOneSpin->setMaximum(max_value);
    refineBetaOneSpin->setDecimals(max_precision);
    refineBetaOneSpin->setValue(refineParas.beta1);
    groupLayout->addWidget(betaOneLabel,2,0);
    groupLayout->addWidget(refineBetaOneSpin,2,1);

    QLabel *beta2Label = new QLabel;
    beta2Label->setText(tr("Beta 2"));
    refineBeta2Spin = new QDoubleSpinBox;
    refineBeta2Spin->setMaximum(max_value);
    refineBeta2Spin->setDecimals(max_precision);
    refineBeta2Spin->setValue(refineParas.beta2);
    groupLayout->addWidget(beta2Label,3,0);
    groupLayout->addWidget(refineBeta2Spin,3,1);

    QLabel *beta3Label = new QLabel;
    beta3Label->setText(tr("Beta 3"));
    refineBeta3Spin = new QDoubleSpinBox;
    refineBeta3Spin->setMaximum(max_value);
    refineBeta3Spin->setDecimals(max_precision);
    refineBeta3Spin->setValue(refineParas.beta3);
    groupLayout->addWidget(beta3Label,4,0);
    groupLayout->addWidget(refineBeta3Spin,4,1);

    QLabel *sigmaLabel = new QLabel;
    sigmaLabel->setText(tr("sigma"));
    refineSigmaSpin = new QDoubleSpinBox;
    refineSigmaSpin->setMaximum(max_value);
    refineSigmaSpin->setDecimals(max_precision);
    refineSigmaSpin->setValue(refineParas.sigma);
    groupLayout->addWidget(sigmaLabel, 5, 0);
    groupLayout->addWidget(refineSigmaSpin, 5, 1);

    refineGroup->setLayout(groupLayout);
    main_layout->addWidget(refineGroup,1,2,1,2);
}

void SettingDialog::initShowGroup()
{
    QGroupBox *showGroup = new QGroupBox;
    showGroup->setTitle(tr("Display"));
    QGridLayout *groupLayout = new QGridLayout;
    
    pointCloud_ComboBox = new QComboBox;
    pointCloud_ComboBox->addItem(tr("vectices"));
    pointCloud_ComboBox->addItem(tr("faces"));
    pointCloud_ComboBox->addItem(tr("none"));
    pointCloud_ComboBox->setCurrentIndex(showParas.pointCloudFormat);
    QLabel *pointCloudLabel = new QLabel;
    pointCloudLabel->setText(tr("PointCloud"));
    groupLayout->addWidget(pointCloudLabel, 0, 0);
    groupLayout->addWidget(pointCloud_ComboBox, 0, 1);

    faceMesh_ComboBox = new QComboBox;
    faceMesh_ComboBox->addItem(tr("faces"));
    faceMesh_ComboBox->addItem(tr("texture"));
    faceMesh_ComboBox->addItem(tr("none"));
    faceMesh_ComboBox->setCurrentIndex(showParas.faceMeshFormat);
    QLabel *faceMeshLabel = new QLabel;
    faceMeshLabel->setText(tr("FaceMesh"));
    groupLayout->addWidget(faceMeshLabel, 1, 0);
    groupLayout->addWidget(faceMesh_ComboBox, 1, 1);

    showGroup->setLayout(groupLayout);
    main_layout->addWidget(showGroup, 2, 0, 1, 2);
}

void SettingDialog::updateConfig()
{
    writeConfig();
    emit update();
    emit close();
}
