#include "mainwindow.h"
#include "qdebug.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    init();
    tabWidget = new QTabWidget;
    tabAnimation = new AnimationWidget(face);
    tabWidget->setStyleSheet("QTabWidget::pane{border: 0px;}"
                             "QTabWidget::tab-bar{alignment:center;}"
                             "QTabBar::tab:selected{border-color: white;background:white;color:green;}");
    QPalette palette;
    palette.setBrush(QPalette::Window, QBrush(QColor(96,98,90)));
    tabWidget->setPalette(palette);

    animationIndex = tabWidget->addTab(tabAnimation, tr("Animation"));
    setCentralWidget(tabWidget);
}
void MainWindow::init()
{
    face = new FacePerformance;
    face->userViewMesh = new UserViewMesh;
    face->avatarViewMesh = new AvatarViewMesh;
    face->userImage = new QLabel;
    face->recognizedImage = new QLabel;

    face->mesh_color.resize(0,0);
    face->userViewMesh->mesh_color = &(face->mesh_color);
    face->userViewMesh->mutex_showParas = &(face->mutex_show);
    face->userViewMesh->setMutex_pointCloud(&(face->mutex_pointCloud));
    face->userViewMesh->showParas = &(face->showParas);
    face->userViewMesh->mutex_isTracking = &(face->mutex_isTracking);
    face->userViewMesh->isTracking = &(face->isTracking);

    face->userViewMesh->setMesh(ConfigParameter::template_mesh_filename);//"test.obj");
    face->userViewMesh->camera()->setZClippingCoefficient(50);

    face->avatarViewMesh->mutex_x = &(face->mutex_x);
    face->avatarViewMesh->mutex_isTracking = &(face->mutex_isTracking);
    face->avatarViewMesh->isTracking = &(face->isTracking);
    face->avatarViewMesh->camera()->setZClippingCoefficient(50);
    face->avatarViewMesh->loadAllAvatars(ConfigParameter::avatarModelsDir);//"avatar-models/");//

    face->InitializePlugin();
    connect(face, SIGNAL(UpdateView()), face->userViewMesh, SLOT(update()));
    connect(face, SIGNAL(UpdateAvatar()), face->avatarViewMesh, SLOT(updateMesh()));
    connect(face, SIGNAL(UpdateView()), face->avatarViewMesh, SLOT(update()));

    QPalette palette;
    palette.setBrush(QPalette::Window, QBrush(QColor(96,98,90)));

    face->userImage->setAutoFillBackground(true);
    face->userImage->setPalette(palette);
    face->userImage->setAlignment(Qt::AlignCenter);
    face->userImage->setFixedSize(640,480);

    face->recognizedImage->setAutoFillBackground(true);
    face->recognizedImage->setPalette(palette);
    face->recognizedImage->setAlignment(Qt::AlignCenter);
}

MainWindow::~MainWindow()
{
}

