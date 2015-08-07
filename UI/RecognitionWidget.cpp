#include "RecognitionWidget.h"


RecognitionWidget::RecognitionWidget(FacePerformance* face, QWidget *parent) :
    TemplateWidget(parent)
{
    this->face = face;
    this->setAutoFillBackground (true);
    QPalette palette;
    palette.setColor(QPalette::Active, static_cast<QPalette::ColorRole>(10), QColor(96,98,90));
    this->setPalette(palette);
    initLeft();
    initRight();
    connect(face, SIGNAL(ShowColorImage(uchar*,int,int)), this, SLOT(ShowColorImage(uchar*,int,int)));
}

void RecognitionWidget::initLeft()
{
    QPushButton * trackingButton = new QPushButton(tr("tracking"));
    QPushButton * configButton = new QPushButton(tr("&Config"));
    QPushButton * changeUser = new QPushButton(tr("change user"));
    SettingDialog * config = new SettingDialog;
    QGridLayout * layout = new QGridLayout;

    layout->addWidget(trackingButton,0,0);
    layout->addWidget(configButton,1,0);
    layout->addWidget(changeUser, 2, 0);
    configWidget->setLayout(layout);

    connect(trackingButton, SIGNAL(clicked()), face, SLOT(StartTrackThread()));
    connect(configButton, SIGNAL(clicked()), config, SLOT(exec()));
    connect(config, SIGNAL(update()), face, SLOT(UpdateParas()));
    connect(changeUser, SIGNAL(clicked()), this, SLOT(ChangeUser()));
}

void RecognitionWidget::initRight()
{
    QHBoxLayout * showLayout = new QHBoxLayout;
    showLayout->addWidget(face->userImage);
    showLayout->addWidget(face->userViewMesh);
    showLayout->addWidget(face->avatarViewMesh);
    showLayout->setSpacing(0);
    showLayout->setContentsMargins(0, 0, 0, 0);
    showAllWidget->setLayout(showLayout);
}
void RecognitionWidget::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_F1)//ChangeUser
    {
        std::cout<<"F1"<<std::endl;
        face->mutex_isInitFrame.lock();
        face->isInitFrame = true;
        face->mutex_isInitFrame.unlock();
    }
}
void RecognitionWidget::ChangeUser()
{
    face->mutex_isInitFrame.lock();
    face->isInitFrame = true;
    face->mutex_isInitFrame.unlock();
    face->userViewMesh->camera()->setZClippingCoefficient(50);
    face->avatarViewMesh->camera()->setZClippingCoefficient(50);
}

void RecognitionWidget::ShowColorImage(uchar *data, int cols, int rows)
{
    face->mutex_colorImage.lock();
    QImage image(data, cols, rows, QImage::Format_RGB888);
    face->userImage->setPixmap(QPixmap::fromImage(image.scaled(face->userImage->size(), Qt::KeepAspectRatio)));
    face->mutex_colorImage.unlock();
}
