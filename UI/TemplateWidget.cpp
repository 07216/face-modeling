#include "TemplateWidget.h"
#include <qdebug.h>

ConfigWidget::ConfigWidget(QWidget *parent):
    QWidget(parent)
{

}

void ConfigWidget::leaveEvent(QEvent *e)
{
    this->setVisible(false);
}

ConfigButton::ConfigButton(ConfigWidget* configWidget, QWidget *parent):
    QPushButton(parent)
{
    this->configWidget = configWidget;
}

void ConfigButton::enterEvent(QEvent* e)
{
    configWidget->setVisible(true);
}

TemplateWidget::TemplateWidget(QWidget *parent) :
    QWidget(parent)
{
    isSideHidden = false;
    mainSplitter = new QSplitter();
    splitter=new QSplitter(Qt::Vertical);
    configWidget = new ConfigWidget();
    showAllWidget = new QWidget();
    idWidget = new QWidget();
    zoomButton = new ConfigButton(configWidget);
    zoomButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    zoomButton->setFixedWidth(6);
    zoomButton->setFocusPolicy(Qt::NoFocus);
    zoomButton->setStyleSheet("background: #E8E8E8; border: none; padding: 0px;");

    initLeft();
    initRight();
    initBottom();

    mainSplitter->addWidget(configWidget);
    //mainSplitter->addWidget(showAllWidget);
    QHBoxLayout *main_layout = new QHBoxLayout();
    QHBoxLayout *main_layout1 = new QHBoxLayout();
    QVBoxLayout *main_layout2 = new QVBoxLayout();

    main_layout->addWidget(zoomButton);
    splitter->addWidget(showAllWidget);
    splitter->addWidget(idWidget);
    splitter->setStretchFactor(0,50);
    splitter->setStretchFactor(1,50);
    //setAutoFullBackground(true);
    mainSplitter->addWidget(splitter);
    main_layout->addWidget(mainSplitter);



    //main_layout1->addWidget(idWidget);

    //main_layout2->addLayout(main_layout);
    //main_layout2->addLayout(main_layout1);
    main_layout->setSpacing(0);
    main_layout->setContentsMargins(0, 0, 0, 0);


    setLayout(main_layout);
}

void TemplateWidget::initLeft()
{
    configWidget->resize(200,100);
    QPalette palette;
    palette.setBrush(QPalette::Window, QBrush(QColor(51,51,51)));
    configWidget->setPalette(palette);
    configWidget->setAutoFillBackground(true);
}

void TemplateWidget::initRight()
{
    QPalette palette;
    palette.setBrush(QPalette::Window, QBrush(QColor(96,98,90)));
    showAllWidget->setPalette(palette);
    showAllWidget->setAutoFillBackground(true);
}


void TemplateWidget::initBottom()
{

    QPalette palette;
    palette.setBrush(QPalette::Window, QBrush(QColor(96,98,90)));
    idWidget->setPalette(palette);
    idWidget->setAutoFillBackground(true);

}

