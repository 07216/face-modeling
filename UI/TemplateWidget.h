/************************************************
 **
 ** Each widget in the tagWidget.
 **
 ***********************************************/
#ifndef TEMPLATEWIDGET_H
#define TEMPLATEWIDGET_H

#include <QtGui>
#include <QWidget>
#include <QPushButton>
#include <QSplitter>
#include <QLayout>

class ConfigWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ConfigWidget(QWidget *parent = 0);
protected:
    void leaveEvent(QEvent* e);
};

class ConfigButton : public QPushButton
{
    Q_OBJECT
public:
    explicit ConfigButton(ConfigWidget* configWidget, QWidget *parent = 0);
protected:
    void enterEvent(QEvent* e);
private:
    ConfigWidget* configWidget;
};

class TemplateWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TemplateWidget(QWidget *parent = 0);

protected:
    virtual void initLeft();
    virtual void initRight();
    virtual void initBottom();

    QSplitter * mainSplitter;
    QSplitter * splitter;
    ConfigWidget * configWidget;
    QWidget * showAllWidget;
    QWidget * idWidget;
    QPushButton* zoomButton;

    bool isSideHidden;
};

#endif // TEMPLATEWIDGET_H
