#ifndef SHOWTHREAD_H
#define SHOWTHREAD_H

#include <QThread>

class ShowThread : public QThread
{
    Q_OBJECT
public:
    explicit ShowThread(QString name = "");

    void run();
    
signals:
    void track();
    
public slots:

private:
    QString name;

    
};

#endif // SHOWTHREAD_H
