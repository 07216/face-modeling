#include "ShowThread.h"
#include "qdebug.h"

ShowThread::ShowThread(QString name)
{
    this->name = name;
}

void ShowThread::run()
{
    emit track();
}
