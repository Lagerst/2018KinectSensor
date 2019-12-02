#pragma once

#include <QThread>

class Thread : public QThread
{
	Q_OBJECT

public:
	Thread();
	void stop();
	void setThread(QString,QString,QString);
protected:
	void run();

public:
	QString personID,roundNo,label;
};