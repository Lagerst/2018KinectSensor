#pragma once
#define WIDGET_H
#include<qwidget.h>
#include<qpushbutton.h>
#include<qlineedit.h>
#include<qlabel.h>
#include<Thread.h>
class Widget :public QWidget {
	Q_OBJECT
public:
	Widget(QWidget*parent = 0);
	~Widget();
private:
	QPushButton*btn1,*btn2;
	QLineEdit*edit1,*edit2,*edit3;
	QLabel*label1,*label2,*label3;
	Thread thread;
private slots:
	void add();
	void stop();
};
