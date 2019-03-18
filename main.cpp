#include "stdafx.h"

#include "trackeronbovwwindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	TrackerOnBoVWwindow w;
	w.show();
	return a.exec();
}
