#define IS_QT_FIVE_OR_MORE 1 // TODO: move this marco to the configure file(.hpp)

#if IS_QT_FIVE_OR_MORE
#include <QtWidgets/QApplication>
#else
#include <QtGui/QApplication>
#endif

#include "maindialog.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainDialog w;
    w.show();
    return a.exec();
}
