#include <cam_pkg/image_view.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setStyle("fusion");
    ImageView w;
    a.setWindowTitle("SpineRobotGUI");
    w.show();

    return a.exec();
}
