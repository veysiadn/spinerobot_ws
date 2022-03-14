#ifndef CONTROLUI_H
#define CONTROLUI_H

#include <QWidget>

namespace Ui {
class ControlUi;
}

class ControlUi : public QWidget
{
    Q_OBJECT

public:
    explicit ControlUi(QWidget *parent = nullptr);
    ~ControlUi();

private:
    Ui::ControlUi *ui;
};

#endif // CONTROLUI_H
