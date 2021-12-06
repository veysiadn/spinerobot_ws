#include "controlui.h"
#include "ui_controlui.h"

ControlUi::ControlUi(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControlUi)
{
    ui->setupUi(this);
}

ControlUi::~ControlUi()
{
    delete ui;
}
