#pragma once
//CPP
#include <chrono>
#include <memory>
#include <iostream>
//ROS2
#include "rclcpp/rclcpp.hpp"

//GUI_Node Headers
#include "gui_node.hpp"


// QT
#include "ui_main_window.h"
#include <QMainWindow>
#include <QApplication>
#include <QStandardItemModel>
#include <QTableView>
#include <QHeaderView>
#include <QString>
#include <QTextStream>
#include <QTimer>

using namespace GUI;
namespace Ui {
class MainWindow;
}
  class MainWindow : public QMainWindow {
  Q_OBJECT
  public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();

  private slots:
    void on_button_reset__clicked();
    void UpdateGUI();

  private:
    Ui::MainWindow *ui;
    int argc_;
    char** argv_;

    QTimer my_timer;

    std::shared_ptr<GuiNode> gui_node_;
    std::thread ros_spin_thread_;
    void rosSpinThread();

  };
