#include "../include/gui_pkg/main_window.hpp"
#include "ui_main_window.h"

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      argc_(argc),
      argv_(argv)
  {
    ui->setupUi(this);
    ros_spin_thread_ = std::thread{std::bind(&MainWindow::rosSpinThread, this)};
    this->my_timer.setInterval(10);
    this->my_timer.start();
    connect(&my_timer, SIGNAL(timeout()), this, SLOT(UpdateGUI()));
  }

  MainWindow::~MainWindow()
  {
      rclcpp::shutdown();
  }

  // Start ROS2 NODE
  void MainWindow::rosSpinThread()
  {
    rclcpp::init(argc_, argv_);
    gui_node_ = std::make_shared<GuiNode>();
    rclcpp::spin(gui_node_);
    rclcpp::shutdown();
  }

void MainWindow::UpdateGUI()
{
    QString qstr1,qstr2,qstr3,qstr4,qstr5,qstr6,qstr7,qstr8,qstr9,qstr0;

    QTextStream(&qstr1) << gui_node_->received_data_[0].right_x_axis;
        ui->table_controller_node->setItem(0,0,new QTableWidgetItem(qstr1));
    QTextStream(&qstr2) << gui_node_->received_data_[0].left_x_axis;
        ui->table_controller_node->setItem(1,0,new QTableWidgetItem(qstr2));

    QTextStream(&qstr3) << gui_node_->received_data_[0].right_limit_switch_val;
       ui->table_easycat->setItem(0,0,new QTableWidgetItem(qstr3));
    QTextStream(&qstr4) << gui_node_->received_data_[0].left_limit_switch_val;
       ui->table_easycat->setItem(1,0,new QTableWidgetItem(qstr4));

    for(int i = 0; i < NUM_OF_SLAVES ;i++){
        for (int j = 0 ; j < NUM_OF_SLAVES ; j++){
            QTextStream(&qstr5) << gui_node_->received_data_[i].target_vel;
            ui->table_master_commands->setItem(i,j,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].target_pos;
            ui->table_master_commands->setItem(i,j,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].control_word;
            ui->table_master_commands->setItem(i,j,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].actual_vel;
            ui->table_motor_feedback->setItem(i,j,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].actual_pos;
            ui->table_motor_feedback->setItem(i,j,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].status_word;
            ui->table_motor_feedback->setItem(i,j,new QTableWidgetItem(qstr5));
            qstr5.clear();

        }
    }

}
void MainWindow::on_button_reset__clicked()
{
   for (int i = 0 ; i < NUM_OF_SLAVES ; i++ ){
        gui_node_->received_data_[i].status_word = 0 ;
        gui_node_->received_data_[i].actual_pos = 0 ;
        gui_node_->received_data_[i].actual_vel = 0 ;

        gui_node_->received_data_[i].control_word = 0 ;
        gui_node_->received_data_[i].target_pos  = 0 ;
        gui_node_->received_data_[i].target_vel = 0 ;
        gui_node_->received_data_[0].left_limit_switch_val = 0 ;
        gui_node_->received_data_[0].right_limit_switch_val = 0;
        gui_node_->received_data_[0].right_x_axis = 0 ;
        gui_node_->received_data_[0].left_x_axis = 0 ;
   }
}
