#include "../include/gui_pkg/main_window.hpp"
#include "ui_main_window.h"

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      argc_(argc),
      argv_(argv)
  {
    ui->setupUi(this);
    ros_spin_thread_ = std::thread{std::bind(&MainWindow::rosSpinThread, this)}; // Activating ROS2 spinning functionality for subscribtion callbacks.
    this->my_timer.setInterval(25);  // Update rate 25 ms for GUI.
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
    QString qstr1,qstr5;
    if(!gui_node_->received_data_[0].p_emergency_switch_val)
        {
          setDisabledStyleSheet();
          gui_node_->emergency_button_val_ = 0;
        }

    QTextStream(&qstr1) << gui_node_->received_data_[0].right_x_axis;
        ui->table_controller_node->setItem(0,0,new QTableWidgetItem(qstr1));
        qstr1.clear();

    QTextStream(&qstr1) << gui_node_->received_data_[0].left_x_axis;
        ui->table_controller_node->setItem(1,0,new QTableWidgetItem(qstr1));
        qstr1.clear();

    QTextStream(&qstr1) << gui_node_->received_data_[0].right_limit_switch_val;
       ui->table_easycat->setItem(0,0,new QTableWidgetItem(qstr1));
       qstr1.clear();

    QTextStream(&qstr1) << gui_node_->received_data_[0].left_limit_switch_val;
       ui->table_easycat->setItem(1,0,new QTableWidgetItem(qstr1));
       qstr1.clear();
    if(gui_node_->received_data_[0].p_emergency_switch_val && gui_node_->emergency_button_val_ ){
        QTextStream(&qstr1) << "IDLE";
          ui->table_easycat->setItem(2,0,new QTableWidgetItem(qstr1));
          ui->table_easycat->item(2,0)->setBackground(Qt::green);
          qstr1.clear();
    }else{
          QTextStream(&qstr1) << "EMERGENCY";
          ui->table_easycat->setItem(2,0,new QTableWidgetItem(qstr1));
          ui->table_easycat->item(2,0)->setBackground(Qt::red);
          ui->table_easycat->item(2,0)->setForeground(Qt::white);
          qstr1.clear();
    }

     if(gui_node_->received_data_[0].com_status == 0x08)
     {
         QTextStream(&qstr1) << "OPERATIONAL";
         ui->table_easycat->setItem(3,0,new QTableWidgetItem(qstr1));
         ui->table_easycat->item(3,0)->setBackground(Qt::green);
         qstr1.clear();
     }

    int j=0;
    for(int i = 0; i < NUM_OF_SLAVES ;i++){
            QTextStream(&qstr5) << gui_node_->received_data_[i].target_vel;
            ui->table_master_commands->setItem(j,i,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].target_pos;
            ui->table_master_commands->setItem(j+1,i,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].control_word;
            ui->table_master_commands->setItem(j+2,i,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].actual_vel;
            ui->table_motor_feedback->setItem(j,i,new QTableWidgetItem(qstr5));
            qstr5.clear();

            QTextStream(&qstr5) << gui_node_->received_data_[i].actual_pos;
            ui->table_motor_feedback->setItem(j+1,i,new QTableWidgetItem(qstr5));
            qstr5.clear();

            if (gui_node_->received_data_[i].status_word==567){
                QTextStream(&qstr5) << "RUNNING";
                ui->table_motor_feedback->setItem(j+2,i,new QTableWidgetItem(qstr5));
                ui->table_motor_feedback->item(j+2,i)->setBackground(Qt::green);
                qstr5.clear();
            } else if(gui_node_->received_data_[i].status_word==5687){
                QTextStream(&qstr5) << "READY";
                ui->table_motor_feedback->setItem(j+2,i,new QTableWidgetItem(qstr5));
                ui->table_motor_feedback->item(j+2,i)->setBackground(Qt::yellow);
                qstr5.clear();
            }else{
                QTextStream(&qstr5) << "NOT READY";
                ui->table_motor_feedback->setItem(j+2,i,new QTableWidgetItem(qstr5));
                ui->table_motor_feedback->item(j+2,i)->setBackground(Qt::red);
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
   if (gui_node_->received_data_[0].p_emergency_switch_val){
       gui_node_->emergency_button_val_ = 1;
       setEnabledStyleSheet();
   }
}

void MainWindow::setDisabledStyleSheet()
{
    ui->emergency_button_->setEnabled(false);
    ui->emergency_button_->setStyleSheet("color: rgb(255, 255, 255);"
                                         "background-color: rgb(0, 0,0);"
                                         "font: bold 75 15pt;");
}

void MainWindow::setEnabledStyleSheet()
{
    ui->emergency_button_->setEnabled(true);
    ui->emergency_button_->setStyleSheet("color: rgb(255, 255, 255);"
                                         "background-color: rgb(255, 0,0);"
                                         "font: bold 75 15pt;");
}

void MainWindow::on_emergency_button__clicked()
{
  gui_node_->emergency_button_val_ = 0;
  setDisabledStyleSheet();
}