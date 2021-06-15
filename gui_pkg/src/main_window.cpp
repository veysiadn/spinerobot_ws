#include "../include/gui_pkg/main_window.hpp"
#include "ui_main_window.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent),
  ui(new Ui::MainWindow),
  argc_(argc),
  argv_(argv)
{
    ui->setupUi(this);
    // Activating ROS2 spinning functionality for subscribtion callbacks.
    ros_spin_thread_ = std::thread{std::bind(&MainWindow::rosSpinThread, this)};
    this->my_timer.setInterval(25);  // Update rate 25 ms for GUI.
    this->my_timer.start();
    connect(&my_timer, SIGNAL(timeout()), this, SLOT(UpdateGUI()));

    opencv_video_cap =  new VideoCapture(this);
    connect(opencv_video_cap, &VideoCapture::NewPixmapCapture, this, [&]()
    {
       ui->image_frame->setPixmap(opencv_video_cap->pixmap().scaled(1000,720));
    });
    opencv_video_cap->start(QThread::HighestPriority);
}

MainWindow::~MainWindow()
{
  rclcpp::shutdown();
  delete ui;
  opencv_video_cap->terminate();
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
    // Updating Additional GUI Part Veysi ADN
    QString qstr;
   if(!gui_node_->received_data_[0].p_emergency_switch_val){
         setDisabledStyleSheet();
         gui_node_->emergency_button_val_ = 0;
   }

   if(gui_node_->received_data_[0].p_emergency_switch_val && gui_node_->emergency_button_val_ ){
         QTextStream(&qstr) << "    IDLE";
         ui->line_emergency_switch->setText(qstr);
         ui->line_emergency_switch->setStyleSheet("QLineEdit{background:green;"
                                                  "color:black;"
                                                  "font:bold 75 12pt \"Noto Sans\";}");
         qstr.clear();
   }else{
         QTextStream(&qstr) << "    EMERGENCY";
         ui->line_emergency_switch->setText(qstr);
         ui->line_emergency_switch->setStyleSheet("QLineEdit{background:red;"
                                                  "color:black;"
                                                  "font:bold 75 12pt \"Noto Sans\";}");
         qstr.clear();
   }

   if(gui_node_->received_data_[0].com_status == 0x08){

        QTextStream(&qstr) << "    OPERATIONAL";
        ui->line_com_status->setText(qstr);
        ui->line_com_status->setStyleSheet("QLineEdit{background:green;"
                                           "color:black;"
                                           "font:bold 75 12pt \"Noto Sans\";}");
        qstr.clear();
   }else{
       QTextStream(&qstr) << "    NOT OPERATIONAL";
       ui->line_com_status->setText(qstr);
       ui->line_com_status->setStyleSheet("QLineEdit{background:red;"
                                          "color:black;"
                                          "font:bold 75 12pt \"Noto Sans\";}");
       qstr.clear();
   }

   for(int i = 0; i < NUM_OF_SERVO_DRIVES ;i++){
           QTextStream(&qstr) << gui_node_->received_data_[i].target_vel;
           ui->line_target_velocity_m1->setText(qstr);
           ui->line_target_velocity_m1->setStyleSheet("QLabel{background:white;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].control_word;
           ui->line_control_word_m1->setText(qstr);
           qstr.clear();

           QTextStream(&qstr) << gui_node_->received_data_[i].actual_vel;
           ui->line_actual_velocity_m1->setText(qstr);
           qstr.clear();

           if (gui_node_->received_data_[i].status_word==1591){
               QTextStream(&qstr) << "RUNNING";
               ui->line_status_word_m1->setText(qstr);
               ui->line_status_word_m1->setStyleSheet("QLineEdit{background:green;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");
               qstr.clear();
           } else if(gui_node_->received_data_[i].status_word==5687){
               QTextStream(&qstr) << "READY";
               ui->line_status_word_m1->setText(qstr);
               ui->line_status_word_m1->setStyleSheet("QLineEdit{background:yellow;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");
               qstr.clear();
           }else{
               QTextStream(&qstr) << "NOT READY";
               ui->line_status_word_m1->setText(qstr);
               ui->line_status_word_m1->setStyleSheet("QLineEdit{background:red;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");
               qstr.clear();
           }
   }
}

void MainWindow::on_button_reset_clicked()
{
    for (int i = 0 ; i < NUM_OF_SERVO_DRIVES ; i++ ){
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
    ui->button_emergency->setEnabled(false);
    ui->button_emergency->setStyleSheet("color: rgb(255, 255, 255);"
                                         "background-color: rgb(0, 0,0);"
                                         "font: bold 75 15pt;");
}

void MainWindow::setEnabledStyleSheet()
{
    ui->button_emergency->setEnabled(true);
    ui->button_emergency->setStyleSheet("color: rgb(255, 255, 255);"
                                        "background-color: rgb(255, 0,0);"
                                        "font: bold 75 15pt;");
}

void MainWindow::on_button_emergency_clicked()
{
    gui_node_->emergency_button_val_ = 0;
    setDisabledStyleSheet();
}
