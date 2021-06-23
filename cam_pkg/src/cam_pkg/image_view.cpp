#include <cam_pkg/image_view.h>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QTextStream>
#include <QDebug>

using namespace std::chrono_literals;

namespace cam_pkg {

    ImageView::ImageView()
      : rqt_gui_cpp::Plugin()
      , widget_(0)
    {
      setObjectName("spine_robot_gui");
    }

    void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
    {
      widget_ = new QWidget();
      ui_.setupUi(widget_);
      widget_->setWindowTitle("SpineRobotView");
      context.addWidget(widget_);

      connect(ui_.button_emergency,SIGNAL(clicked(bool)),this, SLOT(on_button_emergency_clicked()));
      connect(ui_.button_reset,SIGNAL(clicked(bool)),this, SLOT(on_button_reset_clicked()));


      gui_publisher_ = node_->create_publisher<std_msgs::msg::UInt8>("gui_buttons", 10);

      controller_commands_= node_->create_subscription<sensor_msgs::msg::Joy>("Controller", 10,std::bind(&ImageView::HandleControllerCallbacks,
                                                                                                         this, std::placeholders::_1));
      slave_feedback_ = node_->create_subscription<ecat_msgs::msg::DataReceived>("Slave_Feedback", 10,std::bind(&ImageView::HandleSlaveFeedbackCallbacks,
                                                                                                         this, std::placeholders::_1));
      master_commands_ = node_->create_subscription<ecat_msgs::msg::DataSent>("Master_Commands", 10,std::bind(&ImageView::HandleMasterCommandCallbacks,
                                                                                                          this, std::placeholders::_1));

      image_width_ = node_->declare_parameter("image_width", 960);
      image_height_ = node_->declare_parameter("image_height", 720);
      fps_ = node_->declare_parameter("fps", 30.0);
      camera_id = node_->declare_parameter("camera_id", 2);
      cap_video.open(camera_id);
      cap_video.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
      cap_video.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
      last_frame_ = std::chrono::steady_clock::now();

      timer_ = node_->create_wall_timer(20ms, std::bind(&ImageView::callbackImage,this));

      received_data_[0].p_emergency_switch_val=1;
      on_button_reset_clicked();
    }

    void ImageView::shutdownPlugin()
    {
        gui_publisher_.reset();
        controller_commands_.reset();
        master_commands_.reset();
        slave_feedback_.reset();
    }

    void ImageView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {

    }

    void ImageView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
    }

    QImage ImageView::cvMatToQImage( const cv::Mat &inMat )
    {
      switch ( inMat.type() )
      {
         // 8-bit, 4 channel
         case CV_8UC4:
         {
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_ARGB32 );

            return image;
         }

         // 8-bit, 3 channel
         case CV_8UC3:
         {
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_RGB888 );

            return image.rgbSwapped();
         }

         // 8-bit, 1 channel
         case CV_8UC1:
         {
    #if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_Grayscale8 );
    #else
            static QVector<QRgb>  sColorTable;

            // only create our color table the first time
            if ( sColorTable.isEmpty() )
            {
               sColorTable.resize( 256 );

               for ( int i = 0; i < 256; ++i )
               {
                  sColorTable[i] = qRgb( i, i, i );
               }
            }

            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_Indexed8 );

            image.setColorTable( sColorTable );
    #endif

            return image;
         }

      default:
         qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
         break;
   }
      return QImage();
   }

   QPixmap ImageView::cvMatToQPixmap( const cv::Mat &inMat )
   {
      return QPixmap::fromImage( cvMatToQImage( inMat ) );
   }

    void ImageView::callbackImage()
    {
      cap_video >> cap_frame;
      cap_pixmap = cvMatToQPixmap(cap_frame);
      auto now = std::chrono::steady_clock::now();

      if (!cap_frame.empty()&&
              std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count() > 1/fps_*1000){
              last_frame_ = now;
              ui_.my_frame->setPixmap(cap_pixmap.scaled(image_width_,image_height_));
      }

      // VeysiADN gui_update functionality.
      UpdateGUI();

      auto button_info = std_msgs::msg::UInt8();
      button_info.data = emergency_button_val_;
      gui_publisher_->publish(button_info);

    }
/***************************************************************************************************/
    /*VeysiADN modifications start*/
    void ImageView::on_button_emergency_clicked()
    {
        emergency_button_val_ = 0 ;
        setDisabledStyleSheet();
    }

    void ImageView::on_button_reset_clicked()
    {
        for (int i = 0 ; i < NUM_OF_SERVO_DRIVES ; i++ ){
             received_data_[i].status_word = 0 ;
             received_data_[i].actual_pos = 0 ;
             received_data_[i].actual_vel = 0 ;

             received_data_[i].control_word = 0 ;
             received_data_[i].target_pos  = 0 ;
             received_data_[i].target_vel = 0 ;
             received_data_[0].left_limit_switch_val = 0 ;
             received_data_[0].right_limit_switch_val = 0;
             received_data_[0].right_x_axis = 0 ;
             received_data_[0].left_x_axis = 0 ;
        }
        if (received_data_[0].p_emergency_switch_val){
            emergency_button_val_ = 1;
            setEnabledStyleSheet();
        }
    }

    void ImageView::HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
       for(int i=0; i < NUM_OF_SERVO_DRIVES ; i++){
          received_data_[i].right_x_axis = msg->axes[3];
          received_data_[i].left_x_axis =  msg->axes[0];
       }
    }

    void ImageView::HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg)
    {
        for(int i=0; i < NUM_OF_SERVO_DRIVES ; i++){
           received_data_[i].target_pos   =  msg->target_pos[i];
           received_data_[i].target_vel   =  msg->target_vel[i];
           received_data_[i].control_word =  msg->control_word[i];
        }

    }

    void ImageView::HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg)
    {
        std::cout << "Handling callback.." << std::endl;
        for(int i=0; i < NUM_OF_SERVO_DRIVES ; i++){
          received_data_[i].actual_pos             =  msg->actual_pos[i];
          received_data_[i].actual_vel             =  msg->actual_vel[i];
          received_data_[i].status_word            =  msg->status_word[i];
          received_data_[i].left_limit_switch_val  =  msg->left_limit_switch_val;
          received_data_[i].right_limit_switch_val =  msg->right_limit_switch_val;
          received_data_[i].p_emergency_switch_val =  msg->emergency_switch_val;
          received_data_[i].com_status             =  msg->com_status;
      }
    }

    void ImageView::setDisabledStyleSheet()
    {
        ui_.button_emergency->setEnabled(false);
        ui_.button_emergency->setStyleSheet("color: rgb(255, 255, 255);"
                                             "background-color: rgb(0, 0,0);"
                                             "font: bold 75 15pt \"Noto Sans\";");
    }

    void ImageView::setEnabledStyleSheet()
    {
        ui_.button_emergency->setEnabled(true);
        ui_.button_emergency->setStyleSheet("color: rgb(255, 255, 255);"
                                             "background-color: rgb(255, 0,0);"
                                             "font: bold 75 15pt;");
    }

    void ImageView::UpdateGUI()
    {
        // Updating Additional GUI Part Veysi ADN
        QString qstr;
       if(!received_data_[0].p_emergency_switch_val){
             setDisabledStyleSheet();
             emergency_button_val_ = 0;
       }

       if(received_data_[0].p_emergency_switch_val && emergency_button_val_ ){
             QTextStream(&qstr) << "    IDLE";
             ui_.line_emergency_switch->setText(qstr);
             ui_.line_emergency_switch->setStyleSheet("QLineEdit{background:green;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");
             qstr.clear();
       }else{
             QTextStream(&qstr) << "    EMERGENCY";
             ui_.line_emergency_switch->setText(qstr);
             ui_.line_emergency_switch->setStyleSheet("QLineEdit{background:red;"
                                                      "color:black;"
                                                      "font:bold 75 12pt \"Noto Sans\";}");
             qstr.clear();
       }

       if(received_data_[0].com_status == 0x08){

            QTextStream(&qstr) << "    OPERATIONAL";
            ui_.line_com_status->setText(qstr);
            ui_.line_com_status->setStyleSheet("QLineEdit{background:green;"
                                               "color:black;"
                                               "font:bold 75 12pt \"Noto Sans\";}");
            qstr.clear();
       }else{
           QTextStream(&qstr) << "    NOT OPERATIONAL";
           ui_.line_com_status->setText(qstr);
           ui_.line_com_status->setStyleSheet("QLineEdit{background:red;"
                                              "color:black;"
                                              "font:bold 75 12pt \"Noto Sans\";}");
           qstr.clear();
       }

       for(int i = 0; i < NUM_OF_SERVO_DRIVES ;i++){
               QTextStream(&qstr) << received_data_[i].target_vel;
               ui_.line_target_velocity_m1->setText(qstr);
               ui_.line_target_velocity_m1->setStyleSheet("QLabel{background:white;"
                                                          "color:black;"
                                                          "font:bold 75 12pt \"Noto Sans\";}");
               qstr.clear();

               QTextStream(&qstr) << received_data_[i].control_word;
               ui_.line_control_word_m1->setText(qstr);
               qstr.clear();

               QTextStream(&qstr) << received_data_[i].actual_vel;
               ui_.line_actual_velocity_m1->setText(qstr);
               qstr.clear();

               if (received_data_[i].status_word==567){
                   QTextStream(&qstr) << "RUNNING";
                   ui_.line_status_word_m1->setText(qstr);
                   ui_.line_status_word_m1->setStyleSheet("QLineEdit{background:green;"
                                                          "color:black;"
                                                          "font:bold 75 12pt \"Noto Sans\";}");
                   qstr.clear();
               } else if(received_data_[i].status_word==5687){
                   QTextStream(&qstr) << "READY";
                   ui_.line_status_word_m1->setText(qstr);
                   ui_.line_status_word_m1->setStyleSheet("QLineEdit{background:yellow;"
                                                          "color:black;"
                                                          "font:bold 75 12pt \"Noto Sans\";}");
                   qstr.clear();
               }else{
                   QTextStream(&qstr) << "NOT READY";
                   ui_.line_status_word_m1->setText(qstr);
                   ui_.line_status_word_m1->setStyleSheet("QLineEdit{background:red;"
                                                          "color:black;"
                                                          "font:bold 75 12pt \"Noto Sans\";}");
                   qstr.clear();
               }

       }
    }
}//Class ImageView


PLUGINLIB_EXPORT_CLASS(cam_pkg::ImageView, rqt_gui_cpp::Plugin)
