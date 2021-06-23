/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*****************************************************************************
 * \file  imageview.h
 * \brief Header file for spinerobot GUI with endoscop camera viewer.
 *******************************************************************************/
#ifndef cam_pkg__ImageView_H
#define cam_pkg__ImageView_H
// Qt for ROS2 header required.
#include <rqt_gui_cpp/plugin.h>
// ROS2 core library header
#include "rclcpp/rclcpp.hpp"

// Message file headers, -custom and built-in-
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "ecat_msgs/msg/data_received.hpp"
#include "ecat_msgs/msg/data_sent.hpp"

// OpenCV libraries ROS2 for capturing camera.
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>

// Qt header files.
#include <ui_image_view.h>
#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QPixmap>
#include <QCloseEvent>
#include <QMessageBox>

// C++ headers
#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>

// number of drives should de specified first.
#define NUM_OF_SERVO_DRIVES 1

typedef struct
{
    int32_t   target_pos ;
    int32_t   target_vel ;
    int16_t   target_tor ;
    int16_t   max_tor ;
    uint16_t  control_word ;
    int32_t   vel_offset ;
    int16_t   tor_offset ;

    int32_t  actual_pos ;
    int32_t  actual_vel ;
    int16_t  actual_cur ;
    int16_t  actual_tor ;
    uint16_t status_word ;
    int8_t   op_mode_display ;
    uint8_t  left_limit_switch_val ;
    uint8_t  right_limit_switch_val ;
    int32_t  right_x_axis;
    int32_t  left_x_axis;
    uint8_t  p_emergency_switch_val;
    uint8_t  com_status;
}ReceivedData;

namespace cam_pkg {

class ImageView
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  ImageView();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


protected:

    virtual void callbackImage();

    Ui::ImageViewWidget ui_;

    QWidget* widget_;

private slots:
    void on_button_emergency_clicked();
    void on_button_reset_clicked();

private:
  /**
   * @brief Function will be used for subscribtion callbacks from controller node
   *        for Controller topic.
   *
   * @param msg controller command structure published by controller node.
   */
    void HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg);
    /**
     * @brief Function will be used for subscribtion callbacks from EthercatLifecycle node
     *        for Master_Commands topic.
     *
     * @param msg Master commands structure published by EthercatLifecycle node
     */
    void HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg);
    /**
     * @brief Function will be used for subscribtion callbacks from EthercatLifecycle node
     *        for Master_Commands topic.
     *
     * @param msg Slave feedback structure published by EthercatLifecycle node
     */
    void HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg);
    /**
     * @brief Callback function for GUI publisher to publish button values each 10 ms.
     */
    void timer_callback();
    /**
     * @brief Function to configure GUI view in case of Emergency
     */
    void setEnabledStyleSheet();

    /**
     * @brief Function to configure GUI view in case of Emergency
     */
    void setDisabledStyleSheet();

    /**
     * @brief Updates GUI to show subscribed data to user.
     */
    void UpdateGUI();
    /**
     * @brief This function is to convert OpenCV camera reading to QImage structure to show it on GUI.
     * @param inMat
     * @return QImage
     */
    QImage cvMatToQImage(const cv::Mat &inMat);

    /**
     * @brief Converts QImage structure to QPixmap structure to show camera captures on GUI.
     * @param inMat
     * @return
     */
    QPixmap cvMatToQPixmap(const cv::Mat &inMat );

    // ROS2 Publisher / Subscriber Declarations
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_commands_;
    rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr slave_feedback_;
    rclcpp::Subscription<ecat_msgs::msg::DataSent>::SharedPtr master_commands_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gui_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Received data structure from EtherCAT node and controller node.
    ReceivedData received_data_[NUM_OF_SERVO_DRIVES];

    // Emergency button value to publish.
    unsigned int emergency_button_val_ = 1;

    // Parameters for camera capturing.
    int image_height_;
    int image_width_;
    double fps_;
    int camera_id;

    QPixmap cap_pixmap;              //Qt image
    cv::Mat cap_frame;               //OpenCV image
    cv::VideoCapture cap_video;      //video capture
    std::chrono::steady_clock::time_point last_frame_;

};

}

#endif // cam_pkg__ImageView_H
