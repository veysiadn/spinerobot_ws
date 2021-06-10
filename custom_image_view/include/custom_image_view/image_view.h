/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef custom_image_view__ImageView_H
#define custom_image_view__ImageView_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_view.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "std_msgs/msg/u_int8.hpp"

#include "ecat_msgs/msg/data_received.hpp"
#include "ecat_msgs/msg/data_sent.hpp"

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <opencv2/core/core.hpp>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>

#include <vector>

#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>

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

namespace custom_image_view {

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

protected slots:

  virtual void updateTopicList();

protected:

  virtual QSet<QString> getTopics(const QSet<QString>& message_types, const QSet<QString>& message_sub_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);

protected slots:

  virtual void onTopicChanged(int index);

  virtual void onZoom1(bool checked);

  virtual void onDynamicRange(bool checked);

  virtual void saveImage();

  virtual void updateNumGridlines();

  virtual void onMousePublish(bool checked);

  virtual void onMouseLeft(int x, int y);

  virtual void onPubTopicChanged();

  virtual void onHideToolbarChanged(bool hide);

  virtual void onRotateLeft();
  virtual void onRotateRight();

protected:

  virtual void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  virtual void invertPixels(int x, int y);

  QList<int> getGridIndices(int size) const;

  virtual void overlayGrid();

  Ui::ImageViewWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;

  cv::Mat conversion_mat_;

private slots:
  void on_button_emergency_clicked();
  void on_button_reset_clicked();
private:

  enum RotateState {
    ROTATE_0 = 0,
    ROTATE_90 = 1,
    ROTATE_180 = 2,
    ROTATE_270 = 3,

    ROTATE_STATE_COUNT
  };

  void syncRotateLabel();

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

    void setEnabledStyleSheet();

    void setDisabledStyleSheet();

    void UpdateGUI();

  QString arg_topic_name;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_mouse_left_;

  rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr slave_feedback_;
  rclcpp::Subscription<ecat_msgs::msg::DataSent>::SharedPtr master_commands_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_commands_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gui_publisher_;

  bool pub_topic_custom_;

  QAction* hide_toolbar_action_;

  int num_gridlines_;

  RotateState rotate_state_ = ROTATE_0 ;

  // Received data structure from EtherCAT node and controller node.
  ReceivedData received_data_[NUM_OF_SERVO_DRIVES];
  unsigned int emergency_button_val_ = 1;
};

}

#endif // custom_image_view__ImageView_H
