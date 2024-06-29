#ifndef RVIZ_PLUGIN_H
#define RVIZ_PLUGIN_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <QProgressBar>
#include <QTextEdit>
#include <QGroupBox>
#include <QHBoxLayout>
namespace rviz_plugin_interface
{
  class MyPanel : public rviz::Panel
  {
    Q_OBJECT
  public:
    MyPanel(QWidget *parent = 0);

  protected Q_SLOTS:
    void onStartExploration();
    void onCleanFarm();
    void onReturnHome();
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg);
  private:
    QLabel* title_label_;
    QPushButton *start_exploration_button_;
    QPushButton *clean_farm_button_;
    QPushButton *gohome_button_;
    QLabel* battery_label_;
    QLabel* state_label_;
    QProgressBar* battery_status_bar_;
    QTextEdit* error_log_display_;
    ros::NodeHandle nh_;
    ros::ServiceClient start_exploration_client_;
    ros::ServiceClient clean_farm_client_;
    ros::ServiceClient gohome_client_;
    ros::Subscriber battery_sub_;
    ros::Subscriber state_sub_ ;
  };
} // end namespace my_rviz_plugin

#endif // RVIZ_PLUGIN_H