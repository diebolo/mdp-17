#include "rviz_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>

namespace rviz_plugin_interface
{
  MyPanel::MyPanel(QWidget *parent)
      : rviz::Panel(parent)
  {
    QVBoxLayout *layout = new QVBoxLayout;
    title_label_ = new QLabel("Farm Cleaning Robot Interface - Group 17");
    start_exploration_button_ = new QPushButton("Start Exploration");
    clean_farm_button_ = new QPushButton("Clean Farm");
    gohome_button_ = new QPushButton("Return Home");
    battery_label_ = new QLabel("Battery: N/A");
    state_label_ = new QLabel("Robot on state: N/A");
    title_label_->setAlignment(Qt::AlignCenter);
    QFont title_font = title_label_->font();
    title_font.setPointSize(16);
    title_font.setBold(true);
    title_label_->setFont(title_font);
    layout->addWidget(title_label_);

    QGroupBox* button_group = new QGroupBox("Controls");
    QVBoxLayout* button_layout = new QVBoxLayout;


    button_layout->addWidget(start_exploration_button_);
    button_layout->addWidget(clean_farm_button_);
    button_layout->addWidget(gohome_button_);
    
    button_group->setLayout(button_layout);
    layout->addWidget(button_group);

    QGroupBox* battery_group = new QGroupBox("Battery Information");
    QVBoxLayout* battery_layout = new QVBoxLayout;

    battery_layout->addWidget(battery_label_);

    battery_status_bar_ = new QProgressBar();
    battery_status_bar_->setRange(0, 100);
    battery_status_bar_->setValue(50);  // Example value
    battery_status_bar_->setTextVisible(true);
    battery_layout->addWidget(battery_status_bar_);

    battery_group->setLayout(battery_layout);
    layout->addWidget(battery_group);

    // Section 3: Robot State and Error Log
    QGroupBox* state_group = new QGroupBox("Robot State and Error Log");
    QVBoxLayout* state_layout = new QVBoxLayout;

    state_layout->addWidget(state_label_);

    error_log_display_ = new QTextEdit();
    error_log_display_->setReadOnly(true);
    state_layout->addWidget(error_log_display_);

    state_group->setLayout(state_layout);
    layout->addWidget(state_group);

    // layout->addWidget(gohome_button_);
    // layout->addWidget(gohome_button_);
    setLayout(layout);

    connect(start_exploration_button_, SIGNAL(clicked()), this, SLOT(onStartExploration()));
    connect(clean_farm_button_, SIGNAL(clicked()), this, SLOT(onCleanFarm()));
    connect(gohome_button_ , SIGNAL(clicked()), this, SLOT(onReturnHome()));
    start_exploration_client_ = nh_.serviceClient<std_srvs::Empty>("/sm/start_frontier_exploration");
    clean_farm_client_ = nh_.serviceClient<std_srvs::Empty>("/sm/start_darp");
    gohome_client_ = nh_.serviceClient<std_srvs::Empty>("/sm/shutdown");
    battery_sub_ = nh_.subscribe("/mirte/power/power_watcher", 10, &MyPanel::batteryCallback, this);
    state_sub_ = nh_.subscribe("/sm/state", 10, &MyPanel::stateCallback, this);
  }

  void MyPanel::onStartExploration()
  {
    std_srvs::Empty srv;
    if (start_exploration_client_.call(srv))
    {
      ROS_INFO("Exploration started successfully.");
    }
    else
    {
      ROS_ERROR("Failed to start exploration.");
    }
  }

  void MyPanel::onCleanFarm()
  {
    std_srvs::Empty srv;
    if (clean_farm_client_.call(srv))
    {
      ROS_INFO("Farm cleaning started successfully.");
    }
    else
    {
      ROS_ERROR("Failed to start farm cleaning.");
    }
  }

  void MyPanel::onReturnHome()
  {
    std_srvs::Empty srv;
    if (gohome_client_.call(srv))
    {
      ROS_INFO("Robot returning home successfully.");
    }
    else
    {
      ROS_ERROR("Failed to make robot return home.");
    }
  }

  void MyPanel::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  battery_label_->setText(QString("Battery: %1%").arg(msg->percentage * 100.0));
  int battery_percentage = static_cast<int>((msg->percentage) * 100);
  battery_status_bar_->setValue(battery_percentage);
}
  void MyPanel::stateCallback(const std_msgs::String::ConstPtr& msg)
{
  state_label_->setText(QString("Robot on state: %1%").arg(QString::fromStdString(msg->data)));
}


} // end namespace rviz_plugin



PLUGINLIB_EXPORT_CLASS(rviz_plugin_interface::MyPanel, rviz::Panel)
