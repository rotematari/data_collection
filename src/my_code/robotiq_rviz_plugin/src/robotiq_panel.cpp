#include "robotiq_rviz_plugin/robotiq_panel.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace robotiq_rviz_plugin
{

RobotiqPanel::RobotiqPanel(QWidget * parent)
: rviz_common::Panel(parent)
, node_(nullptr)
{
  setupUI();
}

RobotiqPanel::~RobotiqPanel()
{
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

void RobotiqPanel::onInitialize()
{
  // Initialize ROS node
  auto node_options = rclcpp::NodeOptions();
  node_ = std::make_shared<rclcpp::Node>("robotiq_rviz_panel", node_options);
  
  // Publishers
  command_pub_ = node_->create_publisher<robotiq_msgs::msg::RobotiqGripperCommand>(
    "robotiq_gripper/command", 10);
  
  // // Subscribers
  // status_sub_ = node_->create_subscription<robotiq_msgs::msg::RobotiqGripperStatus>(
  //   "robotiq_gripper/status", 10,
  //   std::bind(&RobotiqPanel::statusCallback, this, std::placeholders::_1));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() {
    executor_->spin();
  });
}

void RobotiqPanel::setupUI()
{
  auto * main_layout = new QVBoxLayout;
  
  // Control buttons
  auto * control_group = new QGroupBox("Gripper Control");
  auto * control_layout = new QGridLayout;
  
  activate_btn_ = new QPushButton("Activate");
  reset_btn_ = new QPushButton("Reset");
  open_btn_ = new QPushButton("Open");
  close_btn_ = new QPushButton("Close");
  setup_btn_ = new QPushButton("Setup");
  
  control_layout->addWidget(activate_btn_, 0, 0);
  control_layout->addWidget(reset_btn_, 0, 1);
  control_layout->addWidget(open_btn_, 1, 0);
  control_layout->addWidget(close_btn_, 1, 1);
  control_layout->addWidget(setup_btn_, 2, 0, 1, 2);
  
  control_group->setLayout(control_layout);
  
  // Position control
  auto * position_group = new QGroupBox("Position Control");
  auto * position_layout = new QVBoxLayout;
  
  position_slider_ = new QSlider(Qt::Horizontal);
  position_slider_->setRange(0, 255);
  position_slider_->setValue(0);
  
  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(0, 255);
  speed_slider_->setValue(128);
  
  force_slider_ = new QSlider(Qt::Horizontal);
  force_slider_->setRange(0, 255);
  force_slider_->setValue(128);
  
  goto_btn_ = new QPushButton("Go to Position");
  
  position_layout->addWidget(new QLabel("Position:"));
  position_layout->addWidget(position_slider_);
  position_layout->addWidget(new QLabel("Speed:"));
  position_layout->addWidget(speed_slider_);
  position_layout->addWidget(new QLabel("Force:"));
  position_layout->addWidget(force_slider_);
  position_layout->addWidget(goto_btn_);
  
  position_group->setLayout(position_layout);
  
  // Status display
  // auto * status_group = new QGroupBox("Status");
  // auto * status_layout = new QVBoxLayout;
  
  // status_label_ = new QLabel("Status: Unknown");
  // position_label_ = new QLabel("Position: 0");
  // current_label_ = new QLabel("Current: 0");
  
  // status_layout->addWidget(status_label_);
  // status_layout->addWidget(position_label_);
  // status_layout->addWidget(current_label_);
  
  // status_group->setLayout(status_layout);
  
  // Add to main layout
  main_layout->addWidget(control_group);
  main_layout->addWidget(position_group);
  // main_layout->addWidget(status_group);
  main_layout->addStretch();
  
  setLayout(main_layout);
  
  // Connect signals
  connect(activate_btn_, &QPushButton::clicked, this, &RobotiqPanel::onActivateClicked);
  connect(reset_btn_, &QPushButton::clicked, this, &RobotiqPanel::onResetClicked);
  connect(open_btn_, &QPushButton::clicked, this, &RobotiqPanel::onOpenClicked);
  connect(close_btn_, &QPushButton::clicked, this, &RobotiqPanel::onCloseClicked);
  connect(goto_btn_, &QPushButton::clicked, this, &RobotiqPanel::onGotoClicked);

  connect(setup_btn_, &QPushButton::clicked, this, &RobotiqPanel::onSetupClicked);

  connect(position_slider_, QOverload<int>::of(&QSlider::valueChanged), this, &RobotiqPanel::onPositionChanged);
  // connect(this, &RobotiqPanel::statusReceived, this, &RobotiqPanel::updateStatusLabels);
}

void RobotiqPanel::onActivateClicked()
{
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "activate";
    command_pub_->publish(msg);
  }
}

void RobotiqPanel::onResetClicked()
{
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "reset";
    command_pub_->publish(msg);
  }
}

void RobotiqPanel::onOpenClicked()
{
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "open";
    command_pub_->publish(msg);
  }
}

void RobotiqPanel::onCloseClicked()
{
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "close";
    command_pub_->publish(msg);
  }
}

void RobotiqPanel::onGotoClicked()
{
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "goto";
    msg.position = static_cast<uint8_t>(position_slider_->value());
    msg.speed = static_cast<uint8_t>(speed_slider_->value());
    msg.force = static_cast<uint8_t>(force_slider_->value());
    command_pub_->publish(msg);
  }
}

void RobotiqPanel::onPositionChanged(int value)
{
  // Optional: real-time position feedback
  Q_UNUSED(value)
}

void RobotiqPanel::onSetupClicked()
{

  // open gripper
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "open";
    command_pub_->publish(msg);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "activate";
    command_pub_->publish(msg);
  }

  RCLCPP_INFO(node_->get_logger(), "opened gripper");
  
  // wait for a second
  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "goto";
    msg.position = static_cast<uint8_t>(200);
    msg.speed = static_cast<uint8_t>(10);
    msg.force = static_cast<uint8_t>(50);
    command_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Moving to position 200 with speed 10 and force 50");
  }

  RCLCPP_INFO(node_->get_logger(), "waiting for 5 seconds to reach position 200");
  // wait for a second
  std::this_thread::sleep_for(std::chrono::seconds(5));
  // log the sleep time
  // RCLCPP_INFO(node_->get_logger(), "Waited for 5 seconds");

  if (command_pub_) {
    auto msg = robotiq_msgs::msg::RobotiqGripperCommand();
    msg.command = "goto";
    msg.position = static_cast<uint8_t>(255);
    msg.speed = static_cast<uint8_t>(10);
    msg.force = static_cast<uint8_t>(255);
    command_pub_->publish(msg);
  }
  RCLCPP_INFO(node_->get_logger(), "Moving to position 255 with speed 10 and force 255");
}


} // namespace robotiq_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robotiq_rviz_plugin::RobotiqPanel, rviz_common::Panel)
