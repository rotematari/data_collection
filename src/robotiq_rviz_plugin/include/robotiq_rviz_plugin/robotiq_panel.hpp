#ifndef ROBOTIQ_PANEL_HPP
#define ROBOTIQ_PANEL_HPP

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <robotiq_msgs/msg/robotiq_gripper_command.hpp>
#include <robotiq_msgs/msg/robotiq_gripper_status.hpp>
#endif

#include <QWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpinBox>
#include <thread>

namespace rclcpp
{
namespace executors
{
class SingleThreadedExecutor;
}  // namespace executors
}  // namespace rclcpp

namespace robotiq_rviz_plugin
{

class RobotiqPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit RobotiqPanel(QWidget * parent = nullptr);
  ~RobotiqPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onActivateClicked();
  void onResetClicked();
  void onOpenClicked();
  void onCloseClicked();
  void onGotoClicked();
  void onPositionChanged(int value);
  void updateStatusLabels(const QString & status_text, int position, int current);

Q_SIGNALS:
  void statusReceived(const QString & status_text, int position, int current);

private:
  void setupUI();
  void statusCallback(const robotiq_msgs::msg::RobotiqGripperStatus::SharedPtr msg);
  
  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robotiq_msgs::msg::RobotiqGripperCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<robotiq_msgs::msg::RobotiqGripperStatus>::SharedPtr status_sub_;
  
  // UI Elements
  QPushButton * activate_btn_;
  QPushButton * reset_btn_;
  QPushButton * open_btn_;
  QPushButton * close_btn_;
  QPushButton * goto_btn_;
  
  QSlider * position_slider_;
  QSlider * speed_slider_;
  QSlider * force_slider_;
  
  // QLabel * status_label_;
  // QLabel * position_label_;
  // QLabel * current_label_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

} // namespace robotiq_rviz_plugin

#endif // ROBOTIQ_PANEL_HPP
