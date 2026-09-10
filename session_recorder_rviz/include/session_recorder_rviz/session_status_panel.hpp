#ifndef SESSION_RECORDER_RVIZ__SESSION_STATUS_PANEL_HPP_
#define SESSION_RECORDER_RVIZ__SESSION_STATUS_PANEL_HPP_

#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace session_recorder_rviz
{

/// Dockable RViz panel showing the session player's play/pause state and the
/// current activity. Subscribes to a std_msgs/String on /session/status_text.
class SessionStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SessionStatusPanel(QWidget * parent = nullptr);

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  void onStatus(const std_msgs::msg::String::SharedPtr msg);

  QLabel * label_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  QString topic_;
};

}  // namespace session_recorder_rviz

#endif  // SESSION_RECORDER_RVIZ__SESSION_STATUS_PANEL_HPP_
