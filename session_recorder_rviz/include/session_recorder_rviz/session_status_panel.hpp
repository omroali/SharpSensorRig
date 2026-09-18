#ifndef SESSION_RECORDER_RVIZ__SESSION_STATUS_PANEL_HPP_
#define SESSION_RECORDER_RVIZ__SESSION_STATUS_PANEL_HPP_

#include <QLabel>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace session_recorder_rviz
{

/// Dockable RViz panel showing the session player's play/pause state and the
/// current activity, plus buttons that jump to the previous / next activity.
///
/// The status text comes from /session/status_text (std_msgs/String). The
/// buttons call the player's /session/prev_activity and /session/next_activity
/// services (std_srvs/Trigger) — the same actions as 'p' and 'n' in the player
/// terminal.
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
  void onPrevActivity();
  void onNextActivity();
  void callTrigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const QString & direction);

  QLabel * label_;
  QPushButton * prev_button_;
  QPushButton * next_button_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr prev_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr next_client_;
  QString topic_;
  QString prev_service_;
  QString next_service_;
};

}  // namespace session_recorder_rviz

#endif  // SESSION_RECORDER_RVIZ__SESSION_STATUS_PANEL_HPP_
