#include "session_recorder_rviz/session_status_panel.hpp"

#include <QFont>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace session_recorder_rviz
{

SessionStatusPanel::SessionStatusPanel(QWidget * parent)
: rviz_common::Panel(parent),
  label_(nullptr),
  topic_("/session/status_text")
{
  label_ = new QLabel("session status: waiting for /session/status_text ...");
  QFont font = label_->font();
  font.setFamily("Monospace");
  font.setPointSizeF(font.pointSizeF() + 2);
  label_->setFont(font);
  label_->setWordWrap(true);
  label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  auto * layout = new QVBoxLayout;
  layout->addWidget(label_);
  setLayout(layout);
}

void SessionStatusPanel::onInitialize()
{
  auto context = getDisplayContext();
  if (!context) {
    return;
  }
  auto abstraction = context->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  auto node = abstraction->get_raw_node();
  subscription_ = node->create_subscription<std_msgs::msg::String>(
    topic_.toStdString(), rclcpp::QoS(10),
    [this](std_msgs::msg::String::SharedPtr msg) {onStatus(msg);});
}

void SessionStatusPanel::onStatus(const std_msgs::msg::String::SharedPtr msg)
{
  const QString text = QString::fromStdString(msg->data);
  // The subscription callback runs on the ROS executor thread, so marshal the
  // update onto the Qt GUI thread.
  QMetaObject::invokeMethod(
    label_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void SessionStatusPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic) && !topic.isEmpty()) {
    topic_ = topic;
  }
}

void SessionStatusPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_);
}

}  // namespace session_recorder_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(session_recorder_rviz::SessionStatusPanel, rviz_common::Panel)
