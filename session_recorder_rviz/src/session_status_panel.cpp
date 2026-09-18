#include "session_recorder_rviz/session_status_panel.hpp"

#include <QFont>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace session_recorder_rviz
{

SessionStatusPanel::SessionStatusPanel(QWidget * parent)
: rviz_common::Panel(parent),
  label_(nullptr),
  prev_button_(nullptr),
  next_button_(nullptr),
  topic_("/session/status_text"),
  prev_service_("/session/prev_activity"),
  next_service_("/session/next_activity")
{
  label_ = new QLabel("session status: waiting for /session/status_text ...");
  QFont font = label_->font();
  font.setFamily("Monospace");
  font.setPointSizeF(font.pointSizeF() + 2);
  label_->setFont(font);
  label_->setWordWrap(true);
  label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  prev_button_ = new QPushButton(QString::fromUtf8("\u25c0  Prev activity"));
  next_button_ = new QPushButton(QString::fromUtf8("Next activity  \u25b6"));
  prev_button_->setToolTip("Jump to the previous activity (same as 'p')");
  next_button_->setToolTip("Jump to the next activity (same as 'n')");
  for (auto * button : {prev_button_, next_button_}) {
    button->setMinimumHeight(30);
  }

  auto * buttons = new QHBoxLayout;
  buttons->addWidget(prev_button_);
  buttons->addWidget(next_button_);

  auto * layout = new QVBoxLayout;
  layout->addWidget(label_);
  layout->addLayout(buttons);
  setLayout(layout);

  // New-style connects: the handlers run on the Qt GUI thread, which is safe —
  // rclcpp clients may send requests from any thread.
  connect(prev_button_, &QPushButton::clicked, this, &SessionStatusPanel::onPrevActivity);
  connect(next_button_, &QPushButton::clicked, this, &SessionStatusPanel::onNextActivity);
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
  prev_client_ = node->create_client<std_srvs::srv::Trigger>(prev_service_.toStdString());
  next_client_ = node->create_client<std_srvs::srv::Trigger>(next_service_.toStdString());
}

void SessionStatusPanel::onPrevActivity()
{
  callTrigger(prev_client_, "previous");
}

void SessionStatusPanel::onNextActivity()
{
  callTrigger(next_client_, "next");
}

void SessionStatusPanel::callTrigger(
  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const QString & direction)
{
  if (!client) {
    return;
  }
  // The player may not be up yet (or may have exited); say so rather than
  // failing silently. The next status message will overwrite this line.
  if (!client->service_is_ready()) {
    label_->setText(
      QString("session player not running — cannot jump to the %1 activity")
      .arg(direction));
    return;
  }
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
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
  QString service;
  if (config.mapGetString("PrevActivityService", &service) && !service.isEmpty()) {
    prev_service_ = service;
  }
  if (config.mapGetString("NextActivityService", &service) && !service.isEmpty()) {
    next_service_ = service;
  }
}

void SessionStatusPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", topic_);
  config.mapSetValue("PrevActivityService", prev_service_);
  config.mapSetValue("NextActivityService", next_service_);
}

}  // namespace session_recorder_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(session_recorder_rviz::SessionStatusPanel, rviz_common::Panel)
