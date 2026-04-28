#include "rviz_editor_plugins/csv_marker_display.hpp"

#include <chrono>
#include <string>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "editor_tool_srvs/srv/load_csv.hpp"
#include "editor_tool_srvs/srv/save_csv.hpp"
#include "editor_tool_srvs/srv/select_range.hpp"

namespace {
constexpr auto kServiceWaitTimeout = std::chrono::seconds(1);
} // namespace

namespace rviz_editor_plugins {
CsvMarkerDisplay::CsvMarkerDisplay(QWidget *parent)
    : rviz_common::Panel(parent) {
  node_ = rclcpp::Node::make_shared("csv_marker_display");

  label_ = new QLabel("[No CSV loaded]");
  load_button_ = new QPushButton("Load CSV");
  save_button_ = new QPushButton("Save CSV");
  load_client_ =
      node_->create_client<editor_tool_srvs::srv::LoadCsv>("load_csv");
  save_client_ =
      node_->create_client<editor_tool_srvs::srv::SaveCsv>("save_csv");

  QVBoxLayout *layout = new QVBoxLayout;
  QGridLayout *grid_layout = new QGridLayout;
  layout->addWidget(label_);
  grid_layout->addWidget(load_button_, 0, 0);
  grid_layout->addWidget(save_button_, 0, 1);
  layout->addLayout(grid_layout);
  setLayout(layout);

  connect(load_button_, &QPushButton::clicked, this,
          &CsvMarkerDisplay::loadCsv);
  connect(save_button_, &QPushButton::clicked, this,
          &CsvMarkerDisplay::saveCsv);
}
void CsvMarkerDisplay::onInitialize() {
  // Initialization logic if needed
  RCLCPP_INFO(node_->get_logger(), "CsvMarkerDisplay initialized");
}
void CsvMarkerDisplay::load(const rviz_common::Config &config) {
  Panel::load(config);
}
void CsvMarkerDisplay::save(rviz_common::Config config) const {
  Panel::save(config);
}
void CsvMarkerDisplay::tick() {
  // Periodic update logic if needed
}
void CsvMarkerDisplay::loadCsv() {
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open CSV File"), "",
                                                  tr("CSV Files (*.csv)"));
  if (fileName.isEmpty()) {
    return;
  }

  const std::string file_path = fileName.toStdString();
  RCLCPP_INFO(node_->get_logger(), "Loading CSV file: %s", file_path.c_str());

  if (!load_client_->wait_for_service(kServiceWaitTimeout)) {
    RCLCPP_ERROR(node_->get_logger(), "load_csv service not available");
    label_->setText(QString("Load service unavailable"));
    return;
  }

  auto request = std::make_shared<editor_tool_srvs::srv::LoadCsv::Request>();
  request->filename = file_path;
  auto future = load_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send request to load CSV");
    label_->setText(QString("Failed to load CSV"));
    return;
  }

  label_->setText(QString("Loaded: %1").arg(fileName));
}

void CsvMarkerDisplay::saveCsv() {
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save CSV File"), "",
                                                  tr("CSV Files (*.csv)"));
  if (fileName.isEmpty()) {
    return;
  }

  const std::string file_path = fileName.toStdString();
  RCLCPP_INFO(node_->get_logger(), "Saving CSV file: %s", file_path.c_str());

  if (!save_client_->wait_for_service(kServiceWaitTimeout)) {
    RCLCPP_ERROR(node_->get_logger(), "save_csv service not available");
    label_->setText(QString("Save service unavailable"));
    return;
  }

  auto request = std::make_shared<editor_tool_srvs::srv::SaveCsv::Request>();
  request->filename = file_path;
  auto future = save_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send request to save CSV");
    label_->setText(QString("Failed to save CSV"));
    return;
  }

  label_->setText(QString("Saved: %1").arg(fileName));
  RCLCPP_INFO(node_->get_logger(), "CSV saved successfully to %s",
              file_path.c_str());
}
} // namespace rviz_editor_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_editor_plugins::CsvMarkerDisplay,
                       rviz_common::Panel)
