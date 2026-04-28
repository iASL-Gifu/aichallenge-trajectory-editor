#include "rviz_editor_plugins/editor_tool.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include <rcpputils/filesystem_helper.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include "editor_tool_srvs/srv/save_csv.hpp"
#include "editor_tool_srvs/srv/select_range.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {
constexpr char kSimpleTrajectoryGeneratorNode[] =
    "/planning/scenario_planning/simple_trajectory_generator";
constexpr char kPostDirectory[] = "/aichallenge/workspace/post";
constexpr auto kServiceWaitTimeout = std::chrono::seconds(1);

std::string makePostCsvPath() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto timestamp =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return std::string(kPostDirectory) + "/post_" + std::to_string(timestamp) +
         ".csv";
}
} // namespace

namespace rviz_editor_plugins {
EditorTool::EditorTool(QWidget *parent) : rviz_common::Panel(parent) {
  node_ = rclcpp::Node::make_shared("editor_tool");

  mode_label_ = new QLabel("Mode: Normal");
  velocity_edit_ = new QLineEdit("0.0");
  select_range_button_ = new QPushButton("Select Range");
  start_parallel_button_ = new QPushButton("Start Parallel Move");
  confirm_parallel_button_ = new QPushButton("End Parallel Move");
  undo_button_ = new QPushButton("Undo");
  redo_button_ = new QPushButton("Redo");
  post_button_ = new QPushButton("Post Trajectory");

  select_range_client_ =
      node_->create_client<editor_tool_srvs::srv::SelectRange>("select_range");
  start_parallel_client_ =
      node_->create_client<std_srvs::srv::Trigger>("start_parallel_move");
  confirm_parallel_client_ =
      node_->create_client<std_srvs::srv::Trigger>("confirm_parallel_move");
  undo_client_ = node_->create_client<std_srvs::srv::Trigger>("undo");
  redo_client_ = node_->create_client<std_srvs::srv::Trigger>("redo");
  save_csv_client_ =
      node_->create_client<editor_tool_srvs::srv::SaveCsv>("save_csv");
  param_client_ = std::make_shared<rclcpp::SyncParametersClient>(
      node_, kSimpleTrajectoryGeneratorNode);

  QVBoxLayout *layout = new QVBoxLayout;
  QGridLayout *grid_layout_parallel = new QGridLayout;
  QGridLayout *grid_layout_unredo = new QGridLayout;
  layout->addWidget(mode_label_);
  layout->addWidget(velocity_edit_);
  layout->addWidget(select_range_button_);
  grid_layout_parallel->addWidget(start_parallel_button_, 0, 0);
  grid_layout_parallel->addWidget(confirm_parallel_button_, 0, 1);
  grid_layout_unredo->addWidget(undo_button_, 0, 0);
  grid_layout_unredo->addWidget(redo_button_, 0, 1);
  layout->addLayout(grid_layout_parallel);
  layout->addLayout(grid_layout_unredo);
  layout->addWidget(post_button_);

  setLayout(layout);

  connect(select_range_button_, &QPushButton::clicked, this,
          &EditorTool::startSelection);
  connect(start_parallel_button_, &QPushButton::clicked, this,
          &EditorTool::startParallelMove);
  connect(confirm_parallel_button_, &QPushButton::clicked, this,
          &EditorTool::confirmParallelMove);
  connect(undo_button_, &QPushButton::clicked, this, &EditorTool::undo);
  connect(redo_button_, &QPushButton::clicked, this, &EditorTool::redo);
  connect(post_button_, &QPushButton::clicked, this,
          &EditorTool::postTrajectory);

  use_simple_trajectory_generator_ = simpleTrajectoryGeneratorAvailable();
}

void EditorTool::onInitialize() {
  // Initialization logic if needed
  RCLCPP_INFO(node_->get_logger(), "EditorTool initialized");
}

void EditorTool::load(const rviz_common::Config &config) {
  Panel::load(config);
}

void EditorTool::save(rviz_common::Config config) const { Panel::save(config); }

void EditorTool::tick() {
  // Periodic update logic if needed
}
void EditorTool::startSelection() {
  const std::string velocity_str = velocity_edit_->text().toStdString();
  if (velocity_str.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Velocity input is empty");
    return;
  }

  double velocity = 0.0;
  try {
    size_t parsed = 0;
    velocity = std::stod(velocity_str, &parsed);
    if (velocity_str.find_first_not_of(" \t\r\n", parsed) !=
        std::string::npos) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid velocity input: %s",
                   velocity_str.c_str());
      return;
    }
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid velocity input: %s", e.what());
    return;
  } catch (const std::out_of_range &e) {
    RCLCPP_ERROR(node_->get_logger(), "Velocity out of range: %s", e.what());
    return;
  }

  auto request =
      std::make_shared<editor_tool_srvs::srv::SelectRange::Request>();
  request->velocity = velocity;
  mode_label_->setText(
      QString("Mode: Selection (Velocity: %1)").arg(velocity_str.c_str()));

  if (select_range_client_->wait_for_service(kServiceWaitTimeout)) {
    auto future = select_range_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      const auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(node_->get_logger(),
                    "Selection started with velocity: %.2f", request->velocity);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Server rejected selection start");
        mode_label_->setText(QString("Mode: Normal"));
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to start selection");
      mode_label_->setText(QString("Mode: Normal"));
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "select_range service not available");
    mode_label_->setText(QString("Mode: Normal"));
  }
}

void EditorTool::startParallelMove() {
  if (callTriggerService(start_parallel_client_, "start_parallel_move")) {
    mode_label_->setText(QString("Mode: Parallel Move"));
  }
}

void EditorTool::confirmParallelMove() {
  if (callTriggerService(confirm_parallel_client_, "confirm_parallel_move")) {
    mode_label_->setText(QString("Mode: Normal"));
  }
}

void EditorTool::undo() { callTriggerService(undo_client_, "undo"); }

void EditorTool::redo() { callTriggerService(redo_client_, "redo"); }

void EditorTool::postTrajectory() {
  use_simple_trajectory_generator_ = simpleTrajectoryGeneratorAvailable();
  if (use_simple_trajectory_generator_) {
    const rcpputils::fs::path post_directory(kPostDirectory);
    if (!rcpputils::fs::exists(post_directory)) {
      RCLCPP_INFO(node_->get_logger(), "Creating directory: %s",
                  kPostDirectory);
      rcpputils::fs::create_directories(post_directory);
    }

    const std::string post_csv_path = makePostCsvPath();
    if (!saveCsvToPath(post_csv_path)) {
      return;
    }

    const auto results = param_client_->set_parameters(
        {rclcpp::Parameter("csv_path", post_csv_path)});
    if (results.empty() || !results.front().successful) {
      const auto reason =
          results.empty() ? "empty result" : results.front().reason;
      RCLCPP_ERROR(node_->get_logger(), "Failed to update csv_path: %s",
                   reason.c_str());
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Updated simple trajectory generator csv_path: %s",
                post_csv_path.c_str());
    return;
  }

  RCLCPP_ERROR(node_->get_logger(),
               "%s node is not available; cannot update trajectory CSV path",
               kSimpleTrajectoryGeneratorNode);
}

bool EditorTool::callTriggerService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
    const char *service_name) {
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  if (!client->wait_for_service(kServiceWaitTimeout)) {
    RCLCPP_ERROR(node_->get_logger(), "%s service not available", service_name);
    return false;
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call %s", service_name);
    return false;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "%s rejected request: %s", service_name,
                 response->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "%s succeeded: %s", service_name,
              response->message.c_str());
  return true;
}

bool EditorTool::saveCsvToPath(const std::string &file_path) {
  if (!save_csv_client_->wait_for_service(kServiceWaitTimeout)) {
    RCLCPP_ERROR(node_->get_logger(), "save_csv service not available");
    return false;
  }

  auto save_request =
      std::make_shared<editor_tool_srvs::srv::SaveCsv::Request>();
  save_request->filename = file_path;
  auto save_future = save_csv_client_->async_send_request(save_request);

  if (rclcpp::spin_until_future_complete(node_, save_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save CSV");
    return false;
  }

  const auto response = save_future.get();
  if (!response->success) {
    RCLCPP_ERROR(node_->get_logger(), "save_csv rejected request: %s",
                 response->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "CSV saved to %s", file_path.c_str());
  return true;
}

bool EditorTool::simpleTrajectoryGeneratorAvailable() const {
  for (const auto &node_name : node_->get_node_names()) {
    if (node_name == kSimpleTrajectoryGeneratorNode ||
        node_name == "simple_trajectory_generator") {
      return true;
    }
  }
  return false;
}
} // namespace rviz_editor_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_editor_plugins::EditorTool, rviz_common::Panel)
