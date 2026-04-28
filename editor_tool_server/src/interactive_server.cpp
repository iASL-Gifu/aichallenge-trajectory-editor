// File: interactive_server.cpp
#include "editor_tool_server/interactive_server.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace {
constexpr char kFrameId[] = "map";
constexpr char kTrajectoryMarkerNs[] = "arrow";
constexpr char kLineMarkerNs[] = "line";
constexpr char kSpeedLabelNs[] = "speed_label";
constexpr char kHitBoxNs[] = "hit_box";
constexpr char kHelperMarkerNs[] = "helper";
constexpr double kMpsToKmph = 3.6;
constexpr double kKmphToMps = 1.0 / kMpsToKmph;
constexpr double kGridResolution = 0.01;
constexpr size_t kCsvColumnCount = 8;
constexpr size_t kMaxUndoHistory = 100;
constexpr std::array<float, 4> kSelectionColor = {0.91f, 0.984f, 0.992f, 1.0f};
constexpr std::array<float, 4> kPickColor = {0.15f, 0.55f, 1.0f, 0.22f};

bool parseDoubleStrict(const std::string &value, double &output) {
  try {
    size_t parsed = 0;
    output = std::stod(value, &parsed);
    return value.find_first_not_of(" \t\r\n", parsed) == std::string::npos;
  } catch (const std::exception &) {
    return false;
  }
}

void setMarkerColor(visualization_msgs::msg::Marker &marker, float red,
                    float green, float blue, float alpha) {
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = alpha;
}

void setSelectionColor(visualization_msgs::msg::Marker &marker) {
  setMarkerColor(marker, kSelectionColor[0], kSelectionColor[1],
                 kSelectionColor[2], kSelectionColor[3]);
}

visualization_msgs::msg::Marker makePickSphere(const char *ns, int id,
                                               double scale) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = kFrameId;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.pose.orientation.w = 1.0;
  setMarkerColor(marker, kPickColor[0], kPickColor[1], kPickColor[2],
                 kPickColor[3]);
  return marker;
}

} // namespace

namespace editor_tool_server {
EditorToolServer::EditorToolServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("editor_tool_server", options) {
  // --- InteractiveMarkerServer の初期化 ---
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "editor_tool_server", this->get_node_base_interface(),
      this->get_node_clock_interface(), this->get_node_logging_interface(),
      this->get_node_topics_interface(), this->get_node_services_interface());

  // --- CSV 読み込みサービスの登録 ---
  load_csv_service_ = this->create_service<editor_tool_srvs::srv::LoadCsv>(
      "load_csv", std::bind(&EditorToolServer::LoadCsv, this,
                            std::placeholders::_1, std::placeholders::_2));

  save_csv_service_ = this->create_service<editor_tool_srvs::srv::SaveCsv>(
      "save_csv", std::bind(&EditorToolServer::SaveCsvSrv, this,
                            std::placeholders::_1, std::placeholders::_2));

  // --- 選択モード開始サービスの登録 (SelectRange.srv) ---
  select_range_service_ =
      this->create_service<editor_tool_srvs::srv::SelectRange>(
          "select_range",
          std::bind(&EditorToolServer::StartSelection, this,
                    std::placeholders::_1, std::placeholders::_2));

  start_parallel_move_service_ = this->create_service<std_srvs::srv::Trigger>(
      "start_parallel_move",
      std::bind(&EditorToolServer::StartParallelMove, this,
                std::placeholders::_1, std::placeholders::_2));

  confirm_parallel_move_service_ = this->create_service<std_srvs::srv::Trigger>(
      "confirm_parallel_move",
      std::bind(&EditorToolServer::ConfirmParallelMove, this,
                std::placeholders::_1, std::placeholders::_2));

  undo_service_ = this->create_service<std_srvs::srv::Trigger>(
      "undo", std::bind(&EditorToolServer::undo, this, std::placeholders::_1,
                        std::placeholders::_2));

  redo_service_ = this->create_service<std_srvs::srv::Trigger>(
      "redo", std::bind(&EditorToolServer::redo, this, std::placeholders::_1,
                        std::placeholders::_2));

  // --- MarkerArray パブリッシャの初期化 ---
  const auto marker_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/race_trajectory", marker_qos);
  raw_feedback_sub_ =
      this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
          "/editor_tool_server/feedback", rclcpp::QoS(10),
          [this](const visualization_msgs::msg::InteractiveMarkerFeedback::
                     SharedPtr feedback) {
            RCLCPP_DEBUG(
                get_logger(),
                "Raw InteractiveMarker feedback topic: marker=%s control=%s "
                "event=%d pose=%.3f, %.3f, %.3f",
                feedback->marker_name.c_str(), feedback->control_name.c_str(),
                feedback->event_type, feedback->pose.position.x,
                feedback->pose.position.y, feedback->pose.position.z);
            processFeedback(feedback);
          });

  // 初期状態は選択モードオフ、インデックス -1
  selection_mode_ = false;
  sel_idx1_ = sel_idx2_ = -1;
  selection_velocity_ = 0.0;

  // パラメータ宣言
  this->declare_parameter<std::string>("csv_file_path", "default.csv");
  this->declare_parameter<bool>("publish_on_initialize", true);
  this->declare_parameter<double>("wait_seconds", 5.0);
  this->declare_parameter<double>("grad_min_speed", 0.0);
  this->declare_parameter<double>("grad_mid_speed", 30.0);
  this->declare_parameter<double>("grad_max_speed", 60.0);

  this->get_parameter("csv_file_path", csv_file_path_);
  this->get_parameter("publish_on_initialize", publish_on_initialize_);
  this->get_parameter("wait_seconds", wait_seconds_);
  this->get_parameter("grad_min_speed", min_speed_);
  this->get_parameter("grad_mid_speed", mid_speed_);
  this->get_parameter("grad_max_speed", max_speed_);

  // dynamic parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &EditorToolServer::onParameterChange, this, std::placeholders::_1));

  if (publish_on_initialize_) {

    RCLCPP_INFO(
        this->get_logger(),
        "Waiting for %f seconds before loading CSV and publishing markers.",
        wait_seconds_);
    rclcpp::sleep_for(
        std::chrono::milliseconds(static_cast<int>(wait_seconds_ * 1000)));
    LoadCsvFile(csv_file_path_);
  }
}

rcl_interfaces::msg::SetParametersResult EditorToolServer::onParameterChange(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Parameters updated successfully.";

  bool color_parameters_changed = false;

  for (const auto &param : parameters) {
    if (param.get_name() == "csv_file_path" &&
        param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      const auto new_csv_file_path = param.as_string();
      if (!LoadCsvFile(new_csv_file_path)) {
        result.successful = false;
        result.reason = "Failed to load csv_file_path.";
        return result;
      }
      csv_file_path_ = new_csv_file_path;
      RCLCPP_INFO(this->get_logger(), "Updated csv_file_path: %s",
                  csv_file_path_.c_str());
    } else if (param.get_name() == "publish_on_initialize" &&
               param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      publish_on_initialize_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "Updated publish_on_initialize: %s",
                  publish_on_initialize_ ? "true" : "false");
    } else if (param.get_name() == "wait_seconds" &&
               param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      wait_seconds_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "Updated wait_seconds: %f",
                  wait_seconds_);
    } else if (param.get_name() == "grad_min_speed" &&
               param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      min_speed_ = param.as_double();
      color_parameters_changed = true;
      RCLCPP_INFO(this->get_logger(), "Updated grad_min_speed: %f", min_speed_);
    } else if (param.get_name() == "grad_mid_speed" &&
               param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      mid_speed_ = param.as_double();
      color_parameters_changed = true;
      RCLCPP_INFO(this->get_logger(), "Updated grad_mid_speed: %f", mid_speed_);
    } else if (param.get_name() == "grad_max_speed" &&
               param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      max_speed_ = param.as_double();
      color_parameters_changed = true;
      RCLCPP_INFO(this->get_logger(), "Updated grad_max_speed: %f", max_speed_);
    }
  }

  if (color_parameters_changed && !trajectory_markers_.empty()) {
    refreshTrajectoryColor();
  }

  return result;
}

void EditorToolServer::LoadCsv(
    const std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Request> request,
    std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Response> /*response*/) {
  const std::string &file_name = request->filename;
  if (file_name.empty()) {
    RCLCPP_ERROR(get_logger(), "LoadCsv: filename is empty");
    return;
  }

  if (LoadCsvFile(file_name)) {
    csv_file_path_ = file_name;
  }
}

bool EditorToolServer::LoadCsvFile(const std::string &file_name) {
  RCLCPP_INFO(get_logger(), "Loading CSV file: %s", file_name.c_str());
  std::ifstream ifs(file_name);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_name.c_str());
    return false;
  }

  std::string line;
  // ヘッダ行をスキップ
  if (!std::getline(ifs, line)) {
    RCLCPP_ERROR(get_logger(), "CSV is empty");
    return false;
  }

  std::vector<visualization_msgs::msg::Marker> loaded_markers;
  int id = 0;
  bool any_error = false;
  size_t line_number = 1;

  // CSV の各行を parseLineToMarker で Marker に変換し、trajectory_markers_
  // に追加
  while (std::getline(ifs, line)) {
    ++line_number;
    visualization_msgs::msg::Marker marker;
    if (parseLineToMarker(line, id, marker)) {
      loaded_markers.push_back(marker);
      id++;
    } else {
      any_error = true;
      RCLCPP_WARN(get_logger(), "Failed to parse CSV line %zu", line_number);
    }
  }

  if (loaded_markers.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid data found in CSV");
    return false;
  }

  trajectory_markers_ = std::move(loaded_markers);
  name_to_index_.clear();
  server_->clear(); // 既存のインタラクティブマーカーをすべて削除

  for (const auto &marker : trajectory_markers_) {
    makeMoveTrajectoryMarker(marker);
  }

  // 生成したインタラクティブマーカーを RViz に反映
  server_->applyChanges();

  // 初期状態の MarkerArray（ポイント＋ライン＋速度ラベル）を publish
  publishMarkers();
  undo_stack_.clear();
  redo_stack_.clear();
  saveStateForUndo(); // 初期状態を undo 履歴に保存

  if (any_error) {
    RCLCPP_WARN(get_logger(), "Some lines failed to parse (check log)");
  } else {
    RCLCPP_INFO(get_logger(), "CSV loaded and markers displayed successfully");
  }
  return true;
}

void EditorToolServer::SaveCsvSrv(
    const std::shared_ptr<editor_tool_srvs::srv::SaveCsv::Request> request,
    std::shared_ptr<editor_tool_srvs::srv::SaveCsv::Response> response) {
  const std::string &file_name = request->filename;
  if (file_name.empty()) {
    RCLCPP_ERROR(get_logger(), "SaveCsv: filename is empty");
    response->success = false;
    response->message = "filename is empty";
    return;
  }

  const std::filesystem::path file_path(file_name);
  const auto parent_path = file_path.parent_path();
  if (!parent_path.empty()) {
    std::error_code error_code;
    std::filesystem::create_directories(parent_path, error_code);
    if (error_code) {
      RCLCPP_ERROR(get_logger(), "Failed to create directory %s: %s",
                   parent_path.string().c_str(), error_code.message().c_str());
      response->success = false;
      response->message = "failed to create output directory: " +
                          error_code.message();
      return;
    }
  }

  if (saveCsv(file_name)) {
    RCLCPP_INFO(get_logger(), "CSV saved successfully to %s",
                file_name.c_str());
    response->success = true;
    response->message = "CSV saved";
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to save CSV to %s", file_name.c_str());
    response->success = false;
    response->message = "failed to save CSV";
  }
}

void EditorToolServer::StartSelection(
    const std::shared_ptr<editor_tool_srvs::srv::SelectRange::Request> request,
    std::shared_ptr<editor_tool_srvs::srv::SelectRange::Response> response) {
  // 既に選択モード中なら弾く
  if (selection_mode_ || parallel_mode_) {
    RCLCPP_WARN(get_logger(),
                "SelectRange: already in selection or parallel mode");
    response->success = false;
    return;
  }
  if (trajectory_markers_.empty()) {
    RCLCPP_WARN(get_logger(), "SelectRange: no trajectory is loaded");
    response->success = false;
    return;
  }

  // 速度を格納し、選択モードを ON にする
  selection_velocity_ = request->velocity;
  sel_idx1_ = sel_idx2_ = -1;
  selection_mode_ = true;

  RCLCPP_INFO(get_logger(),
              "SelectRange: selection mode started. click two markers to "
              "select range. velocity=%.2f",
              selection_velocity_);
  response->success = true;
}

void EditorToolServer::processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  const std::string &mname = feedback->marker_name;
  RCLCPP_DEBUG(get_logger(),
               "InteractiveMarker feedback: marker=%s control=%s event=%d",
               mname.c_str(), feedback->control_name.c_str(),
               feedback->event_type);
  // ── 並行移動モード中の処理 ──
  if (parallel_mode_) {
    if (mname == move_marker_name_) {
      if (feedback->event_type !=
          visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
        return;
      }

      geometry_msgs::msg::Point current = feedback->pose.position;
      double dx = current.x - move_marker_prev_pos_.x;
      double dy = current.y - move_marker_prev_pos_.y;
      double dz = current.z - move_marker_prev_pos_.z;

      std::vector<int> indices = getRangeIndices(p_sel_idx1_, p_sel_idx2_);
      for (int idx : indices) {
        trajectory_markers_[idx].pose.position.x += dx;
        trajectory_markers_[idx].pose.position.y += dy;
        trajectory_markers_[idx].pose.position.z += dz;
        trajectory_markers_[idx].header.stamp = this->now();
      }

      move_marker_prev_pos_ = current;

      for (int idx : indices) {
        const std::string name = trajectory_markers_[idx].ns + "_" +
                                 std::to_string(trajectory_markers_[idx].id);
        server_->setPose(name, trajectory_markers_[idx].pose);
      }

      server_->applyChanges();
      publishMarkers();
      RCLCPP_DEBUG(get_logger(),
                   "Published race_trajectory after moving selected range by "
                   "%.3f, %.3f, %.3f",
                   dx, dy, dz);
      return;
    }

    // １） まだ範囲未選択：2点選択待ち（BUTTON_CLICK
    // で2つのインデックスを取得）
    if (p_sel_idx1_ < 0) {
      auto it = name_to_index_.find(mname);
      if (it != name_to_index_.end()) {
        p_sel_idx1_ = it->second;
        RCLCPP_DEBUG(get_logger(), "ParallelMove: first index selected: %d",
                     p_sel_idx1_);
        highlightMarker(p_sel_idx1_);
        redrawMarkers();
      }
      return;
    }
    if (p_sel_idx2_ < 0) {
      auto it = name_to_index_.find(mname);
      if (it != name_to_index_.end() && it->second != p_sel_idx1_) {
        p_sel_idx2_ = it->second;
        RCLCPP_DEBUG(get_logger(), "ParallelMove: second index selected: %d",
                     p_sel_idx2_);

        highlightRange(getRangeIndices(p_sel_idx1_, p_sel_idx2_));
        redrawMarkers();
        createMoveHelperMarker();
      }
      return;
    }
    return;
  }

  // ── 速度変更用の既存処理（選択モード中） ──
  if (selection_mode_) {
    auto it = name_to_index_.find(feedback->marker_name);
    if (it != name_to_index_.end()) {
      const int clicked_idx = it->second;

      if (sel_idx1_ < 0) {
        sel_idx1_ = clicked_idx;
        RCLCPP_DEBUG(get_logger(), "Selected first index: %d", sel_idx1_);
        highlightMarker(sel_idx1_);
        redrawMarkers();
        return;
      } else if (sel_idx2_ < 0 && clicked_idx != sel_idx1_) {
        sel_idx2_ = clicked_idx;
        RCLCPP_DEBUG(get_logger(), "Selected second index: %d", sel_idx2_);

        const bool updated =
            applyVelocityToRange(sel_idx1_, sel_idx2_, selection_velocity_);
        // 結果を publish して、選択モードを終了
        redrawMarkers();
        if (updated) {
          saveStateForUndo();
        }
        selection_mode_ = false;
        sel_idx1_ = sel_idx2_ = -1;
        RCLCPP_INFO(get_logger(),
                    "Selection completed; exiting selection mode.");
      }
    }
    return;
  }

  if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    alignMarker(feedback);
    return;
  }

  if (feedback->event_type ==
      visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    saveStateForUndo();
  }
}
void EditorToolServer::AutoColorizeTraj(visualization_msgs::msg::Marker &marker,
                                        double velocity) {
  const double min_speed = std::min(min_speed_, max_speed_);
  const double max_speed = std::max(min_speed_, max_speed_);
  const double mid_speed = std::clamp(mid_speed_, min_speed, max_speed);

  if (max_speed <= min_speed) {
    setMarkerColor(marker, 0.0f, 1.0f, 0.0f, 1.0f);
    return;
  }

  const double clamped_speed = std::clamp(velocity, min_speed, max_speed);

  if (clamped_speed <= mid_speed) {
    const double denominator = mid_speed - min_speed;
    const auto ratio = static_cast<float>(
        denominator > 0.0 ? (clamped_speed - min_speed) / denominator : 1.0);
    setMarkerColor(marker, ratio, 1.0f, 0.0f, 1.0f);
  } else {
    const double denominator = max_speed - mid_speed;
    const auto ratio = static_cast<float>(
        denominator > 0.0 ? (clamped_speed - mid_speed) / denominator : 1.0);
    setMarkerColor(marker, 1.0f, 1.0f - ratio, 0.0f, 1.0f);
  }
}

void EditorToolServer::highlightMarker(int index) {
  if (index < 0 || index >= static_cast<int>(trajectory_markers_.size())) {
    return;
  }

  setSelectionColor(trajectory_markers_[index]);
}

void EditorToolServer::highlightRange(const std::vector<int> &indices) {
  for (const int index : indices) {
    highlightMarker(index);
  }
}

bool EditorToolServer::applyVelocityToRange(int idx1, int idx2,
                                            double velocity) {
  const auto indices = getRangeIndices(idx1, idx2);
  if (indices.empty()) {
    RCLCPP_WARN(get_logger(),
                "No trajectory markers selected for velocity update");
    return false;
  }

  std::ostringstream velocity_text;
  velocity_text << std::fixed << std::setprecision(1) << velocity;

  for (const int index : indices) {
    AutoColorizeTraj(trajectory_markers_[index], velocity);
    trajectory_markers_[index].text = velocity_text.str();
  }

  return true;
}

double EditorToolServer::markerSpeedKmh(
    const visualization_msgs::msg::Marker &marker) const {
  double speed = 0.0;
  if (parseDoubleStrict(marker.text, speed)) {
    return speed;
  }

  RCLCPP_WARN(get_logger(),
              "Invalid speed text for marker %s_%d: '%s'. Using 0.0 km/h.",
              marker.ns.c_str(), marker.id, marker.text.c_str());
  return 0.0;
}

void EditorToolServer::alignMarker(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {
  RCLCPP_DEBUG(
      get_logger(),
      "POSE_UPDATE feedback: marker=%s control=%s pose=%.3f, %.3f, %.3f",
      feedback->marker_name.c_str(), feedback->control_name.c_str(),
      feedback->pose.position.x, feedback->pose.position.y,
      feedback->pose.position.z);

  auto it = name_to_index_.find(feedback->marker_name);
  if (it == name_to_index_.end()) {
    RCLCPP_WARN(get_logger(), "POSE_UPDATE ignored: unknown marker %s",
                feedback->marker_name.c_str());
    return;
  }

  const int idx = it->second;
  if (idx < 0 || idx >= static_cast<int>(trajectory_markers_.size())) {
    RCLCPP_WARN(get_logger(), "POSE_UPDATE ignored: marker index %d is invalid",
                idx);
    return;
  }

  geometry_msgs::msg::Pose new_pose = feedback->pose;
  new_pose.position.x =
      std::round(new_pose.position.x / kGridResolution) * kGridResolution;
  new_pose.position.y =
      std::round(new_pose.position.y / kGridResolution) * kGridResolution;

  const int marker_count = static_cast<int>(trajectory_markers_.size());
  const int next_idx = (idx + 1) % marker_count;
  const auto &next_marker = trajectory_markers_[next_idx];
  const double dx = next_marker.pose.position.x - new_pose.position.x;
  const double dy = next_marker.pose.position.y - new_pose.position.y;
  const double yaw = std::atan2(dy, dx);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  new_pose.orientation = tf2::toMsg(q);

  trajectory_markers_[idx].pose = new_pose;
  trajectory_markers_[idx].header.stamp = this->now();
  server_->setPose(feedback->marker_name, trajectory_markers_[idx].pose);
  server_->applyChanges();
  publishMarkers();
  RCLCPP_DEBUG(get_logger(),
               "Published race_trajectory after moving %s to %.3f, %.3f, %.3f",
               feedback->marker_name.c_str(), new_pose.position.x,
               new_pose.position.y, new_pose.position.z);
}

void EditorToolServer::makeMoveTrajectoryMarker(
    const visualization_msgs::msg::Marker &marker) {
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = kFrameId;
  int_marker.header.stamp = this->now();
  int_marker.name = marker.ns + "_" + std::to_string(marker.id);
  int_marker.description = "Move Trajectory Marker";
  name_to_index_[int_marker.name] = marker.id;

  // ──
  // インタラクティブマーカー全体のスケールを大きめに設定（クリック可能領域を拡張）
  // ──
  int_marker.scale = 4.0; // 1.0 → 2.0 にすると、かなり広めにクリックできる

  // マーカーの初期ポーズをそのままコピー
  int_marker.pose = marker.pose;

  // MOVE_PLANE は通常編集の直接移動と、選択モードのクリック検出の両方で使う。
  visualization_msgs::msg::InteractiveMarkerControl control;
  tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

  // --- ① 透明 Sphere を作って、クリック領域をさらに大きくする ---
  visualization_msgs::msg::Marker hit_box;
  hit_box.header.stamp = this->now();
  hit_box.ns = kHitBoxNs;
  hit_box.id = marker.id;
  hit_box.type = visualization_msgs::msg::Marker::SPHERE;
  hit_box.scale.x = 2.0;
  hit_box.scale.y = 2.0;
  hit_box.scale.z = 2.0;
  hit_box.color.r = 0.0f;
  hit_box.color.g = 0.0f;
  hit_box.color.b = 0.0f;
  hit_box.color.a = 0.05f;
  hit_box.pose.orientation.w = 1.0;

  // 透明 Sphere を control に追加
  control.markers.push_back(hit_box);

  // --- ② 実際に見える矢印マーカーを追加 ---
  visualization_msgs::msg::Marker visible_arrow = marker;
  visible_arrow.header.frame_id.clear();
  visible_arrow.header.stamp = rclcpp::Time(0);
  visible_arrow.pose = geometry_msgs::msg::Pose();
  visible_arrow.pose.orientation.w = 1.0;
  control.markers.push_back(visible_arrow);

  control.name = "move_xy";
  control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
}

bool EditorToolServer::markerPointsToVelocityLine(
    visualization_msgs::msg::MarkerArray &marker_array) {
  const int marker_count = static_cast<int>(marker_array.markers.size());
  if (marker_count < 1) {
    return false;
  }

  std::vector<visualization_msgs::msg::Marker> line_markers;
  std::vector<visualization_msgs::msg::Marker> velocity_markers;

  // 各連続ペアをつなぎ、ラインマーカーと速度テキストマーカーを作成
  for (int i = 0; i < marker_count - 1; ++i) {
    const auto &marker = marker_array.markers[i];
    const auto &next_marker = marker_array.markers[i + 1];

    geometry_msgs::msg::Point start_point = marker.pose.position;
    geometry_msgs::msg::Point end_point = next_marker.pose.position;

    // ── ラインマーカー ──
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = kFrameId;
    line_marker.header.stamp = this->now();
    line_marker.ns = kLineMarkerNs;
    line_marker.id = i;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1; // 線の太さ
    line_marker.color = marker.color;
    line_marker.color.a = 1.0;
    line_marker.points.push_back(start_point);
    line_marker.points.push_back(end_point);
    line_markers.push_back(line_marker);

    // ── 速度テキストマーカー ──
    visualization_msgs::msg::Marker velocity_marker;
    velocity_marker.header = marker.header;
    velocity_marker.ns = kSpeedLabelNs;
    velocity_marker.id = i;
    velocity_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    velocity_marker.action = visualization_msgs::msg::Marker::ADD;
    velocity_marker.pose.position.x = start_point.x + 0.02;
    velocity_marker.pose.position.y = start_point.y + 0.02;
    velocity_marker.pose.position.z = start_point.z + 0.1;
    velocity_marker.pose.orientation.w = 1.0;
    velocity_marker.scale.z = 1.0; // テキストの大きさ
    velocity_marker.color = marker.color;
    velocity_marker.color.a = 1.0;
    velocity_marker.text = marker.text;
    velocity_markers.push_back(velocity_marker);
  }

  // 最後のマーカー → 最初のマーカーをつなぐ
  const auto &last_marker = marker_array.markers[marker_count - 1];
  const auto &first_marker = marker_array.markers[0];
  geometry_msgs::msg::Point last_start_point = last_marker.pose.position;
  geometry_msgs::msg::Point last_end_point = first_marker.pose.position;

  visualization_msgs::msg::Marker last_line_marker;
  last_line_marker.header.frame_id = kFrameId;
  last_line_marker.header.stamp = this->now();
  last_line_marker.ns = kLineMarkerNs;
  last_line_marker.id = marker_count - 1;
  last_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  last_line_marker.action = visualization_msgs::msg::Marker::ADD;
  last_line_marker.pose.orientation.w = 1.0;
  last_line_marker.scale.x = 0.1;
  last_line_marker.color = last_marker.color;
  last_line_marker.color.a = 1.0;
  last_line_marker.points.push_back(last_start_point);
  last_line_marker.points.push_back(last_end_point);
  line_markers.push_back(last_line_marker);

  // velocity_markers には最終マーカーのラベルは不要なので追加しない

  // オリジナル配列に「ライン」と「速度テキスト」を結合
  marker_array.markers.insert(marker_array.markers.end(), line_markers.begin(),
                              line_markers.end());
  marker_array.markers.insert(marker_array.markers.end(),
                              velocity_markers.begin(), velocity_markers.end());

  return true;
}

void EditorToolServer::publishMarkers() {
  visualization_msgs::msg::MarkerArray publish_array;
  publish_array.markers = trajectory_markers_; // 基礎マーカー

  // ライン＋速度ラベルを追加
  markerPointsToVelocityLine(publish_array);

  // /race_trajectory トピックにパブリッシュ
  marker_pub_->publish(publish_array);
  RCLCPP_DEBUG(get_logger(),
               "Published race_trajectory MarkerArray: %zu markers",
               publish_array.markers.size());
}

bool EditorToolServer::parseLineToMarker(
    const std::string &line, int id, visualization_msgs::msg::Marker &marker) {
  std::stringstream ss(line);
  std::string token;
  std::vector<std::string> elems;
  while (std::getline(ss, token, ',')) {
    elems.push_back(token);
  }
  if (elems.size() != kCsvColumnCount) {
    return false;
  }

  // 座標
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  // 四元数
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  // speed (m/s → km/h)
  double speed_mps = 0.0;
  if (!parseDoubleStrict(elems[0], x) || !parseDoubleStrict(elems[1], y) ||
      !parseDoubleStrict(elems[2], z) || !parseDoubleStrict(elems[3], qx) ||
      !parseDoubleStrict(elems[4], qy) || !parseDoubleStrict(elems[5], qz) ||
      !parseDoubleStrict(elems[6], qw) ||
      !parseDoubleStrict(elems[7], speed_mps)) {
    return false;
  }
  const double speed = speed_mps * kMpsToKmph;

  marker.header.frame_id = kFrameId;
  marker.header.stamp = this->now();
  marker.ns = kTrajectoryMarkerNs;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // テキストに速度 (小数1桁) を入れる
  {
    std::ostringstream text_ss;
    text_ss << std::fixed << std::setprecision(1) << speed;
    marker.text = text_ss.str();
  }

  // ポーズ設定
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = qx;
  marker.pose.orientation.y = qy;
  marker.pose.orientation.z = qz;
  marker.pose.orientation.w = qw;

  // 矢印のスケール (長さ, 幅, 太さ)
  marker.scale.x = 1.0; // length
  marker.scale.y = 0.5; // width
  marker.scale.z = 0.2;

  AutoColorizeTraj(marker, speed); // 色を速度に応じて設定
  return true;
}

bool EditorToolServer::saveCsv(const std::string &file_name) {
  std::ofstream ofs(file_name);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file: %s (%s)",
                 file_name.c_str(), std::strerror(errno));
    return false;
  }
  // ヘッダ行
  ofs << "x,y,z,qx,qy,qz,qw,speed\n";
  for (const auto &marker : trajectory_markers_) {
    const double speed_ms = markerSpeedKmh(marker) * kKmphToMps;
    ofs << marker.pose.position.x << "," << marker.pose.position.y << ","
        << marker.pose.position.z << "," << marker.pose.orientation.x << ","
        << marker.pose.orientation.y << "," << marker.pose.orientation.z << ","
        << marker.pose.orientation.w << "," << speed_ms << "\n";
  }
  ofs.close();
  RCLCPP_INFO(get_logger(), "CSV saved to %s", file_name.c_str());
  return true;
}

std::vector<int> EditorToolServer::getRangeIndices(int idx1, int idx2) {
  std::vector<int> indices;
  int N = static_cast<int>(trajectory_markers_.size());
  if (idx1 < 0 || idx2 < 0 || idx1 >= N || idx2 >= N)
    return indices;

  int i1 = idx1, i2 = idx2;
  // 前方距離・後方距離を比較して最短経路をとる
  int forward_len = (i2 >= i1) ? (i2 - i1) : (N - (i1 - i2));
  int backward_len = (i1 >= i2) ? (i1 - i2) : (N - (i2 - i1));
  bool use_forward = (forward_len <= backward_len);

  if (use_forward) {
    int idx = i1;
    while (true) {
      indices.push_back(idx);
      if (idx == i2)
        break;
      idx = (idx + 1) % N;
    }
  } else {
    int idx = i1;
    while (true) {
      indices.push_back(idx);
      if (idx == i2)
        break;
      idx = (idx - 1 + N) % N;
    }
  }
  return indices;
}

void EditorToolServer::StartParallelMove(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (selection_mode_ || parallel_mode_) {
    RCLCPP_WARN(
        get_logger(),
        "Cannot start parallel move: already in selection or parallel mode");
    response->success = false;
    response->message = "Busy";
    return;
  }
  if (trajectory_markers_.empty()) {
    RCLCPP_WARN(get_logger(),
                "Cannot start parallel move: no trajectory is loaded");
    response->success = false;
    response->message = "No trajectory loaded";
    return;
  }
  p_sel_idx1_ = p_sel_idx2_ = -1;
  parallel_mode_ = true;
  RCLCPP_INFO(get_logger(),
              "ParallelMove: mode started. Click two markers to define range.");
  response->success = true;
  response->message = "Parallel mode ON";
}

void EditorToolServer::ConfirmParallelMove(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;
  if (!parallel_mode_) {
    RCLCPP_WARN(get_logger(),
                "ConfirmParallelMove called but not in parallel mode");
    response->success = false;
    response->message = "Not in parallel mode";
    return;
  }

  // ダミー移動マーカーをサーバから消す
  server_->erase(move_marker_name_);
  server_->applyChanges();

  parallel_mode_ = false;
  p_sel_idx1_ = p_sel_idx2_ = -1;
  refreshTrajectoryColor();
  saveStateForUndo();
  RCLCPP_INFO(get_logger(), "ParallelMove: confirmed and mode exited.");
  response->success = true;
  response->message = "Parallel move confirmed";
}

void EditorToolServer::createMoveHelperMarker() {
  // 1) 選択範囲内インデックスを取得
  std::vector<int> indices = getRangeIndices(p_sel_idx1_, p_sel_idx2_);
  if (indices.empty()) {
    RCLCPP_WARN(get_logger(), "createMoveHelperMarker: no indices found");
    parallel_mode_ = false;
    return;
  }

  // 2) 範囲内マーカーの重心を計算
  geometry_msgs::msg::Point centroid;
  centroid.x = centroid.y = centroid.z = 0.0;
  for (int idx : indices) {
    centroid.x += trajectory_markers_[idx].pose.position.x;
    centroid.y += trajectory_markers_[idx].pose.position.y;
    centroid.z += trajectory_markers_[idx].pose.position.z;
  }
  centroid.x /= indices.size();
  centroid.y /= indices.size();
  centroid.z /= indices.size();

  // 3) ダミー・移動マーカーを作成
  visualization_msgs::msg::InteractiveMarker helper;
  helper.header.frame_id = kFrameId;
  helper.header.stamp = this->now();
  move_marker_name_ = "parallel_move_helper";
  helper.name = move_marker_name_;
  helper.description = "Drag to move selected range";
  helper.pose.position = centroid;
  helper.scale = 0.0;

  visualization_msgs::msg::Marker sphere =
      makePickSphere(kHelperMarkerNs, 0, 1.0);
  sphere.header.stamp = this->now();
  sphere.color.a = 0.85f;
  helper.controls.clear();

  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(sphere);
  helper.controls.push_back(control);

  // MOVE_PLANE コントロールを追加 (XY移動)
  visualization_msgs::msg::InteractiveMarkerControl move_control;
  move_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  move_control.orientation.w = 1;
  move_control.orientation.x = 0;
  move_control.orientation.y = 1;
  move_control.orientation.z = 0;
  move_control.markers.push_back(sphere);
  helper.controls.push_back(move_control);

  server_->insert(helper);

  server_->applyChanges();

  // 初回の前回位置を設定
  move_marker_prev_pos_ = centroid;
}

void EditorToolServer::saveStateForUndo() {
  if (!undo_stack_.empty() && undo_stack_.back() == trajectory_markers_) {
    return;
  }

  undo_stack_.push_back(trajectory_markers_);
  if (undo_stack_.size() > kMaxUndoHistory) {
    undo_stack_.erase(undo_stack_.begin());
  }
  redo_stack_.clear();
}

void EditorToolServer::undo(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)req;
  if (undo_stack_.size() <= 1) {
    RCLCPP_WARN(get_logger(), "Nothing to undo");
    response->success = false;
    response->message = "Undo stack empty";
    return;
  }
  redo_stack_.push_back(undo_stack_.back());
  undo_stack_.pop_back();
  trajectory_markers_ = undo_stack_.back();
  redrawMarkers();
  RCLCPP_INFO(get_logger(), "Undo executed");
  response->success = true;
  response->message = "Undo OK";
}

void EditorToolServer::redo(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)req;
  if (redo_stack_.empty()) {
    RCLCPP_WARN(get_logger(), "Nothing to redo");
    response->success = false;
    response->message = "Redo stack empty";
    return;
  }
  trajectory_markers_ = redo_stack_.back();
  redo_stack_.pop_back();
  undo_stack_.push_back(trajectory_markers_);
  redrawMarkers();
  RCLCPP_INFO(get_logger(), "Redo executed");
  response->success = true;
  response->message = "Redo OK";
}

void EditorToolServer::redrawMarkers() {
  server_->clear();
  name_to_index_.clear();
  // trajectory_markers_ に合わせてインタラクティブマーカーを作り直す
  for (const auto &marker : trajectory_markers_) {
    makeMoveTrajectoryMarker(marker);
  }
  server_->applyChanges();
  publishMarkers();
  // name_to_index_ は makeMoveTrajectoryMarker で埋まる
}

void EditorToolServer::refreshTrajectoryColor() {
  for (auto &marker : trajectory_markers_) {
    double speed = markerSpeedKmh(marker);
    AutoColorizeTraj(marker, speed);
  }
  redrawMarkers();
}

} // namespace editor_tool_server

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<editor_tool_server::EditorToolServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
