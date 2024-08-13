// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AVSTACK_MSGS_RVIZ_PLUGINS__BOX_TRACK_COMMON_HPP_
#define AVSTACK_MSGS_RVIZ_PLUGINS__BOX_TRACK_COMMON_HPP_

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <vector>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <avstack_msgs/msg/box_track.hpp>
#include <avstack_msgs/msg/box_track_array.hpp>

typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;

namespace rviz_plugins
{
template<class MessageType>
class BoxTrackCommon : public rviz_common::RosTopicDisplay<MessageType>
{
public:
  using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
  using Marker = visualization_msgs::msg::Marker;
  using BoxTrack = avstack_msgs::msg::BoxTrack;
  using BoxTrackArray = avstack_msgs::msg::BoxTrackArray;

  BoxTrackCommon()
  : rviz_common::RosTopicDisplay<MessageType>(), line_width(0.05), alpha(),
    m_marker_common(std::make_unique<MarkerCommon>(
        this)), color(Qt::yellow) {}
  ~BoxTrackCommon() {}

protected:
  float line_width, alpha;
  std::unique_ptr<MarkerCommon> m_marker_common;
  QColor color;
  std::vector<BillboardLinePtr> edges_;
  std::unordered_map<int, visualization_msgs::msg::Marker::SharedPtr> score_markers;
  std::unordered_map<int, visualization_msgs::msg::Marker::SharedPtr> identifier_markers;


  visualization_msgs::msg::Marker::SharedPtr get_marker(
    const avstack_msgs::msg::BoxTrack & trk)
  {
    auto marker = std::make_shared<Marker>();

    marker->type = Marker::CUBE;
    marker->action = Marker::ADD;

    marker->pose.position.x = static_cast<double>(trk.box.center.position.x);
    marker->pose.position.y = static_cast<double>(trk.box.center.position.y);
    marker->pose.position.z = static_cast<double>(trk.box.center.position.z);
    marker->pose.orientation.x = static_cast<double>(trk.box.center.orientation.x);
    marker->pose.orientation.y = static_cast<double>(trk.box.center.orientation.y);
    marker->pose.orientation.z = static_cast<double>(trk.box.center.orientation.z);
    marker->pose.orientation.w = static_cast<double>(trk.box.center.orientation.w);

    if (trk.box.size.x < 0.0 || trk.box.size.y < 0.0 || trk.box.size.z < 0.0) {
      std::ostringstream oss;
      oss << "Error received BoxTrack message with size value less than zero.\n";
      oss << "X: " << trk.box.size.x << " Y: " << trk.box.size.y << " Z: " << trk.box.size.z;
      RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
      this->setStatus(
        rviz_common::properties::StatusProperty::Error, "Scale", QString::fromStdString(
          oss.str()));
    }

    // Some systems can return BoxTrack messages with one dimension set to zero.
    // (for example Isaac Sim can have a mesh that is only a plane)
    // This is not supported by Rviz markers so set the scale to a small value if this happens.
    if (trk.box.size.x < 1e-4) {
      marker->scale.x = 1e-4;
    } else {
      marker->scale.x = static_cast<double>(trk.box.size.x);
    }
    if (trk.box.size.y < 1e-4) {
      marker->scale.y = 1e-4;
    } else {
      marker->scale.y = static_cast<double>(trk.box.size.y);
    }
    if (trk.box.size.z < 1e-4) {
      marker->scale.z = 1e-4;
    } else {
      marker->scale.z = static_cast<double>(trk.box.size.z);
    }

    return marker;
  }

  void showBoxes(
    const BoxTrackArray::ConstSharedPtr & msg,
    const bool show_score,
    const bool show_identifier)
  {
    edges_.clear();
    m_marker_common->clearMarkers();
    ClearScores(show_score);
    ClearIdentifiers(show_identifier);

    for (size_t idx = 0U; idx < msg->tracks.size(); idx++) {
      const auto marker_ptr = get_marker(msg->tracks[idx]);
      if (show_score) {
        ShowScore(msg->tracks[idx], msg->tracks[idx].score, idx);
      }
      if (show_identifier) {
        ShowIdentifier(msg->tracks[idx], msg->tracks[idx].identifier, idx);
      }
      marker_ptr->color.r = color.red() / 255.0;
      marker_ptr->color.g = color.green() / 255.0;
      marker_ptr->color.b = color.blue() / 255.0;
      marker_ptr->color.a = alpha;
      marker_ptr->ns = "box_track";
      marker_ptr->header = msg->header;
      marker_ptr->id = idx;
      m_marker_common->addMessage(marker_ptr);
    }
  }

  void showBoxes(
    const BoxTrack::ConstSharedPtr & msg,
    const bool show_score,
    const bool show_identifier)
  {
    edges_.clear();
    m_marker_common->clearMarkers();
    ClearScores(show_score);
    ClearIdentifiers(show_identifier);

    const auto marker_ptr = get_marker(*msg);
    if (show_score) {
      ShowScore(*msg, msg->score, 0);
    }
    if (show_identifier) {
      ShowIdentifier(*msg, msg->identifier, 0);
    }
    marker_ptr->header.frame_id = qPrintable(this->fixed_frame_);
    marker_ptr->header.stamp = rclcpp::Clock().now();
    marker_ptr->color.r = color.red() / 255.0;
    marker_ptr->color.g = color.green() / 255.0;
    marker_ptr->color.b = color.blue() / 255.0;
    marker_ptr->color.a = alpha;
    marker_ptr->ns = "box_track";
    // marker_ptr->header = msg->header;
    marker_ptr->id = 0;
    m_marker_common->addMessage(marker_ptr);
  }

  void allocateBillboardLines(size_t num)
  {
    if (num > edges_.size()) {
      for (size_t i = edges_.size(); i < num; i++) {
        BillboardLinePtr line(new rviz_rendering::BillboardLine(
            this->context_->getSceneManager(), this->scene_node_));
        edges_.push_back(line);
      }
    } else if (num < edges_.size()) {
      edges_.resize(num);
    }
  }

  void showEdges(
    const BoxTrackArray::ConstSharedPtr & msg,
    const bool show_score,
    const bool show_identifier)
  {
    m_marker_common->clearMarkers();
    ClearScores(show_score);
    ClearIdentifiers(show_identifier);

    allocateBillboardLines(msg->tracks.size());

    for (size_t idx = 0; idx < msg->tracks.size(); idx++) {
      avstack_msgs::msg::BoxTrack trk = msg->tracks[idx];
      if (show_score) {
        ShowScore(msg->tracks[idx], msg->tracks[idx].score, idx);
      }
      if (show_identifier) {
        ShowIdentifier(msg->tracks[idx], msg->tracks[idx].identifier, idx);
      }
      BillboardLinePtr edge = edges_[idx];
      edge->clear();
      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      geometry_msgs::msg::Vector3 dimensions = trk.box.size;
      if (!this->context_->getFrameManager()->transform(
          msg->header, trk.box.center,
          position,
          quaternion))
      {
        std::ostringstream oss;
        oss << "Error transforming pose";
        oss << " from frame '" << msg->header.frame_id << "'";
        oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
        RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
        this->setStatus(
          rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(
            oss.str()));
      }
      edge->setPosition(position);
      edge->setOrientation(quaternion);

      edge->setMaxPointsPerLine(2);
      edge->setNumLines(12);
      edge->setLineWidth(line_width);
      edge->setColor(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, this->alpha);

      Ogre::Vector3 A, B, C, D, E, F, G, H;
      A[0] = dimensions.x / 2.0;
      A[1] = dimensions.y / 2.0;
      A[2] = dimensions.z / 2.0;
      B[0] = -dimensions.x / 2.0;
      B[1] = dimensions.y / 2.0;
      B[2] = dimensions.z / 2.0;
      C[0] = -dimensions.x / 2.0;
      C[1] = -dimensions.y / 2.0;
      C[2] = dimensions.z / 2.0;
      D[0] = dimensions.x / 2.0;
      D[1] = -dimensions.y / 2.0;
      D[2] = dimensions.z / 2.0;

      E[0] = dimensions.x / 2.0;
      E[1] = dimensions.y / 2.0;
      E[2] = -dimensions.z / 2.0;
      F[0] = -dimensions.x / 2.0;
      F[1] = dimensions.y / 2.0;
      F[2] = -dimensions.z / 2.0;
      G[0] = -dimensions.x / 2.0;
      G[1] = -dimensions.y / 2.0;
      G[2] = -dimensions.z / 2.0;
      H[0] = dimensions.x / 2.0;
      H[1] = -dimensions.y / 2.0;
      H[2] = -dimensions.z / 2.0;

      edge->addPoint(A);
      edge->addPoint(B);
      edge->finishLine();
      edge->addPoint(B);
      edge->addPoint(C);
      edge->finishLine();
      edge->addPoint(C);
      edge->addPoint(D);
      edge->finishLine();
      edge->addPoint(D);
      edge->addPoint(A);
      edge->finishLine();
      edge->addPoint(E);
      edge->addPoint(F);
      edge->finishLine();
      edge->addPoint(F);
      edge->addPoint(G);
      edge->finishLine();
      edge->addPoint(G);
      edge->addPoint(H);
      edge->finishLine();
      edge->addPoint(H);
      edge->addPoint(E);
      edge->finishLine();
      edge->addPoint(A);
      edge->addPoint(E);
      edge->finishLine();
      edge->addPoint(B);
      edge->addPoint(F);
      edge->finishLine();
      edge->addPoint(C);
      edge->addPoint(G);
      edge->finishLine();
      edge->addPoint(D);
      edge->addPoint(H);
    }
  }

  void showEdges(
    const BoxTrack::ConstSharedPtr & msg,
    const bool show_score,
    const bool show_identifier)
  {
    m_marker_common->clearMarkers();
    ClearScores(show_score);
    ClearIdentifiers(show_identifier);

    allocateBillboardLines(1);
    if (show_score) {
      ShowScore(*msg, msg->score, 0);
    }
    if (show_identifier) {
      ShowIdentifier(*msg, msg->identifier, 0);
    }

    BillboardLinePtr edge = edges_[0];
    edge->clear();
    geometry_msgs::msg::Vector3 dimensions = msg->box.size;

    std_msgs::msg::Header header;
    header.frame_id = qPrintable(this->fixed_frame_);

    Ogre::Vector3 position;
    Ogre::Quaternion quaternion;

    if (!this->context_->getFrameManager()->transform(
        header, msg->box.center,
        position,
        quaternion))
    {
      std::ostringstream oss;
      oss << "Error transforming pose";
      oss << " from frame '" << header.frame_id << "'";
      oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
      RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
      this->setStatus(
        rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(
          oss.str()));
      return;
    }

    edge->setPosition(position);
    edge->setOrientation(quaternion);

    edge->setMaxPointsPerLine(2);
    edge->setNumLines(12);
    edge->setLineWidth(line_width);
    edge->setColor(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, this->alpha);

    Ogre::Vector3 A, B, C, D, E, F, G, H;
    A[0] = dimensions.x / 2.0;
    A[1] = dimensions.y / 2.0;
    A[2] = dimensions.z / 2.0;
    B[0] = -dimensions.x / 2.0;
    B[1] = dimensions.y / 2.0;
    B[2] = dimensions.z / 2.0;
    C[0] = -dimensions.x / 2.0;
    C[1] = -dimensions.y / 2.0;
    C[2] = dimensions.z / 2.0;
    D[0] = dimensions.x / 2.0;
    D[1] = -dimensions.y / 2.0;
    D[2] = dimensions.z / 2.0;

    E[0] = dimensions.x / 2.0;
    E[1] = dimensions.y / 2.0;
    E[2] = -dimensions.z / 2.0;
    F[0] = -dimensions.x / 2.0;
    F[1] = dimensions.y / 2.0;
    F[2] = -dimensions.z / 2.0;
    G[0] = -dimensions.x / 2.0;
    G[1] = -dimensions.y / 2.0;
    G[2] = -dimensions.z / 2.0;
    H[0] = dimensions.x / 2.0;
    H[1] = -dimensions.y / 2.0;
    H[2] = -dimensions.z / 2.0;

    edge->addPoint(A);
    edge->addPoint(B);
    edge->finishLine();
    edge->addPoint(B);
    edge->addPoint(C);
    edge->finishLine();
    edge->addPoint(C);
    edge->addPoint(D);
    edge->finishLine();
    edge->addPoint(D);
    edge->addPoint(A);
    edge->finishLine();
    edge->addPoint(E);
    edge->addPoint(F);
    edge->finishLine();
    edge->addPoint(F);
    edge->addPoint(G);
    edge->finishLine();
    edge->addPoint(G);
    edge->addPoint(H);
    edge->finishLine();
    edge->addPoint(H);
    edge->addPoint(E);
    edge->finishLine();
    edge->addPoint(A);
    edge->addPoint(E);
    edge->finishLine();
    edge->addPoint(B);
    edge->addPoint(F);
    edge->finishLine();
    edge->addPoint(C);
    edge->addPoint(G);
    edge->finishLine();
    edge->addPoint(D);
    edge->addPoint(H);
  }

  void ShowScore(
    const avstack_msgs::msg::BoxTrack track,
    const double score,
    const size_t idx)
  {
    auto marker = std::make_shared<Marker>();
    marker->type = Marker::TEXT_VIEW_FACING;
    marker->action = Marker::ADD;
    marker->header = track.header;
    std::ostringstream oss;
    oss << std::fixed;
    oss << std::setprecision(2);
    oss << score;
    marker->text = oss.str();
    marker->scale.z = 0.5;         // Set the size of the text
    marker->id = idx;
    marker->ns = "score";
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 1.0f;
    marker->color.a = alpha;
    marker->pose.position.x = static_cast<double>(track.box.center.position.x);
    marker->pose.position.y = static_cast<double>(track.box.center.position.y);
    marker->pose.position.z =
      static_cast<double>(track.box.center.position.z + (track.box.size.z / 2.0) * 1.2);

    // Add the marker to the MarkerArray message
    m_marker_common->addMessage(marker);
    score_markers[idx] = marker;
  }

  void ClearScores(const bool show_score)
  {
    if (!show_score) {
      for (auto &[id, marker] : score_markers) {
        marker->action = visualization_msgs::msg::Marker::DELETE;
        m_marker_common->addMessage(marker);
      }
      score_markers.clear();
    }
  }

  void ShowIdentifier(
    const avstack_msgs::msg::BoxTrack track,
    const int identifier,
    const size_t idx)
  {
    auto marker = std::make_shared<Marker>();
    marker->type = Marker::TEXT_VIEW_FACING;
    marker->action = Marker::ADD;
    marker->header = track.header;
    std::ostringstream oss;
    oss << std::fixed;
    oss << std::setprecision(2);
    oss << identifier;
    marker->text = oss.str();
    marker->scale.z = 0.5;         // Set the size of the text
    marker->id = idx;
    marker->ns = "id";
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 1.0f;
    marker->color.a = alpha;
    marker->pose.position.x = static_cast<double>(track.box.center.position.x);
    marker->pose.position.y = static_cast<double>(track.box.center.position.y);
    marker->pose.position.z =
      static_cast<double>(track.box.center.position.z + (track.box.size.z / 2.0) * 1.2);

    // Add the marker to the MarkerArray message
    m_marker_common->addMessage(marker);
    identifier_markers[idx] = marker;
  }

  void ClearIdentifiers(const bool show_identifier)
  {
    if (!show_identifier) {
      for (auto &[id, marker] : identifier_markers) {
        marker->action = visualization_msgs::msg::Marker::DELETE;
        m_marker_common->addMessage(marker);
      }
      identifier_markers.clear();
    }
  }
};
}  // namespace rviz_plugins

#endif  // AVSTACK_MSGS_RVIZ_PLUGINS__BOX_TRACK_COMMON_HPP_