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

#ifndef AVSTACK_MSGS_RVIZ_PLUGINS__OBJECT_STATE_COMMON_HPP_
#define AVSTACK_MSGS_RVIZ_PLUGINS__OBJECT_STATE_COMMON_HPP_

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

#include <avstack_msgs/msg/object_state.hpp>
#include <avstack_msgs/msg/object_state_array.hpp>

typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;

namespace rviz_plugins
{
template<class MessageType>
class ObjectStateCommon : public rviz_common::RosTopicDisplay<MessageType>
{
public:
  using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
  using Marker = visualization_msgs::msg::Marker;
  using ObjectState = avstack_msgs::msg::ObjectState;
  using ObjectStateArray = avstack_msgs::msg::ObjectStateArray;

  ObjectStateCommon()
  : rviz_common::RosTopicDisplay<MessageType>(), line_width(0.05), alpha(),
    m_marker_common(std::make_unique<MarkerCommon>(
        this)), color(Qt::yellow) {}
  ~ObjectStateCommon() {}

protected:
  float line_width, alpha;
  std::unique_ptr<MarkerCommon> m_marker_common;
  QColor color;
  std::vector<BillboardLinePtr> edges_;

  visualization_msgs::msg::Marker::SharedPtr get_marker(
    const avstack_msgs::msg::ObjectState & obj)
  {
    auto marker = std::make_shared<Marker>();

    marker->type = Marker::CUBE;
    marker->action = Marker::ADD;

    marker->pose.position.x = static_cast<double>(obj.box.center.position.x);
    marker->pose.position.y = static_cast<double>(obj.box.center.position.y);
    marker->pose.position.z = static_cast<double>(obj.box.center.position.z);
    marker->pose.orientation.x = static_cast<double>(obj.box.center.orientation.x);
    marker->pose.orientation.y = static_cast<double>(obj.box.center.orientation.y);
    marker->pose.orientation.z = static_cast<double>(obj.box.center.orientation.z);
    marker->pose.orientation.w = static_cast<double>(obj.box.center.orientation.w);

    if (obj.box.size.x < 0.0 || obj.box.size.y < 0.0 || obj.box.size.z < 0.0) {
      std::ostringstream oss;
      oss << "Error received ObjectState message with size value less than zero.\n";
      oss << "X: " << obj.box.size.x << " Y: " << obj.box.size.y << " Z: " << obj.box.size.z;
      RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
      this->setStatus(
        rviz_common::properties::StatusProperty::Error, "Scale", QString::fromStdString(
          oss.str()));
    }

    // Some systems can return ObjectState messages with one dimension set to zero.
    // (for example Isaac Sim can have a mesh that is only a plane)
    // This is not supported by Rviz markers so set the scale to a small value if this happens.
    if (obj.box.size.x < 1e-4) {
      marker->scale.x = 1e-4;
    } else {
      marker->scale.x = static_cast<double>(obj.box.size.x);
    }
    if (obj.box.size.y < 1e-4) {
      marker->scale.y = 1e-4;
    } else {
      marker->scale.y = static_cast<double>(obj.box.size.y);
    }
    if (obj.box.size.z < 1e-4) {
      marker->scale.z = 1e-4;
    } else {
      marker->scale.z = static_cast<double>(obj.box.size.z);
    }

    return marker;
  }

  void showBoxes(const ObjectStateArray::ConstSharedPtr & msg)
  {
    edges_.clear();
    m_marker_common->clearMarkers();

    for (size_t idx = 0U; idx < msg->states.size(); idx++) {
      const auto marker_ptr = get_marker(msg->states[idx]);

      marker_ptr->color.r = color.red() / 255.0;
      marker_ptr->color.g = color.green() / 255.0;
      marker_ptr->color.b = color.blue() / 255.0;
      marker_ptr->color.a = alpha;
      marker_ptr->ns = "object_state";
      marker_ptr->header = msg->header;
      marker_ptr->id = idx;
      m_marker_common->addMessage(marker_ptr);
    }
  }

  void showBoxes(const ObjectState::ConstSharedPtr & msg)
  {
    edges_.clear();
    m_marker_common->clearMarkers();

    const auto marker_ptr = get_marker(*msg);
    marker_ptr->header.frame_id = qPrintable(this->fixed_frame_);
    marker_ptr->header.stamp = rclcpp::Clock().now();
    marker_ptr->color.r = color.red() / 255.0;
    marker_ptr->color.g = color.green() / 255.0;
    marker_ptr->color.b = color.blue() / 255.0;
    marker_ptr->color.a = alpha;
    marker_ptr->ns = "object_state";
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

  void showEdges(const ObjectStateArray::ConstSharedPtr & msg)
  {
    m_marker_common->clearMarkers();

    allocateBillboardLines(msg->states.size());

    for (size_t idx = 0; idx < msg->states.size(); idx++) {
      avstack_msgs::msg::ObjectState obj = msg->states[idx];

      BillboardLinePtr edge = edges_[idx];
      edge->clear();
      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      geometry_msgs::msg::Vector3 dimensions = obj.box.size;
      if (!this->context_->getFrameManager()->transform(
          msg->header, obj.box.center,
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

  void showEdges(const ObjectState::ConstSharedPtr & msg)
  {
    m_marker_common->clearMarkers();
    allocateBillboardLines(1);
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
};
}  // namespace rviz_plugins

#endif  // AVSTACK_MSGS_RVIZ_PLUGINS__OBJECT_STATE_COMMON_HPP_