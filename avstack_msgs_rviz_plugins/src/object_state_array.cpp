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

#include <avstack_msgs_rviz_plugins/object_state_array.hpp>

#include <memory>

namespace rviz_plugins
{

ObjectStateArrayDisplay::ObjectStateArrayDisplay()
{
  only_edge_property_ = new rviz_common::properties::BoolProperty(
    "Only Edge", false, "Display only edges of the boxes", this, SLOT(updateEdge()));
  line_width_property_ = new rviz_common::properties::FloatProperty(
    "Line Width", 0.05, "Line width of edges", this, SLOT(updateLineWidth()));
  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "Transparency", this, SLOT(updateAlpha()));
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", Qt::yellow, "Color of bounding box", this, SLOT(updateColor()));
  color = Qt::yellow;
}

ObjectStateArrayDisplay::~ObjectStateArrayDisplay()
{
  delete only_edge_property_;
  delete line_width_property_;
  delete alpha_property_;
}

void ObjectStateArrayDisplay::onInitialize()
{
  RTDClass::onInitialize();
  m_marker_common->initialize(context_, scene_node_);

  topic_property_->setValue("object_state_array");
  topic_property_->setDescription("ObjectStateArray topic to subscribe to.");

  line_width_property_->setMax(0.1);
  line_width_property_->setMin(0.01);
  line_width_property_->hide();

  alpha_property_->setMax(1.0);
  alpha_property_->setMin(0.1);

  line_width = line_width_property_->getFloat();
  alpha = alpha_property_->getFloat();

  only_edge_ = only_edge_property_->getBool();
}

void ObjectStateArrayDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
  m_marker_common->load(config);
}

void ObjectStateArrayDisplay::processMessage(
  ObjectStateArray::ConstSharedPtr msg)
{
  latest_msg = msg;
  if (!only_edge_) {
    showBoxes(msg);
  } else {
    showEdges(msg);
  }
}

void ObjectStateArrayDisplay::update(float wall_dt, float ros_dt)
{
  m_marker_common->update(wall_dt, ros_dt);
}

void ObjectStateArrayDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common->clearMarkers();
  edges_.clear();
}

void ObjectStateArrayDisplay::updateEdge()
{
  only_edge_ = only_edge_property_->getBool();
  if (only_edge_) {
    line_width_property_->show();
  } else {
    line_width_property_->hide();
  }
  // Imediately apply attribute
  if (latest_msg) {
    if (only_edge_) {
      showEdges(latest_msg);
    } else {
      showBoxes(latest_msg);
    }
  }
}

void ObjectStateArrayDisplay::updateLineWidth()
{
  line_width = line_width_property_->getFloat();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void ObjectStateArrayDisplay::updateAlpha()
{
  alpha = alpha_property_->getFloat();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

void ObjectStateArrayDisplay::updateColor()
{
  color = color_property_->getColor();
  if (latest_msg) {
    processMessage(latest_msg);
  }
}

}  // namespace rviz_plugins

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ObjectStateArrayDisplay, rviz_common::Display)