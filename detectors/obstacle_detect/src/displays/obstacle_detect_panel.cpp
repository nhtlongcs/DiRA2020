/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "displays/obstacle_detect_panel.hpp"

namespace obstacles_display {

ObstacleDetectPanel::ObstacleDetectPanel(QWidget* parent)
    : rviz::Panel(parent), pnh_("obstacle_detect"), drClt_("obstacle_detect") {
  initParams();

  use_split_merge_checkbox_ = new QCheckBox("Use split and merge");
  circ_from_visib_checkbox_ = new QCheckBox("Use visibles");
  publish_circle_segments = new QCheckBox("Publish circle's segments");

  min_n_input_ = new QLineEdit();
  dist_prop_input_ = new QLineEdit();
  group_dist_input_ = new QLineEdit();
  split_dist_input_ = new QLineEdit();
  merge_sep_input_ = new QLineEdit();
  merge_spread_input_ = new QLineEdit();
  max_radius_input_ = new QLineEdit();

  min_n_input_->setAlignment(Qt::AlignRight);
  dist_prop_input_->setAlignment(Qt::AlignRight);
  group_dist_input_->setAlignment(Qt::AlignRight);
  split_dist_input_->setAlignment(Qt::AlignRight);
  merge_sep_input_->setAlignment(Qt::AlignRight);
  merge_spread_input_->setAlignment(Qt::AlignRight);
  max_radius_input_->setAlignment(Qt::AlignRight);

  QFrame* lines[5];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin =
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QGroupBox* segmentation_box = new QGroupBox("Segmentation:");
  QGridLayout* segmentation_layout = new QGridLayout;
  segmentation_layout->addWidget(new QLabel("N<sub>min</sub>:"), 0, 0,
                                 Qt::AlignRight);
  segmentation_layout->addWidget(min_n_input_, 0, 1);
  segmentation_layout->addWidget(new QLabel("   "), 0, 2);
  segmentation_layout->addWidget(new QLabel("d<sub>p</sub>:"), 0, 3,
                                 Qt::AlignRight);
  segmentation_layout->addWidget(dist_prop_input_, 0, 4);
  segmentation_layout->addWidget(new QLabel(""), 0, 5);

  segmentation_layout->addWidget(new QLabel("d<sub>group</sub>:"), 1, 0,
                                 Qt::AlignRight);
  segmentation_layout->addWidget(group_dist_input_, 1, 1);
  segmentation_layout->addWidget(new QLabel("m  "), 1, 2, Qt::AlignLeft);
  segmentation_layout->addWidget(new QLabel("d<sub>split</sub>:"), 1, 3,
                                 Qt::AlignRight);
  segmentation_layout->addWidget(split_dist_input_, 1, 4);
  segmentation_layout->addWidget(new QLabel("m"), 1, 5, Qt::AlignLeft);

  segmentation_layout->addWidget(new QLabel("d<sub>sep</sub>:"), 2, 0,
                                 Qt::AlignRight);
  segmentation_layout->addWidget(merge_sep_input_, 2, 1);
  segmentation_layout->addWidget(new QLabel("m  "), 2, 2, Qt::AlignLeft);
  segmentation_layout->addWidget(new QLabel("d<sub>spread</sub>:"), 2, 3,
                                 Qt::AlignRight);
  segmentation_layout->addWidget(merge_spread_input_, 2, 4);
  segmentation_layout->addWidget(new QLabel("m"), 2, 5, Qt::AlignLeft);

  segmentation_layout->addWidget(use_split_merge_checkbox_, 3, 0, 1, 6,
                                 Qt::AlignCenter);
  segmentation_box->setLayout(segmentation_layout);

  QGroupBox* circle_box = new QGroupBox("Circularization:");
  QGridLayout* circ_limits_layout = new QGridLayout;
  circ_limits_layout->addWidget(new QLabel("r<sub>max</sub>:"), 0, 0,
                                Qt::AlignRight);
  circ_limits_layout->addWidget(max_radius_input_, 0, 1);
  circ_limits_layout->addWidget(new QLabel("m "), 0, 2, Qt::AlignLeft);
  circ_limits_layout->addWidget(new QLabel("r<sub>margin</sub>:"), 0, 3,
                                Qt::AlignRight);
  circ_limits_layout->addWidget(new QLabel("m"), 0, 5, Qt::AlignLeft);

  QHBoxLayout* circ_checks_layout = new QHBoxLayout;
  circ_checks_layout->addWidget(publish_circle_segments, 0, Qt::AlignCenter);
  circ_checks_layout->addWidget(circ_from_visib_checkbox_, 0, Qt::AlignCenter);

  QVBoxLayout* circ_layout = new QVBoxLayout;
  circ_layout->addLayout(circ_limits_layout);
  circ_layout->addLayout(circ_checks_layout);
  circle_box->setLayout(circ_layout);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(lines[0]);
  layout->addWidget(lines[1]);
  layout->addWidget(segmentation_box);
  layout->addWidget(lines[2]);
  layout->addWidget(circle_box);
  layout->addWidget(lines[3]);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(use_split_merge_checkbox_, SIGNAL(clicked()), this,
          SLOT(handleInputs()));
  connect(circ_from_visib_checkbox_, SIGNAL(clicked()), this,
          SLOT(handleInputs()));
  connect(publish_circle_segments, SIGNAL(clicked()), this,
          SLOT(handleInputs()));

  connect(min_n_input_, SIGNAL(editingFinished()), this, SLOT(handleInputs()));
  connect(dist_prop_input_, SIGNAL(editingFinished()), this,
          SLOT(handleInputs()));
  connect(group_dist_input_, SIGNAL(editingFinished()), this,
          SLOT(handleInputs()));
  connect(split_dist_input_, SIGNAL(editingFinished()), this,
          SLOT(handleInputs()));
  connect(merge_sep_input_, SIGNAL(editingFinished()), this,
          SLOT(handleInputs()));
  connect(merge_spread_input_, SIGNAL(editingFinished()), this,
          SLOT(handleInputs()));
  connect(max_radius_input_, SIGNAL(editingFinished()), this,
          SLOT(handleInputs()));

  updateUI();
}

void ObstacleDetectPanel::initParams() {
  pnh_.getParam("min_range", cfg_.min_range);
  pnh_.getParam("max_range", cfg_.max_range);

  pnh_.getParam("use_split_and_merge", cfg_.use_split_and_merge);
  pnh_.getParam("detect_only_visible_circle", cfg_.detect_only_visible_circle);
  pnh_.getParam("publish_seg_of_circle", cfg_.publish_seg_of_circle);
  pnh_.getParam("min_group_points", cfg_.min_group_points);

  pnh_.getParam("p_max_group_distance_", cfg_.max_group_distance);
  pnh_.getParam("p_distance_proportion_", cfg_.distance_proportion);
  pnh_.getParam("max_split_distance", cfg_.max_split_distance);
  pnh_.getParam("max_merge_separation", cfg_.max_merge_separation);
  pnh_.getParam("max_merge_spread", cfg_.max_merge_spread);
  pnh_.getParam("max_circle_radius", cfg_.max_circle_radius);

  pnh_.getParam("frame_id", frameId_);
}

void ObstacleDetectPanel::handleInputs() {
  if (isUpdateUI) {
    getRvizInputs();
    drClt_.setConfiguration(cfg_);
    updateUI();
  }
  isUpdateUI = !isUpdateUI;
}

void ObstacleDetectPanel::getRvizInputs() {
  cfg_.use_split_and_merge = use_split_merge_checkbox_->isChecked();
  cfg_.detect_only_visible_circle = circ_from_visib_checkbox_->isChecked();
  cfg_.publish_seg_of_circle = publish_circle_segments->isChecked();

  try {
    cfg_.min_group_points = min_n_input_->text().toInt();
  } catch (std::invalid_argument&) {
    cfg_.min_group_points = 0;
    min_n_input_->setText("0");
  }

  try {
    cfg_.distance_proportion = dist_prop_input_->text().toDouble();
  } catch (std::invalid_argument&) {
    cfg_.distance_proportion = 0.0;
    dist_prop_input_->setText("0.0");
  }

  try {
    cfg_.max_group_distance = group_dist_input_->text().toDouble();
  } catch (std::invalid_argument&) {
    cfg_.max_group_distance = 0.0;
    group_dist_input_->setText("0.0");
  }

  try {
    cfg_.max_split_distance = split_dist_input_->text().toDouble();
  } catch (std::invalid_argument&) {
    cfg_.max_split_distance = 0.0;
    split_dist_input_->setText("0.0");
  }

  try {
    cfg_.max_merge_separation = merge_sep_input_->text().toDouble();
  } catch (std::invalid_argument&) {
    cfg_.max_merge_separation = 0.0;
    merge_sep_input_->setText("0.0");
  }

  try {
    cfg_.max_merge_spread = merge_spread_input_->text().toDouble();
  } catch (std::invalid_argument&) {
    cfg_.max_merge_spread = 0.0;
    merge_spread_input_->setText("0.0");
  }

  try {
    cfg_.max_circle_radius = max_radius_input_->text().toDouble();
  } catch (std::invalid_argument&) {
    cfg_.max_circle_radius = 0.0;
    max_radius_input_->setText("0.0");
  }
}

void ObstacleDetectPanel::updateUI() {
  use_split_merge_checkbox_->setChecked(cfg_.use_split_and_merge);
  circ_from_visib_checkbox_->setChecked(cfg_.detect_only_visible_circle);
  publish_circle_segments->setChecked(cfg_.publish_seg_of_circle);

  min_n_input_->setText(QString::number(cfg_.min_group_points));
  dist_prop_input_->setText(QString::number(cfg_.distance_proportion));
  group_dist_input_->setText(QString::number(cfg_.max_group_distance));
  split_dist_input_->setText(QString::number(cfg_.max_split_distance));
  merge_sep_input_->setText(QString::number(cfg_.max_merge_separation));
  merge_spread_input_->setText(QString::number(cfg_.max_merge_spread));
  max_radius_input_->setText(QString::number(cfg_.max_circle_radius));
}

void ObstacleDetectPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ObstacleDetectPanel::load(rviz::Config const& config) {
  rviz::Panel::load(config);
}
}  // namespace obstacles_display

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacles_display::ObstacleDetectPanel, rviz::Panel)
