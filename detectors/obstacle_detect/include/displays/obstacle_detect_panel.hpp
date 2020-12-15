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

#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <dynamic_reconfigure/client.h>
#include "obstacle_detect/obstacleDetectConfig.h"

#include <QLabel>
#include <QFrame>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>

#include <mutex>

namespace obstacles_display {

class ObstacleDetectPanel : public rviz::Panel {
  Q_OBJECT
 public:
  ObstacleDetectPanel(QWidget* parent = 0);

  virtual void load(rviz::Config const& config);
  virtual void save(rviz::Config config) const;

 private Q_SLOTS:
  void handleInputs();

 private:
  void initParams();
  void getRvizInputs();
  void updateUI();

 private:
  QCheckBox* use_split_merge_checkbox_;
  QCheckBox* circ_from_visib_checkbox_;
  QCheckBox* publish_circle_segments;

  QLineEdit* min_n_input_;
  QLineEdit* dist_prop_input_;
  QLineEdit* group_dist_input_;
  QLineEdit* split_dist_input_;
  QLineEdit* merge_sep_input_;
  QLineEdit* merge_spread_input_;
  QLineEdit* max_radius_input_;

  ros::NodeHandle pnh_;
  std::string frameId_;
  bool isUpdateUI = false;

  using cfg_t = obstacle_detect::obstacleDetectConfig;
  cfg_t cfg_;
  dynamic_reconfigure::Client<cfg_t> drClt_;
};

}  // namespace obstacles_display
