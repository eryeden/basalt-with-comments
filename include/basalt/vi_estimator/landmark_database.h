/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include <basalt/utils/imu_types.h>

namespace basalt {

// keypoint position defined relative to some frame
struct KeypointPosition {

  //! keyFrame ID
  TimeCamId kf_id;

  //! Direction to the Landmark position
  Eigen::Vector2d dir;
  //! Inverse depth, 多分…
  double id;

  inline void backup() {
    backup_dir = dir;
    backup_id = id;
  }

  inline void restore() {
    dir = backup_dir;
    id = backup_id;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Vector2d backup_dir;
  double backup_id;
};

struct KeypointObservation {
  int kpt_id;
  Eigen::Vector2d pos;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class LandmarkDatabase {
 public:
  // Non-const
  void addLandmark(int lm_id, const KeypointPosition& pos);

  void removeFrame(const FrameId& frame);

  void removeKeyframes(const std::set<FrameId>& kfs_to_marg,
                       const std::set<FrameId>& poses_to_marg,
                       const std::set<FrameId>& states_to_marg_all);

  void addObservation(const TimeCamId& tcid_target,
                      const KeypointObservation& o);

  KeypointPosition& getLandmark(int lm_id);

  // Const
  const KeypointPosition& getLandmark(int lm_id) const;

  std::vector<TimeCamId> getHostKfs() const;

  std::vector<KeypointPosition> getLandmarksForHost(
      const TimeCamId& tcid) const;

  const Eigen::aligned_map<
      TimeCamId, Eigen::aligned_map<
                     TimeCamId, Eigen::aligned_vector<KeypointObservation>>>&
  getObservations() const;

  bool landmarkExists(int lm_id) const;

  size_t numLandmarks() const;

  int numObservations() const;

  int numObservations(int lm_id) const;

  void removeLandmark(int lm_id);

  void removeObservations(int lm_id, const std::set<TimeCamId>& obs);

  inline void backup() {
    for (auto& kv : kpts) kv.second.backup();
  }

  inline void restore() {
    for (auto& kv : kpts) kv.second.restore();
  }

 private:

  //! @brief <KeyPoint ID(int), Landmarkの推定位置？>
  //! @details KeyPointIDはINTになっている。
  Eigen::aligned_unordered_map<int, KeypointPosition> kpts;

  /**
   * @brief <CameraFrameID, <CameraFrameID, KeyPointPosition and KeyPointID>>
   * @details
   * Landmarkは初めて観測されたKeyFrameのカメラ座標系で表現されて保持される。
   * このことから、このMapは以下の対応だと予想される。
   * <LMHostingCameraFrameID, <LMHostingCameraFrameIDで初めて観測されたLMを観測したCameraFrameID, [観測したKeypointの位置情報]>>
   */
  Eigen::aligned_map<
      TimeCamId,
      Eigen::aligned_map<TimeCamId, Eigen::aligned_vector<KeypointObservation>>>
      obs;

  //! @brief <CameraFrameID, [Observing keypoint id]>
  //! @details <(カメラID,FrameID), [観測しているKeyPointIDのリスト？]>という感じのMap？
  std::unordered_map<TimeCamId, std::set<int>> host_to_kpts;

  int num_observations = 0;
  //! @brief <KeyPointID, Number of observation?>
  //! @details <KeyPointID, [KeyPointID]のKeyPointが観測された回数？>という感じのMap？
  Eigen::aligned_unordered_map<int, int> kpts_num_obs;
};

}  // namespace basalt
