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

#include <basalt/vi_estimator/ba_base.h>

#include <tbb/parallel_for.h>

namespace basalt {

/**
 * @brief Poseに関する相対的な何らかの変換を行う。 @todo
 * ここでの計算がどういった意味を持つのか明らかにする
 * @param T_w_i_h : IMU to World, Host Frame
 * @param T_i_c_h : Camera to IMU, Host Frame
 * @param T_w_i_t : IMU to World, Target Frame
 * @param T_i_c_t : Camera to IMU, Target Frame
 * @param d_rel_d_h : Adj(T_c_t_i_h) * diag[R_i_h_w, R_i_h_w] @todo
 * 式の意味を明らかにする
 * @param d_rel_d_t : -Adj(T_c_t_i_t) * diag[R_i_t_w, R_i_t_w] @todo
 * 式の意味を明らかにする
 * @return T_c_t_c_h : Camera,host to Camera,target
 */
Sophus::SE3d BundleAdjustmentBase::computeRelPose(const Sophus::SE3d &T_w_i_h,
                                                  const Sophus::SE3d &T_i_c_h,
                                                  const Sophus::SE3d &T_w_i_t,
                                                  const Sophus::SE3d &T_i_c_t,
                                                  Sophus::Matrix6d *d_rel_d_h,
                                                  Sophus::Matrix6d *d_rel_d_t) {
  //! Camera to IMU, Target Frame
  Sophus::SE3d tmp2 = (T_i_c_t).inverse();

  //! Transformation : IMU,host to IMU,target
  Sophus::SE3d T_t_i_h_i;
  T_t_i_h_i.so3() = T_w_i_t.so3().inverse() * T_w_i_h.so3();
  T_t_i_h_i.translation() =
      T_w_i_t.so3().inverse() * (T_w_i_h.translation() - T_w_i_t.translation());
//  T_t_i_h_i = T_w_i_t.inverse() * T_w_i_h;

  //! IMU,host to Camera,target
  Sophus::SE3d tmp = tmp2 * T_t_i_h_i;
  //! Camera,host to Camera,target
  Sophus::SE3d res = tmp * T_i_c_h;

  if (d_rel_d_h) {
    //! World to IMU,host
    Eigen::Matrix3d R = T_w_i_h.so3().inverse().matrix();

    Sophus::Matrix6d RR;
    RR.setZero();
    RR.topLeftCorner<3, 3>() = R;
    RR.bottomRightCorner<3, 3>() = R;

    // Method 1: Compute Ad(T_i_c_t^-1 T_w_i_t^-1 T_w_i_h) and remove the rotation(R_w_t_h).
    //! Adj(T_c_t_i_h) * diag[R_i_h_w, R_i_h_w]
//    *d_rel_d_h = tmp.Adj() * RR;

    // Method 2: Compute the full SE3 left jacobian and convert it to decoupled jacobian.
    Sophus::Matrix6d decoupled_conversion_T_w_i_h;
    decoupled_conversion_T_w_i_h.setIdentity();
    decoupled_conversion_T_w_i_h.topRightCorner<3,3>() << Sophus::SO3d::hat(T_w_i_h.translation());
    *d_rel_d_h = (T_i_c_t.inverse() * T_w_i_t.inverse()).Adj() * decoupled_conversion_T_w_i_h;
  }

  if (d_rel_d_t) {
    //! World to IMU,target
    Eigen::Matrix3d R = T_w_i_t.so3().inverse().matrix();

    Sophus::Matrix6d RR;
    RR.setZero();
    RR.topLeftCorner<3, 3>() = R;
    RR.bottomRightCorner<3, 3>() = R;

    // Method 1: Compute -Ad(T_i_c_t^-1) and add the rotation(R_w_i_t)
    //! -Adj(T_c_t_i_t) * diag[R_i_t_w, R_i_t_w]
//    *d_rel_d_t = -tmp2.Adj() * RR; // なぜRRなのか？T_w_i_t.inverse().Adj()ではないのか？

    // Method 2: Compute Ad(T_i_c_t^-1) * (-Ad(T_w_i_t^-1)) and remove the translation(t_w_i_t)
    // This method is the same way that compute full SE3 left jacobian and convert it to the decoupled left jacobian.
    Sophus::Matrix6d decoupled_conversion_T_w_i_t;
    decoupled_conversion_T_w_i_t.setIdentity();
    decoupled_conversion_T_w_i_t.topRightCorner<3, 3>() << Sophus::SO3d::hat(T_w_i_t.translation());
    *d_rel_d_t = -tmp2.Adj() * T_w_i_t.inverse().Adj() * decoupled_conversion_T_w_i_t;
  }

  return res;

  //@}
}

void BundleAdjustmentBase::updatePoints(const AbsOrderMap &aom,
                                        const RelLinData &rld,
                                        const Eigen::VectorXd &inc) {
  Eigen::VectorXd rel_inc;
  rel_inc.setZero(rld.order.size() * POSE_SIZE);
  for (size_t i = 0; i < rld.order.size(); i++) {
    const TimeCamId &tcid_h = rld.order[i].first;
    const TimeCamId &tcid_t = rld.order[i].second;

    if (tcid_h.frame_id != tcid_t.frame_id) {
      int abs_h_idx = aom.abs_order_map.at(tcid_h.frame_id).first;
      int abs_t_idx = aom.abs_order_map.at(tcid_t.frame_id).first;

      rel_inc.segment<POSE_SIZE>(i * POSE_SIZE) =
          rld.d_rel_d_h[i] * inc.segment<POSE_SIZE>(abs_h_idx) +
              rld.d_rel_d_t[i] * inc.segment<POSE_SIZE>(abs_t_idx);
    }
  }

  for (const auto &kv : rld.lm_to_obs) {
    int lm_idx = kv.first;
    const auto &other_obs = kv.second;

    Eigen::Vector3d H_l_p_x;
    H_l_p_x.setZero();

    for (size_t k = 0; k < other_obs.size(); k++) {
      int rel_idx = other_obs[k].first;
      const FrameRelLinData &frld_other = rld.Hpppl.at(rel_idx);

      Eigen::Matrix<double, 3, POSE_SIZE> H_l_p_other =
          frld_other.Hpl[other_obs[k].second].transpose();

      H_l_p_x += H_l_p_other * rel_inc.segment<POSE_SIZE>(rel_idx * POSE_SIZE);

      // std::cerr << "inc_p " << inc_p.transpose() << std::endl;
    }

    Eigen::Vector3d inc_p = rld.Hll.at(lm_idx) * (rld.bl.at(lm_idx) - H_l_p_x);

    KeypointPosition &kpt = lmdb.getLandmark(lm_idx);
    kpt.dir -= inc_p.head<2>();
    kpt.id -= inc_p[2];

    kpt.id = std::max(0., kpt.id);
  }
}

void BundleAdjustmentBase::computeError(
    double &error,
    std::map<int, std::vector<std::pair<TimeCamId, double>>> *outliers,
    double outlier_threshold) const {
  error = 0;

  for (const auto &kv : lmdb.getObservations()) {
    const TimeCamId &tcid_h = kv.first;

    for (const auto &obs_kv : kv.second) {
      const TimeCamId &tcid_t = obs_kv.first;

      if (tcid_h != tcid_t) {
        PoseStateWithLin state_h = getPoseStateWithLin(tcid_h.frame_id);
        PoseStateWithLin state_t = getPoseStateWithLin(tcid_t.frame_id);

        Sophus::SE3d T_t_h_sophus =
            computeRelPose(state_h.getPose(), calib.T_i_c[tcid_h.cam_id],
                           state_t.getPose(), calib.T_i_c[tcid_t.cam_id]);

        Eigen::Matrix4d T_t_h = T_t_h_sophus.matrix();

        std::visit(
            [&](const auto &cam) {
              for (size_t i = 0; i < obs_kv.second.size(); i++) {
                const KeypointObservation &kpt_obs = obs_kv.second[i];
                const KeypointPosition &kpt_pos =
                    lmdb.getLandmark(kpt_obs.kpt_id);

                Eigen::Vector2d res;

                bool valid = linearizePoint(kpt_obs, kpt_pos, T_t_h, cam, res);

                if (valid) {
                  double e = res.norm();

                  if (outliers && e > outlier_threshold) {
                    (*outliers)[kpt_obs.kpt_id].emplace_back(tcid_t, e);
                  }

                  double huber_weight =
                      e < huber_thresh ? 1.0 : huber_thresh / e;
                  double obs_weight =
                      huber_weight / (obs_std_dev * obs_std_dev);

                  error +=
                      (2 - huber_weight) * obs_weight * res.transpose() * res;
                } else {
                  if (outliers) {
                    (*outliers)[kpt_obs.kpt_id].emplace_back(tcid_t, -1);
                  }
                }
              }
            },
            calib.intrinsics[tcid_t.cam_id].variant);

      } else {
        // target and host are the same
        // residual does not depend on the pose
        // it just depends on the point

        std::visit(
            [&](const auto &cam) {
              for (size_t i = 0; i < obs_kv.second.size(); i++) {
                const KeypointObservation &kpt_obs = obs_kv.second[i];
                const KeypointPosition &kpt_pos =
                    lmdb.getLandmark(kpt_obs.kpt_id);

                Eigen::Vector2d res;

                bool valid = linearizePoint(kpt_obs, kpt_pos, cam, res);
                if (valid) {
                  double e = res.norm();

                  if (outliers && e > outlier_threshold) {
                    (*outliers)[kpt_obs.kpt_id].emplace_back(tcid_t, -2);
                  }

                  double huber_weight =
                      e < huber_thresh ? 1.0 : huber_thresh / e;
                  double obs_weight =
                      huber_weight / (obs_std_dev * obs_std_dev);

                  error +=
                      (2 - huber_weight) * obs_weight * res.transpose() * res;
                } else {
                  if (outliers) {
                    (*outliers)[kpt_obs.kpt_id].emplace_back(tcid_t, -2);
                  }
                }
              }
            },
            calib.intrinsics[tcid_t.cam_id].variant);
      }
    }
  }
}

/**
 * @berif
 * 何かしらの線形化、つまりJacobianかHessianを計算するのだろうが、具体的な計算過程、意味は不明
 * @param rld_vec : 線形化結果の出力
 * @param obs_to_lin : 線形化を実施するランドマークの観測状況
 * @param error : おそらく線形化誤差かなにかがリターンされるはず
 */
void BundleAdjustmentBase::linearizeHelper(
    Eigen::aligned_vector<RelLinData> &rld_vec,
    const Eigen::aligned_map<TimeCamId,
                             Eigen::aligned_map<TimeCamId,
                                                Eigen::aligned_vector<KeypointObservation>
                             >
                             > &obs_to_lin,
    double &error
) const {
  error = 0;

  rld_vec.clear();

  // HostFrameのFrameIDが保存されている。
  // obs_to_linの i 番目のKey(HostFrame)は、osb_tcid_vec[i]で参照できる。
  std::vector<TimeCamId> obs_tcid_vec;
  for (const auto &kv : obs_to_lin) {
    obs_tcid_vec.emplace_back(
        //! kv.first : KeyPointを保持しているKeyFrame(HostFrame)のFrameID
        kv.first);

    rld_vec.emplace_back(
        //! Sliding windowに入っているLandmarkの数
        lmdb.numLandmarks(),
        //! 自分も含めてHostしているKeyPointのどれか一つでも観測された回数（Poseの個数）
        //! kv.second : kv.firstで初めて観測したLMをもう一度観測したFrameID（自分も含む）と、観測KeyPoint位置のリストのMap
        kv.second.size());
  }

  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, obs_tcid_vec.size()),
      [&](const tbb::blocked_range<size_t> &range) {

        //! HostFrameのLoop
        for (size_t r = range.begin(); r != range.end(); ++r) {

          //! 対象のKeyFrameについて<FrameID, [KeyPoint List]>のMap
          auto kv = obs_to_lin.find(
              //! LMをHostしているKeyFrameのID
              obs_tcid_vec[r]);

          //! r番目のKeyFrameについての情報を保存するためのコンテナへの参照
          RelLinData &rld = rld_vec[r];  // RelLinDataはFrameごとに計算される

          rld.error = 0;

          //! 基準Frame, obs_tcid_vec[r]の参照するFrameになる
          //! tcid_h(host)
          const TimeCamId &tcid_h = kv->first;

          //! TargetFrameのLoop
          for (const auto &obs_kv :
            //! <観測したFrame, [KeyPoints list]>のMap
              kv->second) {

            //! tcid_hでHostしているLMを観測したFrame
            //! tcid_t(target)
            const TimeCamId &tcid_t = obs_kv.first;

            //! Target frameとHost frameが違う場合
            // target and host are not the same
            if (tcid_h != tcid_t) {

              //! Host frameの`RelLinData`に <host frame id, target frame
              //! id>のペアを追加
              rld.order.emplace_back(std::make_pair(tcid_h, tcid_t));

              //! 初めにPoseDBから、見つからなければStateDB(Velocity, IMU
              //! biasなど含まれる)からFrameStateを探索する。
              //! もしどちらにもなければstd::abort
              /**
               * @brief Core of `PoseStateWithLin`
               * @details
               * ``` c++
               *    bool linearized;
                    VecN delta; // 線形化系
                    PoseState pose_linearized; // 線形化系
                    Sophus::SE3d T_w_i_current; // IMU to World のPose
                 ```
               *
               */
              //! Host Frameの状態
              PoseStateWithLin state_h = getPoseStateWithLin(tcid_h.frame_id);
              //! Target Frameの状態
              PoseStateWithLin state_t = getPoseStateWithLin(tcid_t.frame_id);

              //! Adj(T_c_t_i_h) diag(R_i_h_w, R_i_h_w)
              Sophus::Matrix6d d_rel_d_h;
              //! -Adj(T_c_t_i_t) diag(R_i_t_w, R_i_t_w)
              Sophus::Matrix6d d_rel_d_t;

              //! Transformation for Camera,host to Camera,target
              Sophus::SE3d T_t_h_sophus = computeRelPose(
                  state_h.getPoseLin(), calib.T_i_c[tcid_h.cam_id],
                  state_t.getPoseLin(), calib.T_i_c[tcid_t.cam_id], &d_rel_d_h,
                  &d_rel_d_t);

              //! computeRelPoseでの計算結果をrld(rld_vecにおける今回のカメラ分)を追加
              //! Poseに関する情報のはず、微分すると出現するT~なんちゃらだとは思うが…
              rld.d_rel_d_h.emplace_back(d_rel_d_h);
              rld.d_rel_d_t.emplace_back(d_rel_d_t);

              if (state_h.isLinearized() || state_t.isLinearized()) {
                T_t_h_sophus = computeRelPose(
                    state_h.getPose(), calib.T_i_c[tcid_h.cam_id],
                    state_t.getPose(), calib.T_i_c[tcid_t.cam_id]);
              }

              //! Transformation for Camera,host to Cmaera,target
              Eigen::Matrix4d T_t_h = T_t_h_sophus.matrix();

              //! HostFrameで観測されているLandmarkとTargetFrameに関する、Landmark to TargetFramePose, TargetFramePose to TargetFramePoseのHessianとGradientを保持する
              FrameRelLinData frld;

              //! CameraModelごとの処理の分岐がここでstd::variantとstd::visitを使って実装されている。
              std::visit(
                  [&](const auto &cam) {

                    //! TargetFrameで観測されているLMのLoop
                    for (size_t i = 0; i < obs_kv.second.size(); i++) {
                      //! <観測したFrame, [KeyPoints list]>のPair
                      //! obs_kv.first : TargetFrame
                      //! obs_kv.second : TargetFrameが観測したKeyPointのList

                      //! TargetFrameが観測したi番目のKeyPoint
                      const KeypointObservation &kpt_obs = obs_kv.second[i];
                      //! KeyPoint i
                      //! の位置で、おそらくHost座標系になっているはず
                      const KeypointPosition &kpt_pos =
                          lmdb.getLandmark(kpt_obs.kpt_id);

                      //! Residual, Reprojeciton error
                      Eigen::Vector2d res;
                      //! Reprojection error と pose i(x_i)のJacobian
                      Eigen::Matrix<double, 2, POSE_SIZE> d_res_d_xi;
                      //! Reprojection errorとKeyPoint位置のJacobian
                      Eigen::Matrix<double, 2, 3> d_res_d_p;

                      //! KeyPoint i に関するJacobianを計算する
                      bool valid = linearizePoint(
                          //! Keypoint i の観測情報
                          kpt_obs,
                          //! Keypoint i の位置
                          kpt_pos,
                          //! Camera,host to Camera,target
                          T_t_h,
                          //! カメラモデル、カメラパラメータ
                          cam,
                          //! Residual, Reprojeciton error
                          res,
                          //! Reprojection error と pose(xiはグザイ、[vx, vy, vz, wx, wy, wz])のJacobian
                          &d_res_d_xi,
                          //! Reprojection errorとKeyPoint位置のJacobian
                          &d_res_d_p
                      );

                      if (valid) {  //! Invalid
                        //! projectionとかが発生しなければ？？？

                        //! KeyPointについてロバスト化？したコスト関数適用？Weightを計算
                        double e = res.norm();
                        double huber_weight =
                            e < huber_thresh ? 1.0 : huber_thresh / e;
                        double obs_weight =
                            huber_weight / (obs_std_dev * obs_std_dev);

                        rld.error += (2 - huber_weight) * obs_weight *
                            res.transpose() * res;

                        //! クリアされていなければHessianとbをzeroクリアする
                        if (rld.Hll.count(kpt_obs.kpt_id) == 0) {
                          rld.Hll[kpt_obs.kpt_id].setZero();
                          rld.bl[kpt_obs.kpt_id].setZero();
                        }

                        //! rld : HostFrameごと
                        //! frld : HostFrameごと

                        //! Landmark to Landmark のHessian(Hll)を計算
                        rld.Hll[kpt_obs.kpt_id] +=
                            obs_weight * d_res_d_p.transpose() * d_res_d_p;
                        //! Landmark部分のGradient(bl)を計算
                        rld.bl[kpt_obs.kpt_id] +=
                            obs_weight * d_res_d_p.transpose() * res;

                        //! FrameRelLinDataはConstructorでメンバがZeroクリアされるので、rldのように初期化は不要
                        //! Pose to Pose のHessianに加算？
                        frld.Hpp +=
                            obs_weight * d_res_d_xi.transpose() * d_res_d_xi;
                        //! Pose部分のGradientに加算？
                        frld.bp += obs_weight * d_res_d_xi.transpose() * res;
                        //! Pose to LandmarkのHessian(Hpl)に加算
                        frld.Hpl.emplace_back(obs_weight * d_res_d_xi.transpose() * d_res_d_p);
                        //! Pose to LandmarkのHessianとLandmark idの対応をとるためIDをvectorに保存？
                        frld.lm_id.emplace_back(kpt_obs.kpt_id);

                        //! Keypoint ID, [<Hpplにおける今回のTargetFrameのIndex, Hppl(frld)におけるLMのIndex>]のマップを追加する
                        //! これによって、Keypoint IDから、Hpplに保存されているHpp, bp, Hplにアクセスできる。
                        //! より具体的にいうと、KwyPoint IDから、HpplにおけるTargetFrameのスタックインデックスと、Hppl[stack index].Hplにおける、今回のLMのスタックインデクスがわかる。
                        //! ここもマップにしても良かったのではないかを個人的に思う。今後の処理で都合がいいのかもしれない。
                        //! Hppl[rld.lm_to_obs[Keypoint ID].first].Hpl[rld.lm_to_obs[Keypoint ID].second]のような使い方になるはず。
                        rld.lm_to_obs[kpt_obs.kpt_id].emplace_back(
                            //! HsotFrameにおける今回のTargetFrameのIndex
                            rld.Hpppl.size(),
                            //! 今回のTargetFrameにおける`kpt_obs.kpt_id`のLMのIndex
                            frld.lm_id.size() - 1);
                      }
                    }
                  },
                  calib.intrinsics[tcid_t.cam_id].variant);

              /**
               * @brief まとめると…
               * @details
               * - rld : HostFrameに紐づく
               * - frld : HostFrame, TargetFrameの組み合わせに紐づく
               *
               * - rld::error : WeightをかけたReprojection
               * errorをTargetFrameで観測したLM文すべて加算
               * - rld::Hll : HostFrame ID, Keypoint IDの組み合わせ,
               * TargetFrameごとにHessianを計算、Keypoint IDごとに計算結果を加算
               * - rld::bl : HostFrame ID, Keypoint IDの組み合わせ,
               * TargetFrameごとにGradientを計算、Keypoint
               * IDごとに計算結果を加算
               * - frld::Hpp : HostFrame ID, TargetFrame IDの組み合わせ,
               * Host,TargetのIntersectionLMについてJacobian全てを加算
               * - frld::bp : HostFrame ID, TargetFrame IDの組み合わせ,
               * Host,TargetのIntersectionLMについてJacobian全てを加算
               * - frld::Hpl : HostFrame ID, TargetFrame ID, Keypoint
               * IDの組わせでスタックされる
               */

              /**
               * @brief　あとやること
               * @todo HostFrameの座標系で計算しているはず、式としてどういった形になっているか明らかにする
               * @todo Hessian関係で加算処理が見られる。ここが J^T @ J = H の計算とどういったように対応しているか明らかにする
               * @todo ここでの計算について計算式として書き下す
               */

              //! rldにPose to Pose, Pose to Landmarkの情報を追加
              rld.Hpppl.emplace_back(frld);

            } else {
              // target and host are the same
              // residual does not depend on the pose
              // it just depends on the point

              std::visit(
                  [&](const auto &cam) {
                    for (size_t i = 0; i < obs_kv.second.size(); i++) {
                      const KeypointObservation &kpt_obs = obs_kv.second[i];
                      const KeypointPosition &kpt_pos =
                          lmdb.getLandmark(kpt_obs.kpt_id);

                      Eigen::Vector2d res;
                      Eigen::Matrix<double, 2, 3> d_res_d_p;

                      bool valid = linearizePoint(kpt_obs, kpt_pos, cam, res,
                                                  &d_res_d_p);

                      if (valid) {
                        double e = res.norm();
                        double huber_weight =
                            e < huber_thresh ? 1.0 : huber_thresh / e;
                        double obs_weight =
                            huber_weight / (obs_std_dev * obs_std_dev);

                        rld.error += (2 - huber_weight) * obs_weight *
                            res.transpose() * res;

                        if (rld.Hll.count(kpt_obs.kpt_id) == 0) {
                          rld.Hll[kpt_obs.kpt_id].setZero();
                          rld.bl[kpt_obs.kpt_id].setZero();
                        }

                        rld.Hll[kpt_obs.kpt_id] +=
                            obs_weight * d_res_d_p.transpose() * d_res_d_p;
                        rld.bl[kpt_obs.kpt_id] +=
                            obs_weight * d_res_d_p.transpose() * res;
                      }
                    }
                  },
                  calib.intrinsics[tcid_t.cam_id].variant);
            }
          }
        }
      });

  for (const auto &rld : rld_vec) error += rld.error;
}

/**
 * @brief おそらく、HostFrame座標系におけるPoseにMarginalizeされたH, bを計算する。H, bの構成順は`rld.order`の配列順になる。
 * @details
 * ここで計算されるH, bはMarginalizeされてたあとのH, bであることに注意。
 * [Hpp, Hpl; Hpl^T, Hll] [xi_p; xi_l] = [b_p; b_l]
 * を元のシステムとして、以下のようにMarginalizeした結果が、H, bになる
 * (Hpp - Hpl Hll^-1 Hpl^T) xi_p = b_p - Hpl Hll^-1 b_l
 * なので、
 * H = Hpp - Hpl Hll^-1 Hpl^T
 * b = b_p - Hpl Hll^-1 b_l
 * が、ここでの @param H, @param b になる。
 * @param rld : TargetFrame、Landmark個々の計算済みHessianが格納されている。
 * @param H : おそらく、HostFrame座標系におけるHessianの計算結果
 * @param b : おそらく、HostFrame座標系におけるGradientの計算結果
 */
void BundleAdjustmentBase::linearizeRel(const RelLinData &rld,
                                        Eigen::MatrixXd &H,
                                        Eigen::VectorXd &b) {
  //  std::cout << "linearizeRel: KF " << frame_states.size() << " obs "
  //            << obs.size() << std::endl;

  //! Do schur complement

  //! HostFrameに含まれるTargetFrameの数
  size_t msize = rld.order.size();
  H.setZero(POSE_SIZE * msize, POSE_SIZE * msize);
  b.setZero(POSE_SIZE * msize);

  //! TargetFrameごとのLoop
  for (size_t i = 0; i < rld.order.size(); i++) {
    const FrameRelLinData &frld = rld.Hpppl.at(i);

    /**
     * Hpp, bpの成分は、HostFrameと一つのTargetFrameの組み合わせで決定されている。
     */
    H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * i) += frld.Hpp;
    b.segment<POSE_SIZE>(POSE_SIZE * i) += frld.bp;

    //! HostFrameでHostしたLMのうち、TargetFrameで観測しているLM分のLoop。
    for (size_t j = 0; j < frld.lm_id.size(); j++) {
      Eigen::Matrix<double, POSE_SIZE, 3> H_pl_H_ll_inv;
      int lm_id = frld.lm_id[j];

      /**
       * @details
       * H_pl_H_ll_inv == Hpl Hll^-1
       * Hpl Hll^-1の計算は、一つのTargetFrameと一つのLandmarkの情報のみから計算できる。
       * Hllの対角成分（正確には対角ではなく、LL成分？）以外ゼロなので、Hpl Hll^-1を計算すると、
       * 注目しているLM idのみしか残らないため。
       */
      H_pl_H_ll_inv = frld.Hpl[j] * rld.Hll.at(lm_id);
      //! b == b_p - Hpl Hll^-1 b_l, この計算も同様に一つのTargetFrameとLandmarkのペアから計算できる
      b.segment<POSE_SIZE>(POSE_SIZE * i) -= H_pl_H_ll_inv * rld.bl.at(lm_id);

      const auto &other_obs = rld.lm_to_obs.at(lm_id);
      for (size_t k = 0; k < other_obs.size(); k++) {
        const FrameRelLinData &frld_other = rld.Hpppl.at(other_obs[k].first);
        int other_i = other_obs[k].first;

        //! Hpl Hll^-1 Hpl^T はTargetFrameで観測している全てのLMの情報が必要になる。
        //! Hpl Hll^-1はブロック的には対角行列になっているので、後ろからかかる行列全ての情報が必要になる。
        Eigen::Matrix<double, 3, POSE_SIZE> H_l_p_other =
            frld_other.Hpl[other_obs[k].second].transpose();

        H.block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * other_i) -=
            H_pl_H_ll_inv * H_l_p_other;
      }
    }
  }
}

void BundleAdjustmentBase::get_current_points(
    Eigen::aligned_vector<Eigen::Vector3d> &points,
    std::vector<int> &ids) const {
  points.clear();
  ids.clear();

  for (const auto &tcid_host : lmdb.getHostKfs()) {
    Sophus::SE3d T_w_i;

    int64_t id = tcid_host.frame_id;
    if (frame_states.count(id) > 0) {
      PoseVelBiasStateWithLin state = frame_states.at(id);
      T_w_i = state.getState().T_w_i;
    } else if (frame_poses.count(id) > 0) {
      PoseStateWithLin state = frame_poses.at(id);

      T_w_i = state.getPose();
    } else {
      std::cout << "Unknown frame id: " << id << std::endl;
      std::abort();
    }

    const Sophus::SE3d &T_i_c = calib.T_i_c[tcid_host.cam_id];
    Eigen::Matrix4d T_w_c = (T_w_i * T_i_c).matrix();

    for (const KeypointPosition &kpt_pos :
        lmdb.getLandmarksForHost(tcid_host)) {
      Eigen::Vector4d pt_cam =
          StereographicParam<double>::unproject(kpt_pos.dir);
      pt_cam[3] = kpt_pos.id;

      Eigen::Vector4d pt_w = T_w_c * pt_cam;

      points.emplace_back(pt_w.head<3>() / pt_w[3]);
      ids.emplace_back(1);
    }
  }
}

void BundleAdjustmentBase::filterOutliers(double outlier_threshold,
                                          int min_num_obs) {
  double error;
  std::map<int, std::vector<std::pair<TimeCamId, double>>> outliers;
  computeError(error, &outliers, outlier_threshold);

  //  std::cout << "============================================" <<
  //  std::endl; std::cout << "Num landmarks: " << lmdb.numLandmarks() << "
  //  with outliners
  //  "
  //            << outliers.size() << std::endl;

  for (const auto &kv : outliers) {
    int num_obs = lmdb.numObservations(kv.first);
    int num_outliers = kv.second.size();

    bool remove = false;

    if (num_obs - num_outliers < min_num_obs) remove = true;

    //    std::cout << "\tlm_id: " << kv.first << " num_obs: " << num_obs
    //              << " outliers: " << num_outliers << " [";

    for (const auto &kv2 : kv.second) {
      if (kv2.second == -2) remove = true;

      //      std::cout << kv2.second << ", ";
    }

    //    std::cout << "] " << std::endl;

    if (remove) {
      lmdb.removeLandmark(kv.first);
    } else {
      std::set<TimeCamId> outliers;
      for (const auto &kv2 : kv.second) outliers.emplace(kv2.first);
      lmdb.removeObservations(kv.first, outliers);
    }
  }

  // std::cout << "============================================" <<
  // std::endl;
}

void BundleAdjustmentBase::marginalizeHelper(Eigen::MatrixXd &abs_H,
                                             Eigen::VectorXd &abs_b,
                                             const std::set<int> &idx_to_keep,
                                             const std::set<int> &idx_to_marg,
                                             Eigen::MatrixXd &marg_H,
                                             Eigen::VectorXd &marg_b) {
  int keep_size = idx_to_keep.size();
  int marg_size = idx_to_marg.size();

  BASALT_ASSERT(keep_size + marg_size == abs_H.cols());

  // Fill permutation matrix
  Eigen::Matrix<int, Eigen::Dynamic, 1> indices(idx_to_keep.size() +
      idx_to_marg.size());

  {
    auto it = idx_to_keep.begin();
    for (size_t i = 0; i < idx_to_keep.size(); i++) {
      indices[i] = *it;
      it++;
    }
  }

  {
    auto it = idx_to_marg.begin();
    for (size_t i = 0; i < idx_to_marg.size(); i++) {
      indices[idx_to_keep.size() + i] = *it;
      it++;
    }
  }

  const Eigen::PermutationWrapper<Eigen::Matrix<int, Eigen::Dynamic, 1>> p(
      indices);

  const Eigen::PermutationMatrix<Eigen::Dynamic> pt = p.transpose();

  abs_b.applyOnTheLeft(pt);
  abs_H.applyOnTheLeft(pt);
  abs_H.applyOnTheRight(p);

  Eigen::MatrixXd H_mm_inv;
  //  H_mm_inv.setIdentity(marg_size, marg_size);
  //  abs_H.bottomRightCorner(marg_size,
  //  marg_size).ldlt().solveInPlace(H_mm_inv);

  H_mm_inv = abs_H.bottomRightCorner(marg_size, marg_size)
      .fullPivLu()
      .solve(Eigen::MatrixXd::Identity(marg_size, marg_size));

  //  H_mm_inv = abs_H.bottomRightCorner(marg_size, marg_size)
  //                 .fullPivHouseholderQr()
  //                 .solve(Eigen::MatrixXd::Identity(marg_size,
  //                 marg_size));

  abs_H.topRightCorner(keep_size, marg_size) *= H_mm_inv;

  marg_H = abs_H.topLeftCorner(keep_size, keep_size);
  marg_b = abs_b.head(keep_size);

  marg_H -= abs_H.topRightCorner(keep_size, marg_size) *
      abs_H.bottomLeftCorner(marg_size, keep_size);
  marg_b -= abs_H.topRightCorner(keep_size, marg_size) * abs_b.tail(marg_size);

  abs_H.resize(0, 0);
  abs_b.resize(0);
}

void BundleAdjustmentBase::computeDelta(const AbsOrderMap &marg_order,
                                        Eigen::VectorXd &delta) const {
  size_t marg_size = marg_order.total_size;
  delta.setZero(marg_size);
  for (const auto &kv : marg_order.abs_order_map) {
    if (kv.second.second == POSE_SIZE) {
      BASALT_ASSERT(frame_poses.at(kv.first).isLinearized());
      delta.segment<POSE_SIZE>(kv.second.first) =
          frame_poses.at(kv.first).getDelta();
    } else if (kv.second.second == POSE_VEL_BIAS_SIZE) {
      BASALT_ASSERT(frame_states.at(kv.first).isLinearized());
      delta.segment<POSE_VEL_BIAS_SIZE>(kv.second.first) =
          frame_states.at(kv.first).getDelta();
    } else {
      BASALT_ASSERT(false);
    }
  }
}

void BundleAdjustmentBase::linearizeMargPrior(const AbsOrderMap &marg_order,
                                              const Eigen::MatrixXd &marg_H,
                                              const Eigen::VectorXd &marg_b,
                                              const AbsOrderMap &aom,
                                              Eigen::MatrixXd &abs_H,
                                              Eigen::VectorXd &abs_b,
                                              double &marg_prior_error) const {
  // Assumed to be in the top left corner

  BASALT_ASSERT(size_t(marg_H.cols()) == marg_order.total_size);

  // Check if the order of variables is the same.
  for (const auto &kv : marg_order.abs_order_map)
    BASALT_ASSERT(aom.abs_order_map.at(kv.first) == kv.second);

  size_t marg_size = marg_order.total_size;
  abs_H.topLeftCorner(marg_size, marg_size) += marg_H;

  Eigen::VectorXd delta;
  computeDelta(marg_order, delta);

  abs_b.head(marg_size) += marg_b;
  abs_b.head(marg_size) += marg_H * delta;

  marg_prior_error = 0.5 * delta.transpose() * marg_H * delta;
  marg_prior_error += delta.transpose() * marg_b;
}

void BundleAdjustmentBase::computeMargPriorError(
    const AbsOrderMap &marg_order, const Eigen::MatrixXd &marg_H,
    const Eigen::VectorXd &marg_b, double &marg_prior_error) const {
  BASALT_ASSERT(size_t(marg_H.cols()) == marg_order.total_size);

  Eigen::VectorXd delta;
  computeDelta(marg_order, delta);

  marg_prior_error = 0.5 * delta.transpose() * marg_H * delta;
  marg_prior_error += delta.transpose() * marg_b;
}

Eigen::VectorXd BundleAdjustmentBase::checkNullspace(
    const Eigen::MatrixXd &H, const Eigen::VectorXd &b,
    const AbsOrderMap &order,
    const Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin> &frame_states,
    const Eigen::aligned_map<int64_t, PoseStateWithLin> &frame_poses) {
  BASALT_ASSERT(size_t(H.cols()) == order.total_size);
  size_t marg_size = order.total_size;

  Eigen::VectorXd inc_x, inc_y, inc_z, inc_roll, inc_pitch, inc_yaw;
  inc_x.setZero(marg_size);
  inc_y.setZero(marg_size);
  inc_z.setZero(marg_size);
  inc_roll.setZero(marg_size);
  inc_pitch.setZero(marg_size);
  inc_yaw.setZero(marg_size);

  int num_trans = 0;
  Eigen::Vector3d mean_trans;
  mean_trans.setZero();

  // Compute mean translation
  for (const auto &kv : order.abs_order_map) {
    Eigen::Vector3d trans;
    if (kv.second.second == POSE_SIZE) {
      mean_trans += frame_poses.at(kv.first).getPoseLin().translation();
      num_trans++;
    } else if (kv.second.second == POSE_VEL_BIAS_SIZE) {
      mean_trans += frame_states.at(kv.first).getStateLin().T_w_i.translation();
      num_trans++;
    } else {
      std::cerr << "Unknown size of the state: " << kv.second.second
                << std::endl;
      std::abort();
    }
  }
  mean_trans /= num_trans;

  double eps = 0.01;

  // Compute nullspace increments
  for (const auto &kv : order.abs_order_map) {
    inc_x(kv.second.first + 0) = eps;
    inc_y(kv.second.first + 1) = eps;
    inc_z(kv.second.first + 2) = eps;
    inc_roll(kv.second.first + 3) = eps;
    inc_pitch(kv.second.first + 4) = eps;
    inc_yaw(kv.second.first + 5) = eps;

    Eigen::Vector3d trans;
    if (kv.second.second == POSE_SIZE) {
      trans = frame_poses.at(kv.first).getPoseLin().translation();
    } else if (kv.second.second == POSE_VEL_BIAS_SIZE) {
      trans = frame_states.at(kv.first).getStateLin().T_w_i.translation();
    } else {
      BASALT_ASSERT(false);
    }

    trans -= mean_trans;

    Eigen::Matrix3d J = -Sophus::SO3d::hat(trans);
    J *= eps;

    inc_roll.segment<3>(kv.second.first) = J.col(0);
    inc_pitch.segment<3>(kv.second.first) = J.col(1);
    inc_yaw.segment<3>(kv.second.first) = J.col(2);

    if (kv.second.second == POSE_VEL_BIAS_SIZE) {
      Eigen::Vector3d vel = frame_states.at(kv.first).getStateLin().vel_w_i;
      Eigen::Matrix3d J_vel = -Sophus::SO3d::hat(vel);
      J_vel *= eps;

      inc_roll.segment<3>(kv.second.first + POSE_SIZE) = J_vel.col(0);
      inc_pitch.segment<3>(kv.second.first + POSE_SIZE) = J_vel.col(1);
      inc_yaw.segment<3>(kv.second.first + POSE_SIZE) = J_vel.col(2);
    }
  }

  inc_x.normalize();
  inc_y.normalize();
  inc_z.normalize();
  inc_roll.normalize();
  inc_pitch.normalize();
  inc_yaw.normalize();

  //  std::cout << "inc_x   " << inc_x.transpose() << std::endl;
  //  std::cout << "inc_y   " << inc_y.transpose() << std::endl;
  //  std::cout << "inc_z   " << inc_z.transpose() << std::endl;
  //  std::cout << "inc_yaw " << inc_yaw.transpose() << std::endl;

  Eigen::VectorXd inc_random;
  inc_random.setRandom(marg_size);
  inc_random.normalize();

  Eigen::VectorXd xHx(7), xb(7);
  xHx[0] = 0.5 * inc_x.transpose() * H * inc_x;
  xHx[1] = 0.5 * inc_y.transpose() * H * inc_y;
  xHx[2] = 0.5 * inc_z.transpose() * H * inc_z;
  xHx[3] = 0.5 * inc_roll.transpose() * H * inc_roll;
  xHx[4] = 0.5 * inc_pitch.transpose() * H * inc_pitch;
  xHx[5] = 0.5 * inc_yaw.transpose() * H * inc_yaw;
  xHx[6] = 0.5 * inc_random.transpose() * H * inc_random;

  xb[0] = inc_x.transpose() * b;
  xb[1] = inc_y.transpose() * b;
  xb[2] = inc_z.transpose() * b;
  xb[3] = inc_roll.transpose() * b;
  xb[4] = inc_pitch.transpose() * b;
  xb[5] = inc_yaw.transpose() * b;
  xb[6] = inc_random.transpose() * b;

  std::cout << "nullspace x_trans: " << xHx[0] << " + " << xb[0] << std::endl;
  std::cout << "nullspace y_trans: " << xHx[1] << " + " << xb[1] << std::endl;
  std::cout << "nullspace z_trans: " << xHx[2] << " + " << xb[2] << std::endl;
  std::cout << "nullspace roll   : " << xHx[3] << " + " << xb[3] << std::endl;
  std::cout << "nullspace pitch  : " << xHx[4] << " + " << xb[4] << std::endl;
  std::cout << "nullspace yaw    : " << xHx[5] << " + " << xb[5] << std::endl;
  std::cout << "nullspace random : " << xHx[6] << " + " << xb[6] << std::endl;

  return xHx + xb;
}

}  // namespace basalt
