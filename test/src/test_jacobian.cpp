//
// Created by ery on 2020/10/24.
//

#include <basalt/spline/se3_spline.h>
#include <basalt/vi_estimator/keypoint_vio.h>

#include <iostream>

#include "gtest/gtest.h"
#include "test_utils.h"




TEST(JacobianTestSuite, d_MaMb_d_MaTest) {
    /*
     * - se3_expd : Decoupled SE3 exponentiation
     * - SE3d::exp : Coupled SE3 exponentiation
     */

    Sophus::SE3d Ma = Sophus::SE3d::exp(Sophus::Vector6d::Random());
    Sophus::SE3d Mb = Sophus::SE3d::exp(Sophus::Vector6d::Random());


    Sophus::SE3d MaMb = Ma*Mb;
    Sophus::Matrix6d d_MaMb_d_Ma;

    // SE3 left incrementの場合、Jacobianは単位行列。
    d_MaMb_d_Ma.setIdentity();
//    d_MaMb_d_Ma.setZero();
    d_MaMb_d_Ma.topRightCorner<3,3>() << Sophus::SO3d::hat(Ma.translation());

    {
        Sophus::Vector6d x0;
        x0.setZero();
        test_jacobian(
                "d_MaMb_d_Ma", d_MaMb_d_Ma,
                [&](const Sophus::Vector6d& x) {
                    Sophus::SE3d Ma_new = Ma;
                    // Decoupled left increment
                    basalt::PoseState::incPose(x, Ma_new);
//                     SE3 left increment
//                    Ma_new = Sophus::SE3d::exp(x) * Ma_new;
//                    Ma_new = Sophus::se3_expd(x) * Ma_new;

                    Sophus::SE3d MaMb_new = Ma_new * Mb;

//                    return (MaMb_new * MaMb.inverse()).log();
                    return Sophus::se3_logd(MaMb_new * MaMb.inverse());
                },
                x0);
    }

}

TEST(JacobianTestSuite, d_MaMb_d_MbTest) {
    /*
     * - se3_expd : Decoupled SE3 exponentiation
     * - SE3d::exp : Coupled SE3 exponentiation
     */

    Sophus::SE3d Ma = Sophus::SE3d::exp(Sophus::Vector6d::Random());
    Sophus::SE3d Mb = Sophus::SE3d::exp(Sophus::Vector6d::Random());


    Sophus::SE3d MaMb = Ma*Mb;
    Sophus::Matrix6d d_MaMb_d_Mb;

    // SE3 left incrementの場合のJacobian
    d_MaMb_d_Mb = Ma.Adj();

    Sophus::Matrix6d decouple_conversion;
    decouple_conversion.setIdentity();
    decouple_conversion.topRightCorner<3,3>() << Sophus::SO3d::hat(Mb.translation());

    // Convert to decoupled jacobian
//    d_MaMb_d_Mb = decouple_conversion * d_MaMb_d_Mb;
    d_MaMb_d_Mb = d_MaMb_d_Mb * decouple_conversion ;

    {
        Sophus::Vector6d x0;
        x0.setZero();
        test_jacobian(
                "d_MaMb_d_Mb", d_MaMb_d_Mb,
                [&](const Sophus::Vector6d& x) {
                    Sophus::SE3d Mb_new = Mb;
                    // Decoupled left increment
                    basalt::PoseState::incPose(x, Mb_new);
//                     SE3 left increment
//                    Ma_new = Sophus::SE3d::exp(x) * Ma_new;
//                    Ma_new = Sophus::se3_expd(x) * Ma_new;

                    Sophus::SE3d MaMb_new = Ma * Mb_new;

//                    return (MaMb_new * MaMb.inverse()).log();
                    return Sophus::se3_logd(MaMb_new * MaMb.inverse());
                },
                x0);
    }

}


TEST(JacobianTestSuite, d_MInv_d_MTest) {
    /*
     * - se3_expd : Decoupled SE3 exponentiation
     * - SE3d::exp : Coupled SE3 exponentiation
     */

    Sophus::SE3d M = Sophus::SE3d::exp(Sophus::Vector6d::Random());


    Sophus::SE3d MInv = M.inverse();
    Sophus::Matrix6d d_MInv_d_M;

    // SE3 left incrementの場合のJacobian
    d_MInv_d_M = - MInv.Adj();

    Sophus::Matrix6d decouple_conversion;
    decouple_conversion.setIdentity();
    decouple_conversion.topRightCorner<3,3>() << Sophus::SO3d::hat(M.translation());

    // Convert to decoupled jacobian
    //
    d_MInv_d_M = d_MInv_d_M * decouple_conversion ;

    {
        Sophus::Vector6d x0;
        x0.setZero();
        test_jacobian(
                "d_MInv_d_M", d_MInv_d_M,
                [&](const Sophus::Vector6d& x) {
                    Sophus::SE3d M_new = M;
                    // Decoupled left increment
                    basalt::PoseState::incPose(x, M_new);
//                     SE3 left increment
//                    Ma_new = Sophus::SE3d::exp(x) * Ma_new;
//                    Ma_new = Sophus::se3_expd(x) * Ma_new;

                    Sophus::SE3d MInv_new = M_new.inverse();

//                    return (MaMb_new * MaMb.inverse()).log();
                    return Sophus::se3_logd(MInv_new * MInv.inverse());
                },
                x0);
    }

}


TEST(JacobianTestSuite, d_MInv_d_M_Full_Dec_jacobianTest) {
    /*
     * - se3_expd : Decoupled SE3 exponentiation
     * - SE3d::exp : Coupled SE3 exponentiation
     */

    Sophus::SE3d M = Sophus::SE3d::exp(Sophus::Vector6d::Random());


    Sophus::SE3d MInv = M.inverse();
    Sophus::Matrix6d d_MInv_d_M, dec_d_MInv_d_M;


    // SE3 left incrementの場合のJacobian
    d_MInv_d_M = - MInv.Adj();

    Sophus::Matrix6d decouple_conversion;
    decouple_conversion.setIdentity();
    decouple_conversion.topRightCorner<3,3>() << Sophus::SO3d::hat(M.translation());
    // Convert to decoupled jacobian
//    dec_d_MInv_d_M = d_MInv_d_M * decouple_conversion;
    dec_d_MInv_d_M = d_MInv_d_M;

    // 微小変化
    Sophus::Vector6d tau;
//    tau << 1,1,1,1,1,1;
    tau.setRandom();
    tau = tau * 1e-2;

    // Full SE3 jacobianによるMinvの変化
    Sophus::SE3d MInv_new_fullSE3;
    MInv_new_fullSE3 = Sophus::SE3d::exp(d_MInv_d_M * tau) * MInv;
    // Decoupled left incrementによるMinvの変化
    Sophus::SE3d Minv_new_decoupled = MInv;
    basalt::PoseState::incPose(dec_d_MInv_d_M*tau, Minv_new_decoupled);


    std::cout << "Minv: \n"
    << MInv.matrix()
    << "\nMinv_new_fullSE3: \n"
    << MInv_new_fullSE3.matrix()
    << "\nMinv_new_decoupled: \n"
    << Minv_new_decoupled.matrix()
    << "\nfullSE3 - decoupled: \n"
    << MInv_new_fullSE3.matrix() - Minv_new_decoupled.matrix()
    << "\ntau: " << tau
    << "\n norm(dec - fullSE3): " << (Minv_new_decoupled.matrix() - MInv_new_fullSE3.matrix()).norm()
    << "\n norm(dec - MInv): " << (MInv.matrix() - Minv_new_decoupled.matrix()).norm()
    << "\n norm(fullSE3 - MInv): " << (MInv.matrix() - MInv_new_fullSE3.matrix()).norm()
    << std::endl;
    EXPECT_TRUE(MInv_new_fullSE3.matrix().isApprox(Minv_new_decoupled.matrix(), 1e-3));


}
