/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
// #include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include "util.h"

using namespace std;
using namespace cv;
using namespace g2o;

// Pinhole camera model
class PinholeCamera
{
public:
    typedef std::shared_ptr<PinholeCamera> Ptr;
    float fx_, fy_, cx_, cy_, depth_scale_;

    PinholeCamera ( float fx, float fy, float cx, float cy, float depth_scale=0 ) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale ) {}

    // coordinate transform: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w ) { return T_c_w*p_w; }
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w ) { return T_c_w.inverse() *p_c; }
    Vector2d camera2pixel( const Vector3d& p_c ) {
        return Vector2d (
                   fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
                   fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
               );
    }
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 )
    {
        return Vector3d (
                   ( p_p ( 0,0 )-cx_ ) *depth/fx_,
                   ( p_p ( 1,0 )-cy_ ) *depth/fy_,
                   depth
               );
    }
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 ) { return camera2world ( pixel2camera ( p_p, depth ), T_c_w ); }
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w ) { return camera2pixel ( world2camera(p_w, T_c_w) ); }
};


class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        _error = _measurement - camera_->camera2pixel( pose->estimate().map(point_) );
    }


    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
        g2o::SE3Quat T ( pose->estimate() );
        Vector3d xyz_trans = T.map ( point_ );
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;

        _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx_;
        _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
        _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
        _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
        _jacobianOplusXi ( 0,4 ) = 0;
        _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx_;

        _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
        _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy_;
        _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy_;
        _jacobianOplusXi ( 1,3 ) = 0;
        _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy_;
        _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy_;
    }

    virtual bool read( std::istream& in ){return true;}
    virtual bool write(std::ostream& os) const {return true;};

    Vector3d point_;
    PinholeCamera* camera_;
};





class EdgeProjectXYZ2UVPoseAndPoint: public g2o::BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError()
    {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        _error = _measurement - camera_->camera2pixel( v1->estimate().map(v2->estimate()) );
    }

    virtual void linearizeOplus()
    {
        VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
        SE3Quat T(vj->estimate());
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Vector3d xyz = vi->estimate();
        Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z*z;

        Eigen::Matrix<double,2,3,Eigen::ColMajor> tmp;
        tmp(0,0) = camera_->fx_;
        tmp(0,1) = 0;
        tmp(0,2) = -x/z*camera_->fx_;

        tmp(1,0) = 0;
        tmp(1,1) = camera_->fy_;
        tmp(1,2) = -y/z*camera_->fy_;

        _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

        _jacobianOplusXj(0,0) =  x*y/z_2 *camera_->fx_;
        _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *camera_->fx_;
        _jacobianOplusXj(0,2) = y/z *camera_->fx_;
        _jacobianOplusXj(0,3) = -1./z *camera_->fx_;
        _jacobianOplusXj(0,4) = 0;
        _jacobianOplusXj(0,5) = x/z_2 *camera_->fx_;

        _jacobianOplusXj(1,0) = (1+y*y/z_2) *camera_->fy_;
        _jacobianOplusXj(1,1) = -x*y/z_2 *camera_->fy_;
        _jacobianOplusXj(1,2) = -x/z *camera_->fy_;
        _jacobianOplusXj(1,3) = 0;
        _jacobianOplusXj(1,4) = -1./z *camera_->fy_;
        _jacobianOplusXj(1,5) = y/z_2 *camera_->fy_;
    }

    virtual bool read( std::istream& in ){return true;}
    virtual bool write(std::ostream& os) const {return true;};

    PinholeCamera* camera_;
};



#endif

