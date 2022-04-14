/*
 * @Author: your name
 * @Date: 2022-03-07 11:10:53
 * @LastEditTime: 2022-03-08 16:27:25
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/src/ipopt_registration.cpp
 */

#include "../include/pairwise_ICP.h"

// Copyright (C) 2005, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-16

using namespace Ipopt;
using namespace pairwiseICP;
// constructor

void registrationICP::align()
{
    // set ICP parameters
    ICP::Parameters pars;

    // set Sparse-ICP parameters
    SICP::Parameters spars;
    spars.p = 0.4;
    spars.print_icpn = false;
    FRICP<3> fricp;
    Eigen::Vector3d source_mean, target_mean;
    source_mean = vertices_source->rowwise().sum() / double(vertices_source->cols());
    target_mean = vertices_target->rowwise().sum() / double(vertices_target->cols());
    vertices_source->colwise() -= source_mean;
    vertices_target->colwise() -= target_mean;
    switch(method)
    {
        case ICP:
        {
            pars.f = ICP::FAIR;
            pars.use_AA = false;
            fricp.point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, pars);
            res_trans = pars.res_trans;
            break;
        }
        case AA_ICP:
        {
            AAICP::point_to_point_aaicp(*vertices_source, *vertices_target, source_mean, target_mean, pars);
            res_trans = pars.res_trans;
            break;
        }
        case FICP:
        {
            pars.f = ICP::NONE;
            pars.use_AA = true;
            fricp.point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, pars);
            res_trans = pars.res_trans;
            break;
        }
        case RICP:
        {
            pars.f = ICP::WELSCH;
            pars.use_AA = true;
            fricp.point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, pars);
            res_trans = pars.res_trans;
            break;
        }
        case PPL:
        {
            pars.f = ICP::NONE;
            pars.use_AA = false;
            fricp.point_to_plane(*vertices_source, *vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
            res_trans = pars.res_trans;
            break;
        }
        case RPPL:
        {
            pars.nu_end_k = 1.0/6;
            pars.f = ICP::WELSCH;
            pars.use_AA = true;
            fricp.point_to_plane_GN(*vertices_source, *vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
            res_trans = pars.res_trans;
            break;
        }
        case SparseICP:
        {
            SICP::point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, spars);
            res_trans = spars.res_trans;
            break;
        }
        case SICPPPL:
        {
            SICP::point_to_plane(*vertices_source, *vertices_target, normal_target, source_mean, target_mean, spars);
            res_trans = spars.res_trans;
            break;
        }
        default:
            break;
    }
}
