/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz, University of Bonn.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pcl/io/vtk_lib_io.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

typedef pcl::PointXYZ PointT;

int main (int argc, char** argv)
{

  if (argc  < 2)
  {
    PCL_ERROR ("Usage: %s [mesh_file]\n", argv[0]);
    exit (1);
  }

  boost::filesystem::path input_file(argv[1]);
  PCL_INFO("Generating rendered views for %s.\n", input_file.string().c_str());

  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (input_file.string().c_str());

  float model_scale = 0.001f;
  vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New ();
  trans->Scale(model_scale, model_scale, model_scale);
  trans->Modified();
  trans->Update();

  vtkSmartPointer<vtkTransformPolyDataFilter> filter_scale = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
  filter_scale->SetTransform(trans);
  filter_scale->SetInputConnection (reader->GetOutputPort ());
  filter_scale->Update();

  vtkSmartPointer<vtkPolyData> mapper = filter_scale->GetOutput ();
  mapper->Update ();

  int resolution = 150;
  float radius_sphere = 1.5f;
  int tesselation_level = 1;
  float view_angle  = 57.f;

  pcl::apps::RenderViewsTesselatedSphere render_views;
  render_views.setResolution(resolution);
  render_views.setUseVertices(false);
  render_views.setRadiusSphere(radius_sphere);
  render_views.setComputeEntropies(false);
  render_views.setTesselationLevel(tesselation_level);
  render_views.setViewAngle(view_angle);
  render_views.addModelFromPolyData (mapper);
  render_views.setGenOrganized(false);
  // render_views.setCamPosConstraints(campos_constraints_func_);
  render_views.generateViews();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clouds;
  std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
  render_views.getViews (clouds);
  render_views.getPoses (poses);

  PCL_INFO("Rendered %zu points clouds (at %zu poses).\n");
  for (size_t i = 0; i < clouds.size(); ++i)
  {
    clouds[i]->sensor_origin_.head<3>() = poses[i].block<3,1>(3,0);
    clouds[i]->sensor_orientation_ = Eigen::Quaternionf(poses[i].block<3,3>(0,0));

    std::string filename = str(boost::format("%s_rendered_view_%zu.pcd") % boost::filesystem::basename(input_file) % i);  
    pcl::io::savePCDFileBinary(filename, *clouds[i]);\
    PCL_INFO("Writing file %s", filename.c_str());
  }

  return 0;
}


