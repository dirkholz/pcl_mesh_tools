/* -*- Mode: c++; tab-width: 2; c-basic-offset: 2; indent-tabs-mode: nil -*- */
/* vim:set softtabstop=2 shiftwidth=2 tabstop=2 expandtab: */
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Dirk Holz <dirk.holz _at_ ieee.org>
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
#include <pcl/visualization/pcl_visualizer.h>

pcl::PolygonMesh::Ptr input(new pcl::PolygonMesh);

pcl::PCLPointCloud2::Ptr cloud;
pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr color_handler;
pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2>::Ptr geometry_handler;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
bool show_mesh = true;
bool show_cloud = true;
int vis_cloud_active_channel = -1; // <0 for random color per cloud

void updateViewer()
{ 
  viewer->setBackgroundColor(1.0, 1.0, 1.0);
  viewer->removePolygonMesh("input");
  if (show_mesh)
    viewer->addPolygonMesh(*input, "input");

  viewer->removePointCloud("cloud");
  if (show_cloud)
  {

    if (vis_cloud_active_channel < 0)
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (cloud));
    else if (vis_cloud_active_channel < static_cast<int>(cloud->fields.size()))
    {
      if (cloud->fields[vis_cloud_active_channel].name == "rgb" || 
          cloud->fields[vis_cloud_active_channel].name == "rgba")
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> (cloud));
      else
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud, cloud->fields[vis_cloud_active_channel].name));
    }
    geometry_handler.reset(new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2> (cloud));
    viewer->addPointCloud (cloud, 
                           geometry_handler, 
                           color_handler, 
                           Eigen::Vector4f::Zero(), 
                           Eigen::Quaternionf::Identity(), 
                           "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
  }
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, 
                      void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.keyDown ())
  {
    // std::cout  << "keycode " << static_cast<int>(event.getKeyCode()) << " " << event.getKeySym() << std::endl;
    switch (event.getKeyCode())
    {
      case 33: // SHIFT+1
        vis_cloud_active_channel = -1;
        break;
      case 64: // SHIFT+2
        vis_cloud_active_channel = 0;
        break;
      case 35: // SHIFT+3
        vis_cloud_active_channel = 1;
        break;
      case 36: // SHIFT+4
        vis_cloud_active_channel = 2;
        break;
      case 37: // SHIFT+5
        vis_cloud_active_channel = 3;
        break;
      case 94: // SHIFT+6
        vis_cloud_active_channel = 4;
        break;
      case 38: // SHIFT+7
        vis_cloud_active_channel = 5;
        break;
      case 42: // SHIFT+8
        vis_cloud_active_channel = 6;
        break;
      case 40: // SHIFT+9
        vis_cloud_active_channel = 7;
        break;
      case 41: // SHIFT+0
        vis_cloud_active_channel = 8;
        break;

      case 0:
        if (event.getKeySym() == "F1")
          show_mesh = !show_mesh;
        else if (event.getKeySym() == "F2")
          show_cloud = !show_cloud;
        else if (event.getKeySym() == "F5")
          vis_cloud_active_channel = 1;
        else if (event.getKeySym() == "F6")
          vis_cloud_active_channel = 2;
        else if (event.getKeySym() == "F7")
          vis_cloud_active_channel = 3;
        else if (event.getKeySym() == "F8")
          vis_cloud_active_channel = 4;
        break;
    }

    updateViewer();
  }
}


int main (int argc, char** argv)
{

  if (argc  < 2)
  {
    PCL_ERROR ("Usage: %s [mesh_file]\n", argv[0]);
    exit (1);
  }

  int nr_poly_points = pcl::io::loadPolygonFile(argv[1], *input);
  int nr_polygons = (int)input->polygons.size();

  PCL_INFO("Read a mesh containing %d vertices and %d polygons.\n", nr_poly_points, nr_polygons);
  if ((nr_poly_points == 0) || (nr_polygons == 0))
  {
    exit (1);
  }

  cloud.reset(new pcl::PCLPointCloud2(input->cloud));
  // for (size_t i = 0; i < input->cloud.fields.size(); ++i)
  // {
  //   if (input->cloud.fields[i].name == "rgb" || input->cloud.fields[i].name == "rgba")
  //     color_handlers.push_back(pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2> (cloud));
  //   else
  //     color_handlers.push_back(pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2> (cloud, cloud->fields[i].name));
  // }

  viewer.reset(new pcl::visualization::PCLVisualizer(argv[0]));
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&*viewer);

  updateViewer();
  printf("press \"q\" to quit.\n");
  viewer->spin();
  return 0;
}


