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

int main (int argc, char** argv)
{

  if (argc  < 2)
  {
    PCL_ERROR ("Usage: %s [mesh_file]\n", argv[0]);
    exit (1);
  }

  pcl::PolygonMesh::Ptr input(new pcl::PolygonMesh);
  int nr_poly_points = pcl::io::loadPolygonFile(argv[1], *input);
  int nr_polygons = (int)input->polygons.size();

  PCL_INFO("Read a mesh containing %d vertices and %d polygons.\n", nr_poly_points, nr_polygons);
  if ((nr_poly_points == 0) || (nr_polygons == 0))
  {
    exit (1);
  }

  pcl::visualization::PCLVisualizer viewer(argv[0]);
  viewer.setBackgroundColor(1.0, 1.0, 1.0);
  viewer.addPolygonMesh(*input, "input");
  printf("press \"q\" to quit.\n");
  viewer.spin();
  return 0;
}
