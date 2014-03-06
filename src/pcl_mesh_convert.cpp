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
#include <pcl/console/print.h>

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    PCL_ERROR("USAGE: %s [input mesh] [output mesh]\n", argv[0]);
    exit(EXIT_FAILURE);
  }
  
  std::string input(argv[1]), output(argv[2]);

  PCL_DEBUG("Converting %s to %s.\n", input.c_str(), output.c_str());

  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFile(input, mesh) <= 0)
  {
    PCL_ERROR("Error reading from %s", input.c_str());
    exit(EXIT_FAILURE);
  }
  
  if (pcl::io::savePolygonFile(output, mesh) <= 0)
  {
    PCL_ERROR("Error writing to %s", input.c_str());
    exit(EXIT_FAILURE);
  }

  return EXIT_SUCCESS;
}
