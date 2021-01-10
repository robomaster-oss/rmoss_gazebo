#!/bin/bash
path=`pwd`
cd $path/../models/rmua19_battlefield
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot1
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2_red1
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2_blue1
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2_red2
xacro4sdf model.sdf.xacro
cd $path/../models/rmua19_standard_robot2_blue2
xacro4sdf model.sdf.xacro