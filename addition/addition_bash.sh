#!/bin/bash

cp haique_description ../src/Dave_Haique/urdf/robots/

cp prop_meshes/*.* ../src/uuv_simulator/uuv_descriptions/meshes

cp haique.launch ../src/Dave_Haiqu/examples/dave_robot_launch/launch

cp thruster_snippets_haique.xacro ../src/uuv_simulator/uuv_gazebo_plugins/uuv_gazebo_ros_plugins/urdf