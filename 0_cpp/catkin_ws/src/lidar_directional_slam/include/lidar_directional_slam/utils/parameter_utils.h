#pragma once

#ifndef PARAMETER_UTILS_H
#define PARAMETER_UTILS_H

#include <ros/ros.h>
#include <string>

namespace parameter_utils
{

template <class C> bool Get(const std::string& paramName, C& param)
{
  std::string nodeName = ros::this_node::getName();

  std::string paraTemp;
  if (!ros::param::search(paramName, paraTemp))
  {
    ROS_ERROR("%s: Failed to search for parameter '%s'.", nodeName.c_str(),
              paramName.c_str());
    return false;
  }

  if (!ros::param::has(paraTemp))
  {
    ROS_ERROR("%s: Missing required parameter '%s'.", nodeName.c_str(),
              paramName.c_str());
    return false;
  }

  if (!ros::param::get(paraTemp, param))
  {
    ROS_ERROR("%s: Failed to get parameter '%s'.", nodeName.c_str(),
              paramName.c_str());
    return false;
  }

  return true;
}
}

#endif // PARAMETER_UTILS_H
