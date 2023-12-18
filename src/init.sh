#!/bin/sh
# initialize for the ros workspace

# clone stuff
# ros package forked from the project ethzasl_icp_mapping
# [ethzasl_icp_mapping](https://github.com/ethz-asl/ethzasl_icp_mapping/tree/master)
git clone git@github.com:robinloujun/libpointmatcher_ros.git

# removed dependency on geometry_utils on 28.11.2018
# git clone git@github.com:robinloujun/geometry_utils.git 