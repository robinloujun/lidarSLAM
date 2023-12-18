#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H

#include <lidar_directional_slam/utils/basic.h>

namespace mapping_utils
{

template <typename T> struct poseGraph
{

  IMPORT_TYPES(T)

  struct poseFactor
  {
    uint index;
    TransformationParameters pose;
    DPPtr cloudPtr;

  public:
    // Default constructor is the origin
    poseFactor()
        : index(0), pose(TransformationParameters::Identity(4, 4)),
          cloudPtr(NULL)
    {
    }

    // Copy constructor
    poseFactor(const poseFactor& pose_factor)
        : index(pose_factor.index), pose(pose_factor.pose),
          cloudPtr(pose_factor.cloudPtr)
    {
    }

    // Construct from pose and point cloud pointer
    poseFactor(const uint& idx, const TransformationParameters& p, DPPtr ptr)
        : index(idx), pose(p), cloudPtr(ptr)
    {
    }
  }; // poseFactor

public:
  poseGraph() {}
  poseGraph(const poseGraph& graph) { pose_graph = graph; }

  void addNode(const uint index, const TransformationParameters& p, DPPtr ptr)
  {
    pose_graph.push_back(poseFactor(index, p, ptr));
  }

  void clearGraph() { pose_graph.clear(); }

  size_t getNumNodes() { return pose_graph.size(); }

//  DPPtr getLocalMap()
//  {
//    DPPtr local_map;
//    int size = pose_graph.getNumNodes();
//    for (int i = size - 1; i > size - 11; --i)
//    {
//      if (i < 0) { break; }
//      local_map.concatenate(*pose_graph[i].cloudPtr);
//    }
//    return local_map;
//  }

  std::vector<poseFactor> pose_graph;
}; // poseGraph

} // namespace mapping_utils

#endif // POSE_GRAPH_H
