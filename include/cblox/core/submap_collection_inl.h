//
// Created by victor on 07.01.19.
//

#ifndef CBLOX_CORE_SUBMAP_COLLECTION_INL_H_
#define CBLOX_CORE_SUBMAP_COLLECTION_INL_H_

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/protobuf_utils.h>
#include "cblox/core/tsdf_submap.h"

namespace cblox {

template <typename SubmapType>
SubmapCollection<SubmapType>::SubmapCollection(
    const typename SubmapType::Config& submap_config,
    const std::vector<typename SubmapType::Ptr>& tsdf_sub_maps)
    : submap_config_(submap_config) {
  // Constructing from a list of existing submaps
  // NOTE(alexmillane): assigning arbitrary SubmapIDs
  SubmapID submap_id = 0;
  for (const auto& tsdf_submap_ptr : tsdf_sub_maps) {
    id_to_submap_[submap_id] = tsdf_submap_ptr;
    submap_id++;
  }
}

template <typename SubmapType>
std::vector<SubmapID> SubmapCollection<SubmapType>::getIDs() const {
  std::vector<SubmapID> submap_ids;
  submap_ids.reserve(id_to_submap_.size());
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ids.emplace_back(id_submap_pair.first);
  }
  return submap_ids;
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::exists(const SubmapID submap_id) const {
  // Searching for the passed submap ID
  const auto it = id_to_submap_.find(submap_id);
  return (it != id_to_submap_.end());
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::createNewSubMap(const Transformation& T_M_S,
                                                   const SubmapID submap_id) {
  // Checking if the submap already exists
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it == id_to_submap_.end());
  // Creating the new submap and adding it to the list
  typename SubmapType::Ptr tsdf_sub_map(
      new SubmapType(T_M_S, submap_id, submap_config_));
  id_to_submap_.emplace(submap_id, std::move(tsdf_sub_map));
  // Updating the active submap
  active_submap_id_ = submap_id;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::createNewSubMap(
    const Transformation& T_M_S) {
  // Creating a submap with a generated SubmapID
  // NOTE(alexmillane): rbegin() returns the pair with the highest key.
  SubmapID new_ID = 0;
  if (!id_to_submap_.empty()) {
    new_ID = id_to_submap_.rbegin()->first + 1;
  }
  createNewSubMap(T_M_S, new_ID);
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::duplicateSubMap(
    const SubmapID source_submap_id, const SubmapID new_submap_id) {
  // Get pointer to the source submap
  const auto src_submap_ptr_it = id_to_submap_.find(source_submap_id);
  if (src_submap_ptr_it != id_to_submap_.end()) {
    typename SubmapType::Ptr src_submap_ptr = src_submap_ptr_it->second;
    // Create a new submap with the same pose and get its pointer
    const Transformation T_M_S = src_submap_ptr->getPose();
    // Creating the new submap and adding it to the list
    typename SubmapType::Ptr new_tsdf_sub_map(
        new SubmapType(T_M_S, new_submap_id, submap_config_));
    // Reset the TsdfMap based on a copy of the source submap's TSDF layer
    // TODO(victorr): Find a better way to do this, however with .reset(...) as
    // below the new submap appears empty
    // new_tsdf_sub_map->getTsdfMapPtr().reset(new
    // TsdfMap(src_submap_ptr->getTsdfMap().getTsdfLayer()));
    *(new_tsdf_sub_map->getTsdfMapPtr()) =
        *(new TsdfMap(src_submap_ptr->getTsdfMap().getTsdfLayer()));
    id_to_submap_.emplace(new_submap_id, new_tsdf_sub_map);
    return true;
  }
  return false;
}

// Gets a const pointer to a raw submap
template <typename SubmapType>
const SubmapType& SubmapCollection<SubmapType>::getSubMap(
    const SubmapID submap_id) const {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  return *it->second;
}

template <typename SubmapType>
const std::vector<typename SubmapType::Ptr>
SubmapCollection<SubmapType>::getSubMaps() const {
  std::vector<typename SubmapType::Ptr> submap_ptrs;
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ptrs.emplace_back(id_submap_pair.second);
  }
  return submap_ptrs;
}

// Gets a pointer to the tsdf_map member of the active submap
template <typename SubmapType>
TsdfMap::Ptr SubmapCollection<SubmapType>::getActiveTsdfMapPtr() {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getTsdfMapPtr();
}
// Gets a reference to tsdf_map member of the active submap
template <typename SubmapType>
const TsdfMap& SubmapCollection<SubmapType>::getActiveTsdfMap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getTsdfMap();
}

// Gets a reference to the active submap
template <typename SubmapType>
const SubmapType& SubmapCollection<SubmapType>::getActiveSubMap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return *(it->second);
}

template <typename SubmapType>
const Transformation& SubmapCollection<SubmapType>::getActiveSubMapPose()
    const {
  return getActiveSubMap().getPose();
}
template <typename SubmapType>
const SubmapID SubmapCollection<SubmapType>::getActiveSubMapID() const {
  return active_submap_id_;
}

template <typename SubmapType>
TsdfMap::Ptr SubmapCollection<SubmapType>::getProjectedMap() const {
  // Creating the global tsdf map and getting its tsdf layer
  TsdfMap::Ptr projected_tsdf_map_ptr =
      TsdfMap::Ptr(new TsdfMap(submap_config_));
  Layer<TsdfVoxel>* combined_tsdf_layer_ptr =
      projected_tsdf_map_ptr->getTsdfLayerPtr();
  // Looping over the current submaps
  for (const auto& id_submap_pair : id_to_submap_) {
    // Getting the tsdf submap and its pose
    const TsdfMap& tsdf_map = (id_submap_pair.second)->getTsdfMap();
    const Transformation& T_M_S = (id_submap_pair.second)->getPose();
    // Merging layers the submap into the global layer
    mergeLayerAintoLayerB(tsdf_map.getTsdfLayer(), T_M_S,
                          combined_tsdf_layer_ptr);
  }
  // Returning the new map
  return projected_tsdf_map_ptr;
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::setSubMapPose(const SubmapID submap_id,
                                                 const Transformation& pose) {
  // Looking for the submap
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    typename SubmapType::Ptr submap_ptr = (*tsdf_submap_ptr_it).second;
    submap_ptr->setPose(pose);
    return true;
  } else {
    std::cout << "Tried to set the pose of the submap with submap_id: "
              << submap_id << " and could not find the linked submap."
              << std::endl;
    return false;
  }
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::setSubMapPoses(
    const TransformationVector& transforms) {
  CHECK_EQ(transforms.size(), id_to_submap_.size());
  // NOTE(alexmillane): This assumes that the order of transforms matches the
  //                    submap order.
  size_t sub_map_index = 0;
  for (const auto& id_submap_pair : id_to_submap_) {
    (id_submap_pair.second)->setPose(transforms[sub_map_index]);
    sub_map_index++;
  }
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::getSubMapPose(
    const SubmapID submap_id, Transformation* pose_ptr) const {
  // Looking for the submap
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    typename SubmapType::Ptr submap_ptr = (*tsdf_submap_ptr_it).second;
    *pose_ptr = submap_ptr->getPose();
    return true;
  } else {
    std::cout << "Tried to get the pose of the submap with submap_id: "
              << submap_id << " and could not find the linked submap."
              << std::endl;
    return false;
  }
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::getSubMapPoses(
    AlignedVector<Transformation>* submap_poses_ptr) const {
  // Checks
  CHECK_NOTNULL(submap_poses_ptr);
  // Extracting transforms
  submap_poses_ptr->clear();
  submap_poses_ptr->reserve(id_to_submap_.size());
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_poses_ptr->push_back((id_submap_pair.second)->getPose());
  }
}

/*bool SubmapCollection::getAssociatedTsdfSubMapID(
    const SubmapID submap_id, SubmapID* submap_id_ptr) const {
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    *submap_id_ptr = tsdf_submap_ptr_it->second->getID();
    return true;
  } else {
    // std::cout << "Cant find the requested submap_id: " << submap_id
    //          << " associated with any submap" << std::endl;
    return false;
  }
}*/

template <typename SubmapType>
typename SubmapType::ConstPtr SubmapCollection<
    SubmapType>::getSubMapConstPtrById(const SubmapID submap_id) const {
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    return tsdf_submap_ptr_it->second;
  } else {
    // std::cout << "Cant find the requested submap_id: " << submap_id
    //          << " associated with any submap" << std::endl;
    // TODO(Alex.Millane): Why not return a nullptr here?
    return typename SubmapType::ConstPtr();
  }
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::saveToFile(
    const std::string& file_path) const {
  // Opening the file (if we can)
  CHECK(!file_path.empty());
  std::fstream outfile;
  outfile.open(file_path, std::fstream::out | std::fstream::binary);
  if (!outfile.is_open()) {
    LOG(ERROR) << "Could not open file for writing: " << file_path;
    return false;
  }
  // Saving the submap collection header object
  TsdfSubmapCollectionProto tsdf_submap_collection_proto;
  getProto(&tsdf_submap_collection_proto);
  // Write out the layer header.
  if (!voxblox::utils::writeProtoMsgToStream(tsdf_submap_collection_proto,
                                             &outfile)) {
    LOG(ERROR) << "Could not write submap collection header message.";
    outfile.close();
    return false;
  }
  // Saving the tsdf submaps
  for (const auto& id_submap_pair : id_to_submap_) {
    // DEBUG
    std::cout << "Saving tsdf_submap with ID: " << id_submap_pair.first
              << std::endl;
    // Saving the submap
    (id_submap_pair.second)->saveToStream(&outfile);
  }
  // Closing the file
  outfile.close();
  return true;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::getProto(
    TsdfSubmapCollectionProto* proto) const {
  // Checks
  CHECK_NOTNULL(proto);
  // Filling out the description of the submap collection
  proto->set_voxel_size(submap_config_.tsdf_voxel_size);
  proto->set_voxels_per_side(submap_config_.tsdf_voxels_per_side);
  proto->set_num_submaps(num_patches());
}

// Fusing the submap pairs
template <typename SubmapType>
void SubmapCollection<SubmapType>::fuseSubmapPair(
    const SubmapIdPair& submap_id_pair) {
  // Extracting the submap IDs
  SubmapID submap_id_1 = submap_id_pair.first;
  SubmapID submap_id_2 = submap_id_pair.second;
  // DEBUG
  std::cout << "Fusing submap pair: (" << submap_id_1 << ", " << submap_id_2
            << ")" << std::endl;
  // Getting the requested submaps
  auto id_submap_pair_1_it = id_to_submap_.find(submap_id_1);
  auto id_submap_pair_2_it = id_to_submap_.find(submap_id_2);
  // If the submaps are found
  if ((id_submap_pair_1_it != id_to_submap_.end()) &&
      (id_submap_pair_2_it != id_to_submap_.end())) {
    // Getting the submaps
    typename SubmapType::Ptr submap_ptr_1 = (*id_submap_pair_1_it).second;
    typename SubmapType::Ptr submap_ptr_2 = (*id_submap_pair_2_it).second;
    // Checking that we're not trying to fuse a submap into itself. This can
    // occur due to fusing submap pairs in a triangle.
    if (submap_ptr_1->getID() == submap_ptr_2->getID()) {
      std::cout << "Avoided fusing submap into itself." << std::endl;
      return;
    }
    // Getting the tsdf submap and its pose
    const Transformation& T_M_S1 = submap_ptr_1->getPose();
    const Transformation& T_M_S2 = submap_ptr_2->getPose();
    const Transformation& T_S1_S2 = T_M_S1.inverse() * T_M_S2;
    // Merging the submap layers
    mergeLayerAintoLayerB(submap_ptr_2->getTsdfMap().getTsdfLayer(), T_S1_S2,
                          submap_ptr_1->getTsdfMapPtr()->getTsdfLayerPtr());
    // Transfering KeyFrames to the new submap
    // TODO(alexmillane): NEEDED FOR MANIFOLD MAPPING.
    /*    // Searching for ids associated with submap 2
        for (auto& id_submap_pair : id_to_submap_) {
          if (id_submap_pair.second == submap_ptr_2) {
            id_submap_pair.second = submap_ptr_1;
            // std::cout << "Moved submap ID: " << id_submap_pair.first
            //          << " from submap ID: " << submap_ptr_2->getID()
            //          << " to submap ID: " << submap_ptr_1->getID()
            //          << std::endl;
          }
        }*/
    // Deleting Submap #2
    const size_t num_erased = id_to_submap_.erase(submap_id_2);
    CHECK_EQ(num_erased, 1);
    std::cout << "Erased the submap: " << submap_ptr_2->getID()
              << " from the submap collection" << std::endl;

  } else {
    std::cout << "Could not find the requested submap pair during fusion."
              << std::endl;
  }
}

template <typename SubmapType>
size_t SubmapCollection<SubmapType>::getNumberAllocatedBlocks() const {
  // Looping over the submaps totalling the sizes
  size_t total_blocks = 0;
  for (const auto& id_submap_pair : id_to_submap_) {
    total_blocks += (id_submap_pair.second)->getNumberAllocatedBlocks();
  }
  return total_blocks;
}

}  // namespace cblox

#endif  // CBLOX_CORE_SUBMAP_COLLECTION_INL_H_
