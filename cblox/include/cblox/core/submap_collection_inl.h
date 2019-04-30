#ifndef CBLOX_CORE_SUBMAP_COLLECTION_INL_H_
#define CBLOX_CORE_SUBMAP_COLLECTION_INL_H_

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
void SubmapCollection<SubmapType>::createNewSubMap(const Transformation& T_G_S,
                                                   const SubmapID submap_id) {
  // Checking if the submap already exists
  // NOTE(alexmillane): This hard fails the program if the submap already
  // exists. This is fairly brittle behaviour and we may want to change it at a
  // later date. Currently the onus is put in the caller to exists() before
  // creating a submap.
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it == id_to_submap_.end());
  // Creating the new submap and adding it to the list
  typename SubmapType::Ptr tsdf_sub_map(
      new SubmapType(T_G_S, submap_id, submap_config_));
  id_to_submap_.emplace(submap_id, std::move(tsdf_sub_map));
  // Updating the active submap
  active_submap_id_ = submap_id;
}

template <typename SubmapType>
SubmapID SubmapCollection<SubmapType>::createNewSubMap(
    const Transformation& T_G_S) {
  // Creating a submap with a generated SubmapID
  // NOTE(alexmillane): rbegin() returns the pair with the highest key.
  SubmapID new_ID = 0;
  if (!id_to_submap_.empty()) {
    new_ID = id_to_submap_.rbegin()->first + 1;
  }
  createNewSubMap(T_G_S, new_ID);
  return new_ID;
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::duplicateSubMap(
    const SubmapID source_submap_id, const SubmapID new_submap_id) {
  // Get pointer to the source submap
  const auto src_submap_ptr_it = id_to_submap_.find(source_submap_id);
  if (src_submap_ptr_it != id_to_submap_.end()) {
    typename SubmapType::Ptr src_submap_ptr = src_submap_ptr_it->second;
    // Create a new submap with the same pose and get its pointer
    const Transformation T_G_S = src_submap_ptr->getPose();
    // Creating the new submap and adding it to the list
    typename SubmapType::Ptr new_tsdf_sub_map(
        new SubmapType(T_G_S, new_submap_id, submap_config_));
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

template <typename SubmapType>
const SubmapType& SubmapCollection<SubmapType>::getSubMap(
    const SubmapID submap_id) const {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  return *it->second;
}

template <typename SubmapType>
const std::vector<typename SubmapType::Ptr>
SubmapCollection<SubmapType>::getSubMapPtrs() const {
  std::vector<typename SubmapType::Ptr> submap_ptrs;
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ptrs.emplace_back(id_submap_pair.second);
  }
  return submap_ptrs;
}

template <typename SubmapType>
const std::vector<typename SubmapType::ConstPtr>
SubmapCollection<SubmapType>::getSubMapConstPtrs() const {
  std::vector<typename SubmapType::ConstPtr> submap_ptrs;
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ptrs.emplace_back(id_submap_pair.second);
  }
  return submap_ptrs;
}

template <typename SubmapType>
TsdfMap::Ptr SubmapCollection<SubmapType>::getActiveTsdfMapPtr() {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getTsdfMapPtr();
}
template <typename SubmapType>
const TsdfMap& SubmapCollection<SubmapType>::getActiveTsdfMap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getTsdfMap();
}

template <typename SubmapType>
const SubmapType& SubmapCollection<SubmapType>::getActiveSubMap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return *(it->second);
}

// Gets a pointer to the active submap
template <typename SubmapType>
typename SubmapType::Ptr SubmapCollection<SubmapType>::getActiveSubMapPtr() {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return it->second;
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
void SubmapCollection<SubmapType>::activateSubMap(const SubmapID submap_id) {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  active_submap_id_ = submap_id;
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
    const Transformation& T_G_S = (id_submap_pair.second)->getPose();
    // Merging layers the submap into the global layer
    mergeLayerAintoLayerB(tsdf_map.getTsdfLayer(), T_G_S,
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
    LOG(WARNING) << "Tried to set the pose of the submap with submap_id: "
                 << submap_id << " and could not find the linked submap.";
    return false;
  }
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::setSubMapPoses(
    const TransformationVector& transforms) {
  CHECK_EQ(transforms.size(), id_to_submap_.size());
  // NOTE(alexmillane): This assumes that the order of transforms matches the
  //                    submap order.
  // NOTE(alexmillane): Should really be passing a map of ids to transforms.
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
    LOG(INFO) << "Tried to get the pose of the submap with submap_id: "
              << submap_id << " and could not find the linked submap.";
    return false;
  }
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::getSubMapPoses(
    AlignedVector<Transformation>* submap_poses_ptr) const {
  CHECK_NOTNULL(submap_poses_ptr);
  // Extracting transforms
  submap_poses_ptr->clear();
  submap_poses_ptr->reserve(id_to_submap_.size());
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_poses_ptr->push_back((id_submap_pair.second)->getPose());
  }
}

template <typename SubmapType>
typename SubmapType::ConstPtr SubmapCollection<
    SubmapType>::getSubMapConstPtrById(const SubmapID submap_id) const {
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    return tsdf_submap_ptr_it->second;
  } else {
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
    LOG(INFO) << "Saving tsdf_submap with ID: " << id_submap_pair.first;
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
  CHECK_NOTNULL(proto);
  // Filling out the description of the submap collection
  proto->set_num_submaps(num_patches());
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::fuseSubmapPair(
    const SubmapIdPair& submap_id_pair) {
  // Extracting the submap IDs
  SubmapID submap_id_1 = submap_id_pair.first;
  SubmapID submap_id_2 = submap_id_pair.second;
  LOG(INFO) << "Fusing submap pair: (" << submap_id_1 << ", " << submap_id_2
            << ")";
  // Getting the requested submaps
  const auto id_submap_pair_1_it = id_to_submap_.find(submap_id_1);
  const auto id_submap_pair_2_it = id_to_submap_.find(submap_id_2);
  // If the submaps are found
  if ((id_submap_pair_1_it != id_to_submap_.end()) &&
      (id_submap_pair_2_it != id_to_submap_.end())) {
    // Getting the submaps
    typename SubmapType::Ptr submap_ptr_1 = (*id_submap_pair_1_it).second;
    typename SubmapType::Ptr submap_ptr_2 = (*id_submap_pair_2_it).second;
    // Checking that we're not trying to fuse a submap into itself. This can
    // occur due to fusing submap pairs in a triangle.
    if (submap_ptr_1->getID() == submap_ptr_2->getID()) {
      return;
    }
    // Getting the tsdf submap and its pose
    const Transformation& T_G_S1 = submap_ptr_1->getPose();
    const Transformation& T_G_S2 = submap_ptr_2->getPose();
    const Transformation T_S1_S2 = T_G_S1.inverse() * T_G_S2;
    // Merging the submap layers
    mergeLayerAintoLayerB(submap_ptr_2->getTsdfMap().getTsdfLayer(), T_S1_S2,
                          submap_ptr_1->getTsdfMapPtr()->getTsdfLayerPtr());

    // Deleting Submap #2
    const size_t num_erased = id_to_submap_.erase(submap_id_2);
    CHECK_EQ(num_erased, 1);
    LOG(INFO) << "Erased the submap: " << submap_ptr_2->getID()
              << " from the submap collection";

  } else {
    LOG(WARNING) << "Could not find the requested submap pair during fusion.";
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
