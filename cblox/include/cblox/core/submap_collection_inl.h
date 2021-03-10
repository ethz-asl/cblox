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
  // NOTE(alexmillane): Relies on the submaps having unique submap IDs...
  for (const auto& tsdf_submap_ptr : tsdf_sub_maps) {
    const auto ret =
        id_to_submap_.insert({tsdf_submap_ptr->getID(), tsdf_submap_ptr});
    CHECK(ret.second) << "Attempted to construct collection from vector of "
                         "submaps containing at least one duplicate ID.";
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
SubmapID SubmapCollection<SubmapType>::getNextSubmapID() const {
  // NOTE(alexmillane): rbegin() returns the pair with the highest key.
  SubmapID new_ID = 0;
  if (!id_to_submap_.empty()) {
    new_ID = id_to_submap_.rbegin()->first + 1;
  }
  return new_ID;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::createNewSubmap(const Transformation& T_G_S,
                                                   const SubmapID submap_id) {
  // Checking if the submap already exists
  // NOTE(alexmillane): This hard fails the program if the submap already
  // exists. This is fairly brittle behaviour and we may want to change it at a
  // later date. Currently the onus is put in the caller to exists() before
  // creating a submap.
  CHECK(!exists(submap_id));
  // Creating the new submap
  typename SubmapType::Ptr tsdf_sub_map(
      new SubmapType(T_G_S, submap_id, submap_config_));
  // Add it to the collection
  id_to_submap_.emplace(submap_id, std::move(tsdf_sub_map));
  // Updating the active submap
  active_submap_id_ = submap_id;
}

template <typename SubmapType>
SubmapID SubmapCollection<SubmapType>::createNewSubmap(
    const Transformation& T_G_S) {
  // Creating a submap with an auto-generated SubmapID
  SubmapID new_ID = getNextSubmapID();
  createNewSubmap(T_G_S, new_ID);
  return new_ID;
}

template <typename SubmapType>
SubmapType SubmapCollection<SubmapType>::draftNewSubmap() const {
  const Transformation dummy_pose = Transformation();
  const SubmapID new_submap_id = getNextSubmapID();
  return SubmapType(dummy_pose, new_submap_id, submap_config_);
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::addSubmap(const SubmapType& submap) {
  // Check ID not already in the collection
  const SubmapID submap_id = submap.getID();
  CHECK(!exists(submap_id));
  // Add and activate
  id_to_submap_.emplace(submap_id, std::make_shared<SubmapType>(submap));
  active_submap_id_ = submap_id;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::addSubmap(SubmapType&& submap) {
  // Check ID not already in the collection
  const SubmapID submap_id = submap.getID();
  CHECK(!exists(submap_id));
  // Add and activate
  id_to_submap_.emplace(submap_id,
                        std::make_shared<SubmapType>(std::move(submap)));
  active_submap_id_ = submap_id;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::addSubmap(
    const typename SubmapType::Ptr submap) {
  // Check ID not already in the collection
  const SubmapID submap_id = submap->getID();
  CHECK(!exists(submap_id));
  // Add and activate
  id_to_submap_.emplace(submap_id, std::move(submap));
  active_submap_id_ = submap_id;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::duplicateSubmap(
    const SubmapID source_submap_id, const SubmapID new_submap_id) {
  // Get pointer to the source submap
  SubmapType src_submap = getSubmap(source_submap_id);
  const Transformation T_G_S = src_submap.getPose();
  // Create a new submap with the same pose
  typename SubmapType::Ptr new_tsdf_sub_map =
      std::make_shared<SubmapType>(T_G_S, new_submap_id, submap_config_);
  // Deep copy the TSDF map
  *(new_tsdf_sub_map->getTsdfMapPtr()) = src_submap.getTsdfMap();
  // Add the submap to the collection
  addSubmap(new_tsdf_sub_map);
}

template <typename SubmapType>
const SubmapType& SubmapCollection<SubmapType>::getSubmap(
    const SubmapID submap_id) const {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  return *it->second;
}

template <typename SubmapType>
const std::vector<typename SubmapType::Ptr>
SubmapCollection<SubmapType>::getSubmapPtrs() const {
  std::vector<typename SubmapType::Ptr> submap_ptrs;
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ptrs.emplace_back(id_submap_pair.second);
  }
  return submap_ptrs;
}

template <typename SubmapType>
const std::vector<typename SubmapType::ConstPtr>
SubmapCollection<SubmapType>::getSubmapConstPtrs() const {
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
TsdfMap::Ptr SubmapCollection<SubmapType>::getTsdfMapPtr(
    const SubmapID submap_id) {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getTsdfMapPtr();
}

template <typename SubmapType>
std::shared_ptr<const TsdfMap> SubmapCollection<SubmapType>::getTsdfMapConstPtr(
    const SubmapID submap_id) const {
  const auto it = id_to_submap_.find(submap_id);
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
const SubmapType& SubmapCollection<SubmapType>::getActiveSubmap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return *(it->second);
}

// Gets a pointer to the active submap
template <typename SubmapType>
typename SubmapType::Ptr SubmapCollection<SubmapType>::getActiveSubmapPtr() {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return it->second;
}

template <typename SubmapType>
Transformation SubmapCollection<SubmapType>::getActiveSubmapPose() const {
  return getActiveSubmap().getPose();
}
template <typename SubmapType>
SubmapID SubmapCollection<SubmapType>::getActiveSubmapID() const {
  return active_submap_id_;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::activateSubmap(const SubmapID submap_id) {
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
bool SubmapCollection<SubmapType>::setSubmapPose(const SubmapID submap_id,
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
void SubmapCollection<SubmapType>::setSubmapPoses(
    const SubmapIdPoseMap& id_pose_map) {
  for (const SubmapIdPosePair& id_pose_pair : id_pose_map) {
    setSubmapPose(id_pose_pair.first, id_pose_pair.second);
  }
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::getSubmapPose(
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
void SubmapCollection<SubmapType>::getSubmapPoses(
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
typename SubmapType::Ptr SubmapCollection<SubmapType>::getSubmapPtr(
    const SubmapID submap_id) {
  const auto submap_ptr_it = id_to_submap_.find(submap_id);
  if (submap_ptr_it != id_to_submap_.end()) {
    return submap_ptr_it->second;
  } else {
    return typename SubmapType::Ptr();
  }
}

template <typename SubmapType>
typename SubmapType::ConstPtr SubmapCollection<SubmapType>::getSubmapConstPtr(
    const SubmapID submap_id) const {
  const auto submap_ptr_it = id_to_submap_.find(submap_id);
  if (submap_ptr_it != id_to_submap_.end()) {
    return submap_ptr_it->second;
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
  SubmapCollectionProto submap_collection_proto;
  getProto(&submap_collection_proto);
  // Write out the layer header.
  if (!voxblox::utils::writeProtoMsgToStream(submap_collection_proto,
                                             &outfile)) {
    LOG(ERROR) << "Could not write submap collection header message.";
    outfile.close();
    return false;
  }
  // Saving the tsdf submaps
  for (const auto& id_submap_pair : id_to_submap_) {
    LOG(INFO) << "Saving submap with ID: " << id_submap_pair.first;
    // Saving the submap
    bool success = (id_submap_pair.second)->saveToStream(&outfile);
    if (success) {
      LOG(INFO) << "Saving successful";
    } else {
      LOG(WARNING) << "Saving unsuccessful";
    }
  }
  // Closing the file
  outfile.close();
  return true;
}

template <typename SubmapType>
void SubmapCollection<SubmapType>::getProto(
    SubmapCollectionProto* proto) const {
  CHECK_NOTNULL(proto);
  // Filling out the description of the submap collection
  proto->set_num_submaps(num_patches());
}

template <typename SubmapType>
bool SubmapCollection<SubmapType>::LoadFromFile(
    const std::string& file_path,
    typename SubmapCollection<SubmapType>::Ptr* submap_collection_ptr) {
  CHECK_NOTNULL(submap_collection_ptr);
  // Open and check the file
  std::fstream proto_file;
  proto_file.open(file_path, std::fstream::in);
  if (!proto_file.is_open()) {
    LOG(ERROR) << "Could not open protobuf file to load layer: " << file_path;
    return false;
  }
  // Unused byte offset result.
  uint64_t tmp_byte_offset = 0u;
  // Loading the header
  SubmapCollectionProto submap_collection_proto;
  if (!voxblox::utils::readProtoMsgFromStream(
          &proto_file, &submap_collection_proto, &tmp_byte_offset)) {
    LOG(ERROR) << "Could not read tsdf submap collection map protobuf message.";
    return false;
  }

  LOG(INFO) << "tsdf_submap_collection_proto.num_submaps(): "
            << submap_collection_proto.num_submaps();

  // Loading each of the submaps
  for (size_t sub_map_index = 0u;
       sub_map_index < submap_collection_proto.num_submaps(); ++sub_map_index) {
    LOG(INFO) << "Loading submap number: " << sub_map_index;
    // Loading the submaps
    typename SubmapType::Ptr submap_ptr = SubmapType::LoadFromStream(
        (*submap_collection_ptr)->getConfig(), &proto_file, &tmp_byte_offset);
    if (submap_ptr == nullptr) {
      LOG(ERROR) << "Could not load the submap from stream.";
      return false;
    }
    (*submap_collection_ptr)->addSubmap(submap_ptr);
  }
  // Because grown ups clean up after themselves
  proto_file.close();
  return true;
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
size_t SubmapCollection<SubmapType>::getNumberOfAllocatedBlocks() const {
  // Looping over the submaps totalling the sizes
  size_t total_blocks = 0;
  for (const auto& id_submap_pair : id_to_submap_) {
    total_blocks += (id_submap_pair.second)->getNumberOfAllocatedBlocks();
  }
  return total_blocks;
}

template <typename SubmapType>
size_t SubmapCollection<SubmapType>::getMemorySize() const {
  // Looping over the submaps totalling the sizes
  size_t size = 0u;
  for (const auto& id_submap_pair : id_to_submap_) {
    size += (id_submap_pair.second)->getMemorySize();
  }
  return size;
}

}  // namespace cblox

#endif  // CBLOX_CORE_SUBMAP_COLLECTION_INL_H_
