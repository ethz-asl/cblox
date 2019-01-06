#include <iostream>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/protobuf_utils.h>

#include "cblox/core/tsdf_submap_collection.h"

namespace cblox {

TsdfSubmapCollection::TsdfSubmapCollection(
    const TsdfMap::Config& tsdf_map_config,
    const std::vector<TsdfSubmap::Ptr>& tsdf_sub_maps)
    : tsdf_map_config_(tsdf_map_config) {
  // Constructing from a list of existing submaps
  // NOTE(alexmillane): assigning arbitrary SubmapIDs
  SubmapID submap_id = 0;
  for (const auto& tsdf_submap_ptr : tsdf_sub_maps) {
    id_to_submap_[submap_id] = tsdf_submap_ptr;
    submap_id++;
  }
}

std::vector<SubmapID> TsdfSubmapCollection::getIDs() const {
  std::vector<SubmapID> submap_ids;
  submap_ids.reserve(id_to_submap_.size());
  for (const auto& id_submap_pair : id_to_submap_) {
    submap_ids.emplace_back(id_submap_pair.first);
  }
  return submap_ids;
}

bool TsdfSubmapCollection::exists(const SubmapID submap_id) const {
  // Searching for the passed submap ID
  const auto it = id_to_submap_.find(submap_id);
  if (it != id_to_submap_.end()) {
    return true;
  } else {
    return false;
  }
}

void TsdfSubmapCollection::createNewSubMap(const Transformation& T_M_S,
                                           const SubmapID submap_id) {
  // Checking if the submap already exists
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  // Creating the new submap and adding it to the list
  TsdfSubmap::Ptr tsdf_sub_map(
      new TsdfSubmap(T_M_S, submap_id, tsdf_map_config_));
  id_to_submap_.emplace(submap_id, std::move(tsdf_sub_map));
  // Updating the active submap
  active_submap_id_ = submap_id;
}

void TsdfSubmapCollection::createNewSubMap(const Transformation& T_M_S) {
  // Creating a submap with a generated SubmapID
  // NOTE(alexmillane): rbegin() returns the pair with the highest key.
  SubmapID new_ID = 0;
  if (!id_to_submap_.empty()) {
    new_ID = id_to_submap_.rbegin()->first + 1;
  }
  createNewSubMap(T_M_S, new_ID);
}

bool TsdfSubmapCollection::duplicateSubMap(const SubmapID source_submap_id,
                                           const SubmapID new_submap_id) {
  // Get pointer to the source submap
  const auto src_submap_ptr_it = id_to_submap_.find(source_submap_id);
  if (src_submap_ptr_it != id_to_submap_.end()) {
    TsdfSubmap::Ptr src_submap_ptr = src_submap_ptr_it->second;
    // Create a new submap with the same pose and get its pointer
    const Transformation T_M_S = src_submap_ptr->getPose();
    // Creating the new submap and adding it to the list
    TsdfSubmap::Ptr new_tsdf_sub_map(
        new TsdfSubmap(T_M_S, new_submap_id, tsdf_map_config_));
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
const TsdfSubmap& TsdfSubmapCollection::getSubMap(
    const SubmapID submap_id) const {
  const auto it = id_to_submap_.find(submap_id);
  CHECK(it != id_to_submap_.end());
  return *it->second;
}

const std::vector<TsdfSubmap::Ptr> TsdfSubmapCollection::getSubMaps() const {
  std::vector<TsdfSubmap::Ptr> submap_ptrs;
  for (const auto& blah : id_to_submap_) {
    submap_ptrs.emplace_back(blah.second);
  }
  return submap_ptrs;
}

// Gets a pointer to the active tsdf_map
TsdfMap::Ptr TsdfSubmapCollection::getActiveTsdfMapPtr() {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getTsdfMapPtr();
}
// Gets a reference to the active tsdf_map
const TsdfMap& TsdfSubmapCollection::getActiveTsdfMap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return (it->second)->getTsdfMap();
}

// Gets a reference to the active tsdf_map
const TsdfSubmap& TsdfSubmapCollection::getActiveTsdfSubMap() const {
  const auto it = id_to_submap_.find(active_submap_id_);
  CHECK(it != id_to_submap_.end());
  return *(it->second);
}

const Transformation& TsdfSubmapCollection::getActiveSubMapPose() const {
  return getActiveTsdfSubMap().getPose();
}
const SubmapID TsdfSubmapCollection::getActiveSubMapID() const {
  return active_submap_id_;
}

TsdfMap::Ptr TsdfSubmapCollection::getProjectedMap() const {
  // Creating the global tsdf map and getting its tsdf layer
  TsdfMap::Ptr projected_tsdf_map_ptr =
      TsdfMap::Ptr(new TsdfMap(tsdf_map_config_));
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

bool TsdfSubmapCollection::setSubMapPose(const SubmapID submap_id,
                                         const Transformation& pose) {
  // Looking for the submap
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    TsdfSubmap::Ptr submap_ptr = (*tsdf_submap_ptr_it).second;
    submap_ptr->setPose(pose);
    return true;
  } else {
    std::cout << "Tried to set the pose of the submap with submap_id: "
              << submap_id << " and could not find the linked submap."
              << std::endl;
    return false;
  }
}

void TsdfSubmapCollection::setSubMapPoses(
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

bool TsdfSubmapCollection::getSubMapPose(const SubmapID submap_id,
                                         Transformation* pose_ptr) const {
  // Looking for the submap
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    TsdfSubmap::Ptr submap_ptr = (*tsdf_submap_ptr_it).second;
    *pose_ptr = submap_ptr->getPose();
    return true;
  } else {
    std::cout << "Tried to get the pose of the submap with submap_id: "
              << submap_id << " and could not find the linked submap."
              << std::endl;
    return false;
  }
}

void TsdfSubmapCollection::getSubMapPoses(
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

/*bool TsdfSubmapCollection::getAssociatedTsdfSubMapID(
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

TsdfSubmap::ConstPtr TsdfSubmapCollection::getTsdfSubmapConstPtrById(
    const SubmapID submap_id) const {
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    return tsdf_submap_ptr_it->second;
  } else {
    // std::cout << "Cant find the requested submap_id: " << submap_id
    //          << " associated with any submap" << std::endl;
    return TsdfSubmap::ConstPtr();
  }
}

bool TsdfSubmapCollection::saveToFile(const std::string& file_path) const {
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
  if (!utils::writeProtoMsgToStream(tsdf_submap_collection_proto, &outfile)) {
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

void TsdfSubmapCollection::getProto(TsdfSubmapCollectionProto* proto) const {
  // Checks
  CHECK_NOTNULL(proto);
  // Filling out the description of the submap collection
  proto->set_voxel_size(tsdf_map_config_.tsdf_voxel_size);
  proto->set_voxels_per_side(tsdf_map_config_.tsdf_voxels_per_side);
  proto->set_num_submaps(num_patches());
}

// Fusing the submap pairs
void TsdfSubmapCollection::fuseSubmapPair(const SubmapIdPair& submap_id_pair) {
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
    TsdfSubmap::Ptr submap_ptr_1 = (*id_submap_pair_1_it).second;
    TsdfSubmap::Ptr submap_ptr_2 = (*id_submap_pair_2_it).second;
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

size_t TsdfSubmapCollection::getNumberAllocatedBlocks() const {
  // Looping over the submaps totalling the sizes
  size_t total_blocks = 0;
  for (const auto& id_submap_pair : id_to_submap_) {
    total_blocks += (id_submap_pair.second)->getNumberAllocatedBlocks();
  }
  return total_blocks;
}

}  // namespace cblox
