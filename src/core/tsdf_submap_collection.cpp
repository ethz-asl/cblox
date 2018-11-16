#include <iostream>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <voxblox/integrator/merge_integration.h>
#include <voxblox/utils/protobuf_utils.h>

#include "cblox/core/tsdf_submap_collection.h"

namespace cblox {

void TsdfSubmapCollection::getIDs(std::vector<SubmapID> *submap_ids) const {
  // Checks
  CHECK_NOTNULL(submap_ids);
  submap_ids->reserve(tsdf_sub_maps_.size());
  // Populating the vector
  for (TsdfSubmap::ConstPtr tsdf_sub_map_ptr : tsdf_sub_maps_) {
    submap_ids->push_back(tsdf_sub_map_ptr->getID());
  }
}

bool TsdfSubmapCollection::isBaseFrame(const SubmapID &submap_id) const {
  // Searching for the passed submap ID
  const auto submap_ptr_it =
      std::find_if(tsdf_sub_maps_.begin(), tsdf_sub_maps_.end(),
                   [submap_id](const TsdfSubmap::Ptr &sub_map_ptr) {
                     return sub_map_ptr->getID() == submap_id;
                   });
  if (submap_ptr_it != tsdf_sub_maps_.end()) {
    return true;
  } else {
    return false;
  }
}

void TsdfSubmapCollection::createNewSubMap(const Transformation &T_M_S,
                                           SubmapID submap_id) {
  // Creating the new submap and adding it to the list
  TsdfSubmap::Ptr tsdf_sub_map(
      new TsdfSubmap(T_M_S, submap_id, tsdf_map_config_));
  tsdf_sub_maps_.push_back(tsdf_sub_map);
}

TsdfMap::Ptr TsdfSubmapCollection::getProjectedMap() const {
  // Creating the global tsdf map and getting its tsdf layer
  TsdfMap::Ptr projected_tsdf_map_ptr =
      TsdfMap::Ptr(new TsdfMap(tsdf_map_config_));
  Layer<TsdfVoxel>* combined_tsdf_layer_ptr =
      projected_tsdf_map_ptr->getTsdfLayerPtr();
  // Looping over the current submaps
  for (TsdfSubmap::ConstPtr tsdf_sub_map_ptr : tsdf_sub_maps_) {
    // Getting the tsdf submap and its pose
    const TsdfMap& tsdf_map = tsdf_sub_map_ptr->getTsdfMap();
    const Transformation& T_M_S = tsdf_sub_map_ptr->getPose();
    // Merging layers the submap into the global layer
    mergeLayerAintoLayerB(tsdf_map.getTsdfLayer(), T_M_S,
                          combined_tsdf_layer_ptr);
  }
  // Returning the new map
  return projected_tsdf_map_ptr;
}

bool TsdfSubmapCollection::setSubMapPose(const SubmapID submap_id,
                                         const Transformation &pose) {
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
    const TransformationVector &transforms) {
  // Updating the poses
  // NOTE(alexmillane): This assumes that the order of transforms matches the
  //                    submap order.
  for (size_t sub_map_index = 0; sub_map_index < transforms.size();
       sub_map_index++) {
    tsdf_sub_maps_[sub_map_index]->setPose(transforms[sub_map_index]);
  }
}

void TsdfSubmapCollection::getSubMapPoses(
    AlignedVector<Transformation>* submap_poses_ptr) const {
  // Checks
  CHECK_NOTNULL(submap_poses_ptr);
  // Extracting transforms
  submap_poses_ptr->clear();
  submap_poses_ptr->reserve(tsdf_sub_maps_.size());
  for (TsdfSubmap::ConstPtr tsdf_sub_map : tsdf_sub_maps_) {
    submap_poses_ptr->push_back(tsdf_sub_map->getPose());
  }
}

bool TsdfSubmapCollection::getAssociatedTsdfSubMapID(
    const SubmapID submap_id, SubmapID *submap_id_ptr) const {
  const auto tsdf_submap_ptr_it = id_to_submap_.find(submap_id);
  if (tsdf_submap_ptr_it != id_to_submap_.end()) {
    *submap_id_ptr = (*tsdf_submap_ptr_it).second->getID();
    return true;
  } else {
    // std::cout << "Cant find the requested submap_id: " << submap_id
    //          << " associated with any submap" << std::endl;
    return false;
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
  size_t sub_map_index = 0;
  for (TsdfSubmap::ConstPtr tsdf_sub_map_ptr : tsdf_sub_maps_) {
    // DEBUG
    std::cout << "Saving tsdf_sub_map number: " << sub_map_index << std::endl;
    // Saving the submap
    tsdf_sub_map_ptr->saveToStream(&outfile);
    sub_map_index++;
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
void TsdfSubmapCollection::fuseSubmapPair(const SubmapIdPair &submap_id_pair) {
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
    // Searching for ids associated with submap 2
    for (auto &id_submap_pair : id_to_submap_) {
      if (id_submap_pair.second == submap_ptr_2) {
        id_submap_pair.second = submap_ptr_1;
        // std::cout << "Moved submap ID: " << id_submap_pair.first
        //          << " from submap ID: " << submap_ptr_2->getID()
        //          << " to submap ID: " << submap_ptr_1->getID()
        //          << std::endl;
      }
    }
    // Searching the submaps and deleting submap 2
    for (auto submap_ptr_it = tsdf_sub_maps_.begin();
         submap_ptr_it != tsdf_sub_maps_.end(); submap_ptr_it++) {
      if (*submap_ptr_it == submap_ptr_2) {
        tsdf_sub_maps_.erase(submap_ptr_it);
        std::cout << "Erased the submap: " << submap_ptr_2->getID()
                  << " from the submap collection" << std::endl;
        break;
      }
    }
  } else {
    std::cout << "Could not find the requested submap pair during fusion."
              << std::endl;
  }
}

size_t TsdfSubmapCollection::getNumberAllocatedBlocks() const {
  // Looping over the submaps totalling the sizes
  size_t total_blocks = 0;
  for (const TsdfSubmap::Ptr& tsdf_sub_map_ptr : tsdf_sub_maps_) {
    total_blocks += tsdf_sub_map_ptr->getNumberAllocatedBlocks();
  }
  return total_blocks;
}

} // namespace cblox
