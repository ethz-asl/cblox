#ifndef CBLOX_CORE_TSDF_SUBMAP_COLLECTION_MAP_H_
#define CBLOX_CORE_TSDF_SUBMAP_COLLECTION_MAP_H_

#include <voxblox/core/tsdf_map.h>

#include "./TsdfSubmapCollection.pb.h"

#include "cblox/core/common.hpp"
#include "cblox/core/tsdf_submap.hpp"

namespace cblox {

using namespace voxblox;

class TsdfSubmapCollection {
 public:
  typedef std::shared_ptr<TsdfSubmapCollection> Ptr;
  typedef std::shared_ptr<const TsdfSubmapCollection> ConstPtr;

  // Constructor. Constructs an empty submap collection map
  TsdfSubmapCollection(const TsdfMap::Config& tsdf_map_config)
      : tsdf_map_config_(tsdf_map_config){};

  // Constructor. Constructs a submap collection from a list of submaps
  TsdfSubmapCollection(const TsdfMap::Config &tsdf_map_config,
                       const std::vector<TsdfSubmap::Ptr> &tsdf_sub_maps)
      : tsdf_map_config_(tsdf_map_config), tsdf_sub_maps_(tsdf_sub_maps){};

  // Gets a vector of the linked keyframe IDs
  void getLinkedKeyframeIds(std::vector<KFId>* keyframe_ids) const;
  bool isBaseFrame(const KFId& kf_id) const;

  // Creates a new submap on the top of the collection
  void createNewSubMap(const Transformation& T_M_S, KFId keyframe_id);

  // Gets a const reference to a raw submap
  const TsdfSubmap& getSubMap(size_t sub_map_index) const {
    CHECK_LT(sub_map_index, tsdf_sub_maps_.size());
    return *(tsdf_sub_maps_[sub_map_index]);
  };

  // Gets a const reference to the raw submap vector
  const std::vector<TsdfSubmap::Ptr>& getSubMaps() const {
    return tsdf_sub_maps_;
  };

  // Flattens the collection map down to a normal TSDF map
  TsdfMap::Ptr getProjectedMap() const;

  // Gets the pose of the patch on the tip of the collection
  const Transformation getActiveSubMapPose() const {
    return tsdf_sub_maps_.back()->getPose();
  }

  // Gets a pointer to the active tsdf_map
  TsdfMap::Ptr getActiveTsdfMapPtr() {
    return tsdf_sub_maps_.back()->getTsdfMapPtr();
  };
  // Gets a reference to the active tsdf_map
  const TsdfMap& getActiveTsdfMap() const {
    return tsdf_sub_maps_.back()->getTsdfMap();
  };

  // Gets a reference to the active tsdf_map
  const TsdfSubmap& getActiveTsdfSubMap() const {
    return *(tsdf_sub_maps_.back());
  };

  // Associates a keyframe to the active submap
  void associateKFToActiveSubmap(const KFId kf_id) {
    kf_to_submap_[kf_id] = tsdf_sub_maps_.back();
  }

  // Gets the tsdf submap associated with the passed keyframe ID
  bool getAssociatedTsdfSubMapID(const KFId kf_id, KFId* submap_id_ptr) const;

  // Interacting with the submap poses
  bool setSubMapPose(const KFId kf_id, const Transformation& pose);
  void setSubMapPoses(const TransformationVector& transforms);
  void getSubMapPoses(AlignedVector<Transformation>* submap_poses) const;

  // Clears the collection, leaving an empty map
  void clear() { tsdf_sub_maps_.clear(); };

  // Returns true if the collection is empty
  bool empty() const { return tsdf_sub_maps_.empty(); };

  // The size of the collection (number of patches)
  size_t size() const { return tsdf_sub_maps_.size(); };
  size_t num_patches() const { return tsdf_sub_maps_.size(); };

  // Returns the block size of the blocks in the tsdf patches
  FloatingPoint block_size() const {
    return tsdf_sub_maps_.back()->block_size();
  };

  // Save the collection to file
  bool saveToFile(const std::string& file_path) const;

  // Getting various protos for this object
  void getProto(TsdfSubmapCollectionProto* proto) const;

  // Returns the config of the tsdf sub maps
  const TsdfMap::Config& getConfig() const { return tsdf_map_config_; };

  // Fusing the submap pairs
  void fuseSubmapPair(const KFIdPair& kf_id_pair);

  // Gets the number of allocated blocks in the collection
  size_t getNumberAllocatedBlocks() const;

 private:
  // TODO(alexmillane): Get some concurrency guards

  // The config used for the patches
  TsdfMap::Config tsdf_map_config_;

  // The vectors of patches
  std::vector<TsdfSubmap::Ptr> tsdf_sub_maps_;

  // A map keeping track of which KF belongs to which submap
  std::map<KFId, TsdfSubmap::Ptr> kf_to_submap_;
};

}  // namespace cblox

#endif /* CBLOX_CORE_TSDF_SUBMAP_COLLECTION_MAP_H_ */
