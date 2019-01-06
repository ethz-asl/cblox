#ifndef CBLOX_CORE_TSDF_SUBMAP_COLLECTION_MAP_H_
#define CBLOX_CORE_TSDF_SUBMAP_COLLECTION_MAP_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <voxblox/core/tsdf_map.h>

#include "./TsdfSubmapCollection.pb.h"

#include "cblox/core/common.h"
#include "cblox/core/tsdf_submap.h"

namespace cblox {

using namespace voxblox;

class TsdfSubmapCollection {
 public:
  typedef std::shared_ptr<TsdfSubmapCollection> Ptr;
  typedef std::shared_ptr<const TsdfSubmapCollection> ConstPtr;

  // Constructor. Constructs an empty submap collection map
  explicit TsdfSubmapCollection(const TsdfMap::Config &tsdf_map_config)
      : tsdf_map_config_(tsdf_map_config) {}

  // Constructor. Constructs a submap collection from a list of submaps
  TsdfSubmapCollection(const TsdfMap::Config &tsdf_map_config,
                       const std::vector<TsdfSubmap::Ptr> &tsdf_sub_maps);

  // Gets a vector of the linked IDs
  std::vector<SubmapID> getIDs() const;
  bool exists(const SubmapID submap_id) const;

  // Creates a new submap on the top of the collection
  void createNewSubMap(const Transformation &T_M_S, const SubmapID submap_id);
  void createNewSubMap(const Transformation &T_M_S);

  // Create a new submap which duplicates an existing source submap
  bool duplicateSubMap(const SubmapID source_submap_id,
                       const SubmapID new_submap_id);

  // Gets a const pointer to a raw submap
  // NOTE(alexmillane): This function hard fails when the submap doesn't
  // exist... This puts the onus on the caller to call exists() first. I don't
  // like this but I can't see a solution.
  const TsdfSubmap &getSubMap(const SubmapID submap_id) const;
  // Note(alexmillane): Unlike the above this function returns a nullptr when
  // the map doesn't exist. No hard crash.
  TsdfSubmap::ConstPtr getTsdfSubmapConstPtrById(
      const SubmapID submap_id) const;
  // A list of the submaps
  const std::vector<TsdfSubmap::Ptr> getSubMaps() const;

  // Interactions with the active submap
  TsdfMap::Ptr getActiveTsdfMapPtr();
  const TsdfMap &getActiveTsdfMap() const;
  const TsdfSubmap &getActiveTsdfSubMap() const;
  const Transformation &getActiveSubMapPose() const;
  const SubmapID getActiveSubMapID() const;

  // Interacting with the submap poses
  bool setSubMapPose(const SubmapID submap_id, const Transformation &pose);
  void setSubMapPoses(const TransformationVector &transforms);
  bool getSubMapPose(const SubmapID submap_id, Transformation *pose_ptr) const;
  void getSubMapPoses(AlignedVector<Transformation> *submap_poses) const;

  // Clears the collection, leaving an empty map
  void clear() { id_to_submap_.clear(); }

  // Size information
  bool empty() const { return id_to_submap_.empty(); }
  size_t size() const { return id_to_submap_.size(); }
  size_t num_patches() const { return id_to_submap_.size(); }
  FloatingPoint block_size() const {
    return (id_to_submap_.begin()->second)->block_size();
  }
  size_t getNumberAllocatedBlocks() const;

  // Returns the config of the tsdf sub maps
  const TsdfMap::Config &getConfig() const { return tsdf_map_config_; }

  // Save the collection to file
  bool saveToFile(const std::string &file_path) const;
  void getProto(TsdfSubmapCollectionProto *proto) const;

  // Fusing the submap pairs
  void fuseSubmapPair(const SubmapIdPair &submap_id_pair);

  // Flattens the collection map down to a normal TSDF map
  TsdfMap::Ptr getProjectedMap() const;

  // KEYFRAME RELATED FUNCTIONS.
  // COMMENTED OUT FOR NOW, BUT NEED TO BE MOVED TO MANIFOLD MAPPING.
  // Associates a to the active submap
  // void associateIDToActiveSubmap(const SubmapID submap_id) {
  //  id_to_submap_[submap_id] = tsdf_sub_maps_.back();
  //}
  // bool getAssociatedTsdfSubMapID(const SubmapID submap_id,
  //                               SubmapID *submap_id_ptr) const;
  //
  // bool getAssociatedTsdfSubMapID(const SubmapID submap_id,
  //                               SubmapID *submap_id_ptr) const;

 private:
  // TODO(alexmillane): Get some concurrency guards

  // The config used for the patches
  TsdfMap::Config tsdf_map_config_;

  // The active SubmapID
  SubmapID active_submap_id_;

  // Submap storage and access
  std::map<SubmapID, TsdfSubmap::Ptr> id_to_submap_;
};

}  // namespace cblox

#endif /* CBLOX_CORE_TSDF_SUBMAP_COLLECTION_MAP_H_ */
