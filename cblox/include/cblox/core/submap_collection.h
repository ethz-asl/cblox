#ifndef CBLOX_CORE_SUBMAP_COLLECTION_H_
#define CBLOX_CORE_SUBMAP_COLLECTION_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cblox/SubmapCollection.pb.h"
#include "cblox/core/common.h"
#include "cblox/core/tsdf_esdf_submap.h"

namespace cblox {

// A interface for use where the type of submap doesnt matter.
class SubmapCollectionInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef std::shared_ptr<SubmapCollectionInterface> Ptr;
  typedef std::shared_ptr<const SubmapCollectionInterface> ConstPtr;

  SubmapCollectionInterface() {}
  virtual ~SubmapCollectionInterface() {}

  // NOTE(alexmillane): I'm moving methods over only as I need them. There's no
  // design intent here in leaving some out. There is only the intent to be
  // lazy.
  virtual const Transformation& getActiveSubmapPose() const = 0;
  virtual SubmapID getActiveSubmapID() const = 0;
  virtual bool getSubmapPose(const SubmapID submap_id,
                             Transformation* pose_ptr) const = 0;

  virtual TsdfMap::Ptr getActiveTsdfMapPtr() = 0;
  virtual const TsdfMap& getActiveTsdfMap() const = 0;
  virtual TsdfMap::Ptr getTsdfMapPtr(const SubmapID submap_id) = 0;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual size_t num_patches() const = 0;
  virtual FloatingPoint block_size() const = 0;
};

// Collection of submaps
template <typename SubmapType>
class SubmapCollection : public SubmapCollectionInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SubmapCollection> Ptr;
  typedef std::shared_ptr<const SubmapCollection> ConstPtr;

  // Constructor. Constructs an empty submap collection map
  explicit SubmapCollection(const typename SubmapType::Config& submap_config)
      : SubmapCollectionInterface(),
        submap_config_(submap_config),
        active_submap_id_(-1) {}

  // Constructor. Constructs a submap collection from a list of submaps
  SubmapCollection(const typename SubmapType::Config& submap_config,
                   const std::vector<typename SubmapType::Ptr>& tsdf_sub_maps);

  virtual ~SubmapCollection() {}

  // Gets a vector of the linked IDs
  std::vector<SubmapID> getIDs() const;
  bool exists(const SubmapID submap_id) const;

  // Creates a new submap on the top of the collection
  // NOTE(alexmillane): T_G_S - Transformation between submap frame (S) and
  //                           the global tracking frame (G).
  // NOTE(alexmillane): Creating a new submap automatically makes it active.
  void createNewSubmap(const Transformation& T_G_S, const SubmapID submap_id);
  SubmapID createNewSubmap(const Transformation& T_G_S);

  void addSubmap(const typename SubmapType::Ptr submap);

  // Create a new submap which duplicates an existing source submap
  bool duplicateSubmap(const SubmapID source_submap_id,
                       const SubmapID new_submap_id);

  // Gets a const pointer to a raw submap
  // NOTE(alexmillane): This function hard fails when the submap doesn't
  // exist... This puts the onus on the caller to call exists() first. I don't
  // like this but I can't see a solution.
  const SubmapType& getSubmap(const SubmapID submap_id) const;
  // Note(alexmillane): Unlike the above method, the two methods below return a
  // nullptr when the map doesn't exist. No hard crash.
  typename SubmapType::Ptr getSubmapPtr(const SubmapID submap_id);
  typename SubmapType::ConstPtr getSubmapConstPtr(
      const SubmapID submap_id) const;
  // A list of the submaps
  const std::vector<typename SubmapType::Ptr> getSubmapPtrs() const;
  const std::vector<typename SubmapType::ConstPtr> getSubmapConstPtrs() const;

  // Removal
  void deleteSubmap(const SubmapID submap_id);

  // Interactions with the active submap
  const SubmapType& getActiveSubmap() const;
  typename SubmapType::Ptr getActiveSubmapPtr();
  const Transformation& getActiveSubmapPose() const;
  SubmapID getActiveSubmapID() const;

  // Access the tsdf_map member of the active submap
  TsdfMap::Ptr getActiveTsdfMapPtr();
  const TsdfMap& getActiveTsdfMap() const;
  // Access the tsdf_map member of any submap
  virtual TsdfMap::Ptr getTsdfMapPtr(const SubmapID submap_id);

  // Activate a submap
  // NOTE(alexmillane): Note that creating a new submap automatically activates
  //                    it.
  void activateSubmap(const SubmapID submap_id);

  // Interacting with the submap poses
  bool setSubmapPose(const SubmapID submap_id, const Transformation& pose);
  void setSubmapPoses(const SubmapIdPoseMap& id_pose_map);
  bool getSubmapPose(const SubmapID submap_id, Transformation* pose_ptr) const;
  void getSubmapPoses(TransformationVector* submap_poses) const;

  // Clears the collection, leaving an empty map
  void clear() { id_to_submap_.clear(); }

  // Size information
  bool empty() const { return id_to_submap_.empty(); }
  size_t size() const { return id_to_submap_.size(); }
  size_t num_patches() const { return id_to_submap_.size(); }

  // Note(alexmillane): These functions with result in a failed check if the
  // collection is empty. We could return 0 instead however that could results
  // in weird stuff in the client code.
  FloatingPoint block_size() const {
    CHECK(!id_to_submap_.empty());
    return (id_to_submap_.begin()->second)->block_size();
  }
  FloatingPoint voxel_size() const {
    CHECK(!id_to_submap_.empty());
    return (id_to_submap_.begin()->second)->block_size();
  }

  size_t getNumberOfAllocatedBlocks() const;

  // Returns the config of the submaps
  const typename SubmapType::Config& getConfig() const {
    return submap_config_;
  }

  // Save the collection to file
  bool saveToFile(const std::string& file_path) const;
  void getProto(SubmapCollectionProto* proto) const;

  // Fusing the submap pairs
  // Note(alexmillane): This function is not thread-safe. The user must take
  // care to ensure the fused submaps are not being modified while the fusion
  // takes place. Or someone can implement locking of the underlying submaps and
  // make a PR. :).
  void fuseSubmapPair(const SubmapIdPair& submap_id_pair);

  // Flattens the collection map down to a normal TSDF map
  TsdfMap::Ptr getProjectedMap() const;

  // Gets the combined memory size of the layers in this collection.
  size_t getMemorySize() const;

  // Loading from file
  static bool LoadFromFile(
      const std::string& file_path,
      typename SubmapCollection<SubmapType>::Ptr* submap_collection_ptr);
  static bool LoadFromStream(
      std::istream* proto_file_ptr,
      typename SubmapCollection<SubmapType>::Ptr* submap_collection_ptr);

 private:
  // TODO(alexmillane): Get some concurrency guards

  // The config used for the patches
  typename SubmapType::Config submap_config_;

  // The active SubmapID
  SubmapID active_submap_id_;

  // Submap storage and access
  std::map<SubmapID, typename SubmapType::Ptr> id_to_submap_;
};

}  // namespace cblox

#endif  // CBLOX_CORE_SUBMAP_COLLECTION_H_

#include "cblox/core/submap_collection_inl.h"
