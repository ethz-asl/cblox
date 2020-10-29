#ifndef CBLOX_CORE_TSDF_SUBMAP_H_
#define CBLOX_CORE_TSDF_SUBMAP_H_

#include <memory>
#include <mutex>

#include <Eigen/Geometry>

#include <glog/logging.h>

#include "cblox/Submap.pb.h"
#include "cblox/core/common.h"
#include "cblox/core/submap.h"

namespace cblox {

// Class representing TSDF submap.
class TsdfSubmap : public Submap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TsdfSubmap> Ptr;
  typedef std::shared_ptr<const TsdfSubmap> ConstPtr;
  typedef TsdfMap::Config Config;

  TsdfSubmap(const Transformation& T_M_S, const SubmapID submap_id,
             const Config& config)
      : Submap(T_M_S, submap_id),
        config_(config),
        tsdf_map_(new TsdfMap(config)) {}

  // Makes a submap by SHARING the underlying TsdfMap with another party.
  TsdfSubmap(const Transformation& T_M_S, const SubmapID submap_id,
             TsdfMap::Ptr tsdf_map_ptr)
      : Submap(T_M_S, submap_id), tsdf_map_(tsdf_map_ptr) {
    CHECK(tsdf_map_ptr);
    // I think uniform initialization wont work until c++14 apparently. Lame.
    config_.tsdf_voxel_size = tsdf_map_ptr->getTsdfLayer().voxel_size();
    config_.tsdf_voxels_per_side =
        tsdf_map_ptr->getTsdfLayer().voxels_per_side();
  }

  virtual ~TsdfSubmap() {
    if (!tsdf_map_.unique()) {
      LOG(WARNING) << "Underlying tsdf map from SubmapID: " << submap_id_
                   << " is NOT unique. Therefore its memory may leak.";
    } else {
      LOG(INFO) << "TsdfSubmap " << submap_id_ << " is being deleted.";
    }
  }

  // Returns the underlying TSDF map pointers.
  inline TsdfMap::Ptr getTsdfMapPtr() { return tsdf_map_; }
  inline const TsdfMap& getTsdfMap() const { return *tsdf_map_; }

  inline FloatingPoint block_size() const { return tsdf_map_->block_size(); }
  inline FloatingPoint voxel_size() const { return tsdf_map_->voxel_size(); }
  inline size_t voxels_per_side() const {
    return tsdf_map_->getTsdfLayer().voxels_per_side();
  }

  Config getTsdfConfig() const { return config_; }

  // Set interval in which submap was actively mapping.
  inline void startMappingTime(int64_t time) { mapping_interval_.first = time; }
  inline void stopMappingTime(int64_t time) { mapping_interval_.second = time; }

  // Access mapping interval.
  inline const std::pair<int64_t, int64_t>& getMappingInterval() const {
    return mapping_interval_;
  }

  virtual size_t getNumberOfAllocatedBlocks() const override {
    return tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks();
  }

  virtual size_t getMemorySize() const override {
    return tsdf_map_->getTsdfLayer().getMemorySize();
  }

  virtual void finishSubmap() override;

  virtual void prepareForPublish() override;

  // Getting the proto for this submap.
  virtual void getProto(SubmapProto* proto) const;

  // Save the submap to file.
  virtual bool saveToStream(std::fstream* outfile_ptr) const;

  // Load a submap from stream.
  // Note(alexmillane): Returns a nullptr if load is unsuccessful.
  static TsdfSubmap::Ptr LoadFromStream(const Config& config,
                                        std::istream* proto_file_ptr,
                                        uint64_t* tmp_byte_offset_ptr);

 protected:
  Config config_;
  TsdfMap::Ptr tsdf_map_;
  std::pair<int64_t, int64_t> mapping_interval_;
};

}  // namespace cblox

#endif  // CBLOX_CORE_TSDF_SUBMAP_H_
