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

  // Constructor
  TsdfSubmap(const Transformation& T_M_S, SubmapID submap_id, Config config)
      : Submap(T_M_S, submap_id),
        tsdf_map_(std::make_shared<TsdfMap>(config)) {}
  // Copy constructor
  // NOTE: This performs a deep copy (full TSDF is copied)
  TsdfSubmap(const TsdfSubmap& rhs)
      : Submap(rhs),
        tsdf_map_(std::make_shared<TsdfMap>(*rhs.tsdf_map_)),
        mapping_interval_(rhs.mapping_interval_) {}
  // Move constructor
  // NOTE: This will move the TSDF by transferring smart pointer ownership
  TsdfSubmap(TsdfSubmap&& rhs) = default;

  ~TsdfSubmap() override {
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
                                        std::fstream* proto_file_ptr,
                                        uint64_t* tmp_byte_offset_ptr);

 protected:
  TsdfMap::Ptr tsdf_map_;
  std::pair<int64_t, int64_t> mapping_interval_;
};

}  // namespace cblox

#endif  // CBLOX_CORE_TSDF_SUBMAP_H_
