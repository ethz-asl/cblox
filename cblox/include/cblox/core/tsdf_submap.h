#ifndef CBLOX_CORE_TSDF_SUBMAP_H_
#define CBLOX_CORE_TSDF_SUBMAP_H_

#include <memory>
#include <mutex>

#include <Eigen/Geometry>

#include <glog/logging.h>

#include "cblox/Submap.pb.h"
#include "cblox/core/common.h"

namespace cblox {

// Class representing TSDF submap
class TsdfSubmap {
 public:
  typedef std::shared_ptr<TsdfSubmap> Ptr;
  typedef std::shared_ptr<const TsdfSubmap> ConstPtr;
  typedef TsdfMap::Config Config;

  // Constructor
  TsdfSubmap(const Transformation& T_M_S, SubmapID submap_id, Config config)
      : submap_id_(submap_id), T_M_S_(T_M_S) {
    tsdf_map_.reset(new TsdfMap(config));
  }

  ~TsdfSubmap() {
    if (!tsdf_map_.unique()) {
      LOG(WARNING) << "Underlying tsdf map from SubmapID: " << submap_id_
                   << " is NOT unique. Therefore its memory may leak.";
    } else {
      LOG(INFO) << "TsdfSubmap " << submap_id_ << " is being deleted.";
    }
  }

  // Returns the underlying TSDF map pointers
  TsdfMap::Ptr getTsdfMapPtr() { return tsdf_map_; }
  const TsdfMap& getTsdfMap() const { return *tsdf_map_; }

  // Submap pose interaction
  const Transformation& getPose() const {
    std::unique_lock<std::mutex> lock(transformation_mutex_);
    return T_M_S_;
  }

  void setPose(const Transformation& T_M_S) {
    std::unique_lock<std::mutex> lock(transformation_mutex_);
    T_M_S_ = T_M_S;
  }

  // Set interval in which submap was actively mapping
  void startMappingTime(int32_t time) { mapping_interval_.first = time; }
  void stopMappingTime(int32_t time) { mapping_interval_.second = time; }
  // Access mapping interval
  const std::pair<int32_t, int32_t>& getMappingInterval() const {
    return mapping_interval_;
  }

  SubmapID getID() const { return submap_id_; }

  FloatingPoint block_size() const { return tsdf_map_->block_size(); }

  size_t getNumberAllocatedBlocks() const {
    return tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks();
  }

  size_t getMemorySize() const {
    return tsdf_map_->getTsdfLayer().getMemorySize();
  }

  virtual void finishSubmap();

  virtual void prepareForPublish();

  // Getting the proto for this submap
  virtual void getProto(SubmapProto* proto) const;

  // Save the submap to file
  virtual bool saveToStream(std::fstream* outfile_ptr) const;

 protected:
  SubmapID submap_id_;
  TsdfMap::Ptr tsdf_map_;

  Transformation T_M_S_;
  std::pair<int32_t, int32_t> mapping_interval_;

 private:
  mutable std::mutex transformation_mutex_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace cblox

#endif  // CBLOX_CORE_TSDF_SUBMAP_H_
