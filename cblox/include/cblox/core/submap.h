#ifndef CBLOX_CORE_SUBMAP_H_
#define CBLOX_CORE_SUBMAP_H_

#include <mutex>

#include "cblox/core/common.h"

namespace cblox {

// Pure virtual class representing a submap.
class Submap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor.
  Submap(const Transformation& T_M_S, SubmapID submap_id)
      : submap_id_(submap_id), T_M_S_(T_M_S) {}

  virtual ~Submap() {}

  // Submap pose interaction.
  inline const Transformation& getPose() const {
    std::unique_lock<std::mutex> lock(transformation_mutex_);
    return T_M_S_;
  }

  inline void setPose(const Transformation& T_M_S) {
    std::unique_lock<std::mutex> lock(transformation_mutex_);
    T_M_S_ = T_M_S;
  }

  inline SubmapID getID() const { return submap_id_; }

  virtual size_t getNumberOfAllocatedBlocks() const = 0;

  virtual size_t getMemorySize() const = 0;

  virtual void finishSubmap() = 0;

  virtual void prepareForPublish() = 0;

  /*
  // Note(ntonci): In order to provide saving/loading functionality to the
  // derived class, the following methods should be implemented (see TsdfSubmap
  // as an example):

  // Getting the proto for this submap.
  virtual void getProto(SubmapProto* proto) const = 0;
  // Save the submap to file.
  virtual bool saveToStream(std::fstream* outfile_ptr) const;
  // Load a submap from stream.
  // Note(alexmillane): Returns a nullptr if load is unsuccessful.
  static Submap::Ptr LoadFromStream(const Config& config,
                                    std::fstream* proto_file_ptr,
                                    uint64_t* tmp_byte_offset_ptr);
  */

 protected:
  const SubmapID submap_id_;
  Transformation T_M_S_;

 private:
  mutable std::mutex transformation_mutex_;
};

}  // namespace cblox

#endif  // CBLOX_CORE_SUBMAP_H_
