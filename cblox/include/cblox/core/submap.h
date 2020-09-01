#ifndef CBLOX_CORE_SUBMAP_H_
#define CBLOX_CORE_SUBMAP_H_

#include <mutex>

#include "cblox/core/common.h"

namespace cblox {

// Pure virtual class representing a submap.
class Submap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructors
  // Standard constructor
  Submap(const Transformation& T_M_S, SubmapID submap_id)
      : submap_id_(submap_id), T_M_S_(T_M_S) {}
  // Copy constructor (threadsafe)
  Submap(const Submap& rhs) : submap_id_(rhs.submap_id_) {
    std::unique_lock<std::mutex> rhs_lk(rhs.transformation_mutex_);
    T_M_S_ = rhs.T_M_S_;
  }
  // Move constructor (threadsafe)
  // NOTE: The implicit move constructor is deleted due to the
  //       transformation_mutex_, hence we need this constructor if we want to
  //       be able to move instances of this class or its derived classes
  Submap(Submap&& rhs) noexcept : submap_id_(rhs.submap_id_) {
    std::unique_lock<std::mutex> rhs_lk(rhs.transformation_mutex_);
    T_M_S_ = rhs.T_M_S_;
  }

  // Destructor
  // NOTE: It must be virtual, to ensure that derived class members are also
  //       destructed when the destructor is called through a base class pointer
  virtual ~Submap() = default;

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
