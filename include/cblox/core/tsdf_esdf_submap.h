//
// Created by victor on 07.01.19.
//

#ifndef CBLOX_CORE_TSDF_ESDF_SUBMAP_H_
#define CBLOX_CORE_TSDF_ESDF_SUBMAP_H_

#include <memory>
#include "cblox/core/tsdf_submap.h"

namespace cblox {

class TsdfEsdfSubmap : public TsdfSubmap {
 public:
  typedef std::shared_ptr<TsdfEsdfSubmap> Ptr;
  typedef std::shared_ptr<const TsdfEsdfSubmap> ConstPtr;

  struct Config : TsdfSubmap::Config, EsdfMap::Config {};

  TsdfEsdfSubmap(const Transformation &T_M_S, SubmapID submap_id, Config config)
      : TsdfSubmap(T_M_S, submap_id, config), config_(config) {
    esdf_map_.reset(new EsdfMap(config));
  }

  ~TsdfEsdfSubmap() {
    if (!esdf_map_.unique()) {
      std::cout << "Underlying esdf map from SubmapID: " << submap_id_
                << " is NOT unique. Therefore its memory may leak."
                << std::endl;
    } else {
      std::cout << "EsdfSubmap " << submap_id_ << " is being deleted."
                << std::endl;
    }
  }

  // Generate the ESDF from the TSDF
  void generateEsdf();

  // Returns the underlying ESDF map pointers
  EsdfMap::Ptr getEsdfMapPtr() { return esdf_map_; }
  const EsdfMap &getEsdfMap() const { return *esdf_map_; }

  /* NOTE: When converting TsdfEsdf submaps into protobuffs, only their
   *       TSDF map is converted. The ESDF can be recomputed when needed.
   *       If you'd like to also store the ESDF, override the getProto() and
   *       saveToStream() methods from tsdf_submap.
   */

 protected:
  Config config_;
  EsdfMap::Ptr esdf_map_;
};
}  // namespace cblox

#endif  // CBLOX_CORE_TSDF_ESDF_SUBMAP_H_
