#ifndef CBLOX_CORE_TSDF_ESDF_SUBMAP_H_
#define CBLOX_CORE_TSDF_ESDF_SUBMAP_H_

#include <memory>

#include <voxblox/integrator/esdf_integrator.h>

#include "cblox/core/tsdf_submap.h"

namespace cblox {

class TsdfEsdfSubmap : public TsdfSubmap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<TsdfEsdfSubmap> Ptr;
  typedef std::shared_ptr<const TsdfEsdfSubmap> ConstPtr;

  struct Config : TsdfSubmap::Config, EsdfMap::Config {
    // default constructor
    Config() : TsdfSubmap::Config(), EsdfMap::Config(){};
    // constructor based on tsdf and esdf config
    Config(const TsdfSubmap::Config& tsdf_map_config,
           const EsdfMap::Config& esdf_map_config)
        : TsdfSubmap::Config(tsdf_map_config),
          EsdfMap::Config(esdf_map_config){};
  };

  TsdfEsdfSubmap(const Transformation& T_M_S, const SubmapID submap_id,
                 const Config& config,
                 const voxblox::EsdfIntegrator::Config& esdf_integrator_config =
                     voxblox::EsdfIntegrator::Config())
      : TsdfSubmap(T_M_S, submap_id, config),
        config_(config),
        esdf_map_(new EsdfMap(config)),
        esdf_integrator_config_(esdf_integrator_config) {}

  // Creating a TsdfEsdfSubmap from a TsdfSubmap by SHARING the underlying
  // TsdfMap Makes a submap by SHARING the underlying TsdfMap with another
  // party.
  // NOTE(alexmillane): The submaps share a submap_id and transform.
  TsdfEsdfSubmap(TsdfSubmap::Ptr tsdf_submap,
                 const voxblox::EsdfIntegrator::Config& esdf_integrator_config =
                     voxblox::EsdfIntegrator::Config())
      : TsdfSubmap(tsdf_submap->getPose(), 
                   tsdf_submap->getID(),
                   tsdf_submap->getTsdfMapPtr()),
        esdf_integrator_config_(esdf_integrator_config) {
    CHECK(tsdf_submap);
    // Copying the config
    config_.tsdf_voxel_size = tsdf_submap->voxel_size();
    config_.tsdf_voxels_per_side = tsdf_submap->voxels_per_side();
    config_.esdf_voxel_size = tsdf_submap->voxel_size();
    config_.esdf_voxels_per_side = tsdf_submap->voxels_per_side();
    esdf_map_ = std::make_shared<EsdfMap>(config_);
  }

  virtual ~TsdfEsdfSubmap() {
    if (!esdf_map_.unique()) {
      LOG(WARNING) << "Underlying esdf map from SubmapID: " << submap_id_
                   << " is NOT unique. Therefore its memory may leak.";
    } else {
      LOG(INFO) << "EsdfSubmap " << submap_id_ << " is being deleted.";
    }
  }

  // Generate the ESDF from the TSDF.
  void generateEsdf();

  // Returns the underlying ESDF map pointers
  EsdfMap::Ptr getEsdfMapPtr() { return esdf_map_; }
  const EsdfMap& getEsdfMap() const { return *esdf_map_; }

  virtual Config getEsdfConfig() const { return config_; }

  virtual void finishSubmap() override;

  virtual void prepareForPublish() override;

  virtual void getProto(cblox::SubmapProto* proto) const override;

  virtual bool saveToStream(std::fstream* outfile_ptr) const override;

  // Load a submap from stream.
  // Note(alexmillane): Returns a nullptr if load is unsuccessful.
  static TsdfEsdfSubmap::Ptr LoadFromStream(const Config& config,
                                            std::istream* proto_file_ptr,
                                            uint64_t* tmp_byte_offset_ptr);

 protected:
  Config config_;
  EsdfMap::Ptr esdf_map_;
  voxblox::EsdfIntegrator::Config esdf_integrator_config_;
};

}  // namespace cblox

#endif  // CBLOX_CORE_TSDF_ESDF_SUBMAP_H_
