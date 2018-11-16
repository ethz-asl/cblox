#ifndef CBLOX_CORE_COMMON_H_
#define CBLOX_CORE_COMMON_H_

#include <deque>
#include <map>
#include <utility>
#include <vector>

#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

#include <voxblox/core/common.h>
#include <voxblox/utils/timing.h>

namespace cblox {

// Matching voxblox's floating point settings
typedef voxblox::FloatingPoint FloatingPoint;

// Transformation typedefs
typedef kindr::minimal::QuatTransformationTemplate<FloatingPoint>
    Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<FloatingPoint> Quaternion;

// Defines
typedef unsigned int SubmapID;
typedef std::pair<SubmapID, SubmapID> SubmapIdPair;

// Aligned Eigen containers
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;
template <typename Type>
using AlignedDeque = std::deque<Type, Eigen::aligned_allocator<Type>>;

// Containers of transforms
typedef AlignedVector<Transformation> TransformationVector;

// Taking timing from voxblox
namespace timing {
using namespace voxblox::timing;
}  // namespace timing

}  // namespace cblox

// Linking common Eigen stuff to floating point
namespace Eigen {
typedef Matrix<cblox::FloatingPoint, 3, 1> Vector3;
}  // namespace Eigen

#endif /* CBLOX_CORE_COMMON_H_ */
