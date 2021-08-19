#ifndef VOXBLOX_CORE_OBJECT_H_
#define VOXBLOX_CORE_OBJECT_H_

#include "voxblox/core/common.h"

namespace voxblox {

class ObjectInstance {
    public:
		ObjectInstance() :
			id_(-1),
			semantic_label_(-1),
			pose_(Eigen::Vector3d::Zero()),
			confidence_(0) {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Member functions declaration


        void setID(int id) {
        	id_ = id;
        }

        void setConfidence(double conf) {
        	confidence_ = conf;
        }

        const int& getID() const {
        	return id_;
        }

        const double& getConfidence() const {
        	return confidence_;
        }

        void addIndex(const GlobalIndex& idx) {
        	voxel_indices_.push_back(idx);
        }

        void resetIndices() {
        	voxel_indices_.clear();
        }

        std::vector<GlobalIndex>& getIndices() {
        	return voxel_indices_;
        }

        const std::vector<GlobalIndex>& getIndices() const {
        	return voxel_indices_;
        }



    private:
        int id_;              // ID of the Object
        size_t semantic_label_;        // Semantic label of the object
        Eigen::Vector3d pose_;         // Position (centroid) of the object
        double confidence_;            // V-value of the object
        std::vector<GlobalIndex> voxel_indices_;

};

}

#endif  // VOXBLOX_CORE_OBJECT_H_
