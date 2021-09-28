#ifndef VOXBLOX_CORE_OBJECT_H_
#define VOXBLOX_CORE_OBJECT_H_

#include "voxblox/core/common.h"
#include "voxblox/core/block_hash.h"

namespace voxblox {

auto index_cmp = [](GlobalIndex a, GlobalIndex b) {
	return 1*a(0) + 2*a(1) + 3*a(2) < 1*b(0) + 2*b(1) + 3*b(2);
};


class ObjectInstance {
    public:
		ObjectInstance() :
			id_(-1),
			semantic_label_(-1),
			pose_(Eigen::Vector3f::Zero()),
			confidence_(0) {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // Member functions declaration


        void setID(int id) {
        	id_ = id;
        }

        void setConfidence(double conf) {
        	confidence_ = conf;
        }

        void setPose(double x, double y, double z) {
        	pose_ = Eigen::Vector3f(x, y, z);
        }

        const int& getID() const {
        	return id_;
        }

        const double& getConfidence() const {
        	return confidence_;
        }

        const Eigen::Vector3f& getPose() const {
            return pose_;
        }

        const size_t& getLabel() const{
        	return semantic_label_;
        }

        void addIndex(const GlobalIndex& idx) {
        	voxel_indices_.insert(idx);
        }

        void addIndices(const GlobalIndexVector& ids) {
            //voxel_indices_.reserve(voxel_indices_.size() + ids.size());
            //for (const auto& id : ids) addIndex(id);
        	voxel_indices_.insert(ids.begin(), ids.end() );
        }

        void resetIndices() {
        	voxel_indices_.clear();
        }

        LongIndexSet& getIndices() {
        	return voxel_indices_;
        }

        const LongIndexSet& getIndices() const {
        	return voxel_indices_;
        }



    private:
        int id_;              // ID of the Object
        size_t semantic_label_;        // Semantic label of the object
        Eigen::Vector3f pose_;         // Position (centroid) of the object
        double confidence_;            // V-value of the object
        LongIndexSet voxel_indices_;

};

}

#endif  // VOXBLOX_CORE_OBJECT_H_
