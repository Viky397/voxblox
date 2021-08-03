// Author: Keenan Burnett
// Copyright (C) 2020 aUToronto
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#pragma once
#include <vector>
#include <boost/shared_ptr.hpp>

namespace zeus_pcl {

class PointXYZ {
 public:
    float x, y, z, c;
    PointXYZ() {x = 0; y = 0; z = 0; c = 0;}
    PointXYZ(float x0, float y0, float z0, float c0 = 0) {x = x0; y = y0; z = z0; c = c0;}
    // array-type accesor
    float operator [] (int i) const {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else if (i == 2)
            return z;
        else
            return c;
    }
    // operations
    float dot(const PointXYZ P) const {
        return x * P.x + y * P.y + z * P.z;
    }
    float sq_norm() {
        return x*x + y*y + z*z;
    }
    void normalize() {
        float norm = sqrt(sq_norm());
        x /= norm;
        y /= norm;
        z /= norm;
    }
    PointXYZ cross(const PointXYZ P) const {
        return PointXYZ(y*P.z - z*P.y, z*P.x - x*P.z, x*P.y - y*P.x, c);
    }
    PointXYZ operator - (const PointXYZ &p) {
        return PointXYZ(this->x - p.x, this->y - p.y, this->z - p.z, c);
    }
    PointXYZ operator + (const PointXYZ &p) {
        return PointXYZ(this->x + p.x, this->y + p.y, this->z + p.z, c);
    }
    PointXYZ operator * (const float &a) {
        return PointXYZ(this->x * a, this->y * a, this->z * a, c);
    }
    bool operator == (const PointXYZ &p) {
        return this->x == p.x && this->y == p.y && this->z == p.z;
    }
    friend std::ostream &operator << (std::ostream &os, const PointXYZ &p) {
        os << p.x << " " << p.y << " " << p.z << " " << p.c;
        return os;
    }
};

class PointXYZI : public PointXYZ {
 public:
    float intensity;
    PointXYZI() {x = 0; y = 0; z = 0; intensity = 0;}
    PointXYZI(float x0, float y0, float z0, float intensity_) {x = x0; y = y0; z = z0; intensity = intensity_;}
    // array-type accesor
    float operator [] (int i) const {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else if (i == 2)
            return z;
        else
            return intensity;
    }
};

class PointXYZIR : public PointXYZ {
 public:
    float intensity;
    uint16_t ring;
    PointXYZIR() {x = 0; y = 0; z = 0; intensity = 0; ring = 0;}
    PointXYZIR(float x0, float y0, float z0, float intensity_, uint16_t ring_){
        x = x0; y = y0; z = z0; intensity = intensity_; ring = ring_;
    }
    // array-type accesor
    float operator [] (int i) const {
        if (i == 0)
            return x;
        else if (i == 1)
            return y;
        else if (i == 2)
            return z;
        else if (i == 3)
            return intensity;
        else
            return (float)ring;
    }
};

template <class T>
class PointCloudTemplate {
 public:
    std::vector<T> points;
    uint size() {return points.size();}
    void resize(uint new_size) {points.resize(new_size);}
    size_t kdtree_get_point_count() const { return points.size(); }
    float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0)
            return points[idx].x;
        else if (dim == 1)
            return points[idx].y;
        else
            return points[idx].z;
    }
    template <class BBOX> bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

typedef PointXYZIR VPoint;
typedef PointXYZI IPoint;
typedef PointXYZ Point;
typedef PointCloudTemplate<VPoint> VPointCloud;
typedef boost::shared_ptr<VPointCloud> VPointCloudPtr;
typedef PointCloudTemplate<IPoint> IPointCloud;
typedef boost::shared_ptr<IPointCloud> IPointCloudPtr;
typedef PointCloudTemplate<Point> PointCloud;
typedef boost::shared_ptr<PointCloud> PointCloudPtr;
}  // namespace zeus_pcl
