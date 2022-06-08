#pragma once

#include "gtsam/base/Vector.h"
#include "gtsam/base/Manifold.h"
#include "gtsam/base/ThreadsafeException.h"

#include <iostream>

namespace gtsam
{
  class SemiParametricLandmark
  {
  public:

    /**
     * SemiParametricLandmark 
     * 
     * @param  {double} bearing : Bearing in radians
     * @param  {double} range   : Range in m
     */
    SemiParametricLandmark(double bearing, double range)
    {
      setBearing(bearing);
      setRange(range);
    }

    // Tangent space dimensionality
    enum
    {
      dimension = 2
    };

    inline size_t dim() const {
      return dimension;
    }

    /// Retract delta to manifold
    SemiParametricLandmark retract(const Vector2 &v) const {

      double x=v[0];
      double y=v[1];

      double bearing_new = bearing_ + asin(x);
      double range_new = range_ + y;

      return SemiParametricLandmark(bearing_new, range_new);
    }

    /// Compute the coordinates in the tangent space
    Vector2 localCoordinates(const SemiParametricLandmark &value) const {
      double x = sin(value.bearing_ - this->bearing_);
      double y = value.range_ - this->range_;

      return Vector2(x, y);
    }

    /// The print function
    void print(const std::string &s = std::string()) const {
      std::cout << (s.empty() ? s : s + " ") << "Bearing: " << this->bearing_ << " | Range: " << this->range_ << std::endl;
    }

    /// The equals function with tolerance
    bool equals(const SemiParametricLandmark &s, double tol = 1e-9) const {

      double err= sqrt(pow((s.bearing_ - this->bearing_),2) + pow((s.range_ - this->range_),2));

      if (err < tol){
        return true;
      }
      return false;
    }
   
    /**
     * Bearing getter
     * @return {double}  : Bearing of the landmark (rad)
     */
    double getBearing() const{
      return bearing_;
    }

    /**
     * Bearing setter
     * Wraps input to domain [-pi, pi)
     * @param  {double} bearing : New bearing of the landmark (rad)
     */
    void setBearing(double bearing){
      bearing_ = atan2(sin(bearing), cos(bearing));
    }
    
    /**
     * Range getter
     * @return {double}  : Range of the landmark (rad)
     */
    double getRange() const {
      return range_;
    }

    /**
     * Range setter
     * Will reject values <= 0
     * @param  {double} range : New range of the landmark (rad)
     */
    void setRange(double range){

      if (range <= 0){
        throw gtsam::OutOfRangeThreadsafe("Range must be positive");
      }else{
        range_ = range;
      }
    }


    private:
      double bearing_;
      double range_;
  };

  // required traits - could alternatively be VectorSpace, LieGroup, etc.
  template <>
  struct traits<SemiParametricLandmark> : public internal::Manifold<SemiParametricLandmark>
  {
  };

  template <>
  struct traits<const SemiParametricLandmark> : public internal::Manifold<SemiParametricLandmark>
  {
  };

}
