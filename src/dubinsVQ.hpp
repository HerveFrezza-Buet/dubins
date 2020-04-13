/*
 *   Copyright (C) 2020,  CentraleSupelec
 *
 *   Author : Hervé Frezza-Buet
 *
 *   Contributor :
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public
 *   License (GPL) as published by the Free Software Foundation; either
 *   version 3 of the License, or any later version.
 *   
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *   Contact : herve.frezza-buet@centralesupelec.fr
 *
 */

#pragma once

#include <utility>
#include <algorithm>

#include <dubinsPose.hpp>
#include <dubinsPath.hpp>

namespace dubins {

  namespace concept {
    struct Param {
      double R() const;              //!< This is the miminum curvature radius.
      double tol_angle() const;      //!< This is the angle tolerance for arcs.
      double tol_distance_2() const; //!< This is the minimal squared distance between centers for considering circles as distincts.
    };
  }

  /**
   * This is a Pose that supports vq3 required operations. More
   * precisely, the update \f$w \mathrel{+}= \alpha (\xi - w)\f$ is a valid
   * expression, where \f$w\f$ and \f$\xi\f$ are pose instances.
   */
  template<typename PARAM>
  class pose : public Pose {
  private:
  public:

    using Pose::Pose;
    
    pose()                       = default;
    pose(const pose&)            = default;
    pose& operator=(const pose&) = default;

    pose(const Pose& p) : Pose(p) {}
    
    pose& operator=(const Pose& value) {
      this->Pose::operator=(value);
      return *this;
    }

    /**
     * @param w is the starting point
     * @param this *this is the ending point.
     * @returns ((P1, b1), (P2, b2)) : (P1, b1) is the shortest path between *this and w. If b1 is true, it is the path from w to *this, if b1 is false, it is the path from *this to w. (P2, b2) is the other path, with the same meaning for b2.
     */
    std::pair<std::pair<Path, bool>, std::pair<Path, bool>> operator-(const pose& w) const {
      auto p_true  = path(PARAM().tol_distance_2(), PARAM().tol_angle(), w, *this, PARAM().R()); 
      auto p_false = path(PARAM().tol_distance_2(), PARAM().tol_angle(), *this, w, PARAM().R());
      if(p_true.length() < p_false.length())
	return {{p_true, true}, {p_false, false}};
      else
	return {{p_false, false}, {p_true, true}} ;
    }

    pose& operator+=(const Pose& value) {
      return (*this) = value;
    }
  };

  Pose operator*(double alpha, const std::pair<std::pair<Path, bool>, std::pair<Path, bool>>& paths) {
    if(paths.first.second)
      return alpha*paths.first.first;
    else
      return (1-alpha)*paths.first.first;
  }

  template<typename PARAM>
  double d(const pose<PARAM>& p1, const pose<PARAM>& p2) {
    return std::min(path(PARAM().tol_distance_2(), PARAM().tol_angle(), p1, p2, PARAM().R()).length(),
    		    path(PARAM().tol_distance_2(), PARAM().tol_angle(), p2, p1, PARAM().R()).length());
  }

  
}
