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
      double R() const;
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
    pose& operator=(const Pose& value) {
      this->Pose::operator=(value);
      return *this;
    }

    std::pair<Path, bool> operator-(const pose& w) const {
      auto p_true  = path(w, *this, PARAM().R()); 
      auto p_false = path(*this, w, PARAM().R());
      if(p_true.length() < p_false.length())
	return {p_true, true};
      else
	return {p_false, false};
    }

    pose& operator+=(const Pose& value) {
      return (*this) = value;
    }
  };

  Pose operator*(double alpha, const std::pair<Path, bool>& path_bool) {
    if(path_bool.second)
      return alpha*path_bool.first;
    else
      return (1-alpha)*path_bool.first;
  }

  template<typename PARAM>
  double d(const pose<PARAM>& p1, const pose<PARAM>& p2) {
    return std::min(path(p1, p2, PARAM().R()).length(),
		    path(p2, p1, PARAM().R()).length());
  }

  
}
