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
      double R();
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
    bla bla
    pose()                       = default;
    pose(const pose&)            = default;
    pose& operator=(const pose&) = default;
    pose(const Pose& value) : value(value) {}
    pose& operator=(const Pose& value) {
      this->value = value;
      return *this;
    }

    std::pair<Path, bool> operator-(const pose& w) const {
      auto p_true  = path(w.value, value, PARAM::R()); 
      auto p_false = path(w.value, value, PARAM::R());
      if(p_true.length() < p_false.length())
	return {p_true, true};
      else
	return {p_false, false};
    }

    pose& operator+=(const Pose& value) {
      // This is the same as =
      this->value = value;
      return *this;
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
    return std::min(path(p1.value, p2.value, PARAM::R()).length(),
		    path(p2.value, p1.value, PARAM::R()).length());
  }

  
}
