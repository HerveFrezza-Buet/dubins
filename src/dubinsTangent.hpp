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

#include <optional>
#include <ostream>

#include <dubinsCircle.hpp>

namespace dubins {
  enum class Direction : bool {
    Clockwise = true,
    CounterClockwise = false
  };

  inline Direction operator!(Direction d) {
    switch(d) {
    case Direction::Clockwise:
      return Direction::CounterClockwise;
    default:
      return Direction::Clockwise;
    }
  }
  
  inline std::ostream& operator<<(std::ostream& os, Direction d) {
    switch(d) {
    case Direction::Clockwise:
      os << "clockwise";
      break;
    case Direction::CounterClockwise:
      os << "counter-clockwise";
      break;
    }
    return os;
  }

  /**
   * @param O1 The center of the starting circle.
   * @param d1 The orientation of the starting circle.
   * @param O2 The center of the destination circle.
   * @param d2 The orientation of the destination circle.
   * @param radius The radius of both circles.
   * @returns the eventual segment that is tangent to the two circles.
   */
  inline std::optional<std::pair<demo2d::Point, demo2d::Point>> tangent(const demo2d::Point& O1, Direction d1,
								 const demo2d::Point& O2, Direction d2,
								 double radius) {
    std::optional<std::pair<demo2d::Point, demo2d::Point>> res;
    
    auto O1O2  = O2 - O1;
    auto nO1O2 = *O1O2;

    if(d1 == Direction::Clockwise) {
      if(d2 == Direction::Clockwise) {
	// d1 = clockwise, d2 = clockwise
	auto T1  = O1 + radius * nO1O2.rotate_left();
	auto T2  = T1 + O1O2;
	res = {T1, T2};
      }
      else {
	// d1 = clockwise, d2 = counter-clockwise
	auto half = .5*(O1 + O2);
	if(demo2d::d2(O1, half) > radius * radius) {
	  Circle c1 {O1, radius};
	  Circle c2 {.5*(O1 + half), O1O2.norm()*.25};
	  if(auto inter = c1 && c2; inter) {
	    auto& [P1, P2] = *inter;
	    demo2d::Point P {P1};
	    if((P2 - O1) * O1O2.rotate_left() > 0)
	      P = P2;
	    res = {P, O2 + O1 - P};
	  }
	}
      }
    }
    else {
      if(d2 == Direction::Clockwise) {
	// d1 = counter-clockwise, d2 = clockwise
	auto half = .5*(O1 + O2);
	if(demo2d::d2(O1, half) > radius * radius) {
	  Circle c1 {O1, radius};
	  Circle c2 {.5*(O1 + half), O1O2.norm()*.25};
	  if(auto inter = c1 && c2; inter) {
	    auto& [P1, P2] = *inter;
	    demo2d::Point P {P1};
	    if((P2 - O1) * O1O2.rotate_left() < 0)
	      P = P2;
	    res = {P, O2 + O1 - P};
	  }
	}
      }
      else {
	// d1 = counter-clockwise, d2 = counter-clockwise
	auto T1  = O1 + radius * nO1O2.rotate_right();
	auto T2  = T1 + O1O2;
	res = {T1, T2};
      }
    }

    return res;
  }
}
