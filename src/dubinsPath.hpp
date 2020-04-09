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

#include <stdexcept>
#include <limits>
#include <iostream>

#include <optional>
#include <utility>

#include <opencv2/opencv.hpp>
#include <demo2d.hpp>

#include <dubinsCircle.hpp>
#include <dubinsTangent.hpp>
#include <dubinsPose.hpp>

namespace dubins {

  /**
   * This is a Dubins'path, i.e an eventual arc, and eventual segment,
   * and an eventual arc.
   */
  class Path {
  public:
    struct Lengths {
      std::optional<double> lb;
      std::optional<double> lm;
      std::optional<double> le;
      std::optional<double> l;
      Lengths(const std::optional<double>& lb,
	      const std::optional<double>& lm,
	      const std::optional<double>& le,
	      const std::optional<double>& l)
	: lb(lb), lm(lm), le(le), l(l) {}
    };
    
  private:
    mutable std::optional<double> l;
    mutable std::optional<double> lb;
    mutable std::optional<double> lm;
    mutable std::optional<double> le;

    void update_lengths() const {
      if(!l) {
	if(begin || middle || end) {
	  if(begin)  lb = begin->length(); else lb = 0;
	  if(middle) lm = demo2d::d(middle->first, middle->second); else lm = 0;
	  if(end)    le = end->length(); else le = 0;
	  l = *lb + *lm + *le;
	}
	else
	  l = std::numeric_limits<double>::max();
      }
    }
  public:
    Pose start; //!< The starting pose.
    Pose destination; //!< The destination pose.
    std::optional<Arc> begin; //!< The starting arc
    std::optional<std::pair<demo2d::Point, demo2d::Point>> middle; //!< The middle segment
    std::optional<Arc> end; //!< The ending arc
    
    Path()                       = default;
    Path(const Path&)            = default;
    Path& operator=(const Path&) = default;

    Path(const Pose& start,
	 const Pose& destination)
      : start(start), destination(destination) {}
    
    Path(const Pose& start,
	 const Pose& destination,
	 const std::optional<Arc>& begin,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle,
	 const std::optional<Arc>& end)
      : start(start), destination(destination), begin(begin), middle(middle), end(end) {}

    Path(const Pose& start,
	 const Pose& destination,
	 const std::optional<Arc>& begin)
      : Path(start, destination,
	     begin,
	     std::optional<std::pair<demo2d::Point, demo2d::Point>>(),
	     std::optional<Arc>()) {}

    Path(const Pose& start,
	 const Pose& destination,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle)
      : Path(start, destination,
	     std::optional<Arc>(),
	     middle,
	     std::optional<Arc>()) {}

    Path(const Pose& start,
	 const Pose& destination,
	 const std::optional<Arc>& begin,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle)
      : Path(start, destination,
	     begin,
	     middle,
	     std::optional<Arc>()) {}
    
    Path(const Pose& start,
	 const Pose& destination,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle,
	 const std::optional<Arc>& end)
      : Path(start, destination,
	     std::optional<Arc>(),
	     middle,
	     end) {}

    double length() const {
      update_lengths();
      return *l;
    }
    
    Path::Lengths lengths() const {
      update_lengths();
      return {lb, lm, le, l};
    }

    /**
     * This walks the path until a fraction lambda of the total length
     * is reached.
     * @param lambda in [0, 1]
     * @returns The pose at the end of the walk.
     */
    Pose walk(double lambda) const {
      double total_length = length();
      if(total_length == 0 || total_length == std::numeric_limits<double>::max() || lambda <= 0)
    	return start;
      if(lambda >= 1)
	return destination;

      double l = total_length*lambda;
      if(l < *lb)
	return (l/(*lb)) * (*begin);
      l -= *lb;
      if(l < *lm) {
	l /= *lm;
	return {(1-l) * (middle->first) + l * (middle->second), (middle->second - middle->first).angle()};
      }

      l -= *lm;
      return (l/ *le) * (*end);
    }
  };

  inline std::ostream& operator<<(std::ostream& os, const Path::Lengths& lengths) {
    std::optional<double> lg;
    lg = lengths.l;
    if(lg) {if (*(lg) != std::numeric_limits<double>::max()) os << *lg; else os << "Inf";} else os << "None";
    os << " = {left = "; lg = lengths.lb;
    if(lg) {if (*(lg) != std::numeric_limits<double>::max()) os << *lg; else os << "Inf";} else os << "None";
    os << ", middle = "; lg = lengths.lm;
    if(lg) {if (*(lg) != std::numeric_limits<double>::max()) os << *lg; else os << "Inf";} else os << "None";
    os << ", end = "; lg = lengths.le;
    if(lg) {if (*(lg) != std::numeric_limits<double>::max()) os << *lg; else os << "Inf";} else os << "None";
    os << '}';
    return os;
  }

  /**
   * alpha * path is path.walk(alpha)
   */
  inline Pose operator*(double alpha, const Path& p) {
    return p.walk(alpha);
  }
  
  /**
   * @param start starting pose.
   * @param end destination pose.
   * @param radius the minimal radius.
   * @returns the Dubins' path.
   */
  inline Path path(const Pose& start, const Pose& end, double radius) {
    Path res(start, end);

    if(start == end)
      return res;

    std::cout << "Path : " << std::endl;

    double min_l = std::numeric_limits<double>::max();
    auto [c1l, c1r] = start.left_right_circles(radius);
    auto [c2l, c2r] = end.left_right_circles(radius);

    if(auto tangent = dubins::tangent(c1l.O, Direction::CounterClockwise,
				      c2l.O, Direction::CounterClockwise,
				      radius);
       tangent) {
      auto s1 = start.theta() - dubins_PI/2;
      auto e1 = (tangent->first - c1l.O).angle();
      auto s2 = (tangent->second - c2l.O).angle();
      auto e2 = end.theta() - dubins_PI/2;
      while(e1 < s1) e1 += 2 * dubins_PI;
      while(e2 < s2) e2 += 2 * dubins_PI;
      Path P {start, end, Arc(c1l, s1, e1), *tangent, Arc(c2l, s2, e2)};
      std::cout << "  - LSL : " << P.lengths() << std::endl;
      if(auto l = P.length(); l < min_l) {
	min_l = l;
	res = P;
      }
    }

    if(auto tangent = dubins::tangent(c1r.O, Direction::Clockwise,
				      c2r.O, Direction::Clockwise,
				      radius);
       tangent) {
      auto s1 = start.theta() + dubins_PI/2;
      auto e1 = (tangent->first - c1r.O).angle();
      auto s2 = (tangent->second - c2r.O).angle();
      auto e2 = end.theta() + dubins_PI/2;
      while(e1 > s1) e1 -= 2 * dubins_PI;
      while(e2 > s2) e2 -= 2 * dubins_PI;
      Path P {start, end, Arc(c1r, s1, e1), *tangent, Arc(c2r, s2, e2)};
      std::cout << "  - RSR : " << P.lengths() << std::endl;
      if(auto l = P.length(); l < min_l) {
	min_l = l;
	res = P;
      }
    }

    if(auto tangent = dubins::tangent(c1l.O, Direction::CounterClockwise,
				      c2r.O, Direction::Clockwise,
				      radius);
       tangent) {
      auto s1 = start.theta() - dubins_PI/2;
      auto e1 = (tangent->first - c1l.O).angle();
      auto s2 = (tangent->second - c2r.O).angle();
      auto e2 = end.theta() + dubins_PI/2;
      while(e1 < s1) e1 += 2 * dubins_PI;
      while(e2 > s2) e2 -= 2 * dubins_PI;
      Path P {start, end, Arc(c1l, s1, e1), *tangent, Arc(c2r, s2, e2)};
      std::cout << "  - LSR : " << P.lengths() << std::endl
		<< "          " << std::make_pair(0, 0) << std::endl;
      if(auto l = P.length(); l < min_l) {
	min_l = l;
	res = P;
      }
    }

    if(auto tangent = dubins::tangent(c1r.O, Direction::Clockwise,
				      c2l.O, Direction::CounterClockwise,
				      radius);
       tangent) {
      auto s1 = start.theta() + dubins_PI/2;
      auto e1 = (tangent->first - c1r.O).angle();
      auto s2 = (tangent->second - c2l.O).angle();
      auto e2 = end.theta() - dubins_PI/2;
      while(e1 > s1) e1 -= 2 * dubins_PI;
      while(e2 < s2) e2 += 2 * dubins_PI;
      Path P {start, end, Arc(c1r, s1, e1), *tangent, Arc(c2l, s2, e2)};
      std::cout << "  - RSL : " << P.lengths() << std::endl;
      if(auto l = P.length(); l < min_l) {
	min_l = l;
	res = P;
      }
    }

    return res;
  }
  
  // inline std::ostream& operator<<(std::ostream& os, const Path& p) {
  //   os << '{';
  //   if(p.begin)
  //     os << *(p.begin);
  //   else
  //     os << "None";
  //   os << " --> ";
  //   if(p.middle)
  //     os << *(p.middle);
  //   else
  //     os << "None";
  //   os << " --> ";
  //   if(p.end)
  //     os << *(p.end);
  //   else
  //     os << "None";
  //   os << '}';
  //   return os;
  // }

  inline std::ostream& operator<<(std::ostream& os, const std::pair<int, int>& p) {
    os << "caca" << std::endl;
    return os;
  }

  inline void draw(cv::Mat& display, demo2d::opencv::Frame& frame,
		   const Path& path,
		   const cv::Scalar& color, int thickness) {
    if(path.begin)
      draw(display, frame, *(path.begin), color, thickness);
    if(path.middle)
      cv::line(display, frame(path.middle->first), frame(path.middle->second), color, thickness);
    if(path.end)
      draw(display, frame, *(path.end), color, thickness);
  }
}


