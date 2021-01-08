/*
 *   Copyright (C) 2020,  CentraleSupelec
 *
 *   Author : Hervé Frezza-Buet
 *
 *   Contributor :  Anass El Idrissi
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
      std::optional<double> la;
      std::optional<double> le;
      std::optional<double> l;
      Lengths(const std::optional<double>& lb,
              const std::optional<double>& lm,
	      const std::optional<double>& la,
	      const std::optional<double>& le,
	      const std::optional<double>& l)
	: lb(lb), lm(lm), la(la), le(le), l(l) {}
    };
    
  private:
    mutable std::optional<double> l;
    mutable std::optional<double> lb;
    mutable std::optional<double> lm;
    mutable std::optional<double> la;
    mutable std::optional<double> le;

    void update_lengths() const {
      if(!l) {
	if(begin || middle || middle_arc || end) {
	  if(begin)  lb = begin->length(); else lb = 0;
	  if(middle) lm = demo2d::d(middle->first, middle->second); else lm = 0;
      if(middle_arc) la = middle_arc->length(); else la = 0;
	  if(end)    le = end->length(); else le = 0;
	  l = *lb + *lm + *la + *le;
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
    std::optional<Arc> middle_arc; //!< The middle arc
    std::optional<Arc> end; //!< The ending arc
    
    Path()                       = default;
    Path(const Path&)            = default;
    Path& operator=(const Path&) = default;

    Path(const Pose& start,
	 const Pose& destination)
      : start(start), destination(destination) {}


    /**
     * @param tol_angle see dubins::Arc::in_one_turn.
     */
    Path(double tol_angle,
	 const Pose& start,
	 const Pose& destination,
	 const std::optional<Arc>& begin,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle,
     const std::optional<Arc>& middle_arc,
	 const std::optional<Arc>& end)
      : start(start), destination(destination), begin(begin), middle(middle),middle_arc(middle_arc), end(end) {
      if(this->begin) this->begin->in_one_turn(tol_angle);
      if(this->middle_arc) this->middle_arc->in_one_turn(tol_angle);
      if(this->end)   this->end->in_one_turn(tol_angle);
    }

    /**
     * @param tol_angle see dubins::Arc::in_one_turn.
     */
    Path(double tol_angle,
	 const Pose& start,
	 const Pose& destination,
	 const std::optional<Arc>& begin)
      : Path(tol_angle,
	     start, destination,
	     begin,
	     std::nullopt,
             std::nullopt,
	     std::nullopt) {}

    Path(const Pose& start,
	 const Pose& destination,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle)
      : Path(0,
	     start, destination,
	     std::nullopt,
	     middle,
             std::nullopt,
	     std::nullopt) {}

    /**
     * @param tol_angle see dubins::Arc::in_one_turn.
     */
    Path(double tol_angle,
	 const Pose& start,
	 const Pose& destination,
	 const std::optional<Arc>& begin,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle)
      : Path(tol_angle,
	     start, destination,
	     begin,
	     middle,
             std::nullopt,
	     std::nullopt) {}
    
    /**
     * @param tol_angle see dubins::Arc::in_one_turn.
     */
    Path(double tol_angle,
	 const Pose& start,
	 const Pose& destination,
	 const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle,
	 const std::optional<Arc>& end)
      : Path(tol_angle,
	     start, destination,
	     std::nullopt,
	     middle,
             std::nullopt,
	     end) {}

    /**
     * @param tol_angle see dubins::Arc::in_one_turn.
     */
    Path(double tol_angle,
            const Pose& start,
            const Pose& destination,
            const std::optional<Arc>& begin,
            const std::optional<std::pair<demo2d::Point, demo2d::Point>>& middle,
            const std::optional<Arc>& end)
        : Path(tol_angle,
                start, destination,
                begin,
                middle,
                std::nullopt,
                end) {}

    /**
     * @param tol_angle see dubins::Arc::in_one_turn.
     */
    Path(double tol_angle,
            const Pose& start,
            const Pose& destination,
            const std::optional<Arc>& begin,
            const std::optional<Arc>& middle_arc,
            const std::optional<Arc>& end)
        : Path(tol_angle,
                start, destination,
                begin,
                std::nullopt,
                middle_arc,
                end) {}

    double length() const {
      update_lengths();
      return *l;
    }
    
    Path::Lengths lengths() const {
      update_lengths();
      return {lb, lm, la, le, l};
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
      if(l < *la) return (l/(*la)) * (*middle_arc);
      l -= *la;
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
    os << ", middle arc= "; lg = lengths.la;
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
  
  inline std::ostream& operator<<(std::ostream& os, const Path& p) {
    os << '{';
    if(p.begin)
      os << *(p.begin);
    else
      os << "None";
    os << " --> ";
    if(p.middle)
      os << *(p.middle);
    else
      os << "None";
    os << " --> ";
    if(p.middle_arc)
        os << *(p.middle_arc);
    else
        os << "None";
    os << " --> ";
    if(p.end)
      os << *(p.end);
    else
      os << "None";
    os << '}';
    return os;
  }

  enum class Side : int {
      Left = 0,
      Right = 1
  };

  inline Side operator!(Side d) {
      switch(d) {
          case Side::Left:
              return Side::Right;
          default:
              return Side::Left;
      }
  }

  inline std::ostream& operator<<(std::ostream& os, Side d) {
      switch(d) {
          case Side::Right:
              os << "R";
              break;
          case Side::Left:
              os << "L";
              break;
      }
      return os;
  }

  inline Path asa_path(double tol_angle, double radius, const Pose& start, const Pose& end, const Circle& c1, Side side1, const Circle& c2, Side side2)
  {
      auto d1 = side1 == Side::Left ? Direction::CounterClockwise : Direction::Clockwise;
      auto d2 = side2 == Side::Left ? Direction::CounterClockwise : Direction::Clockwise;

      auto tangent = dubins::tangent(c1.O, d1, c2.O, d2, radius);
      if(!tangent) return Path();

      auto s1 = start.theta();
      auto e1 = (tangent->first - c1.O).angle();
      if(side1 == Side::Left){ 
          s1 -= dubins_PI/2;
          while(e1 < s1) e1 += 2 * dubins_PI;
      }
      else{
          s1 += dubins_PI/2;
          while(e1 > s1) e1 -= 2 * dubins_PI;
      }

      auto s2 = (tangent->second - c2.O).angle();
      auto e2 = end.theta();
      if(side2 == Side::Left){ 
          e2 -= dubins_PI/2;
          while(e2 < s2) e2 += 2 * dubins_PI;
      }
      else{
          e2 += dubins_PI/2;
          while(e2 > s2) e2 -= 2 * dubins_PI;
      }
      return {tol_angle, start, end, Arc(c1, s1, e1), *tangent, Arc(c2, s2, e2)};
  }
  
  inline Path aaa_path(double tol_angle, double radius, const Pose& start, const Pose& end, const Circle& c1, const Circle& c2, Side side)
  {
      auto d = side == Side::Right ? Direction::CounterClockwise : Direction::Clockwise;
      auto tangent = dubins::tangent_circle(c1.O, c2.O, d, radius);
      if(!tangent) return Path();

      auto s1 = start.theta();
      auto e1 = (tangent->C.O - c1.O).angle();
      auto s2 = (tangent->C.O - c2.O).angle();
      auto e2 = end.theta();
      if(side == Side::Left){ 
          s1 -= dubins_PI/2;
          e2 -= dubins_PI/2;
          while(e1 < s1) e1 += 2 * dubins_PI;
          while(e2 < s2) e2 += 2 * dubins_PI;
      }
      else{
          s1 += dubins_PI/2;
          e2 += dubins_PI/2;
          while(e1 > s1) e1 -= 2 * dubins_PI;
          while(e2 > s2) e2 -= 2 * dubins_PI;
      }

      return {tol_angle, start, end, Arc(c1, s1, e1), *tangent, Arc(c2, s2, e2)};
  }

  /**
   * @param tol_distance_2 If the squared distance between two cicles is lower that this, the circles are considered as identical.
   * @param tol_angle see dubins::Arc::in_one_turn.
   * @param start starting pose.
   * @param end destination pose.
   * @param radius the minimal radius.
   * @returns the Dubins' path.
   */
  //#define dubinsDEBUG_PATH
  inline Path path(double tol_distance_2, double tol_angle, const Pose& start, const Pose& end, double radius) {

    if(start == end)
      return {start, end, std::make_pair(start.O, start.O)};

#ifdef dubinsDEBUG_PATH
    std::cout << "Path : " << std::endl;
#endif

    double min_l = std::numeric_limits<double>::max();
    auto [c1l, c1r] = start.left_right_circles(radius);
    auto [c2l, c2r] = end.left_right_circles(radius);
    std::map<int, Circle> c1 = {{0, c1l}, {1, c1r}};
    std::map<int, Circle> c2 = {{0, c2l}, {1, c2r}};

    if(demo2d::d2(c1l.O, c2l.O) < tol_distance_2) {
      auto s1 = (start.O - c1l.O).angle();
      auto e1 = (end.O - c1l.O).angle();
      while(e1 < s1) e1 += 2 * dubins_PI;
      Path P {tol_angle, start, end, Arc(c1l, s1, e1)};
#ifdef dubinsDEBUG_PATH
      std::cout << "  - left circles match :" << P.lengths() << std::endl
       		<< "                        " << P << std::endl;
#endif
      return P;
    }

    if(demo2d::d2(c1r.O, c2r.O) < tol_distance_2) {
      auto s1 = (start.O - c1r.O).angle();
      auto e1 = (end.O - c1r.O).angle();
      while(e1 > s1) e1 -= 2 * dubins_PI;
      Path P {tol_angle, start, end, Arc(c1r, s1, e1)};
#ifdef dubinsDEBUG_PATH
      std::cout << "  - right circles match :" << P.lengths() << std::endl
       		<< "                         " << P << std::endl;
#endif
      return P;
    }

    Path res(start, end);
     for(const Side side1: {Side::Left, Side::Right})
     {
         for(const Side side2: {Side::Left, Side::Right})
         {
             Path P = asa_path(tol_angle, radius, start, end, c1[(int) side1], side1, c2[(int) side2], side2);
 #ifdef dubinsDEBUG_PATH
             std::cout << "  -" << side1 << "S" << side2 << " : " << P.lengths() << std::endl
                 << "          " << P << std::endl;
 #endif
             if(auto l = P.length(); l < min_l) {
                 min_l = l;
                 res = P;
             }
         }
     }

    for(const Side side: {Side::Left, Side::Right})
    {
        Path P = aaa_path(tol_angle, radius, start, end, c1[(int) side], c2[(int) side], side);
#ifdef dubinsDEBUG_PATH
        std::cout << "  -" << side << !side << side << " : " << P.lengths() << std::endl
            << "          " << P << std::endl;
#endif
        if(auto l = P.length(); l < min_l) {
            min_l = l;
            res = P;
        }
    }

    return res;
  }

  inline void draw(cv::Mat& display, const demo2d::opencv::Frame& frame,
		   const Path& path,
		   const cv::Scalar& color, int thickness) {
    if(path.begin)
      draw(display, frame, *(path.begin), color, thickness);
    if(path.middle)
      cv::line(display, frame(path.middle->first), frame(path.middle->second), color, thickness);
    if(path.middle_arc)
        draw(display, frame, *(path.middle_arc), color, thickness);
    if(path.end)
      draw(display, frame, *(path.end), color, thickness);
  }
}

inline std::ostream& operator<<(std::ostream& os, const std::pair<int, int>& p) {
  os << "caca" << std::endl;
  return os;
}


