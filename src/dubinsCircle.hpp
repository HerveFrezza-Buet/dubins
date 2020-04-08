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
#include <optional>
#include <cmath>
#include <demo2d.hpp>

#define dubins_PI 3.141592653589793


namespace dubins {

  double to_deg(double a) {return a*180/dubins_PI;}
  double to_rad(double a) {return a*dubins_PI/180;}
  
  class Circle {
  public:
    
    demo2d::Point O; //!< This is the center of the circle.
    double        radius;

    Circle(const demo2d::Point& O, double radius) : O(O), radius(radius) {}
    Circle(double x, double y,     double radius) : Circle(demo2d::Point(x, y), radius) {}
    Circle() : Circle(demo2d::Point(0,0), 1) {}
    Circle(const Circle&)            = default;
    Circle& operator=(const Circle&) = default;

    /**
     * Returns the eventual intersection points betwee two circles.
     */
    std::optional<std::pair<demo2d::Point, demo2d::Point>> operator&&(const Circle& other) {
      std::optional<std::pair<demo2d::Point, demo2d::Point>> res;

      auto O1O2 = other.O - O;
      auto d_2  = O1O2.norm2();
      auto d    = std::sqrt(d_2);
      
      if((d == 0)
	 || (d > radius + other.radius)
	 || (d < std::fabs(radius - other.radius)))
	return res;

      auto d_1   = 1/d;
      auto r0_2  = radius       * radius;
      auto r1_2  = other.radius * other.radius;
      auto a     = (r0_2 - r1_2 + d_2) / d / 2;
      auto h     = std::sqrt(r0_2 - a*a);
      auto P2    = O + (a * d_1) * O1O2 ;
      auto D     = (h * d_1) * O1O2.rotate_left();
      res        = {P2 + D, P2 - D};
      
      return res;
    }
  };
  
  std::ostream& operator<<(std::ostream& os, const Circle& c) {
    os << '[' << c.O << ", " << c.radius << ']';
    return os;
  }


  /** This represent an arc. */
  class Arc {
    
  public:
    Circle C;
    mutable double theta_start = 0; //!< The angle where the arc starts.
    mutable double theta_end = 0;   //!< The angle where the arc ends.
    

    Arc()                      = default;
    Arc(const Arc&)            = default;
    Arc& operator=(const Arc&) = default;
    
    Arc(const Circle& c, double theta_start, double theta_end)
      : C(c), theta_start(theta_start), theta_end(theta_end) {}
    
    Arc(const demo2d::Point& O, double radius, double theta_start, double theta_end)
      : Arc(Circle(O, radius), theta_start, theta_end) {}
    
    Arc(double x, double y, double radius, double theta_start, double theta_end)
      : Arc(Circle(x, y, radius), theta_start, theta_end) {}
    
    double length() const {
      return std::fabs(C.radius*(theta_end-theta_start));
    }
  };

  
  std::ostream& operator<<(std::ostream& os, const Arc& a) {
    os << '{' << a.C << ", "
       << int(a.theta_start*1800/dubins_PI+.5)*.1 << " -> "
       << int(a.theta_end*1800/dubins_PI+.5)*.1 << '}';
    return os;
  }
  
  /**
   * @param thickness use -1 for filling the circle.
   */
  void draw(cv::Mat& display, const demo2d::opencv::Frame& frame,
	    const Circle& circle,
	    const cv::Scalar& color, int thickness) {
    cv::circle(display, frame(circle.O), frame(circle.radius), color, thickness);
  }

#define ARC_SEGMENT_PIXEL_LENGTH 5
  void draw(cv::Mat& display, const demo2d::opencv::Frame& frame,
	    const Arc& arc,
	    const cv::Scalar& color, int thickness) {
    auto pixel_length = frame(arc.length());
    auto nb_steps = (unsigned int)(pixel_length/(double)ARC_SEGMENT_PIXEL_LENGTH)+2;

    double coef  = (arc.theta_end - arc.theta_start)/(nb_steps-1.0); // Ok since nb_steps >= 2
    double theta = arc.theta_start;
    demo2d::Point prev = arc.C.O + demo2d::Point(std::cos(theta), std::sin(theta)) * arc.C.radius;
    demo2d::Point curr;
    theta += coef;
    for(unsigned int i=1; i < nb_steps; ++i, theta += coef, prev = curr) {
      curr = arc.C.O + demo2d::Point(std::cos(theta), std::sin(theta)) * arc.C.radius;
      cv::line(display, frame(prev), frame(curr), color, thickness);
    }
  }
}
