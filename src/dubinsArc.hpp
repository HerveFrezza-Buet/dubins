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

#include <dubinsCircle.hpp>
#include <dubinsPose.hpp>

namespace dubins {


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

    /**
     * @param lambda in [0, 1]
     * @returns A pose corresponding go the oriented tangent at lambda*theta_start + (1-lambda)*theta_end.
     */
    Pose walk(double lambda) const {
      double theta = theta_start;
      if(lambda >= 1)
	theta = theta_end;
      else if(lambda > 0)
	theta = (1-lambda)*theta_start + lambda*theta_end;

      Pose res =  {demo2d::Point::unitary(theta) * C.radius + C.O, 0};
      if(theta_end >= theta_start)
	res = theta + dubins_PI/2;
      else
	res = theta - dubins_PI/2;
      return res;
    }
  };

  /**
   * alpha * arc is arc.walk(alpha)
   */
  Pose operator*(double alpha, const Arc& a) {
    return a.walk(alpha);
  }
  
  std::ostream& operator<<(std::ostream& os, const Arc& a) {
    os << '{' << a.C << ", "
       << int(a.theta_start*1800/dubins_PI+.5)*.1 << " -> "
       << int(a.theta_end*1800/dubins_PI+.5)*.1 << '}';
    return os;
  }

#define ARC_SEGMENT_PIXEL_LENGTH 5
  void draw(cv::Mat& display, const demo2d::opencv::Frame& frame,
	    const Arc& arc,
	    const cv::Scalar& color, int thickness) {
    auto pixel_length = frame(arc.length());
    auto nb_steps = (unsigned int)(pixel_length/(double)ARC_SEGMENT_PIXEL_LENGTH)+2;

    double coef  = (arc.theta_end - arc.theta_start)/(nb_steps-1.0); // Ok since nb_steps >= 2
    double theta = arc.theta_start;
    demo2d::Point prev = arc.C.O + demo2d::Point::unitary(theta) * arc.C.radius;
    demo2d::Point curr;
    theta += coef;
    for(unsigned int i=1; i < nb_steps; ++i, theta += coef, prev = curr) {
      curr = arc.C.O + demo2d::Point::unitary(theta) * arc.C.radius;
      cv::line(display, frame(prev), frame(curr), color, thickness);
    }
  }
}
