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

#include <opencv2/opencv.hpp>
#include <demo2d.hpp>
#include <cmath>
#include <optional>
#include <utility>
#include <array>
#include <algorithm>
#include <iostream>

#include <dubinsCircle.hpp>

namespace dubins {

  /**
   * This is a 2D pose (i.e location and orientation).
   */
  class Pose {
  public:
    
    demo2d::Point                                    O; //!< This is the location.
    
  private:
    
    double                                           angle;
    mutable std::optional<std::pair<double, double>> cs;
    
  public:

    Pose(const demo2d::Point& O, double theta) : O(O), angle(theta), cs() {}
    Pose(double x, double y,     double theta) : Pose(demo2d::Point(x, y), theta) {}
    Pose() : Pose(demo2d::Point(0,0), 0) {}
    Pose(const Pose&)            = default;
    Pose& operator=(const Pose&) = default;

    /** This returns the pose orientation. */
    double theta() const {return angle;}

    /** This sets the pose orientation. */
    void operator=(double theta) {
      angle = theta;
      cs.reset();
    }

    /** As the sinus and the cosinus may be frequently needed for a
	pose (i.e. for display), it can be computed only when needed,
	and stored for further reuse.
	@returns the cosinus and the sinus of the orientation. */
    const std::pair<double, double>& cos_sin() const {
      if(!cs)
	cs = {std::cos(angle), std::sin(angle)};
      return *cs;
    }

    /**
     * @radius the radius of the circles.
     * @returns the tangential circles (left and right) of the pose.
     */
    std::pair<Circle, Circle> left_right_circles(double radius) const {
      auto [ct, st] = cos_sin();
      demo2d::Point N {ct, st};
      N *= radius;
      return {{O + N.rotate_left(), radius}, {O + N.rotate_right(), radius}};
    }

    bool operator==(const Pose& other) const {
      return O == other.O && angle == other.angle;
    }

    bool operator!=(const Pose& other) const {
      return O != other.O || angle != other.angle;
    }
  };

  inline std::ostream& operator<<(std::ostream& os, const Pose& p) {
    os << '[' << p.O << ", " << int(p.theta()*1800/dubins_PI+.5)*.1 << "°]";
    return os;
  }
  
  inline void draw(cv::Mat& display, const demo2d::opencv::Frame& frame,
	    const Pose& pose,
	    int radius, double length,
	    const cv::Scalar& color, int thickness) {
    cv::circle(display, frame(pose.O), radius, color, -1);
    auto& [cos_t, sin_t] = pose.cos_sin();
    auto M = demo2d::Point(cos_t, sin_t) * length + pose.O;
    cv::line(display, frame(pose.O), frame(M), color, thickness);
  }


  /**
   * This is an output iterator for drawing collections of poses.
   */
  template<typename OBJECT>
  class PoseDrawer {

  private:
	
    cv::Mat image; // a share pointer.
    demo2d::opencv::Frame frame;
    std::function<bool (const OBJECT&)>          do_draw;
    std::function<Pose (const OBJECT&)>          pose_of;
    std::function<int (const OBJECT&)>           radius_of;
    std::function<double (const OBJECT&)>        length_of;
    std::function<cv::Scalar (const OBJECT&)>    color_of;
    std::function<int (const OBJECT&)>           thickness_of;
	
	
  public:

    using difference_type   = long;
    using value_type        = OBJECT;
    using pointer           = OBJECT*;
    using reference         = OBJECT&;
    using iterator_category = std::output_iterator_tag;
	
    template<typename DO_DRAW, typename POSE_OF, typename RADIUS_OF, typename LENGTH_OF, typename COLOR_OF, typename THICKNESS_OF>
    PoseDrawer(cv::Mat& image,
	       demo2d::opencv::Frame frame,
	       const DO_DRAW&      do_draw,
	       const POSE_OF&      pose_of,
	       const RADIUS_OF&    radius_of,
	       const LENGTH_OF&    length_of,
	       const COLOR_OF&     color_of,
	       const THICKNESS_OF& thickness_of)
      : image(image),
	frame(frame),
	do_draw(do_draw),
	pose_of(pose_of),
	radius_of(radius_of),
	length_of(length_of),
	color_of(color_of),
	thickness_of(thickness_of) {}

    PoseDrawer()                            = delete;
    PoseDrawer(const PoseDrawer&)            = default;
    PoseDrawer& operator=(const PoseDrawer&) = default; 

    PoseDrawer& operator++()    {return *this;}
    PoseDrawer& operator++(int) {return *this;}
    PoseDrawer& operator*()     {return *this;}
    PoseDrawer& operator=(const OBJECT& o) {
      if(do_draw(o))
	draw(image, frame, pose_of(o), radius_of(o), length_of(o), color_of(o), thickness_of(o));
      return *this;
    }
  };

  template<typename OBJECT, typename DO_DRAW, typename POSE_OF, typename RADIUS_OF, typename LENGTH_OF, typename COLOR_OF, typename THICKNESS_OF>
  PoseDrawer<OBJECT> pose_drawer(cv::Mat& image,
				 demo2d::opencv::Frame frame,
				 const DO_DRAW&      do_draw,
				 const POSE_OF&      pose_of,
				 const RADIUS_OF&    radius_of,
				 const LENGTH_OF&    length_of,
				 const COLOR_OF&     color_of,
				 const THICKNESS_OF& thickness_of) {
    return PoseDrawer<OBJECT>(image, frame, do_draw, pose_of, radius_of, length_of, color_of, thickness_of);
  }
}
