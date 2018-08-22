/**
 * @file polygon.h
 * @author Parker Lusk <parkerclusk@gmail.com>
 */

//
// see https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/
//

#pragma once

#include <iostream>
#include <vector>
#include <utility>

#include <Eigen/Dense>

namespace montecarlo {

  /**
   * @brief      A Polygon has three vertices
   */
  class Polygon
  {
  public:
    /**
     * @brief      A simple structure to define a point of the polygon
     */
    struct Point {
        double x;
        double y;
    };

    Polygon(const std::vector<Point>& vertices) : vertices_(vertices) {};
    Polygon() = default;
    ~Polygon() = default;

    /**
     * @brief      Update the internal vertices
     *
     * @param[in]  vertices  The vertices
     */
    void update(const std::vector<Point>& vertices);

    /**
     * @brief      Checks if the polygon contains the given point
     *
     * @param[in]  point  The point
     *
     * @return     bool
     */
    bool contains(Point p);

  private:
    std::vector<Point> vertices_;   ///< points that define the polygon

    static constexpr double INF = 10000.0;

    bool on_segment(Point p, Point q, Point r);
    int orientation(Point p, Point q, Point r);
    bool do_intersect(Point p1, Point q1, Point p2, Point q2);
  };

}