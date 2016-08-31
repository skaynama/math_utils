/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, all rights reserved.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Shahab Kaynama
*/

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

struct Point
{
  Point() : x(0.0), y(0.0) {}

  Point(double px, double py) : x(px), y(py) {}

  double l1Norm()
  {
    return std::fabs(x) + std::fabs(y);
  }

  double l2Norm()
  {
    return std::sqrt(x * x + y * y);
  }

  double lInfNorm()
  {
    return std::max(std::fabs(x), std::fabs(y) );
  }

  double x, y;
};


// TODO assumes convexity
class Polygon
{
public:
  Polygon() {}

  Polygon(size_t size)
  {
    poly_.reserve(size);
  }

  Polygon(const std::vector<Point>& pt_vec)
  {
    poly_ = pt_vec;
  }

  void addVertex(const Point& pt)
  {
    poly_.push_back(pt);
  }

  std::vector<Point> getVertices()
  {
    return poly_;
  }

  bool isInside(const Point& pt)
  {
    if (poly_.empty() )
    {
      std::cout << "Polygon is empty!" << std::endl;
      return false;
    }

    // If the polygon is degenerate down to a point then check if the
    // point is the same as the degenerate polygon
    if (poly_.size() == 1)
    {
      const double eps = 1.0e-8;
      bool res = std::fabs(poly_[0].x - pt.x) < eps
        && std::fabs(poly_[0].y - pt.y) < eps;
      return res;
    }

    // Otherwise iterate through the vertices of the polygon
    for (size_t i = 0; i < poly_.size(); ++i)
    {
      // Get the line connecting the two adjucent vertices
      std::vector<double> n;
      double d;
      size_t ind = (i == poly_.size() - 1)? 0 : i + 1;

      getLine(poly_[i], poly_[ind], n, d);

      std::cout << "n: [" << n[0] << " " << n[1] << "] d: " << d << std::endl;

      // Now check if the point falls on the correct side of the line
      if (n.at(0) * pt.x + n.at(1) * pt.y < d)
      {
        std::cout << "Point on the wrong side of the halfspace." << std::endl;
        // Point is definitely outside of the polygon
        return false;
      }
    }

    // If we got to this point it means that the point lies on or in the
    // interior of the polygon
    return true;
  }

private:
  void getLine(const Point& pt1, const Point& pt2,
    std::vector<double>& normal, double& distance)
  {
    double y_diff = pt2.y - pt1.y;
    double x_diff = pt2.x - pt1.x;

    // The equation of the line given two point is:
    // [-y_diff x_diff] * [x y]^T = distance := x_diff * pt1.y - y_diff * pt1.x;
    normal.resize(2);
    normal[0] = -y_diff;
    normal[1] =  x_diff;
    distance  =  x_diff * pt1.y - y_diff * pt1.x;
  }

private:
  std::vector<Point> poly_;
};


int main()
{
  /* An example use case would be the following
  Point p1(1, 1), p2(-1, 1), p3(-1, -1), p4(1, -1);
  std::vector<Point> vertices;
  vertices.push_back(p1);
  vertices.push_back(p2);
  vertices.push_back(p3);
  vertices.push_back(p4);
  Polygon poly(vertices);
  Point pt(0.1, 0.5);
  poly.isInside(pt);
  */

  size_t num_vertices;
  std::cout << "How many vertices does the polygon have? ";
  std::cin >> num_vertices;

  // Create the polygon object
  Polygon poly(num_vertices);

  // Ask the user to enter the vertices of this polygon
  for (size_t i = 0; i < num_vertices; ++i)
  {
    double vx, vy;
    std::cout << "Enter the vertex x y coordinates: ";
    std::cin >> vx >> vy;

    // Create the vertex and add it to the polygon
    Point vertex(vx, vy);
    poly.addVertex(vertex);
  }

  // Get the point that we wish to check for polygon inclusion
  double px, py;
  std::cout << "Now enter the point to check for inclusion: ";
  std::cin >> px >> py;
  Point pt(px, py);

  // Check if point is inside of the polygon
  bool res = poly.isInside(pt);
  std::cout << "Point is inside? " << res << std::endl;
  
  return 0;
}
