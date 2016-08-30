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

#include "math_utils/polygon.h"

Polygon::Polygon()
{
}

Polygon::Polygon(size_t size)
{
  poly_.reserve(size);
}

void Polygon::addVertex(const Point& pt)
{
  poly_.push_back(pt);
}

std::vector<Point> Polygon::getVertices()
{
  return poly_;
}

bool Polygon::isInside(const Point& pt)
{
  // Iterate through the vertices of the polygon
  for (size_t i = 0; i < poly_.size() - 1; ++i)
  {
    // Get the line connecting the two adjucent vertices
    std::vector<double> n;
    double d;
    getLine(poly_[i], poly_[i + 1], n, d);

    // Now check if the point falls on the correct side of the line
    if (n.at(0) * pt.x + n.at(1) * pt.y > d)
    {
      // Point is definitely outside of the polygon
      return false;
    }
  }

  // If we got to this point it means that the point lies on or in the
  // interior of the polygon
  return true;
}

void Polygon::getLine(const Point& pt1, const Point& pt2,
  std::vector<double>& normal, double& distance)
{
  double y_diff = pt2.y - pt1.y;
  double x_diff = pt2.x - pt1.x;

  // The equation of the line given two point is:
  // x_diff * y - x_diff * pt1.y = y_diff * x - y_diff * pt1.x;
  // In other words: [-y_diff x_diff] * [x y]' = x_diff * pt1.y - y_diff * pt1.x;
  normal.resize(2);
  normal[0] = -y_diff;
  normal[1] =  x_diff;
  distance  =  x_diff * pt1.y - y_diff * pt1.x;
}
