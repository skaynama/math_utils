/**
 * NOTE: This entire file and its classes can be replaced by just a few lines
 * of code using the boost library. Specifically boost's geometry::wthin
 * can be invoked to decide if a geometry::model::polygon contains
 * a geometry::model::d2::point_xy. I strongly recommend using that library
 * instead as it is generic and can handle non-convex complicated shapes.
 * This current library is a simple collection of code snippets with
 * limited functionality and has been written recreationally over the span of
 * a few minutes as an alternative to boost's powerful version.
 *
 * Author: Shahab Kaynama, copyright 2016, BSD License
 */

#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

/**
 * Simple data structure to handle 2D points.
 */
struct Point2D
{
  /**
   * @brief Default constructor.
   */
  Point2D() : x(0.0), y(0.0) {}

  /**
   * @brief Constructor.
   * @param px, py The x-y coordinates defining the point.
   */
  Point2D(double px, double py) : x(px), y(py) {}

  /**
   * @brief Calculates the l1 norm of the point vector.
   * @return The l1 norm.
   */
  double l1Norm()
  {
    return std::fabs(x) + std::fabs(y);
  }

  /**
   * @brief Calculates the l2 norm of the point vector.
   * @return The l2 norm.
   */
  double l2Norm()
  {
    return std::sqrt(x * x + y * y);
  }

  /**
   * @brief Calculates the l-infinity norm of the point vector.
   * @return The l-infinity norm.
   */
  double lInfNorm()
  {
    return std::max(std::fabs(x), std::fabs(y) );
  }

  double x, y;  /**< Holders of the coordinates of the point */
};


/**
 * Class that defines and handles simple (without holes) convex 2D polygons.
 * Contains basic functionalities such as checking if a polygon contains
 * a given point.
 */
class Polygon
{
public:
  /**
   * @brief Default constructor.
   */
  Polygon() {}

  /**
   * @brief Constructor.
   * @param size The size of the polygon (number of vertices).
   */
  Polygon(size_t size)
  {
    poly_.reserve(size);
  }

  /**
   * @brief Constructor.
   * @param pt_vec Vector of vertices describing the polygon.
   */
  Polygon(const std::vector<Point2D>& pt_vec)
  {
    poly_ = pt_vec;
  }

  /**
   * @brief Function that adds a vertex to the polygon.
   * @param pt The vertex point to be added.
   */
  void addVertex(const Point2D& pt)
  {
    poly_.push_back(pt);
  }

  /**
   * @brief Function that retrieves the polygon as a list of its vertices.
   * @return The polygon in vertex representation form.
   */
  std::vector<Point2D> getVertices()
  {
    return poly_;
  }

  /**
   * @brief Function that checks if a point is inside of the polygon. The check
   * here is done by converting the vertex represented polygon into its facet
   * representation and check if the point falls in the intersection of the half
   * spaces created by the polygon's facets.
   * @param pt The point to be checked.
   * @return True if polygon contains the point; false otherwise.
   */
  bool isInside(const Point2D& pt)
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
  /**
   * @brief Function that calculates a line connecting two given points as an
   * inward pointing normal and a distance value so that the equation
   * [n1 n2] * [x y]' = d describes the line.
   * @param[in] pt1, pt2 The two points that pass through the line.
   * @param[out] normal The normal vector to the line.
   * @param[out] distance The distance from the line to the origin.
   */
  void getLine(const Point2D& pt1, const Point2D& pt2,
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
  std::vector<Point2D> poly_;  /**< The container that holds vertices of the polygon */
};


/**
 * @brief The main function of the program to allow user interaction
 * with the Point2D and Polygon classes.
 * An example use case would be the following degenerate polygon
 * Point2D p1(1, 1), p2(-1, 1);
 * std::vector<Point2D> vertices;
 * vertices.push_back(p1);
 * vertices.push_back(p2);
 * Polygon poly(vertices);
 * Point2D pt(0, 1);
 * poly.isInside(pt);
 */
int main()
{
  // Get the number of vertices this polygon is to have
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
    Point2D vertex(vx, vy);
    poly.addVertex(vertex);
  }

  // Get the point that we wish to check for polygon inclusion
  double px, py;
  std::cout << "Now enter the point to check for inclusion: ";
  std::cin >> px >> py;
  Point2D pt(px, py);

  // Check if point is inside of the polygon
  bool res = poly.isInside(pt);
  std::cout << "Point is inside? " << (res? "yes" : "no") << std::endl;
  
  return 0;
}
