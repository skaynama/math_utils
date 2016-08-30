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
#ifndef MATH_UTILS_POINT_H
#define MATH_UTILS_POINT_H

#include <math.h>
#include <algorithm>

class Point
{
public:
  Point(double x, double y) : x_(x), y_(y) {}

  double x()
  {
    return x_;
  }

  double y()
  {
    return y_;
  }

  void setX(double value)
  {
    x_ = value;
  }

  void setY(double value)
  {
    y_ = value;
  }

  double l1Norm()
  {
    return std::fabs(x_) + std::fabs(y_);
  }

  double l2Norm()
  {
    return std::sqrt(x_ * x_ + y_ * y_);
  }

  double lInfNorm()
  {
    return std::max(std::fabs(x_), std::fabs(y_));
  }

private:
  double x_, y_;
};


#endif  // MATH_UTILS_POINT_H
