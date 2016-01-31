/* 
 * The MIT License (MIT)
 *  
 * Copyright (c) 2014 Wayne82
 *  
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

#ifndef __Position__
#define __Position__

#include <Eigen/Dense>
#include <vector>

typedef Eigen::Vector2d Pos2d;
typedef Eigen::Vector3d Pos3d;

template <class T>
class PosAndDistance {
public:
   PosAndDistance()
   { }
   PosAndDistance(const T& pos, double dist)
      : m_pos(pos)
      , m_distance(dist)
   {}
   T m_pos;
   double m_distance;
};

typedef PosAndDistance<Pos2d> PosAndDistance2d;
typedef PosAndDistance<Pos3d> PosAndDistance3d;

typedef std::vector<PosAndDistance2d> PosAndDistance2dVec;
typedef std::vector<PosAndDistance3d> PosAndDistance3dVec;

#endif /* defined(__Position__) */
