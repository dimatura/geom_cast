# geom_cast

Author: Daniel Maturana (`dimatura@cmu.edu`)

Maintainer: Daniel Maturana (`dimatura@cmu.edu`)

Clean conversion between different point and rotation types used throughout ROS.
Conversion is type-safe and has NO performance penalty relative to manual conversion.

Supports Eigen, tf, geometry_msgs.

See also: `geom_cast_extra`, with additional conversions (OpenCV, pcl).

Examples in `src/examples.cpp` and `utest/test.cpp`.

```cpp

#include "geom_cast/rot_cast.hpp"
#include "geom_cast/point_cast.hpp"
//...
Eigen::Vector3f foo(2, 8, 0);
tf::Vector3 bar = ca::point_cast<tf::Vector3>(foo);
Eigen::Quaterniond q1(1, 0, 0, 0);
geometry_msgs::Quaternion q2 = ca::rot_cast<geometry_msgs::Quaternion>(q1);
```



# TODO

- Re-enable tests
- Remove PCL/opencv from example
- Use C++11 features
- More types: std::vector, KDL, std::array
- Types with timestamps?
- Extensibility?
- Anything new for tf2?


### License ###
[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)
 
Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.