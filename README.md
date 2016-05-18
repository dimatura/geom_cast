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