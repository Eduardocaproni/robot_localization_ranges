#ifndef ROBOT_LOCALIZATION_RANGES_EKF_H
#define ROBOT_LOCALIZATION_RANGES_EKF_H

#include <robot_localization/ekf.hpp>

namespace robot_localization_ranges
{

class Ekf : public robot_localization::Ekf
{
public:
  Ekf();
};


}


#endif // ROBOT_LOCALIZATION_RANGES_EKF_H
