#ifndef _OBSERVER_
#define _OBSERVER_

#include <Eigen/Dense>

using namespace Eigen;

class EKF_Observer
{
public:
  EKF_Observer();

  void start();
  VectorXd update(Matrix3d R_c_q,Vector4d y,Vector3d v);

private:
  MatrixXd P;
  VectorXd hat_z;
};


#endif
