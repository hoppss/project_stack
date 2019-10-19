//
//  utils.cpp
//  EKF


#include "utils.hpp"

Eigen::MatrixXd calculate_joacobian(const Eigen::VectorXd& v, const double dt)
{
    // Assumes Jacobian is 6 x 6
    Eigen::MatrixXd JA = Eigen::MatrixXd::Zero(5,5);
    
    // Assumes the size of input vector is 6
    const double psi_dot = v(3);
    
    const double THRESHOLD = 0.001;
    
//    if (psi_dot > THRESHOLD) //Avoid dividing by zero
//    {

      double r23=-v(4)*sin(v(2)+v(3)*dt);
      double r24=-v(4)*dt*sin(v(2)+v(3)*dt);
      double r25=cos(v(2)+v(3)*dt);
      double r13=v(4)*cos(v(2)+v(3)*dt);
      double r14=v(4)*dt*cos(v(2)+v(3)*dt);
      double r15=sin(v(2)+v(3)*dt);
        JA <<
        1.0, 0.0, r13, r14, r15,
        0.0, 1.0, r23, r24, r25,
        0.0, 0.0, 1.0, dt, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0;
//    }
    
    return JA;
}
