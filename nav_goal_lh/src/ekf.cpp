#include "ekf.hpp"
#include "utils.hpp"
#include <iostream>
using namespace std;
//x, y, psi, vel_x,psi_dot, a,sita,s;
void EKF::start(const int nin, const Eigen::VectorXd &xin, const Eigen::MatrixXd &Pin, const Eigen::MatrixXd &Fin, const Eigen::MatrixXd &Qin)
{
    _num_states = nin;
    _I = Eigen::MatrixXd::Identity(_num_states, _num_states);
    if(this->verbose) std::cout << "    EKF: Number of states ->" << nin << "\n";
    this->_state.resize(nin);
    this->_state <<xin(0),xin(1),xin(2),xin(3),xin(5);//x, y, psi, psi_dot,sita,s;
    if(this->verbose) std::cout << "    EKF: Size of Input states ->" << xin.size() << "\n";
    _P = Pin;
    _JA = Fin;
    _Q = Qin;

    return;
}
void EKF::setQ(const Eigen::MatrixXd &Q_in)
{
    _Q = Q_in;
}
void EKF::updateJA(const double dt)
{

    if(this->verbose) std::cout << "Updating JA: About to update state equations" << "\n";
    if(this->verbose) std::cout << "Updating JA: size of states" << this->_state.rows() << "x" <<this->_state.cols() << "\n";

//x, y, psi, vel_x, a,psi_dot,
    
    _state(0)=_state(0)+_state(4)*sin(_state(2)+_state(3)*dt);
    _state(1)=_state(1)+_state(4)*cos(_state(2)+_state(3)*dt);
    _state(2)=_state(2)+_state(3)*dt;
    _state(3)=_state(3);
    _state(4)=_state(4);

    if(this->verbose) std::cout << "Updating JA: About to calculate jacobian" << "\n";
    // Calculate jacobian
   _JA = calculate_joacobian(_state, dt);
}

void EKF::predict()
{
    // Prediction step
    _P = _JA * _P * _JA.transpose() + _Q;
}

void EKF::update(const Eigen::VectorXd& Z, const Eigen::VectorXd& Hx, const Eigen::MatrixXd &JH, const Eigen::MatrixXd &R)
{
    Eigen::MatrixXd JHT = _P * JH.transpose();
    // Temporary variable for storing this intermediate value
    Eigen::MatrixXd _S = JH * JHT  + R;
    // Compute the Kalman gain
    _K = JHT * _S.inverse();
    // Update the estimate
    
    Eigen::VectorXd y = Z - Hx;


    _state = _state + _K * y;
    // Update the error covariance
    _P = (_I - _K * JH) * _P;

  
}

Eigen::VectorXd EKF::get_resulting_state() const
{
  return _state;
}




