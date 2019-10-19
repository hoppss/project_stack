
#include <iostream>
#include "fusion.hpp"
using namespace std;
using namespace Eigen;
Fusion::Fusion(double max_acceleration, double max_turn_rate, double max_yaw_accel, double varGPS,
    double varSpeed, double varYaw, double varAcc, double xOffset, double yOffset, bool verbose)
: _initialized(false), _max_turn_rate(max_turn_rate), _max_acceleration(max_acceleration), _max_yaw_accel(max_yaw_accel), _xOffset(xOffset),
  _yOffset(yOffset), _KF(verbose)
{
    // Initialize initial uncertainity P0
    _P = Eigen::MatrixXd(_n, _n);
    _P << 1000.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1000.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1000.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1000.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1000.0;
    _R = Eigen::MatrixXd(5, 5); //Assuming 5 sources of measurement
    _R << pow(varGPS, 2), 0.0, 0.0, 0.0, 0.0, 
          0.0, pow(varGPS, 2), 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0001, 0.0, 0.0, 
          0.0, 0.0, 0.0, pow(varYaw, 2), 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0001;

  this->verbose = verbose;
  if(verbose) std::cout << " =========================== FUSION:  Initializing --- " << "\r\n";
}

void const Fusion::updateQ(double dt)
{
  if(this->verbose) std::cout << " =========================== FUSION:  Updating Q --- " << "\r\n";


    _Q = Eigen::MatrixXd(_n, _n);
    _sGPS = 0.5 * _max_acceleration * pow(dt, 2);
    _sVelocity = _max_acceleration * dt;
    _sCourse = _max_turn_rate * dt;
    _sYaw = _max_yaw_accel * dt;
    _sAccel = _max_acceleration;
    _Q << pow(_sGPS, 2), 0.0, 0.0, 0.0, 0.0, 
          0.0, pow(_sGPS, 2), 0.0, 0.0, 0.0, 
          0.0, 0.0,pow(_sYaw, 2), 0.0,  0.0,
          0.0, 0.0, 0.0, pow(_sYaw, 2), 0.0,
          0.0, 0.0, 0.0, 0.0,pow(_sCourse, 2);

    _KF.setQ(_Q);
}

Eigen::VectorXd Fusion::start(const DataPoint &data)
{
  if(this->verbose) std::cout << "    Fusion: ------ In start.....\r\n";
    _timestamp = data.get_timestamp();
    Eigen::VectorXd state = data.get_state();
    _KF.start(_n, state, _P, _F, _Q);
    _initialized = true;
    Eigen::VectorXd state0 = _KF.get_resulting_state();
    return state0;
}
//
Eigen::VectorXd Fusion::compute(const DataPoint &data)
{
    /*******************************************
     * Prediction Step
     - Assumes current velocity is the same for this dt
     *******************************************/
  if(this->verbose) std::cout << "    Fusion: ------ In compute.....\r\n";
    // Assuming 1.e6 for timestamp - confirm after running on the system
    //const double dt = (data.get_timestamp())/ 1.e6;
 
 //    const double dt = 0.1;
   
    _timestamp = data.get_timestamp();

    const double dt=_timestamp;
    // Update Q
    this->updateQ(dt);
    // Update state and calculate jacobian
    _KF.updateJA(dt);
    // Prediction
    _KF.predict();

    /*******************************************
     * Update Step
     - Updates appropriate matrices given a measurement
     - Assumes measurement is received either from GPS or IMU
     *******************************************/
    Eigen::VectorXd zz = data.get_state();
    Eigen::VectorXd z;
    z.resize(5);//x, y, psi, psi_dot,sita,s;
    z << zz(0), //x
         zz(1), //y
       zz(4), //sita
       zz(3), //pisdot
       zz(5);//s

    const Eigen::VectorXd state = _KF.get_resulting_state();

    Eigen::VectorXd Hx;
    Eigen::MatrixXd JH;

  Hx.resize(5);
  JH.resize(5,5);
    //Hx是观测值
    // measurement function
    Hx << state(0),
          state(1),
          state(2),
          state(3),
          state(4);
if(data.get_d_point_type() == 1)
{
	JH <<  1.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 
	       0.0, 0.0, 0.0, 0.0, 1.0;

    _KF.update(z, Hx, JH, _R);
}
else if(data.get_d_point_type() == 0)
{
	JH <<  0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 
	       0.0, 0.0, 0.0, 0.0, 1.0;

    _KF.update(z, Hx, JH, _R);  
}
    Eigen::VectorXd state0 = _KF.get_resulting_state();
    return state0;
    

}

VectorXd Fusion::process(const DataPoint &data)
{
  if(this->verbose) std::cout << "    Fusion: ------ In process.....\r\n";
    if(data.get_timestamp() > 0.0)
    {
        VectorXd state0;
      if(_initialized)
      {
	state0=this->compute(data) ;
	return state0;
      }
	else
	{
	  state0=this->start(data);
	  return state0;
	}
    }
}

Eigen::VectorXd Fusion::get_resulting_state() const
{
    return _KF.get_resulting_state();
}
