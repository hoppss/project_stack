
#ifndef fusion_hpp
#define fusion_hpp

#include <stdio.h>
#include "Eigen/Dense"
#include "ekf.hpp"
#include <iostream>


//Input data format assumed as - latitude, longitude, vel, psi_dot, accel, alt

enum class DataPointType
{
    IMU, GPS
};


class DataPoint
{
public:

    /**
     * @brief Default constructor
     */
    DataPoint(bool verbose = false)
    :_initialized(false), _first_data_point(true)
    {
        _dx = 0;
        _dy = 0;
        _mx = 0;
        _my = 0;
        _ds = 0;

        _RadiusEarth = 6378388.0; //m
        _arc = 2.0 * M_PI * (_RadiusEarth + 230)/360.0; // degree
       this->verbose = verbose;
       if(this->verbose) std::cout << "     DATAPOINT: ----- Initialized.....\r\n";
    }

    /**
     * @brief Retrieves raw sensor data and stores it in private variables
     *
     * @param timestamp Current timestamp for the sensor data
     * @param data_type Data type: Either GPS: which includes GPS+IMU data or IMU: which only includes the IMU data
     * @param raw_data Raw sensor data
     */

    void set(const double dt, const int d_type, const Eigen::VectorXd& raw_data)
    {
        if(this->verbose) std::cout << "        DATAPOINT: ----- In set\r\n";
        _raw_data.resize(raw_data.size());
        _timestamp =dt;
        _raw_data = raw_data;
        _initialized = true;

        if(_first_data_point && raw_data(3)!=0.0 && raw_data(4)!=0.0&& raw_data(3)!=100 && raw_data(4)!=100)
        {
            _dx = 0;
            _dy = 0;
            _mx = 0;
            _my = 0;
            _prev_lat = raw_data(3);
            _prev_long = raw_data(4);
            _arc = 2.0 * M_PI * (_RadiusEarth + 51)/360.0;
            _first_data_point = false;
        }
      else if(!_first_data_point)
      {
          _arc = 2.0 * M_PI * (_RadiusEarth + 51)/360.0;
          _dx = _arc * cos(raw_data(3) * M_PI/180.0) * (raw_data(4) - _prev_long);
          _dy = _arc * (raw_data(3) - _prev_lat);
          _ds = sqrt(_dx * _dx + _dy * _dy);

          if(_ds == 0.0||raw_data(3)==0.0 || raw_data(4)==0||raw_data(3)==100 || raw_data(4)==100)
          {
              _d_type = 0;
         }
          else
          {
              _d_type =1;
              _mx += _dx;
              _my += _dy;
          }

          if(raw_data(3)!=0.0 && raw_data(4)!=0&&raw_data(3)!=100 && raw_data(4)!=100) {
              _prev_lat = raw_data(3);
              _prev_long = raw_data(4);
          }
      }
    }

    /**
     * @brief Returns saved raw data for sensor fusion
     *
     * @return Sensor data measurements
     */
    Eigen::VectorXd get_state() const
    {
        Eigen::VectorXd state(6);

        //yaw_rate,sita,s,lat,lon
        double x = _mx;
        double y = _my;
        double psi =_raw_data(1);//sita
        double psi_dot =_raw_data(0);//yaw_rate
	double sita=_raw_data(1);//sita
	double s=_raw_data(2);//s

        state << x, y, psi, psi_dot,sita,s;

        return state;
    }

    /**
     * @brief Get raw sensor data
     *
     * @return Raw sensor data
     */
    Eigen::VectorXd get_raw_data() const
    {
        return _raw_data;
    }

    /**
     * @brief Get data type associated with the data at current timestep
     *
     * @return Data type: Either GPS or IMU
     */


    int get_d_point_type() const
    {
        return _d_type;
    }
    
    /**
     * @brief Get current timestamp
     *
     * @return Timestamp associated with current data
     */
    double get_timestamp() const
    {
        return _timestamp;
    }
    double get_arc() const
    {
        return _arc;
    }

private:
    double _x;
    double _y;
    double _z;
    double _north;
    double _east;
    double _down;

    bool _initialized;
    bool _first_data_point;
    double _timestamp;
    double pre_taw_data3;
    double oumiga;
    Eigen::VectorXd _raw_data;

    int _d_type;

    double _prev_lat;
    double _prev_long;
    double _curr_lat;
    double _curr_long;
    double _dx;
    double _dy;
    double _mx;
    double _my;
    double _ds;

    double _RadiusEarth;
    double _arc;
    bool verbose;
};


/**
 * @brief Class which uses EKF and DataPoint to run the filter
 */
class Fusion
{
private:
    const int _n = 5;
    bool _initialized;
    long long timestamp;
    Eigen::MatrixXd _P;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _F;
    Eigen::MatrixXd _R;
    EKF _KF;

    double _sGPS;
    double _sCourse;
    double _sVelocity;
    double _sYaw;
    double _sAccel;
    double _dt;
    double _max_turn_rate;
    double _max_acceleration;
    double _max_yaw_accel;
    double _timestamp;
    // GPS offsets
    double _xOffset, _yOffset;
    bool verbose;
public:

    /**
     * @brief Constructor
     *
     * @param max_acceleration System parameter specifying maximum acceleration
     * @param max_turn_rate System parameter specifying maximum turn rate
     * @param max_yaw_accel System parameter specifying max yaw acceleration
     */
    Fusion(double max_acceleration, double max_turn_rate, double max_yaw_accel, double varGPS,
    double varSpeed, double varYaw, double varAcc, double xOffset, double yOffset, bool verbose);

    /**
     * @brief Updates the Q matrix
     *
     * @param dt Timestep
     */
    void const updateQ(double dt);

    /**
     * @brief Starts the filter
     *
     * @param data Current sensor data
     */
    Eigen::VectorXd start(const DataPoint& data);

    /**
     * @brief Performs the prediction and update step for the filter based on the datatype of the input data
     *
     * @param data Current sensor data
     */
    Eigen::VectorXd compute(const DataPoint& data);

    /**
     * @brief Looping function to either start or run the filter in a loop
     *
     * @param data Current sensor data
     */
    Eigen::VectorXd process(const DataPoint& data);

    /**
     * @brief Returns the estimated state of the system
     *
     * @return Vector containing the estimated states
     */
    Eigen::VectorXd get_resulting_state() const;
};

#endif /* fusion_hpp */
