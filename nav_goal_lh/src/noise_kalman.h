#include <iostream>

#include <fstream>

#include <string>

#include <vector>

#include <Eigen/Dense>//包含Eigen矩阵运算库，用于矩阵计算

#include <cmath>


using namespace std;
using namespace Eigen;
class KF
{
public:
KF(int num_states,double _dt):_num_states(num_states),dt(_dt)
{
  P=MatrixXd::Identity(_num_states, _num_states);
  P=P*1000;
  Q.resize(_num_states,_num_states);
  R.resize(_num_states,_num_states);
  if(num_states==1)
  {
  sYaw = _max_yaw_accel * dt;
  Q<<pow(sYaw,2);
  R<<pow(varYaw, 2);
  }
  else
  {
    _sGPS = 0.5 * _max_acceleration * pow(dt, 2);
    Q<<pow(_sGPS, 2),0,0,pow(_sGPS, 2);
    R<<pow(varGPS, 2),0,0,pow(varGPS, 2);
  }
  A=MatrixXd::Identity(_num_states, _num_states);
  H=MatrixXd::Identity(_num_states, _num_states);
}
double dt;
double sYaw;
double _max_yaw_accel=0.1;
double varYaw=0.05;
double _max_acceleration=0.5;
double _sGPS;
double varGPS=0.05;
int _num_states;
MatrixXd P;
MatrixXd Q;
MatrixXd R;
int  initial=0;
MatrixXd A,H,K;
VectorXd Z_meas;
VectorXd sita_pdct;
VectorXd sita0;
MatrixXd I = MatrixXd::Identity(_num_states, _num_states);
VectorXd kalman(VectorXd sita)
{

// 	FILE *outFile = fopen("measure.txt", "a+");
 
// 	if(!outFile) cout<<"error"<<endl;   
// 	fprintf(outFile, "%lf\n",sita[0]);
 
// 	fclose(outFile);
	if(!initial)
	{
	  initial=1;
	  sita0=sita;
	  return sita0;
	}
	
		//预测值

	sita_pdct = A*sita0;
		//预测状态与真实状态的协方差矩阵，Pk'

	P = A * P* A.transpose() + Q;
		//K:2x1
	MatrixXd tmp(1,1);

	tmp = H * P * H.transpose() + R;

	K = P * H.transpose() * tmp.inverse();

		//测量值z
	Z_meas = sita;

		//估计值
	sita0 = sita_pdct + K * (Z_meas - H * sita_pdct);


	//估计状态和真实状态的协方差矩阵，Pk

	P = (I - K * H) * P;
// 	FILE *outFile1 = fopen("estimate.txt", "a+");
 //outFile.open("./date.txt");
// 	if(!outFile1) cout<<"error"<<endl;   
// 	fprintf(outFile1, "%lf\n",sita0[0]);
 // outFile.close();
// 	fclose(outFile1);
//	cout<<sita0[0]<<endl;
	return sita0;
}
	
};
