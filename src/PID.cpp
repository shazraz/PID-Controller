#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	//p_error_ = 0;
	//i_error_ = 0;
	//d_error_ = 0;
	error_vector_.resize(3);

}

std::vector<double> PID::UpdateError(double cte) {
	//Calculate d_error_ using previous p_error_ value
	error_vector_[2] = cte - error_vector_[0];
	error_vector_[0] = cte;
	error_vector_[1] += cte;

	//d_error_ = cte - p_error_;
	//p_error_ = cte;
	//i_error_ += cte;

	return error_vector_;
}

double PID::TotalError() {
	return -(Kp_*error_vector_[0] + Ki_*error_vector_[1] + Kd_*error_vector_[2]);
}

