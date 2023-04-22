#pragma once
#include <BasicLinearAlgebra.h>    //Use this library to work with matrices:
using namespace BLA;               //This allows you to declare a matrix

//////// KF Variables ////////

extern float d_val;  // drag
extern float m_val; // mass

// A, B, C matrices
extern Matrix<2,2> A_mat;
extern Matrix<2,1> B_mat;
extern Matrix<1,2> C_mat;

// Process and measurement noise
extern Matrix<2,2> sig_u;
extern Matrix<1,1> sig_z;

// Discretize A & B
extern float delta_t;
extern Matrix<2,2> I_mat;
extern Matrix<2,2> A_d;
extern Matrix<2,1> B_d;

// Initial states
extern Matrix<2,2> sig; // initial state uncertainty
extern Matrix<2,1> x_val; // initial state output

//////// KF Function ////////
// output PWM
Matrix<2,1> kf(int dist, int pwm_val);