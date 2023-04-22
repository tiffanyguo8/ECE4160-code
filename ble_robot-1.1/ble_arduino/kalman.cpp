#include "kalman.h"

float d_val = 0.000555;  // drag
float m_val = 0.00326; // mass

// A, B, C matrices
Matrix<2,2> A_mat = { 0, 1,
                      0, -0.17};
Matrix<2,1> B_mat = { 0, 
                      306.75};
Matrix<1,2> C_mat = { -1, 0 };

// Process and measurement noise
Matrix<2,2> sig_u = { pow(32.79,2), 0,
                      0, pow(32.79,2) };
Matrix<1,1> sig_z = { pow(5,2) };

// Discretize A & B
float delta_t = 0.093;
Matrix<2,2> I_mat = { 1, 0,
                      0, 1      };
Matrix<2,2> A_d   = { 1, 0.093,
                      0, 0.98416718 };
Matrix<2,1> B_d   = { 0,
                      28.52760736   };

// Initial states
Matrix<2,2> sig   = { 25, 0,
                      0, 25 }; // initial state uncertainty
Matrix<2,1> x_val = { -2000, 
                      0      }; // initial state output

Matrix<2,1> kf(int dist, int pwm_val) {

  Matrix<2,1> x_p = A_d*x_val + B_d*pwm_val;
  Matrix<2,2> sig_p = A_d*sig*(~A_d) + sig_u;

  Matrix<1,1> y_curr = dist;
  Matrix<1,1> y_m = y_curr - C_mat*x_p;
  Matrix<1,1> sig_m = C_mat*sig_p*(~C_mat) + sig_z;

  Matrix<1,1> sig_m_inv = sig_m;
  Invert(sig_m_inv);

  Matrix<2,1> kf_gain = sig_p*(~C_mat)*(sig_m_inv);

  // Update
  x_val = x_p + kf_gain*y_m;
  sig = (I_mat - kf_gain*C_mat)*sig_p;
    return x_val;
}