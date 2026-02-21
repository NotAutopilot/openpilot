#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5316222817505122714) {
   out_5316222817505122714[0] = delta_x[0] + nom_x[0];
   out_5316222817505122714[1] = delta_x[1] + nom_x[1];
   out_5316222817505122714[2] = delta_x[2] + nom_x[2];
   out_5316222817505122714[3] = delta_x[3] + nom_x[3];
   out_5316222817505122714[4] = delta_x[4] + nom_x[4];
   out_5316222817505122714[5] = delta_x[5] + nom_x[5];
   out_5316222817505122714[6] = delta_x[6] + nom_x[6];
   out_5316222817505122714[7] = delta_x[7] + nom_x[7];
   out_5316222817505122714[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8582395358567751274) {
   out_8582395358567751274[0] = -nom_x[0] + true_x[0];
   out_8582395358567751274[1] = -nom_x[1] + true_x[1];
   out_8582395358567751274[2] = -nom_x[2] + true_x[2];
   out_8582395358567751274[3] = -nom_x[3] + true_x[3];
   out_8582395358567751274[4] = -nom_x[4] + true_x[4];
   out_8582395358567751274[5] = -nom_x[5] + true_x[5];
   out_8582395358567751274[6] = -nom_x[6] + true_x[6];
   out_8582395358567751274[7] = -nom_x[7] + true_x[7];
   out_8582395358567751274[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6682921460368538930) {
   out_6682921460368538930[0] = 1.0;
   out_6682921460368538930[1] = 0.0;
   out_6682921460368538930[2] = 0.0;
   out_6682921460368538930[3] = 0.0;
   out_6682921460368538930[4] = 0.0;
   out_6682921460368538930[5] = 0.0;
   out_6682921460368538930[6] = 0.0;
   out_6682921460368538930[7] = 0.0;
   out_6682921460368538930[8] = 0.0;
   out_6682921460368538930[9] = 0.0;
   out_6682921460368538930[10] = 1.0;
   out_6682921460368538930[11] = 0.0;
   out_6682921460368538930[12] = 0.0;
   out_6682921460368538930[13] = 0.0;
   out_6682921460368538930[14] = 0.0;
   out_6682921460368538930[15] = 0.0;
   out_6682921460368538930[16] = 0.0;
   out_6682921460368538930[17] = 0.0;
   out_6682921460368538930[18] = 0.0;
   out_6682921460368538930[19] = 0.0;
   out_6682921460368538930[20] = 1.0;
   out_6682921460368538930[21] = 0.0;
   out_6682921460368538930[22] = 0.0;
   out_6682921460368538930[23] = 0.0;
   out_6682921460368538930[24] = 0.0;
   out_6682921460368538930[25] = 0.0;
   out_6682921460368538930[26] = 0.0;
   out_6682921460368538930[27] = 0.0;
   out_6682921460368538930[28] = 0.0;
   out_6682921460368538930[29] = 0.0;
   out_6682921460368538930[30] = 1.0;
   out_6682921460368538930[31] = 0.0;
   out_6682921460368538930[32] = 0.0;
   out_6682921460368538930[33] = 0.0;
   out_6682921460368538930[34] = 0.0;
   out_6682921460368538930[35] = 0.0;
   out_6682921460368538930[36] = 0.0;
   out_6682921460368538930[37] = 0.0;
   out_6682921460368538930[38] = 0.0;
   out_6682921460368538930[39] = 0.0;
   out_6682921460368538930[40] = 1.0;
   out_6682921460368538930[41] = 0.0;
   out_6682921460368538930[42] = 0.0;
   out_6682921460368538930[43] = 0.0;
   out_6682921460368538930[44] = 0.0;
   out_6682921460368538930[45] = 0.0;
   out_6682921460368538930[46] = 0.0;
   out_6682921460368538930[47] = 0.0;
   out_6682921460368538930[48] = 0.0;
   out_6682921460368538930[49] = 0.0;
   out_6682921460368538930[50] = 1.0;
   out_6682921460368538930[51] = 0.0;
   out_6682921460368538930[52] = 0.0;
   out_6682921460368538930[53] = 0.0;
   out_6682921460368538930[54] = 0.0;
   out_6682921460368538930[55] = 0.0;
   out_6682921460368538930[56] = 0.0;
   out_6682921460368538930[57] = 0.0;
   out_6682921460368538930[58] = 0.0;
   out_6682921460368538930[59] = 0.0;
   out_6682921460368538930[60] = 1.0;
   out_6682921460368538930[61] = 0.0;
   out_6682921460368538930[62] = 0.0;
   out_6682921460368538930[63] = 0.0;
   out_6682921460368538930[64] = 0.0;
   out_6682921460368538930[65] = 0.0;
   out_6682921460368538930[66] = 0.0;
   out_6682921460368538930[67] = 0.0;
   out_6682921460368538930[68] = 0.0;
   out_6682921460368538930[69] = 0.0;
   out_6682921460368538930[70] = 1.0;
   out_6682921460368538930[71] = 0.0;
   out_6682921460368538930[72] = 0.0;
   out_6682921460368538930[73] = 0.0;
   out_6682921460368538930[74] = 0.0;
   out_6682921460368538930[75] = 0.0;
   out_6682921460368538930[76] = 0.0;
   out_6682921460368538930[77] = 0.0;
   out_6682921460368538930[78] = 0.0;
   out_6682921460368538930[79] = 0.0;
   out_6682921460368538930[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7484610832011471530) {
   out_7484610832011471530[0] = state[0];
   out_7484610832011471530[1] = state[1];
   out_7484610832011471530[2] = state[2];
   out_7484610832011471530[3] = state[3];
   out_7484610832011471530[4] = state[4];
   out_7484610832011471530[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7484610832011471530[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7484610832011471530[7] = state[7];
   out_7484610832011471530[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3693895843151971843) {
   out_3693895843151971843[0] = 1;
   out_3693895843151971843[1] = 0;
   out_3693895843151971843[2] = 0;
   out_3693895843151971843[3] = 0;
   out_3693895843151971843[4] = 0;
   out_3693895843151971843[5] = 0;
   out_3693895843151971843[6] = 0;
   out_3693895843151971843[7] = 0;
   out_3693895843151971843[8] = 0;
   out_3693895843151971843[9] = 0;
   out_3693895843151971843[10] = 1;
   out_3693895843151971843[11] = 0;
   out_3693895843151971843[12] = 0;
   out_3693895843151971843[13] = 0;
   out_3693895843151971843[14] = 0;
   out_3693895843151971843[15] = 0;
   out_3693895843151971843[16] = 0;
   out_3693895843151971843[17] = 0;
   out_3693895843151971843[18] = 0;
   out_3693895843151971843[19] = 0;
   out_3693895843151971843[20] = 1;
   out_3693895843151971843[21] = 0;
   out_3693895843151971843[22] = 0;
   out_3693895843151971843[23] = 0;
   out_3693895843151971843[24] = 0;
   out_3693895843151971843[25] = 0;
   out_3693895843151971843[26] = 0;
   out_3693895843151971843[27] = 0;
   out_3693895843151971843[28] = 0;
   out_3693895843151971843[29] = 0;
   out_3693895843151971843[30] = 1;
   out_3693895843151971843[31] = 0;
   out_3693895843151971843[32] = 0;
   out_3693895843151971843[33] = 0;
   out_3693895843151971843[34] = 0;
   out_3693895843151971843[35] = 0;
   out_3693895843151971843[36] = 0;
   out_3693895843151971843[37] = 0;
   out_3693895843151971843[38] = 0;
   out_3693895843151971843[39] = 0;
   out_3693895843151971843[40] = 1;
   out_3693895843151971843[41] = 0;
   out_3693895843151971843[42] = 0;
   out_3693895843151971843[43] = 0;
   out_3693895843151971843[44] = 0;
   out_3693895843151971843[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3693895843151971843[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3693895843151971843[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3693895843151971843[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3693895843151971843[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3693895843151971843[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3693895843151971843[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3693895843151971843[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3693895843151971843[53] = -9.8100000000000005*dt;
   out_3693895843151971843[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3693895843151971843[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3693895843151971843[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3693895843151971843[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3693895843151971843[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3693895843151971843[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3693895843151971843[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3693895843151971843[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3693895843151971843[62] = 0;
   out_3693895843151971843[63] = 0;
   out_3693895843151971843[64] = 0;
   out_3693895843151971843[65] = 0;
   out_3693895843151971843[66] = 0;
   out_3693895843151971843[67] = 0;
   out_3693895843151971843[68] = 0;
   out_3693895843151971843[69] = 0;
   out_3693895843151971843[70] = 1;
   out_3693895843151971843[71] = 0;
   out_3693895843151971843[72] = 0;
   out_3693895843151971843[73] = 0;
   out_3693895843151971843[74] = 0;
   out_3693895843151971843[75] = 0;
   out_3693895843151971843[76] = 0;
   out_3693895843151971843[77] = 0;
   out_3693895843151971843[78] = 0;
   out_3693895843151971843[79] = 0;
   out_3693895843151971843[80] = 1;
}
void h_25(double *state, double *unused, double *out_7571596293050719952) {
   out_7571596293050719952[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1888090617440669821) {
   out_1888090617440669821[0] = 0;
   out_1888090617440669821[1] = 0;
   out_1888090617440669821[2] = 0;
   out_1888090617440669821[3] = 0;
   out_1888090617440669821[4] = 0;
   out_1888090617440669821[5] = 0;
   out_1888090617440669821[6] = 1;
   out_1888090617440669821[7] = 0;
   out_1888090617440669821[8] = 0;
}
void h_24(double *state, double *unused, double *out_5898437783271290990) {
   out_5898437783271290990[0] = state[4];
   out_5898437783271290990[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2980724247587037031) {
   out_2980724247587037031[0] = 0;
   out_2980724247587037031[1] = 0;
   out_2980724247587037031[2] = 0;
   out_2980724247587037031[3] = 0;
   out_2980724247587037031[4] = 1;
   out_2980724247587037031[5] = 0;
   out_2980724247587037031[6] = 0;
   out_2980724247587037031[7] = 0;
   out_2980724247587037031[8] = 0;
   out_2980724247587037031[9] = 0;
   out_2980724247587037031[10] = 0;
   out_2980724247587037031[11] = 0;
   out_2980724247587037031[12] = 0;
   out_2980724247587037031[13] = 0;
   out_2980724247587037031[14] = 1;
   out_2980724247587037031[15] = 0;
   out_2980724247587037031[16] = 0;
   out_2980724247587037031[17] = 0;
}
void h_30(double *state, double *unused, double *out_7296402230766214063) {
   out_7296402230766214063[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2639605712686938377) {
   out_2639605712686938377[0] = 0;
   out_2639605712686938377[1] = 0;
   out_2639605712686938377[2] = 0;
   out_2639605712686938377[3] = 0;
   out_2639605712686938377[4] = 1;
   out_2639605712686938377[5] = 0;
   out_2639605712686938377[6] = 0;
   out_2639605712686938377[7] = 0;
   out_2639605712686938377[8] = 0;
}
void h_26(double *state, double *unused, double *out_6148668575282491908) {
   out_6148668575282491908[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1853412701433386403) {
   out_1853412701433386403[0] = 0;
   out_1853412701433386403[1] = 0;
   out_1853412701433386403[2] = 0;
   out_1853412701433386403[3] = 0;
   out_1853412701433386403[4] = 0;
   out_1853412701433386403[5] = 0;
   out_1853412701433386403[6] = 0;
   out_1853412701433386403[7] = 1;
   out_1853412701433386403[8] = 0;
}
void h_27(double *state, double *unused, double *out_2569923025389874307) {
   out_2569923025389874307[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4814369024487363288) {
   out_4814369024487363288[0] = 0;
   out_4814369024487363288[1] = 0;
   out_4814369024487363288[2] = 0;
   out_4814369024487363288[3] = 1;
   out_4814369024487363288[4] = 0;
   out_4814369024487363288[5] = 0;
   out_4814369024487363288[6] = 0;
   out_4814369024487363288[7] = 0;
   out_4814369024487363288[8] = 0;
}
void h_29(double *state, double *unused, double *out_2294728963105368418) {
   out_2294728963105368418[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2129374368372546193) {
   out_2129374368372546193[0] = 0;
   out_2129374368372546193[1] = 1;
   out_2129374368372546193[2] = 0;
   out_2129374368372546193[3] = 0;
   out_2129374368372546193[4] = 0;
   out_2129374368372546193[5] = 0;
   out_2129374368372546193[6] = 0;
   out_2129374368372546193[7] = 0;
   out_2129374368372546193[8] = 0;
}
void h_28(double *state, double *unused, double *out_3513432878291536742) {
   out_3513432878291536742[0] = state[0];
}
void H_28(double *state, double *unused, double *out_165744096807219942) {
   out_165744096807219942[0] = 1;
   out_165744096807219942[1] = 0;
   out_165744096807219942[2] = 0;
   out_165744096807219942[3] = 0;
   out_165744096807219942[4] = 0;
   out_165744096807219942[5] = 0;
   out_165744096807219942[6] = 0;
   out_165744096807219942[7] = 0;
   out_165744096807219942[8] = 0;
}
void h_31(double *state, double *unused, double *out_925064388213020040) {
   out_925064388213020040[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2479620803666737879) {
   out_2479620803666737879[0] = 0;
   out_2479620803666737879[1] = 0;
   out_2479620803666737879[2] = 0;
   out_2479620803666737879[3] = 0;
   out_2479620803666737879[4] = 0;
   out_2479620803666737879[5] = 0;
   out_2479620803666737879[6] = 0;
   out_2479620803666737879[7] = 0;
   out_2479620803666737879[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5316222817505122714) {
  err_fun(nom_x, delta_x, out_5316222817505122714);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8582395358567751274) {
  inv_err_fun(nom_x, true_x, out_8582395358567751274);
}
void car_H_mod_fun(double *state, double *out_6682921460368538930) {
  H_mod_fun(state, out_6682921460368538930);
}
void car_f_fun(double *state, double dt, double *out_7484610832011471530) {
  f_fun(state,  dt, out_7484610832011471530);
}
void car_F_fun(double *state, double dt, double *out_3693895843151971843) {
  F_fun(state,  dt, out_3693895843151971843);
}
void car_h_25(double *state, double *unused, double *out_7571596293050719952) {
  h_25(state, unused, out_7571596293050719952);
}
void car_H_25(double *state, double *unused, double *out_1888090617440669821) {
  H_25(state, unused, out_1888090617440669821);
}
void car_h_24(double *state, double *unused, double *out_5898437783271290990) {
  h_24(state, unused, out_5898437783271290990);
}
void car_H_24(double *state, double *unused, double *out_2980724247587037031) {
  H_24(state, unused, out_2980724247587037031);
}
void car_h_30(double *state, double *unused, double *out_7296402230766214063) {
  h_30(state, unused, out_7296402230766214063);
}
void car_H_30(double *state, double *unused, double *out_2639605712686938377) {
  H_30(state, unused, out_2639605712686938377);
}
void car_h_26(double *state, double *unused, double *out_6148668575282491908) {
  h_26(state, unused, out_6148668575282491908);
}
void car_H_26(double *state, double *unused, double *out_1853412701433386403) {
  H_26(state, unused, out_1853412701433386403);
}
void car_h_27(double *state, double *unused, double *out_2569923025389874307) {
  h_27(state, unused, out_2569923025389874307);
}
void car_H_27(double *state, double *unused, double *out_4814369024487363288) {
  H_27(state, unused, out_4814369024487363288);
}
void car_h_29(double *state, double *unused, double *out_2294728963105368418) {
  h_29(state, unused, out_2294728963105368418);
}
void car_H_29(double *state, double *unused, double *out_2129374368372546193) {
  H_29(state, unused, out_2129374368372546193);
}
void car_h_28(double *state, double *unused, double *out_3513432878291536742) {
  h_28(state, unused, out_3513432878291536742);
}
void car_H_28(double *state, double *unused, double *out_165744096807219942) {
  H_28(state, unused, out_165744096807219942);
}
void car_h_31(double *state, double *unused, double *out_925064388213020040) {
  h_31(state, unused, out_925064388213020040);
}
void car_H_31(double *state, double *unused, double *out_2479620803666737879) {
  H_31(state, unused, out_2479620803666737879);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
