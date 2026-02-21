#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6480965603127692382);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7794393530327101272);
void pose_H_mod_fun(double *state, double *out_6847342441623025206);
void pose_f_fun(double *state, double dt, double *out_7448658717317011616);
void pose_F_fun(double *state, double dt, double *out_3051819978456506628);
void pose_h_4(double *state, double *unused, double *out_4242734118973278604);
void pose_H_4(double *state, double *unused, double *out_2097019224434796136);
void pose_h_10(double *state, double *unused, double *out_2858621772777210172);
void pose_H_10(double *state, double *unused, double *out_3733794250216138204);
void pose_h_13(double *state, double *unused, double *out_2147303471254468456);
void pose_H_13(double *state, double *unused, double *out_1532417304752952032);
void pose_h_14(double *state, double *unused, double *out_7424563568260947704);
void pose_H_14(double *state, double *unused, double *out_5179807656730168432);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}