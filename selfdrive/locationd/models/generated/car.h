#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5316222817505122714);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8582395358567751274);
void car_H_mod_fun(double *state, double *out_6682921460368538930);
void car_f_fun(double *state, double dt, double *out_7484610832011471530);
void car_F_fun(double *state, double dt, double *out_3693895843151971843);
void car_h_25(double *state, double *unused, double *out_7571596293050719952);
void car_H_25(double *state, double *unused, double *out_1888090617440669821);
void car_h_24(double *state, double *unused, double *out_5898437783271290990);
void car_H_24(double *state, double *unused, double *out_2980724247587037031);
void car_h_30(double *state, double *unused, double *out_7296402230766214063);
void car_H_30(double *state, double *unused, double *out_2639605712686938377);
void car_h_26(double *state, double *unused, double *out_6148668575282491908);
void car_H_26(double *state, double *unused, double *out_1853412701433386403);
void car_h_27(double *state, double *unused, double *out_2569923025389874307);
void car_H_27(double *state, double *unused, double *out_4814369024487363288);
void car_h_29(double *state, double *unused, double *out_2294728963105368418);
void car_H_29(double *state, double *unused, double *out_2129374368372546193);
void car_h_28(double *state, double *unused, double *out_3513432878291536742);
void car_H_28(double *state, double *unused, double *out_165744096807219942);
void car_h_31(double *state, double *unused, double *out_925064388213020040);
void car_H_31(double *state, double *unused, double *out_2479620803666737879);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}