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
void car_err_fun(double *nom_x, double *delta_x, double *out_4209795161126653978);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6997367614069238905);
void car_H_mod_fun(double *state, double *out_7416336690310360429);
void car_f_fun(double *state, double dt, double *out_8859519254629166733);
void car_F_fun(double *state, double dt, double *out_8364572389978044378);
void car_h_25(double *state, double *unused, double *out_8300028965105448174);
void car_H_25(double *state, double *unused, double *out_2512644865125695589);
void car_h_24(double *state, double *unused, double *out_9038116966944756929);
void car_H_24(double *state, double *unused, double *out_7381459730153402441);
void car_h_30(double *state, double *unused, double *out_8024834902820942285);
void car_H_30(double *state, double *unused, double *out_7040341195253303787);
void car_h_26(double *state, double *unused, double *out_768691185727490576);
void car_H_26(double *state, double *unused, double *out_6254148183999751813);
void car_h_27(double *state, double *unused, double *out_3143952404811037663);
void car_H_27(double *state, double *unused, double *out_4816747124069360570);
void car_h_29(double *state, double *unused, double *out_7421519018144464768);
void car_H_29(double *state, double *unused, double *out_6530109850938911603);
void car_h_28(double *state, double *unused, double *out_8145625672471883308);
void car_H_28(double *state, double *unused, double *out_6834235205701109439);
void car_h_31(double *state, double *unused, double *out_5905544393970864459);
void car_H_31(double *state, double *unused, double *out_2481998903248735161);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}