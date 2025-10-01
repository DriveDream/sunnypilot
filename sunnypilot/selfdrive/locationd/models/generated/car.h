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
void car_err_fun(double *nom_x, double *delta_x, double *out_2827551707420463633);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8408426563124879774);
void car_H_mod_fun(double *state, double *out_8585748337696332209);
void car_f_fun(double *state, double dt, double *out_8033397392187571078);
void car_F_fun(double *state, double dt, double *out_4114962685454462076);
void car_h_25(double *state, double *unused, double *out_4057584842250850052);
void car_H_25(double *state, double *unused, double *out_6438589400418951797);
void car_h_24(double *state, double *unused, double *out_5883550569039178904);
void car_H_24(double *state, double *unused, double *out_1618267895762963534);
void car_h_30(double *state, double *unused, double *out_7618324005108350628);
void car_H_30(double *state, double *unused, double *out_6309250453275711727);
void car_h_26(double *state, double *unused, double *out_8561833858010013063);
void car_H_26(double *state, double *unused, double *out_2697086081544895573);
void car_h_27(double *state, double *unused, double *out_944088425409995593);
void car_H_27(double *state, double *unused, double *out_4134487141475286816);
void car_h_29(double *state, double *unused, double *out_4805771901838497245);
void car_H_29(double *state, double *unused, double *out_6819481797590103911);
void car_h_28(double *state, double *unused, double *out_746359409325685367);
void car_H_28(double *state, double *unused, double *out_1737082780520573337);
void car_h_31(double *state, double *unused, double *out_2606953449621966885);
void car_H_31(double *state, double *unused, double *out_6469235362295912225);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}