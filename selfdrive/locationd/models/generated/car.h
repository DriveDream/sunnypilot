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
void car_err_fun(double *nom_x, double *delta_x, double *out_1200516966254576139);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6340514496067370621);
void car_H_mod_fun(double *state, double *out_1568428756196638601);
void car_f_fun(double *state, double dt, double *out_4338504881593374904);
void car_F_fun(double *state, double dt, double *out_4187513685334090243);
void car_h_25(double *state, double *unused, double *out_4879031403620109471);
void car_H_25(double *state, double *unused, double *out_8956977284539053210);
void car_h_24(double *state, double *unused, double *out_368488579487421442);
void car_H_24(double *state, double *unused, double *out_1538585650862126317);
void car_h_30(double *state, double *unused, double *out_1241581875366087121);
void car_H_30(double *state, double *unused, double *out_6438644326031804583);
void car_h_26(double *state, double *unused, double *out_7740387296640753644);
void car_H_26(double *state, double *unused, double *out_5748263470296442182);
void car_h_27(double *state, double *unused, double *out_122641864040736174);
void car_H_27(double *state, double *unused, double *out_7185664530226833425);
void car_h_29(double *state, double *unused, double *out_3760091392294758524);
void car_H_29(double *state, double *unused, double *out_5928412981717412399);
void car_h_28(double *state, double *unused, double *out_7312501570387964639);
void car_H_28(double *state, double *unused, double *out_7435932074922608643);
void car_h_31(double *state, double *unused, double *out_2581527360628778854);
void car_H_31(double *state, double *unused, double *out_5122055368063090706);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}