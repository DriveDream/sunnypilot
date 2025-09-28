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
void car_err_fun(double *nom_x, double *delta_x, double *out_5228023266310668267);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5306856698853457707);
void car_H_mod_fun(double *state, double *out_600551320946559308);
void car_f_fun(double *state, double dt, double *out_8129035357947983121);
void car_F_fun(double *state, double dt, double *out_8383710118246981636);
void car_h_25(double *state, double *unused, double *out_148242157840398241);
void car_H_25(double *state, double *unused, double *out_2930383418170214680);
void car_h_24(double *state, double *unused, double *out_697508762897435003);
void car_H_24(double *state, double *unused, double *out_8940788848947818835);
void car_h_30(double *state, double *unused, double *out_6072135025006615728);
void car_H_30(double *state, double *unused, double *out_5448716376677463307);
void car_h_26(double *state, double *unused, double *out_5975567531289086742);
void car_H_26(double *state, double *unused, double *out_811119900703841544);
void car_h_27(double *state, double *unused, double *out_4853431109820447404);
void car_H_27(double *state, double *unused, double *out_7672310447861406524);
void car_h_29(double *state, double *unused, double *out_2909834147000225037);
void car_H_29(double *state, double *unused, double *out_5958947720991855491);
void car_h_28(double *state, double *unused, double *out_2149331593972618375);
void car_H_28(double *state, double *unused, double *out_876548703922324917);
void car_h_31(double *state, double *unused, double *out_3785691686094420591);
void car_H_31(double *state, double *unused, double *out_2961029380047175108);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}