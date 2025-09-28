#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_552322323456612592);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4941387461979504541);
void pose_H_mod_fun(double *state, double *out_1196212354931887211);
void pose_f_fun(double *state, double dt, double *out_8376554467190809622);
void pose_F_fun(double *state, double dt, double *out_9160386404678547461);
void pose_h_4(double *state, double *unused, double *out_7457733264180481157);
void pose_H_4(double *state, double *unused, double *out_442051352877174604);
void pose_h_10(double *state, double *unused, double *out_7956640001151253329);
void pose_H_10(double *state, double *unused, double *out_2661350816086051052);
void pose_h_13(double *state, double *unused, double *out_5574642289715389555);
void pose_H_13(double *state, double *unused, double *out_7168579855439526325);
void pose_h_14(double *state, double *unused, double *out_3993226962874543760);
void pose_H_14(double *state, double *unused, double *out_3521189503462309925);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}