#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_4642697741308526443);
void live_err_fun(double *nom_x, double *delta_x, double *out_2165770424783456757);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3351720479291780110);
void live_H_mod_fun(double *state, double *out_6584352024001636745);
void live_f_fun(double *state, double dt, double *out_1043206997310578708);
void live_F_fun(double *state, double dt, double *out_188722001523302554);
void live_h_4(double *state, double *unused, double *out_6655992373746929060);
void live_H_4(double *state, double *unused, double *out_3248634536311175072);
void live_h_9(double *state, double *unused, double *out_4122145493608870189);
void live_H_9(double *state, double *unused, double *out_3489824182940765717);
void live_h_10(double *state, double *unused, double *out_645947892789743159);
void live_H_10(double *state, double *unused, double *out_7621534239664898320);
void live_h_12(double *state, double *unused, double *out_4988288577312707100);
void live_H_12(double *state, double *unused, double *out_8268090944343136867);
void live_h_35(double *state, double *unused, double *out_5113912582120643214);
void live_H_35(double *state, double *unused, double *out_7433090097041401040);
void live_h_32(double *state, double *unused, double *out_3188138417755073815);
void live_H_32(double *state, double *unused, double *out_4649431932622983447);
void live_h_13(double *state, double *unused, double *out_3527303430740254772);
void live_H_13(double *state, double *unused, double *out_2933411284323017290);
void live_h_14(double *state, double *unused, double *out_4122145493608870189);
void live_H_14(double *state, double *unused, double *out_3489824182940765717);
void live_h_33(double *state, double *unused, double *out_4479732143254830184);
void live_H_33(double *state, double *unused, double *out_8680890475386911564);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}