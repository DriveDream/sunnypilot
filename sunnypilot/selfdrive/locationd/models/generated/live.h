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
void live_H(double *in_vec, double *out_944076617601105747);
void live_err_fun(double *nom_x, double *delta_x, double *out_801211714328885299);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8253365556107816812);
void live_H_mod_fun(double *state, double *out_6152163765497626030);
void live_f_fun(double *state, double dt, double *out_2632092182064410023);
void live_F_fun(double *state, double dt, double *out_6152802002539678822);
void live_h_4(double *state, double *unused, double *out_4179741611882061313);
void live_H_4(double *state, double *unused, double *out_1371302884800574314);
void live_h_9(double *state, double *unused, double *out_3192872672732649000);
void live_H_9(double *state, double *unused, double *out_2785864851554203169);
void live_h_10(double *state, double *unused, double *out_435323846347685986);
void live_H_10(double *state, double *unused, double *out_4327435354791364482);
void live_h_12(double *state, double *unused, double *out_6317644099969766836);
void live_H_12(double *state, double *unused, double *out_1992401909848167981);
void live_h_35(double *state, double *unused, double *out_8526678555302928578);
void live_H_35(double *state, double *unused, double *out_4737964942173181690);
void live_h_32(double *state, double *unused, double *out_7998059690765395533);
void live_H_32(double *state, double *unused, double *out_3879091678483095508);
void live_h_13(double *state, double *unused, double *out_8845839469221301208);
void live_H_13(double *state, double *unused, double *out_479766949924039896);
void live_h_14(double *state, double *unused, double *out_3192872672732649000);
void live_H_14(double *state, double *unused, double *out_2785864851554203169);
void live_h_33(double *state, double *unused, double *out_2442187313763308882);
void live_H_33(double *state, double *unused, double *out_7888521946812039294);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}