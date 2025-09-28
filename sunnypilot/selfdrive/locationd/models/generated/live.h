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
void live_H(double *in_vec, double *out_2218613280910052719);
void live_err_fun(double *nom_x, double *delta_x, double *out_821360518623481157);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1314741730512039890);
void live_H_mod_fun(double *state, double *out_3630972730483892510);
void live_f_fun(double *state, double dt, double *out_7671460368630606467);
void live_F_fun(double *state, double dt, double *out_4686695138732055790);
void live_h_4(double *state, double *unused, double *out_77156149660514568);
void live_H_4(double *state, double *unused, double *out_2939217597075968759);
void live_h_9(double *state, double *unused, double *out_7673201636580189952);
void live_H_9(double *state, double *unused, double *out_4348001338188478711);
void live_h_10(double *state, double *unused, double *out_6973211195815797461);
void live_H_10(double *state, double *unused, double *out_8846474086716798225);
void live_h_12(double *state, double *unused, double *out_4437466309729266841);
void live_H_12(double *state, double *unused, double *out_4727910716606481733);
void live_h_35(double *state, double *unused, double *out_4389847154008101682);
void live_H_35(double *state, double *unused, double *out_427444460296638617);
void live_h_32(double *state, double *unused, double *out_5557828964194475961);
void live_H_32(double *state, double *unused, double *out_3132468973198388554);
void live_h_13(double *state, double *unused, double *out_9080111664965347010);
void live_H_13(double *state, double *unused, double *out_3492495889672719349);
void live_h_14(double *state, double *unused, double *out_7673201636580189952);
void live_H_14(double *state, double *unused, double *out_4348001338188478711);
void live_h_33(double *state, double *unused, double *out_5874741186496731186);
void live_H_33(double *state, double *unused, double *out_7822713320139198570);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}