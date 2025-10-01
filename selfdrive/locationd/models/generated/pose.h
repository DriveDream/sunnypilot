#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_9033090375826759827);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9158915157634621903);
void pose_H_mod_fun(double *state, double *out_7413758488612487521);
void pose_f_fun(double *state, double dt, double *out_3232314017905079538);
void pose_F_fun(double *state, double dt, double *out_5501644288694722521);
void pose_h_4(double *state, double *unused, double *out_2917410844925153063);
void pose_H_4(double *state, double *unused, double *out_7261178375253156523);
void pose_h_10(double *state, double *unused, double *out_2425004257133495675);
void pose_H_10(double *state, double *unused, double *out_735877534270755889);
void pose_h_13(double *state, double *unused, double *out_2858741791526306440);
void pose_H_13(double *state, double *unused, double *out_4048904549920823722);
void pose_h_14(double *state, double *unused, double *out_7352000875437629003);
void pose_H_14(double *state, double *unused, double *out_8102777266161022797);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}