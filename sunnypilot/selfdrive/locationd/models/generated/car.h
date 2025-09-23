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
void car_err_fun(double *nom_x, double *delta_x, double *out_1572442176302239697);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4956545906582697173);
void car_H_mod_fun(double *state, double *out_9059683018143247437);
void car_f_fun(double *state, double dt, double *out_1422803241239556269);
void car_F_fun(double *state, double dt, double *out_6754652119866340026);
void car_h_25(double *state, double *unused, double *out_2596994452580592600);
void car_H_25(double *state, double *unused, double *out_2890038095676274228);
void car_h_24(double *state, double *unused, double *out_2368398905048849498);
void car_H_24(double *state, double *unused, double *out_5884854312135896666);
void car_h_30(double *state, double *unused, double *out_2872188514865098489);
void car_H_30(double *state, double *unused, double *out_2760699148533034158);
void car_h_26(double *state, double *unused, double *out_2129484752795747156);
void car_H_26(double *state, double *unused, double *out_851465223197781996);
void car_h_27(double *state, double *unused, double *out_3802047064833256546);
void car_H_27(double *state, double *unused, double *out_585935836732609247);
void car_h_29(double *state, double *unused, double *out_6148036327612555078);
void car_H_29(double *state, double *unused, double *out_1127426890136941786);
void car_h_28(double *state, double *unused, double *out_4745556917734918981);
void car_H_28(double *state, double *unused, double *out_6209825907206472360);
void car_h_31(double *state, double *unused, double *out_1040455075673429750);
void car_H_31(double *state, double *unused, double *out_2920684057553234656);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}