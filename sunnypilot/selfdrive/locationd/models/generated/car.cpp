#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1572442176302239697) {
   out_1572442176302239697[0] = delta_x[0] + nom_x[0];
   out_1572442176302239697[1] = delta_x[1] + nom_x[1];
   out_1572442176302239697[2] = delta_x[2] + nom_x[2];
   out_1572442176302239697[3] = delta_x[3] + nom_x[3];
   out_1572442176302239697[4] = delta_x[4] + nom_x[4];
   out_1572442176302239697[5] = delta_x[5] + nom_x[5];
   out_1572442176302239697[6] = delta_x[6] + nom_x[6];
   out_1572442176302239697[7] = delta_x[7] + nom_x[7];
   out_1572442176302239697[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4956545906582697173) {
   out_4956545906582697173[0] = -nom_x[0] + true_x[0];
   out_4956545906582697173[1] = -nom_x[1] + true_x[1];
   out_4956545906582697173[2] = -nom_x[2] + true_x[2];
   out_4956545906582697173[3] = -nom_x[3] + true_x[3];
   out_4956545906582697173[4] = -nom_x[4] + true_x[4];
   out_4956545906582697173[5] = -nom_x[5] + true_x[5];
   out_4956545906582697173[6] = -nom_x[6] + true_x[6];
   out_4956545906582697173[7] = -nom_x[7] + true_x[7];
   out_4956545906582697173[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_9059683018143247437) {
   out_9059683018143247437[0] = 1.0;
   out_9059683018143247437[1] = 0.0;
   out_9059683018143247437[2] = 0.0;
   out_9059683018143247437[3] = 0.0;
   out_9059683018143247437[4] = 0.0;
   out_9059683018143247437[5] = 0.0;
   out_9059683018143247437[6] = 0.0;
   out_9059683018143247437[7] = 0.0;
   out_9059683018143247437[8] = 0.0;
   out_9059683018143247437[9] = 0.0;
   out_9059683018143247437[10] = 1.0;
   out_9059683018143247437[11] = 0.0;
   out_9059683018143247437[12] = 0.0;
   out_9059683018143247437[13] = 0.0;
   out_9059683018143247437[14] = 0.0;
   out_9059683018143247437[15] = 0.0;
   out_9059683018143247437[16] = 0.0;
   out_9059683018143247437[17] = 0.0;
   out_9059683018143247437[18] = 0.0;
   out_9059683018143247437[19] = 0.0;
   out_9059683018143247437[20] = 1.0;
   out_9059683018143247437[21] = 0.0;
   out_9059683018143247437[22] = 0.0;
   out_9059683018143247437[23] = 0.0;
   out_9059683018143247437[24] = 0.0;
   out_9059683018143247437[25] = 0.0;
   out_9059683018143247437[26] = 0.0;
   out_9059683018143247437[27] = 0.0;
   out_9059683018143247437[28] = 0.0;
   out_9059683018143247437[29] = 0.0;
   out_9059683018143247437[30] = 1.0;
   out_9059683018143247437[31] = 0.0;
   out_9059683018143247437[32] = 0.0;
   out_9059683018143247437[33] = 0.0;
   out_9059683018143247437[34] = 0.0;
   out_9059683018143247437[35] = 0.0;
   out_9059683018143247437[36] = 0.0;
   out_9059683018143247437[37] = 0.0;
   out_9059683018143247437[38] = 0.0;
   out_9059683018143247437[39] = 0.0;
   out_9059683018143247437[40] = 1.0;
   out_9059683018143247437[41] = 0.0;
   out_9059683018143247437[42] = 0.0;
   out_9059683018143247437[43] = 0.0;
   out_9059683018143247437[44] = 0.0;
   out_9059683018143247437[45] = 0.0;
   out_9059683018143247437[46] = 0.0;
   out_9059683018143247437[47] = 0.0;
   out_9059683018143247437[48] = 0.0;
   out_9059683018143247437[49] = 0.0;
   out_9059683018143247437[50] = 1.0;
   out_9059683018143247437[51] = 0.0;
   out_9059683018143247437[52] = 0.0;
   out_9059683018143247437[53] = 0.0;
   out_9059683018143247437[54] = 0.0;
   out_9059683018143247437[55] = 0.0;
   out_9059683018143247437[56] = 0.0;
   out_9059683018143247437[57] = 0.0;
   out_9059683018143247437[58] = 0.0;
   out_9059683018143247437[59] = 0.0;
   out_9059683018143247437[60] = 1.0;
   out_9059683018143247437[61] = 0.0;
   out_9059683018143247437[62] = 0.0;
   out_9059683018143247437[63] = 0.0;
   out_9059683018143247437[64] = 0.0;
   out_9059683018143247437[65] = 0.0;
   out_9059683018143247437[66] = 0.0;
   out_9059683018143247437[67] = 0.0;
   out_9059683018143247437[68] = 0.0;
   out_9059683018143247437[69] = 0.0;
   out_9059683018143247437[70] = 1.0;
   out_9059683018143247437[71] = 0.0;
   out_9059683018143247437[72] = 0.0;
   out_9059683018143247437[73] = 0.0;
   out_9059683018143247437[74] = 0.0;
   out_9059683018143247437[75] = 0.0;
   out_9059683018143247437[76] = 0.0;
   out_9059683018143247437[77] = 0.0;
   out_9059683018143247437[78] = 0.0;
   out_9059683018143247437[79] = 0.0;
   out_9059683018143247437[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1422803241239556269) {
   out_1422803241239556269[0] = state[0];
   out_1422803241239556269[1] = state[1];
   out_1422803241239556269[2] = state[2];
   out_1422803241239556269[3] = state[3];
   out_1422803241239556269[4] = state[4];
   out_1422803241239556269[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1422803241239556269[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1422803241239556269[7] = state[7];
   out_1422803241239556269[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6754652119866340026) {
   out_6754652119866340026[0] = 1;
   out_6754652119866340026[1] = 0;
   out_6754652119866340026[2] = 0;
   out_6754652119866340026[3] = 0;
   out_6754652119866340026[4] = 0;
   out_6754652119866340026[5] = 0;
   out_6754652119866340026[6] = 0;
   out_6754652119866340026[7] = 0;
   out_6754652119866340026[8] = 0;
   out_6754652119866340026[9] = 0;
   out_6754652119866340026[10] = 1;
   out_6754652119866340026[11] = 0;
   out_6754652119866340026[12] = 0;
   out_6754652119866340026[13] = 0;
   out_6754652119866340026[14] = 0;
   out_6754652119866340026[15] = 0;
   out_6754652119866340026[16] = 0;
   out_6754652119866340026[17] = 0;
   out_6754652119866340026[18] = 0;
   out_6754652119866340026[19] = 0;
   out_6754652119866340026[20] = 1;
   out_6754652119866340026[21] = 0;
   out_6754652119866340026[22] = 0;
   out_6754652119866340026[23] = 0;
   out_6754652119866340026[24] = 0;
   out_6754652119866340026[25] = 0;
   out_6754652119866340026[26] = 0;
   out_6754652119866340026[27] = 0;
   out_6754652119866340026[28] = 0;
   out_6754652119866340026[29] = 0;
   out_6754652119866340026[30] = 1;
   out_6754652119866340026[31] = 0;
   out_6754652119866340026[32] = 0;
   out_6754652119866340026[33] = 0;
   out_6754652119866340026[34] = 0;
   out_6754652119866340026[35] = 0;
   out_6754652119866340026[36] = 0;
   out_6754652119866340026[37] = 0;
   out_6754652119866340026[38] = 0;
   out_6754652119866340026[39] = 0;
   out_6754652119866340026[40] = 1;
   out_6754652119866340026[41] = 0;
   out_6754652119866340026[42] = 0;
   out_6754652119866340026[43] = 0;
   out_6754652119866340026[44] = 0;
   out_6754652119866340026[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6754652119866340026[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6754652119866340026[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6754652119866340026[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6754652119866340026[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6754652119866340026[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6754652119866340026[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6754652119866340026[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6754652119866340026[53] = -9.8100000000000005*dt;
   out_6754652119866340026[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6754652119866340026[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6754652119866340026[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6754652119866340026[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6754652119866340026[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6754652119866340026[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6754652119866340026[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6754652119866340026[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6754652119866340026[62] = 0;
   out_6754652119866340026[63] = 0;
   out_6754652119866340026[64] = 0;
   out_6754652119866340026[65] = 0;
   out_6754652119866340026[66] = 0;
   out_6754652119866340026[67] = 0;
   out_6754652119866340026[68] = 0;
   out_6754652119866340026[69] = 0;
   out_6754652119866340026[70] = 1;
   out_6754652119866340026[71] = 0;
   out_6754652119866340026[72] = 0;
   out_6754652119866340026[73] = 0;
   out_6754652119866340026[74] = 0;
   out_6754652119866340026[75] = 0;
   out_6754652119866340026[76] = 0;
   out_6754652119866340026[77] = 0;
   out_6754652119866340026[78] = 0;
   out_6754652119866340026[79] = 0;
   out_6754652119866340026[80] = 1;
}
void h_25(double *state, double *unused, double *out_2596994452580592600) {
   out_2596994452580592600[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2890038095676274228) {
   out_2890038095676274228[0] = 0;
   out_2890038095676274228[1] = 0;
   out_2890038095676274228[2] = 0;
   out_2890038095676274228[3] = 0;
   out_2890038095676274228[4] = 0;
   out_2890038095676274228[5] = 0;
   out_2890038095676274228[6] = 1;
   out_2890038095676274228[7] = 0;
   out_2890038095676274228[8] = 0;
}
void h_24(double *state, double *unused, double *out_2368398905048849498) {
   out_2368398905048849498[0] = state[4];
   out_2368398905048849498[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5884854312135896666) {
   out_5884854312135896666[0] = 0;
   out_5884854312135896666[1] = 0;
   out_5884854312135896666[2] = 0;
   out_5884854312135896666[3] = 0;
   out_5884854312135896666[4] = 1;
   out_5884854312135896666[5] = 0;
   out_5884854312135896666[6] = 0;
   out_5884854312135896666[7] = 0;
   out_5884854312135896666[8] = 0;
   out_5884854312135896666[9] = 0;
   out_5884854312135896666[10] = 0;
   out_5884854312135896666[11] = 0;
   out_5884854312135896666[12] = 0;
   out_5884854312135896666[13] = 0;
   out_5884854312135896666[14] = 1;
   out_5884854312135896666[15] = 0;
   out_5884854312135896666[16] = 0;
   out_5884854312135896666[17] = 0;
}
void h_30(double *state, double *unused, double *out_2872188514865098489) {
   out_2872188514865098489[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2760699148533034158) {
   out_2760699148533034158[0] = 0;
   out_2760699148533034158[1] = 0;
   out_2760699148533034158[2] = 0;
   out_2760699148533034158[3] = 0;
   out_2760699148533034158[4] = 1;
   out_2760699148533034158[5] = 0;
   out_2760699148533034158[6] = 0;
   out_2760699148533034158[7] = 0;
   out_2760699148533034158[8] = 0;
}
void h_26(double *state, double *unused, double *out_2129484752795747156) {
   out_2129484752795747156[0] = state[7];
}
void H_26(double *state, double *unused, double *out_851465223197781996) {
   out_851465223197781996[0] = 0;
   out_851465223197781996[1] = 0;
   out_851465223197781996[2] = 0;
   out_851465223197781996[3] = 0;
   out_851465223197781996[4] = 0;
   out_851465223197781996[5] = 0;
   out_851465223197781996[6] = 0;
   out_851465223197781996[7] = 1;
   out_851465223197781996[8] = 0;
}
void h_27(double *state, double *unused, double *out_3802047064833256546) {
   out_3802047064833256546[0] = state[3];
}
void H_27(double *state, double *unused, double *out_585935836732609247) {
   out_585935836732609247[0] = 0;
   out_585935836732609247[1] = 0;
   out_585935836732609247[2] = 0;
   out_585935836732609247[3] = 1;
   out_585935836732609247[4] = 0;
   out_585935836732609247[5] = 0;
   out_585935836732609247[6] = 0;
   out_585935836732609247[7] = 0;
   out_585935836732609247[8] = 0;
}
void h_29(double *state, double *unused, double *out_6148036327612555078) {
   out_6148036327612555078[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1127426890136941786) {
   out_1127426890136941786[0] = 0;
   out_1127426890136941786[1] = 1;
   out_1127426890136941786[2] = 0;
   out_1127426890136941786[3] = 0;
   out_1127426890136941786[4] = 0;
   out_1127426890136941786[5] = 0;
   out_1127426890136941786[6] = 0;
   out_1127426890136941786[7] = 0;
   out_1127426890136941786[8] = 0;
}
void h_28(double *state, double *unused, double *out_4745556917734918981) {
   out_4745556917734918981[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6209825907206472360) {
   out_6209825907206472360[0] = 1;
   out_6209825907206472360[1] = 0;
   out_6209825907206472360[2] = 0;
   out_6209825907206472360[3] = 0;
   out_6209825907206472360[4] = 0;
   out_6209825907206472360[5] = 0;
   out_6209825907206472360[6] = 0;
   out_6209825907206472360[7] = 0;
   out_6209825907206472360[8] = 0;
}
void h_31(double *state, double *unused, double *out_1040455075673429750) {
   out_1040455075673429750[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2920684057553234656) {
   out_2920684057553234656[0] = 0;
   out_2920684057553234656[1] = 0;
   out_2920684057553234656[2] = 0;
   out_2920684057553234656[3] = 0;
   out_2920684057553234656[4] = 0;
   out_2920684057553234656[5] = 0;
   out_2920684057553234656[6] = 0;
   out_2920684057553234656[7] = 0;
   out_2920684057553234656[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1572442176302239697) {
  err_fun(nom_x, delta_x, out_1572442176302239697);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4956545906582697173) {
  inv_err_fun(nom_x, true_x, out_4956545906582697173);
}
void car_H_mod_fun(double *state, double *out_9059683018143247437) {
  H_mod_fun(state, out_9059683018143247437);
}
void car_f_fun(double *state, double dt, double *out_1422803241239556269) {
  f_fun(state,  dt, out_1422803241239556269);
}
void car_F_fun(double *state, double dt, double *out_6754652119866340026) {
  F_fun(state,  dt, out_6754652119866340026);
}
void car_h_25(double *state, double *unused, double *out_2596994452580592600) {
  h_25(state, unused, out_2596994452580592600);
}
void car_H_25(double *state, double *unused, double *out_2890038095676274228) {
  H_25(state, unused, out_2890038095676274228);
}
void car_h_24(double *state, double *unused, double *out_2368398905048849498) {
  h_24(state, unused, out_2368398905048849498);
}
void car_H_24(double *state, double *unused, double *out_5884854312135896666) {
  H_24(state, unused, out_5884854312135896666);
}
void car_h_30(double *state, double *unused, double *out_2872188514865098489) {
  h_30(state, unused, out_2872188514865098489);
}
void car_H_30(double *state, double *unused, double *out_2760699148533034158) {
  H_30(state, unused, out_2760699148533034158);
}
void car_h_26(double *state, double *unused, double *out_2129484752795747156) {
  h_26(state, unused, out_2129484752795747156);
}
void car_H_26(double *state, double *unused, double *out_851465223197781996) {
  H_26(state, unused, out_851465223197781996);
}
void car_h_27(double *state, double *unused, double *out_3802047064833256546) {
  h_27(state, unused, out_3802047064833256546);
}
void car_H_27(double *state, double *unused, double *out_585935836732609247) {
  H_27(state, unused, out_585935836732609247);
}
void car_h_29(double *state, double *unused, double *out_6148036327612555078) {
  h_29(state, unused, out_6148036327612555078);
}
void car_H_29(double *state, double *unused, double *out_1127426890136941786) {
  H_29(state, unused, out_1127426890136941786);
}
void car_h_28(double *state, double *unused, double *out_4745556917734918981) {
  h_28(state, unused, out_4745556917734918981);
}
void car_H_28(double *state, double *unused, double *out_6209825907206472360) {
  H_28(state, unused, out_6209825907206472360);
}
void car_h_31(double *state, double *unused, double *out_1040455075673429750) {
  h_31(state, unused, out_1040455075673429750);
}
void car_H_31(double *state, double *unused, double *out_2920684057553234656) {
  H_31(state, unused, out_2920684057553234656);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
