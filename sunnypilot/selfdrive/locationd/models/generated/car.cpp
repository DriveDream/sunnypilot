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
void err_fun(double *nom_x, double *delta_x, double *out_2827551707420463633) {
   out_2827551707420463633[0] = delta_x[0] + nom_x[0];
   out_2827551707420463633[1] = delta_x[1] + nom_x[1];
   out_2827551707420463633[2] = delta_x[2] + nom_x[2];
   out_2827551707420463633[3] = delta_x[3] + nom_x[3];
   out_2827551707420463633[4] = delta_x[4] + nom_x[4];
   out_2827551707420463633[5] = delta_x[5] + nom_x[5];
   out_2827551707420463633[6] = delta_x[6] + nom_x[6];
   out_2827551707420463633[7] = delta_x[7] + nom_x[7];
   out_2827551707420463633[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8408426563124879774) {
   out_8408426563124879774[0] = -nom_x[0] + true_x[0];
   out_8408426563124879774[1] = -nom_x[1] + true_x[1];
   out_8408426563124879774[2] = -nom_x[2] + true_x[2];
   out_8408426563124879774[3] = -nom_x[3] + true_x[3];
   out_8408426563124879774[4] = -nom_x[4] + true_x[4];
   out_8408426563124879774[5] = -nom_x[5] + true_x[5];
   out_8408426563124879774[6] = -nom_x[6] + true_x[6];
   out_8408426563124879774[7] = -nom_x[7] + true_x[7];
   out_8408426563124879774[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8585748337696332209) {
   out_8585748337696332209[0] = 1.0;
   out_8585748337696332209[1] = 0.0;
   out_8585748337696332209[2] = 0.0;
   out_8585748337696332209[3] = 0.0;
   out_8585748337696332209[4] = 0.0;
   out_8585748337696332209[5] = 0.0;
   out_8585748337696332209[6] = 0.0;
   out_8585748337696332209[7] = 0.0;
   out_8585748337696332209[8] = 0.0;
   out_8585748337696332209[9] = 0.0;
   out_8585748337696332209[10] = 1.0;
   out_8585748337696332209[11] = 0.0;
   out_8585748337696332209[12] = 0.0;
   out_8585748337696332209[13] = 0.0;
   out_8585748337696332209[14] = 0.0;
   out_8585748337696332209[15] = 0.0;
   out_8585748337696332209[16] = 0.0;
   out_8585748337696332209[17] = 0.0;
   out_8585748337696332209[18] = 0.0;
   out_8585748337696332209[19] = 0.0;
   out_8585748337696332209[20] = 1.0;
   out_8585748337696332209[21] = 0.0;
   out_8585748337696332209[22] = 0.0;
   out_8585748337696332209[23] = 0.0;
   out_8585748337696332209[24] = 0.0;
   out_8585748337696332209[25] = 0.0;
   out_8585748337696332209[26] = 0.0;
   out_8585748337696332209[27] = 0.0;
   out_8585748337696332209[28] = 0.0;
   out_8585748337696332209[29] = 0.0;
   out_8585748337696332209[30] = 1.0;
   out_8585748337696332209[31] = 0.0;
   out_8585748337696332209[32] = 0.0;
   out_8585748337696332209[33] = 0.0;
   out_8585748337696332209[34] = 0.0;
   out_8585748337696332209[35] = 0.0;
   out_8585748337696332209[36] = 0.0;
   out_8585748337696332209[37] = 0.0;
   out_8585748337696332209[38] = 0.0;
   out_8585748337696332209[39] = 0.0;
   out_8585748337696332209[40] = 1.0;
   out_8585748337696332209[41] = 0.0;
   out_8585748337696332209[42] = 0.0;
   out_8585748337696332209[43] = 0.0;
   out_8585748337696332209[44] = 0.0;
   out_8585748337696332209[45] = 0.0;
   out_8585748337696332209[46] = 0.0;
   out_8585748337696332209[47] = 0.0;
   out_8585748337696332209[48] = 0.0;
   out_8585748337696332209[49] = 0.0;
   out_8585748337696332209[50] = 1.0;
   out_8585748337696332209[51] = 0.0;
   out_8585748337696332209[52] = 0.0;
   out_8585748337696332209[53] = 0.0;
   out_8585748337696332209[54] = 0.0;
   out_8585748337696332209[55] = 0.0;
   out_8585748337696332209[56] = 0.0;
   out_8585748337696332209[57] = 0.0;
   out_8585748337696332209[58] = 0.0;
   out_8585748337696332209[59] = 0.0;
   out_8585748337696332209[60] = 1.0;
   out_8585748337696332209[61] = 0.0;
   out_8585748337696332209[62] = 0.0;
   out_8585748337696332209[63] = 0.0;
   out_8585748337696332209[64] = 0.0;
   out_8585748337696332209[65] = 0.0;
   out_8585748337696332209[66] = 0.0;
   out_8585748337696332209[67] = 0.0;
   out_8585748337696332209[68] = 0.0;
   out_8585748337696332209[69] = 0.0;
   out_8585748337696332209[70] = 1.0;
   out_8585748337696332209[71] = 0.0;
   out_8585748337696332209[72] = 0.0;
   out_8585748337696332209[73] = 0.0;
   out_8585748337696332209[74] = 0.0;
   out_8585748337696332209[75] = 0.0;
   out_8585748337696332209[76] = 0.0;
   out_8585748337696332209[77] = 0.0;
   out_8585748337696332209[78] = 0.0;
   out_8585748337696332209[79] = 0.0;
   out_8585748337696332209[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8033397392187571078) {
   out_8033397392187571078[0] = state[0];
   out_8033397392187571078[1] = state[1];
   out_8033397392187571078[2] = state[2];
   out_8033397392187571078[3] = state[3];
   out_8033397392187571078[4] = state[4];
   out_8033397392187571078[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8033397392187571078[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8033397392187571078[7] = state[7];
   out_8033397392187571078[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4114962685454462076) {
   out_4114962685454462076[0] = 1;
   out_4114962685454462076[1] = 0;
   out_4114962685454462076[2] = 0;
   out_4114962685454462076[3] = 0;
   out_4114962685454462076[4] = 0;
   out_4114962685454462076[5] = 0;
   out_4114962685454462076[6] = 0;
   out_4114962685454462076[7] = 0;
   out_4114962685454462076[8] = 0;
   out_4114962685454462076[9] = 0;
   out_4114962685454462076[10] = 1;
   out_4114962685454462076[11] = 0;
   out_4114962685454462076[12] = 0;
   out_4114962685454462076[13] = 0;
   out_4114962685454462076[14] = 0;
   out_4114962685454462076[15] = 0;
   out_4114962685454462076[16] = 0;
   out_4114962685454462076[17] = 0;
   out_4114962685454462076[18] = 0;
   out_4114962685454462076[19] = 0;
   out_4114962685454462076[20] = 1;
   out_4114962685454462076[21] = 0;
   out_4114962685454462076[22] = 0;
   out_4114962685454462076[23] = 0;
   out_4114962685454462076[24] = 0;
   out_4114962685454462076[25] = 0;
   out_4114962685454462076[26] = 0;
   out_4114962685454462076[27] = 0;
   out_4114962685454462076[28] = 0;
   out_4114962685454462076[29] = 0;
   out_4114962685454462076[30] = 1;
   out_4114962685454462076[31] = 0;
   out_4114962685454462076[32] = 0;
   out_4114962685454462076[33] = 0;
   out_4114962685454462076[34] = 0;
   out_4114962685454462076[35] = 0;
   out_4114962685454462076[36] = 0;
   out_4114962685454462076[37] = 0;
   out_4114962685454462076[38] = 0;
   out_4114962685454462076[39] = 0;
   out_4114962685454462076[40] = 1;
   out_4114962685454462076[41] = 0;
   out_4114962685454462076[42] = 0;
   out_4114962685454462076[43] = 0;
   out_4114962685454462076[44] = 0;
   out_4114962685454462076[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4114962685454462076[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4114962685454462076[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4114962685454462076[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4114962685454462076[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4114962685454462076[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4114962685454462076[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4114962685454462076[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4114962685454462076[53] = -9.8100000000000005*dt;
   out_4114962685454462076[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4114962685454462076[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4114962685454462076[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4114962685454462076[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4114962685454462076[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4114962685454462076[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4114962685454462076[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4114962685454462076[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4114962685454462076[62] = 0;
   out_4114962685454462076[63] = 0;
   out_4114962685454462076[64] = 0;
   out_4114962685454462076[65] = 0;
   out_4114962685454462076[66] = 0;
   out_4114962685454462076[67] = 0;
   out_4114962685454462076[68] = 0;
   out_4114962685454462076[69] = 0;
   out_4114962685454462076[70] = 1;
   out_4114962685454462076[71] = 0;
   out_4114962685454462076[72] = 0;
   out_4114962685454462076[73] = 0;
   out_4114962685454462076[74] = 0;
   out_4114962685454462076[75] = 0;
   out_4114962685454462076[76] = 0;
   out_4114962685454462076[77] = 0;
   out_4114962685454462076[78] = 0;
   out_4114962685454462076[79] = 0;
   out_4114962685454462076[80] = 1;
}
void h_25(double *state, double *unused, double *out_4057584842250850052) {
   out_4057584842250850052[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6438589400418951797) {
   out_6438589400418951797[0] = 0;
   out_6438589400418951797[1] = 0;
   out_6438589400418951797[2] = 0;
   out_6438589400418951797[3] = 0;
   out_6438589400418951797[4] = 0;
   out_6438589400418951797[5] = 0;
   out_6438589400418951797[6] = 1;
   out_6438589400418951797[7] = 0;
   out_6438589400418951797[8] = 0;
}
void h_24(double *state, double *unused, double *out_5883550569039178904) {
   out_5883550569039178904[0] = state[4];
   out_5883550569039178904[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1618267895762963534) {
   out_1618267895762963534[0] = 0;
   out_1618267895762963534[1] = 0;
   out_1618267895762963534[2] = 0;
   out_1618267895762963534[3] = 0;
   out_1618267895762963534[4] = 1;
   out_1618267895762963534[5] = 0;
   out_1618267895762963534[6] = 0;
   out_1618267895762963534[7] = 0;
   out_1618267895762963534[8] = 0;
   out_1618267895762963534[9] = 0;
   out_1618267895762963534[10] = 0;
   out_1618267895762963534[11] = 0;
   out_1618267895762963534[12] = 0;
   out_1618267895762963534[13] = 0;
   out_1618267895762963534[14] = 1;
   out_1618267895762963534[15] = 0;
   out_1618267895762963534[16] = 0;
   out_1618267895762963534[17] = 0;
}
void h_30(double *state, double *unused, double *out_7618324005108350628) {
   out_7618324005108350628[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6309250453275711727) {
   out_6309250453275711727[0] = 0;
   out_6309250453275711727[1] = 0;
   out_6309250453275711727[2] = 0;
   out_6309250453275711727[3] = 0;
   out_6309250453275711727[4] = 1;
   out_6309250453275711727[5] = 0;
   out_6309250453275711727[6] = 0;
   out_6309250453275711727[7] = 0;
   out_6309250453275711727[8] = 0;
}
void h_26(double *state, double *unused, double *out_8561833858010013063) {
   out_8561833858010013063[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2697086081544895573) {
   out_2697086081544895573[0] = 0;
   out_2697086081544895573[1] = 0;
   out_2697086081544895573[2] = 0;
   out_2697086081544895573[3] = 0;
   out_2697086081544895573[4] = 0;
   out_2697086081544895573[5] = 0;
   out_2697086081544895573[6] = 0;
   out_2697086081544895573[7] = 1;
   out_2697086081544895573[8] = 0;
}
void h_27(double *state, double *unused, double *out_944088425409995593) {
   out_944088425409995593[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4134487141475286816) {
   out_4134487141475286816[0] = 0;
   out_4134487141475286816[1] = 0;
   out_4134487141475286816[2] = 0;
   out_4134487141475286816[3] = 1;
   out_4134487141475286816[4] = 0;
   out_4134487141475286816[5] = 0;
   out_4134487141475286816[6] = 0;
   out_4134487141475286816[7] = 0;
   out_4134487141475286816[8] = 0;
}
void h_29(double *state, double *unused, double *out_4805771901838497245) {
   out_4805771901838497245[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6819481797590103911) {
   out_6819481797590103911[0] = 0;
   out_6819481797590103911[1] = 1;
   out_6819481797590103911[2] = 0;
   out_6819481797590103911[3] = 0;
   out_6819481797590103911[4] = 0;
   out_6819481797590103911[5] = 0;
   out_6819481797590103911[6] = 0;
   out_6819481797590103911[7] = 0;
   out_6819481797590103911[8] = 0;
}
void h_28(double *state, double *unused, double *out_746359409325685367) {
   out_746359409325685367[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1737082780520573337) {
   out_1737082780520573337[0] = 1;
   out_1737082780520573337[1] = 0;
   out_1737082780520573337[2] = 0;
   out_1737082780520573337[3] = 0;
   out_1737082780520573337[4] = 0;
   out_1737082780520573337[5] = 0;
   out_1737082780520573337[6] = 0;
   out_1737082780520573337[7] = 0;
   out_1737082780520573337[8] = 0;
}
void h_31(double *state, double *unused, double *out_2606953449621966885) {
   out_2606953449621966885[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6469235362295912225) {
   out_6469235362295912225[0] = 0;
   out_6469235362295912225[1] = 0;
   out_6469235362295912225[2] = 0;
   out_6469235362295912225[3] = 0;
   out_6469235362295912225[4] = 0;
   out_6469235362295912225[5] = 0;
   out_6469235362295912225[6] = 0;
   out_6469235362295912225[7] = 0;
   out_6469235362295912225[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2827551707420463633) {
  err_fun(nom_x, delta_x, out_2827551707420463633);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8408426563124879774) {
  inv_err_fun(nom_x, true_x, out_8408426563124879774);
}
void car_H_mod_fun(double *state, double *out_8585748337696332209) {
  H_mod_fun(state, out_8585748337696332209);
}
void car_f_fun(double *state, double dt, double *out_8033397392187571078) {
  f_fun(state,  dt, out_8033397392187571078);
}
void car_F_fun(double *state, double dt, double *out_4114962685454462076) {
  F_fun(state,  dt, out_4114962685454462076);
}
void car_h_25(double *state, double *unused, double *out_4057584842250850052) {
  h_25(state, unused, out_4057584842250850052);
}
void car_H_25(double *state, double *unused, double *out_6438589400418951797) {
  H_25(state, unused, out_6438589400418951797);
}
void car_h_24(double *state, double *unused, double *out_5883550569039178904) {
  h_24(state, unused, out_5883550569039178904);
}
void car_H_24(double *state, double *unused, double *out_1618267895762963534) {
  H_24(state, unused, out_1618267895762963534);
}
void car_h_30(double *state, double *unused, double *out_7618324005108350628) {
  h_30(state, unused, out_7618324005108350628);
}
void car_H_30(double *state, double *unused, double *out_6309250453275711727) {
  H_30(state, unused, out_6309250453275711727);
}
void car_h_26(double *state, double *unused, double *out_8561833858010013063) {
  h_26(state, unused, out_8561833858010013063);
}
void car_H_26(double *state, double *unused, double *out_2697086081544895573) {
  H_26(state, unused, out_2697086081544895573);
}
void car_h_27(double *state, double *unused, double *out_944088425409995593) {
  h_27(state, unused, out_944088425409995593);
}
void car_H_27(double *state, double *unused, double *out_4134487141475286816) {
  H_27(state, unused, out_4134487141475286816);
}
void car_h_29(double *state, double *unused, double *out_4805771901838497245) {
  h_29(state, unused, out_4805771901838497245);
}
void car_H_29(double *state, double *unused, double *out_6819481797590103911) {
  H_29(state, unused, out_6819481797590103911);
}
void car_h_28(double *state, double *unused, double *out_746359409325685367) {
  h_28(state, unused, out_746359409325685367);
}
void car_H_28(double *state, double *unused, double *out_1737082780520573337) {
  H_28(state, unused, out_1737082780520573337);
}
void car_h_31(double *state, double *unused, double *out_2606953449621966885) {
  h_31(state, unused, out_2606953449621966885);
}
void car_H_31(double *state, double *unused, double *out_6469235362295912225) {
  H_31(state, unused, out_6469235362295912225);
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
