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
void err_fun(double *nom_x, double *delta_x, double *out_1200516966254576139) {
   out_1200516966254576139[0] = delta_x[0] + nom_x[0];
   out_1200516966254576139[1] = delta_x[1] + nom_x[1];
   out_1200516966254576139[2] = delta_x[2] + nom_x[2];
   out_1200516966254576139[3] = delta_x[3] + nom_x[3];
   out_1200516966254576139[4] = delta_x[4] + nom_x[4];
   out_1200516966254576139[5] = delta_x[5] + nom_x[5];
   out_1200516966254576139[6] = delta_x[6] + nom_x[6];
   out_1200516966254576139[7] = delta_x[7] + nom_x[7];
   out_1200516966254576139[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6340514496067370621) {
   out_6340514496067370621[0] = -nom_x[0] + true_x[0];
   out_6340514496067370621[1] = -nom_x[1] + true_x[1];
   out_6340514496067370621[2] = -nom_x[2] + true_x[2];
   out_6340514496067370621[3] = -nom_x[3] + true_x[3];
   out_6340514496067370621[4] = -nom_x[4] + true_x[4];
   out_6340514496067370621[5] = -nom_x[5] + true_x[5];
   out_6340514496067370621[6] = -nom_x[6] + true_x[6];
   out_6340514496067370621[7] = -nom_x[7] + true_x[7];
   out_6340514496067370621[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1568428756196638601) {
   out_1568428756196638601[0] = 1.0;
   out_1568428756196638601[1] = 0.0;
   out_1568428756196638601[2] = 0.0;
   out_1568428756196638601[3] = 0.0;
   out_1568428756196638601[4] = 0.0;
   out_1568428756196638601[5] = 0.0;
   out_1568428756196638601[6] = 0.0;
   out_1568428756196638601[7] = 0.0;
   out_1568428756196638601[8] = 0.0;
   out_1568428756196638601[9] = 0.0;
   out_1568428756196638601[10] = 1.0;
   out_1568428756196638601[11] = 0.0;
   out_1568428756196638601[12] = 0.0;
   out_1568428756196638601[13] = 0.0;
   out_1568428756196638601[14] = 0.0;
   out_1568428756196638601[15] = 0.0;
   out_1568428756196638601[16] = 0.0;
   out_1568428756196638601[17] = 0.0;
   out_1568428756196638601[18] = 0.0;
   out_1568428756196638601[19] = 0.0;
   out_1568428756196638601[20] = 1.0;
   out_1568428756196638601[21] = 0.0;
   out_1568428756196638601[22] = 0.0;
   out_1568428756196638601[23] = 0.0;
   out_1568428756196638601[24] = 0.0;
   out_1568428756196638601[25] = 0.0;
   out_1568428756196638601[26] = 0.0;
   out_1568428756196638601[27] = 0.0;
   out_1568428756196638601[28] = 0.0;
   out_1568428756196638601[29] = 0.0;
   out_1568428756196638601[30] = 1.0;
   out_1568428756196638601[31] = 0.0;
   out_1568428756196638601[32] = 0.0;
   out_1568428756196638601[33] = 0.0;
   out_1568428756196638601[34] = 0.0;
   out_1568428756196638601[35] = 0.0;
   out_1568428756196638601[36] = 0.0;
   out_1568428756196638601[37] = 0.0;
   out_1568428756196638601[38] = 0.0;
   out_1568428756196638601[39] = 0.0;
   out_1568428756196638601[40] = 1.0;
   out_1568428756196638601[41] = 0.0;
   out_1568428756196638601[42] = 0.0;
   out_1568428756196638601[43] = 0.0;
   out_1568428756196638601[44] = 0.0;
   out_1568428756196638601[45] = 0.0;
   out_1568428756196638601[46] = 0.0;
   out_1568428756196638601[47] = 0.0;
   out_1568428756196638601[48] = 0.0;
   out_1568428756196638601[49] = 0.0;
   out_1568428756196638601[50] = 1.0;
   out_1568428756196638601[51] = 0.0;
   out_1568428756196638601[52] = 0.0;
   out_1568428756196638601[53] = 0.0;
   out_1568428756196638601[54] = 0.0;
   out_1568428756196638601[55] = 0.0;
   out_1568428756196638601[56] = 0.0;
   out_1568428756196638601[57] = 0.0;
   out_1568428756196638601[58] = 0.0;
   out_1568428756196638601[59] = 0.0;
   out_1568428756196638601[60] = 1.0;
   out_1568428756196638601[61] = 0.0;
   out_1568428756196638601[62] = 0.0;
   out_1568428756196638601[63] = 0.0;
   out_1568428756196638601[64] = 0.0;
   out_1568428756196638601[65] = 0.0;
   out_1568428756196638601[66] = 0.0;
   out_1568428756196638601[67] = 0.0;
   out_1568428756196638601[68] = 0.0;
   out_1568428756196638601[69] = 0.0;
   out_1568428756196638601[70] = 1.0;
   out_1568428756196638601[71] = 0.0;
   out_1568428756196638601[72] = 0.0;
   out_1568428756196638601[73] = 0.0;
   out_1568428756196638601[74] = 0.0;
   out_1568428756196638601[75] = 0.0;
   out_1568428756196638601[76] = 0.0;
   out_1568428756196638601[77] = 0.0;
   out_1568428756196638601[78] = 0.0;
   out_1568428756196638601[79] = 0.0;
   out_1568428756196638601[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4338504881593374904) {
   out_4338504881593374904[0] = state[0];
   out_4338504881593374904[1] = state[1];
   out_4338504881593374904[2] = state[2];
   out_4338504881593374904[3] = state[3];
   out_4338504881593374904[4] = state[4];
   out_4338504881593374904[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4338504881593374904[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4338504881593374904[7] = state[7];
   out_4338504881593374904[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4187513685334090243) {
   out_4187513685334090243[0] = 1;
   out_4187513685334090243[1] = 0;
   out_4187513685334090243[2] = 0;
   out_4187513685334090243[3] = 0;
   out_4187513685334090243[4] = 0;
   out_4187513685334090243[5] = 0;
   out_4187513685334090243[6] = 0;
   out_4187513685334090243[7] = 0;
   out_4187513685334090243[8] = 0;
   out_4187513685334090243[9] = 0;
   out_4187513685334090243[10] = 1;
   out_4187513685334090243[11] = 0;
   out_4187513685334090243[12] = 0;
   out_4187513685334090243[13] = 0;
   out_4187513685334090243[14] = 0;
   out_4187513685334090243[15] = 0;
   out_4187513685334090243[16] = 0;
   out_4187513685334090243[17] = 0;
   out_4187513685334090243[18] = 0;
   out_4187513685334090243[19] = 0;
   out_4187513685334090243[20] = 1;
   out_4187513685334090243[21] = 0;
   out_4187513685334090243[22] = 0;
   out_4187513685334090243[23] = 0;
   out_4187513685334090243[24] = 0;
   out_4187513685334090243[25] = 0;
   out_4187513685334090243[26] = 0;
   out_4187513685334090243[27] = 0;
   out_4187513685334090243[28] = 0;
   out_4187513685334090243[29] = 0;
   out_4187513685334090243[30] = 1;
   out_4187513685334090243[31] = 0;
   out_4187513685334090243[32] = 0;
   out_4187513685334090243[33] = 0;
   out_4187513685334090243[34] = 0;
   out_4187513685334090243[35] = 0;
   out_4187513685334090243[36] = 0;
   out_4187513685334090243[37] = 0;
   out_4187513685334090243[38] = 0;
   out_4187513685334090243[39] = 0;
   out_4187513685334090243[40] = 1;
   out_4187513685334090243[41] = 0;
   out_4187513685334090243[42] = 0;
   out_4187513685334090243[43] = 0;
   out_4187513685334090243[44] = 0;
   out_4187513685334090243[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4187513685334090243[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4187513685334090243[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4187513685334090243[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4187513685334090243[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4187513685334090243[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4187513685334090243[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4187513685334090243[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4187513685334090243[53] = -9.8100000000000005*dt;
   out_4187513685334090243[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4187513685334090243[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4187513685334090243[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4187513685334090243[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4187513685334090243[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4187513685334090243[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4187513685334090243[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4187513685334090243[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4187513685334090243[62] = 0;
   out_4187513685334090243[63] = 0;
   out_4187513685334090243[64] = 0;
   out_4187513685334090243[65] = 0;
   out_4187513685334090243[66] = 0;
   out_4187513685334090243[67] = 0;
   out_4187513685334090243[68] = 0;
   out_4187513685334090243[69] = 0;
   out_4187513685334090243[70] = 1;
   out_4187513685334090243[71] = 0;
   out_4187513685334090243[72] = 0;
   out_4187513685334090243[73] = 0;
   out_4187513685334090243[74] = 0;
   out_4187513685334090243[75] = 0;
   out_4187513685334090243[76] = 0;
   out_4187513685334090243[77] = 0;
   out_4187513685334090243[78] = 0;
   out_4187513685334090243[79] = 0;
   out_4187513685334090243[80] = 1;
}
void h_25(double *state, double *unused, double *out_4879031403620109471) {
   out_4879031403620109471[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8956977284539053210) {
   out_8956977284539053210[0] = 0;
   out_8956977284539053210[1] = 0;
   out_8956977284539053210[2] = 0;
   out_8956977284539053210[3] = 0;
   out_8956977284539053210[4] = 0;
   out_8956977284539053210[5] = 0;
   out_8956977284539053210[6] = 1;
   out_8956977284539053210[7] = 0;
   out_8956977284539053210[8] = 0;
}
void h_24(double *state, double *unused, double *out_368488579487421442) {
   out_368488579487421442[0] = state[4];
   out_368488579487421442[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1538585650862126317) {
   out_1538585650862126317[0] = 0;
   out_1538585650862126317[1] = 0;
   out_1538585650862126317[2] = 0;
   out_1538585650862126317[3] = 0;
   out_1538585650862126317[4] = 1;
   out_1538585650862126317[5] = 0;
   out_1538585650862126317[6] = 0;
   out_1538585650862126317[7] = 0;
   out_1538585650862126317[8] = 0;
   out_1538585650862126317[9] = 0;
   out_1538585650862126317[10] = 0;
   out_1538585650862126317[11] = 0;
   out_1538585650862126317[12] = 0;
   out_1538585650862126317[13] = 0;
   out_1538585650862126317[14] = 1;
   out_1538585650862126317[15] = 0;
   out_1538585650862126317[16] = 0;
   out_1538585650862126317[17] = 0;
}
void h_30(double *state, double *unused, double *out_1241581875366087121) {
   out_1241581875366087121[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6438644326031804583) {
   out_6438644326031804583[0] = 0;
   out_6438644326031804583[1] = 0;
   out_6438644326031804583[2] = 0;
   out_6438644326031804583[3] = 0;
   out_6438644326031804583[4] = 1;
   out_6438644326031804583[5] = 0;
   out_6438644326031804583[6] = 0;
   out_6438644326031804583[7] = 0;
   out_6438644326031804583[8] = 0;
}
void h_26(double *state, double *unused, double *out_7740387296640753644) {
   out_7740387296640753644[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5748263470296442182) {
   out_5748263470296442182[0] = 0;
   out_5748263470296442182[1] = 0;
   out_5748263470296442182[2] = 0;
   out_5748263470296442182[3] = 0;
   out_5748263470296442182[4] = 0;
   out_5748263470296442182[5] = 0;
   out_5748263470296442182[6] = 0;
   out_5748263470296442182[7] = 1;
   out_5748263470296442182[8] = 0;
}
void h_27(double *state, double *unused, double *out_122641864040736174) {
   out_122641864040736174[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7185664530226833425) {
   out_7185664530226833425[0] = 0;
   out_7185664530226833425[1] = 0;
   out_7185664530226833425[2] = 0;
   out_7185664530226833425[3] = 1;
   out_7185664530226833425[4] = 0;
   out_7185664530226833425[5] = 0;
   out_7185664530226833425[6] = 0;
   out_7185664530226833425[7] = 0;
   out_7185664530226833425[8] = 0;
}
void h_29(double *state, double *unused, double *out_3760091392294758524) {
   out_3760091392294758524[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5928412981717412399) {
   out_5928412981717412399[0] = 0;
   out_5928412981717412399[1] = 1;
   out_5928412981717412399[2] = 0;
   out_5928412981717412399[3] = 0;
   out_5928412981717412399[4] = 0;
   out_5928412981717412399[5] = 0;
   out_5928412981717412399[6] = 0;
   out_5928412981717412399[7] = 0;
   out_5928412981717412399[8] = 0;
}
void h_28(double *state, double *unused, double *out_7312501570387964639) {
   out_7312501570387964639[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7435932074922608643) {
   out_7435932074922608643[0] = 1;
   out_7435932074922608643[1] = 0;
   out_7435932074922608643[2] = 0;
   out_7435932074922608643[3] = 0;
   out_7435932074922608643[4] = 0;
   out_7435932074922608643[5] = 0;
   out_7435932074922608643[6] = 0;
   out_7435932074922608643[7] = 0;
   out_7435932074922608643[8] = 0;
}
void h_31(double *state, double *unused, double *out_2581527360628778854) {
   out_2581527360628778854[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5122055368063090706) {
   out_5122055368063090706[0] = 0;
   out_5122055368063090706[1] = 0;
   out_5122055368063090706[2] = 0;
   out_5122055368063090706[3] = 0;
   out_5122055368063090706[4] = 0;
   out_5122055368063090706[5] = 0;
   out_5122055368063090706[6] = 0;
   out_5122055368063090706[7] = 0;
   out_5122055368063090706[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1200516966254576139) {
  err_fun(nom_x, delta_x, out_1200516966254576139);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6340514496067370621) {
  inv_err_fun(nom_x, true_x, out_6340514496067370621);
}
void car_H_mod_fun(double *state, double *out_1568428756196638601) {
  H_mod_fun(state, out_1568428756196638601);
}
void car_f_fun(double *state, double dt, double *out_4338504881593374904) {
  f_fun(state,  dt, out_4338504881593374904);
}
void car_F_fun(double *state, double dt, double *out_4187513685334090243) {
  F_fun(state,  dt, out_4187513685334090243);
}
void car_h_25(double *state, double *unused, double *out_4879031403620109471) {
  h_25(state, unused, out_4879031403620109471);
}
void car_H_25(double *state, double *unused, double *out_8956977284539053210) {
  H_25(state, unused, out_8956977284539053210);
}
void car_h_24(double *state, double *unused, double *out_368488579487421442) {
  h_24(state, unused, out_368488579487421442);
}
void car_H_24(double *state, double *unused, double *out_1538585650862126317) {
  H_24(state, unused, out_1538585650862126317);
}
void car_h_30(double *state, double *unused, double *out_1241581875366087121) {
  h_30(state, unused, out_1241581875366087121);
}
void car_H_30(double *state, double *unused, double *out_6438644326031804583) {
  H_30(state, unused, out_6438644326031804583);
}
void car_h_26(double *state, double *unused, double *out_7740387296640753644) {
  h_26(state, unused, out_7740387296640753644);
}
void car_H_26(double *state, double *unused, double *out_5748263470296442182) {
  H_26(state, unused, out_5748263470296442182);
}
void car_h_27(double *state, double *unused, double *out_122641864040736174) {
  h_27(state, unused, out_122641864040736174);
}
void car_H_27(double *state, double *unused, double *out_7185664530226833425) {
  H_27(state, unused, out_7185664530226833425);
}
void car_h_29(double *state, double *unused, double *out_3760091392294758524) {
  h_29(state, unused, out_3760091392294758524);
}
void car_H_29(double *state, double *unused, double *out_5928412981717412399) {
  H_29(state, unused, out_5928412981717412399);
}
void car_h_28(double *state, double *unused, double *out_7312501570387964639) {
  h_28(state, unused, out_7312501570387964639);
}
void car_H_28(double *state, double *unused, double *out_7435932074922608643) {
  H_28(state, unused, out_7435932074922608643);
}
void car_h_31(double *state, double *unused, double *out_2581527360628778854) {
  h_31(state, unused, out_2581527360628778854);
}
void car_H_31(double *state, double *unused, double *out_5122055368063090706) {
  H_31(state, unused, out_5122055368063090706);
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
