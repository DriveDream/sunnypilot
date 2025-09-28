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
void err_fun(double *nom_x, double *delta_x, double *out_5228023266310668267) {
   out_5228023266310668267[0] = delta_x[0] + nom_x[0];
   out_5228023266310668267[1] = delta_x[1] + nom_x[1];
   out_5228023266310668267[2] = delta_x[2] + nom_x[2];
   out_5228023266310668267[3] = delta_x[3] + nom_x[3];
   out_5228023266310668267[4] = delta_x[4] + nom_x[4];
   out_5228023266310668267[5] = delta_x[5] + nom_x[5];
   out_5228023266310668267[6] = delta_x[6] + nom_x[6];
   out_5228023266310668267[7] = delta_x[7] + nom_x[7];
   out_5228023266310668267[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5306856698853457707) {
   out_5306856698853457707[0] = -nom_x[0] + true_x[0];
   out_5306856698853457707[1] = -nom_x[1] + true_x[1];
   out_5306856698853457707[2] = -nom_x[2] + true_x[2];
   out_5306856698853457707[3] = -nom_x[3] + true_x[3];
   out_5306856698853457707[4] = -nom_x[4] + true_x[4];
   out_5306856698853457707[5] = -nom_x[5] + true_x[5];
   out_5306856698853457707[6] = -nom_x[6] + true_x[6];
   out_5306856698853457707[7] = -nom_x[7] + true_x[7];
   out_5306856698853457707[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_600551320946559308) {
   out_600551320946559308[0] = 1.0;
   out_600551320946559308[1] = 0.0;
   out_600551320946559308[2] = 0.0;
   out_600551320946559308[3] = 0.0;
   out_600551320946559308[4] = 0.0;
   out_600551320946559308[5] = 0.0;
   out_600551320946559308[6] = 0.0;
   out_600551320946559308[7] = 0.0;
   out_600551320946559308[8] = 0.0;
   out_600551320946559308[9] = 0.0;
   out_600551320946559308[10] = 1.0;
   out_600551320946559308[11] = 0.0;
   out_600551320946559308[12] = 0.0;
   out_600551320946559308[13] = 0.0;
   out_600551320946559308[14] = 0.0;
   out_600551320946559308[15] = 0.0;
   out_600551320946559308[16] = 0.0;
   out_600551320946559308[17] = 0.0;
   out_600551320946559308[18] = 0.0;
   out_600551320946559308[19] = 0.0;
   out_600551320946559308[20] = 1.0;
   out_600551320946559308[21] = 0.0;
   out_600551320946559308[22] = 0.0;
   out_600551320946559308[23] = 0.0;
   out_600551320946559308[24] = 0.0;
   out_600551320946559308[25] = 0.0;
   out_600551320946559308[26] = 0.0;
   out_600551320946559308[27] = 0.0;
   out_600551320946559308[28] = 0.0;
   out_600551320946559308[29] = 0.0;
   out_600551320946559308[30] = 1.0;
   out_600551320946559308[31] = 0.0;
   out_600551320946559308[32] = 0.0;
   out_600551320946559308[33] = 0.0;
   out_600551320946559308[34] = 0.0;
   out_600551320946559308[35] = 0.0;
   out_600551320946559308[36] = 0.0;
   out_600551320946559308[37] = 0.0;
   out_600551320946559308[38] = 0.0;
   out_600551320946559308[39] = 0.0;
   out_600551320946559308[40] = 1.0;
   out_600551320946559308[41] = 0.0;
   out_600551320946559308[42] = 0.0;
   out_600551320946559308[43] = 0.0;
   out_600551320946559308[44] = 0.0;
   out_600551320946559308[45] = 0.0;
   out_600551320946559308[46] = 0.0;
   out_600551320946559308[47] = 0.0;
   out_600551320946559308[48] = 0.0;
   out_600551320946559308[49] = 0.0;
   out_600551320946559308[50] = 1.0;
   out_600551320946559308[51] = 0.0;
   out_600551320946559308[52] = 0.0;
   out_600551320946559308[53] = 0.0;
   out_600551320946559308[54] = 0.0;
   out_600551320946559308[55] = 0.0;
   out_600551320946559308[56] = 0.0;
   out_600551320946559308[57] = 0.0;
   out_600551320946559308[58] = 0.0;
   out_600551320946559308[59] = 0.0;
   out_600551320946559308[60] = 1.0;
   out_600551320946559308[61] = 0.0;
   out_600551320946559308[62] = 0.0;
   out_600551320946559308[63] = 0.0;
   out_600551320946559308[64] = 0.0;
   out_600551320946559308[65] = 0.0;
   out_600551320946559308[66] = 0.0;
   out_600551320946559308[67] = 0.0;
   out_600551320946559308[68] = 0.0;
   out_600551320946559308[69] = 0.0;
   out_600551320946559308[70] = 1.0;
   out_600551320946559308[71] = 0.0;
   out_600551320946559308[72] = 0.0;
   out_600551320946559308[73] = 0.0;
   out_600551320946559308[74] = 0.0;
   out_600551320946559308[75] = 0.0;
   out_600551320946559308[76] = 0.0;
   out_600551320946559308[77] = 0.0;
   out_600551320946559308[78] = 0.0;
   out_600551320946559308[79] = 0.0;
   out_600551320946559308[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8129035357947983121) {
   out_8129035357947983121[0] = state[0];
   out_8129035357947983121[1] = state[1];
   out_8129035357947983121[2] = state[2];
   out_8129035357947983121[3] = state[3];
   out_8129035357947983121[4] = state[4];
   out_8129035357947983121[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8129035357947983121[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8129035357947983121[7] = state[7];
   out_8129035357947983121[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8383710118246981636) {
   out_8383710118246981636[0] = 1;
   out_8383710118246981636[1] = 0;
   out_8383710118246981636[2] = 0;
   out_8383710118246981636[3] = 0;
   out_8383710118246981636[4] = 0;
   out_8383710118246981636[5] = 0;
   out_8383710118246981636[6] = 0;
   out_8383710118246981636[7] = 0;
   out_8383710118246981636[8] = 0;
   out_8383710118246981636[9] = 0;
   out_8383710118246981636[10] = 1;
   out_8383710118246981636[11] = 0;
   out_8383710118246981636[12] = 0;
   out_8383710118246981636[13] = 0;
   out_8383710118246981636[14] = 0;
   out_8383710118246981636[15] = 0;
   out_8383710118246981636[16] = 0;
   out_8383710118246981636[17] = 0;
   out_8383710118246981636[18] = 0;
   out_8383710118246981636[19] = 0;
   out_8383710118246981636[20] = 1;
   out_8383710118246981636[21] = 0;
   out_8383710118246981636[22] = 0;
   out_8383710118246981636[23] = 0;
   out_8383710118246981636[24] = 0;
   out_8383710118246981636[25] = 0;
   out_8383710118246981636[26] = 0;
   out_8383710118246981636[27] = 0;
   out_8383710118246981636[28] = 0;
   out_8383710118246981636[29] = 0;
   out_8383710118246981636[30] = 1;
   out_8383710118246981636[31] = 0;
   out_8383710118246981636[32] = 0;
   out_8383710118246981636[33] = 0;
   out_8383710118246981636[34] = 0;
   out_8383710118246981636[35] = 0;
   out_8383710118246981636[36] = 0;
   out_8383710118246981636[37] = 0;
   out_8383710118246981636[38] = 0;
   out_8383710118246981636[39] = 0;
   out_8383710118246981636[40] = 1;
   out_8383710118246981636[41] = 0;
   out_8383710118246981636[42] = 0;
   out_8383710118246981636[43] = 0;
   out_8383710118246981636[44] = 0;
   out_8383710118246981636[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8383710118246981636[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8383710118246981636[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8383710118246981636[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8383710118246981636[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8383710118246981636[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8383710118246981636[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8383710118246981636[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8383710118246981636[53] = -9.8100000000000005*dt;
   out_8383710118246981636[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8383710118246981636[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8383710118246981636[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8383710118246981636[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8383710118246981636[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8383710118246981636[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8383710118246981636[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8383710118246981636[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8383710118246981636[62] = 0;
   out_8383710118246981636[63] = 0;
   out_8383710118246981636[64] = 0;
   out_8383710118246981636[65] = 0;
   out_8383710118246981636[66] = 0;
   out_8383710118246981636[67] = 0;
   out_8383710118246981636[68] = 0;
   out_8383710118246981636[69] = 0;
   out_8383710118246981636[70] = 1;
   out_8383710118246981636[71] = 0;
   out_8383710118246981636[72] = 0;
   out_8383710118246981636[73] = 0;
   out_8383710118246981636[74] = 0;
   out_8383710118246981636[75] = 0;
   out_8383710118246981636[76] = 0;
   out_8383710118246981636[77] = 0;
   out_8383710118246981636[78] = 0;
   out_8383710118246981636[79] = 0;
   out_8383710118246981636[80] = 1;
}
void h_25(double *state, double *unused, double *out_148242157840398241) {
   out_148242157840398241[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2930383418170214680) {
   out_2930383418170214680[0] = 0;
   out_2930383418170214680[1] = 0;
   out_2930383418170214680[2] = 0;
   out_2930383418170214680[3] = 0;
   out_2930383418170214680[4] = 0;
   out_2930383418170214680[5] = 0;
   out_2930383418170214680[6] = 1;
   out_2930383418170214680[7] = 0;
   out_2930383418170214680[8] = 0;
}
void h_24(double *state, double *unused, double *out_697508762897435003) {
   out_697508762897435003[0] = state[4];
   out_697508762897435003[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8940788848947818835) {
   out_8940788848947818835[0] = 0;
   out_8940788848947818835[1] = 0;
   out_8940788848947818835[2] = 0;
   out_8940788848947818835[3] = 0;
   out_8940788848947818835[4] = 1;
   out_8940788848947818835[5] = 0;
   out_8940788848947818835[6] = 0;
   out_8940788848947818835[7] = 0;
   out_8940788848947818835[8] = 0;
   out_8940788848947818835[9] = 0;
   out_8940788848947818835[10] = 0;
   out_8940788848947818835[11] = 0;
   out_8940788848947818835[12] = 0;
   out_8940788848947818835[13] = 0;
   out_8940788848947818835[14] = 1;
   out_8940788848947818835[15] = 0;
   out_8940788848947818835[16] = 0;
   out_8940788848947818835[17] = 0;
}
void h_30(double *state, double *unused, double *out_6072135025006615728) {
   out_6072135025006615728[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5448716376677463307) {
   out_5448716376677463307[0] = 0;
   out_5448716376677463307[1] = 0;
   out_5448716376677463307[2] = 0;
   out_5448716376677463307[3] = 0;
   out_5448716376677463307[4] = 1;
   out_5448716376677463307[5] = 0;
   out_5448716376677463307[6] = 0;
   out_5448716376677463307[7] = 0;
   out_5448716376677463307[8] = 0;
}
void h_26(double *state, double *unused, double *out_5975567531289086742) {
   out_5975567531289086742[0] = state[7];
}
void H_26(double *state, double *unused, double *out_811119900703841544) {
   out_811119900703841544[0] = 0;
   out_811119900703841544[1] = 0;
   out_811119900703841544[2] = 0;
   out_811119900703841544[3] = 0;
   out_811119900703841544[4] = 0;
   out_811119900703841544[5] = 0;
   out_811119900703841544[6] = 0;
   out_811119900703841544[7] = 1;
   out_811119900703841544[8] = 0;
}
void h_27(double *state, double *unused, double *out_4853431109820447404) {
   out_4853431109820447404[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7672310447861406524) {
   out_7672310447861406524[0] = 0;
   out_7672310447861406524[1] = 0;
   out_7672310447861406524[2] = 0;
   out_7672310447861406524[3] = 1;
   out_7672310447861406524[4] = 0;
   out_7672310447861406524[5] = 0;
   out_7672310447861406524[6] = 0;
   out_7672310447861406524[7] = 0;
   out_7672310447861406524[8] = 0;
}
void h_29(double *state, double *unused, double *out_2909834147000225037) {
   out_2909834147000225037[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5958947720991855491) {
   out_5958947720991855491[0] = 0;
   out_5958947720991855491[1] = 1;
   out_5958947720991855491[2] = 0;
   out_5958947720991855491[3] = 0;
   out_5958947720991855491[4] = 0;
   out_5958947720991855491[5] = 0;
   out_5958947720991855491[6] = 0;
   out_5958947720991855491[7] = 0;
   out_5958947720991855491[8] = 0;
}
void h_28(double *state, double *unused, double *out_2149331593972618375) {
   out_2149331593972618375[0] = state[0];
}
void H_28(double *state, double *unused, double *out_876548703922324917) {
   out_876548703922324917[0] = 1;
   out_876548703922324917[1] = 0;
   out_876548703922324917[2] = 0;
   out_876548703922324917[3] = 0;
   out_876548703922324917[4] = 0;
   out_876548703922324917[5] = 0;
   out_876548703922324917[6] = 0;
   out_876548703922324917[7] = 0;
   out_876548703922324917[8] = 0;
}
void h_31(double *state, double *unused, double *out_3785691686094420591) {
   out_3785691686094420591[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2961029380047175108) {
   out_2961029380047175108[0] = 0;
   out_2961029380047175108[1] = 0;
   out_2961029380047175108[2] = 0;
   out_2961029380047175108[3] = 0;
   out_2961029380047175108[4] = 0;
   out_2961029380047175108[5] = 0;
   out_2961029380047175108[6] = 0;
   out_2961029380047175108[7] = 0;
   out_2961029380047175108[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5228023266310668267) {
  err_fun(nom_x, delta_x, out_5228023266310668267);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5306856698853457707) {
  inv_err_fun(nom_x, true_x, out_5306856698853457707);
}
void car_H_mod_fun(double *state, double *out_600551320946559308) {
  H_mod_fun(state, out_600551320946559308);
}
void car_f_fun(double *state, double dt, double *out_8129035357947983121) {
  f_fun(state,  dt, out_8129035357947983121);
}
void car_F_fun(double *state, double dt, double *out_8383710118246981636) {
  F_fun(state,  dt, out_8383710118246981636);
}
void car_h_25(double *state, double *unused, double *out_148242157840398241) {
  h_25(state, unused, out_148242157840398241);
}
void car_H_25(double *state, double *unused, double *out_2930383418170214680) {
  H_25(state, unused, out_2930383418170214680);
}
void car_h_24(double *state, double *unused, double *out_697508762897435003) {
  h_24(state, unused, out_697508762897435003);
}
void car_H_24(double *state, double *unused, double *out_8940788848947818835) {
  H_24(state, unused, out_8940788848947818835);
}
void car_h_30(double *state, double *unused, double *out_6072135025006615728) {
  h_30(state, unused, out_6072135025006615728);
}
void car_H_30(double *state, double *unused, double *out_5448716376677463307) {
  H_30(state, unused, out_5448716376677463307);
}
void car_h_26(double *state, double *unused, double *out_5975567531289086742) {
  h_26(state, unused, out_5975567531289086742);
}
void car_H_26(double *state, double *unused, double *out_811119900703841544) {
  H_26(state, unused, out_811119900703841544);
}
void car_h_27(double *state, double *unused, double *out_4853431109820447404) {
  h_27(state, unused, out_4853431109820447404);
}
void car_H_27(double *state, double *unused, double *out_7672310447861406524) {
  H_27(state, unused, out_7672310447861406524);
}
void car_h_29(double *state, double *unused, double *out_2909834147000225037) {
  h_29(state, unused, out_2909834147000225037);
}
void car_H_29(double *state, double *unused, double *out_5958947720991855491) {
  H_29(state, unused, out_5958947720991855491);
}
void car_h_28(double *state, double *unused, double *out_2149331593972618375) {
  h_28(state, unused, out_2149331593972618375);
}
void car_H_28(double *state, double *unused, double *out_876548703922324917) {
  H_28(state, unused, out_876548703922324917);
}
void car_h_31(double *state, double *unused, double *out_3785691686094420591) {
  h_31(state, unused, out_3785691686094420591);
}
void car_H_31(double *state, double *unused, double *out_2961029380047175108) {
  H_31(state, unused, out_2961029380047175108);
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
