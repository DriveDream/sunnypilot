#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_9033090375826759827) {
   out_9033090375826759827[0] = delta_x[0] + nom_x[0];
   out_9033090375826759827[1] = delta_x[1] + nom_x[1];
   out_9033090375826759827[2] = delta_x[2] + nom_x[2];
   out_9033090375826759827[3] = delta_x[3] + nom_x[3];
   out_9033090375826759827[4] = delta_x[4] + nom_x[4];
   out_9033090375826759827[5] = delta_x[5] + nom_x[5];
   out_9033090375826759827[6] = delta_x[6] + nom_x[6];
   out_9033090375826759827[7] = delta_x[7] + nom_x[7];
   out_9033090375826759827[8] = delta_x[8] + nom_x[8];
   out_9033090375826759827[9] = delta_x[9] + nom_x[9];
   out_9033090375826759827[10] = delta_x[10] + nom_x[10];
   out_9033090375826759827[11] = delta_x[11] + nom_x[11];
   out_9033090375826759827[12] = delta_x[12] + nom_x[12];
   out_9033090375826759827[13] = delta_x[13] + nom_x[13];
   out_9033090375826759827[14] = delta_x[14] + nom_x[14];
   out_9033090375826759827[15] = delta_x[15] + nom_x[15];
   out_9033090375826759827[16] = delta_x[16] + nom_x[16];
   out_9033090375826759827[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9158915157634621903) {
   out_9158915157634621903[0] = -nom_x[0] + true_x[0];
   out_9158915157634621903[1] = -nom_x[1] + true_x[1];
   out_9158915157634621903[2] = -nom_x[2] + true_x[2];
   out_9158915157634621903[3] = -nom_x[3] + true_x[3];
   out_9158915157634621903[4] = -nom_x[4] + true_x[4];
   out_9158915157634621903[5] = -nom_x[5] + true_x[5];
   out_9158915157634621903[6] = -nom_x[6] + true_x[6];
   out_9158915157634621903[7] = -nom_x[7] + true_x[7];
   out_9158915157634621903[8] = -nom_x[8] + true_x[8];
   out_9158915157634621903[9] = -nom_x[9] + true_x[9];
   out_9158915157634621903[10] = -nom_x[10] + true_x[10];
   out_9158915157634621903[11] = -nom_x[11] + true_x[11];
   out_9158915157634621903[12] = -nom_x[12] + true_x[12];
   out_9158915157634621903[13] = -nom_x[13] + true_x[13];
   out_9158915157634621903[14] = -nom_x[14] + true_x[14];
   out_9158915157634621903[15] = -nom_x[15] + true_x[15];
   out_9158915157634621903[16] = -nom_x[16] + true_x[16];
   out_9158915157634621903[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_7413758488612487521) {
   out_7413758488612487521[0] = 1.0;
   out_7413758488612487521[1] = 0.0;
   out_7413758488612487521[2] = 0.0;
   out_7413758488612487521[3] = 0.0;
   out_7413758488612487521[4] = 0.0;
   out_7413758488612487521[5] = 0.0;
   out_7413758488612487521[6] = 0.0;
   out_7413758488612487521[7] = 0.0;
   out_7413758488612487521[8] = 0.0;
   out_7413758488612487521[9] = 0.0;
   out_7413758488612487521[10] = 0.0;
   out_7413758488612487521[11] = 0.0;
   out_7413758488612487521[12] = 0.0;
   out_7413758488612487521[13] = 0.0;
   out_7413758488612487521[14] = 0.0;
   out_7413758488612487521[15] = 0.0;
   out_7413758488612487521[16] = 0.0;
   out_7413758488612487521[17] = 0.0;
   out_7413758488612487521[18] = 0.0;
   out_7413758488612487521[19] = 1.0;
   out_7413758488612487521[20] = 0.0;
   out_7413758488612487521[21] = 0.0;
   out_7413758488612487521[22] = 0.0;
   out_7413758488612487521[23] = 0.0;
   out_7413758488612487521[24] = 0.0;
   out_7413758488612487521[25] = 0.0;
   out_7413758488612487521[26] = 0.0;
   out_7413758488612487521[27] = 0.0;
   out_7413758488612487521[28] = 0.0;
   out_7413758488612487521[29] = 0.0;
   out_7413758488612487521[30] = 0.0;
   out_7413758488612487521[31] = 0.0;
   out_7413758488612487521[32] = 0.0;
   out_7413758488612487521[33] = 0.0;
   out_7413758488612487521[34] = 0.0;
   out_7413758488612487521[35] = 0.0;
   out_7413758488612487521[36] = 0.0;
   out_7413758488612487521[37] = 0.0;
   out_7413758488612487521[38] = 1.0;
   out_7413758488612487521[39] = 0.0;
   out_7413758488612487521[40] = 0.0;
   out_7413758488612487521[41] = 0.0;
   out_7413758488612487521[42] = 0.0;
   out_7413758488612487521[43] = 0.0;
   out_7413758488612487521[44] = 0.0;
   out_7413758488612487521[45] = 0.0;
   out_7413758488612487521[46] = 0.0;
   out_7413758488612487521[47] = 0.0;
   out_7413758488612487521[48] = 0.0;
   out_7413758488612487521[49] = 0.0;
   out_7413758488612487521[50] = 0.0;
   out_7413758488612487521[51] = 0.0;
   out_7413758488612487521[52] = 0.0;
   out_7413758488612487521[53] = 0.0;
   out_7413758488612487521[54] = 0.0;
   out_7413758488612487521[55] = 0.0;
   out_7413758488612487521[56] = 0.0;
   out_7413758488612487521[57] = 1.0;
   out_7413758488612487521[58] = 0.0;
   out_7413758488612487521[59] = 0.0;
   out_7413758488612487521[60] = 0.0;
   out_7413758488612487521[61] = 0.0;
   out_7413758488612487521[62] = 0.0;
   out_7413758488612487521[63] = 0.0;
   out_7413758488612487521[64] = 0.0;
   out_7413758488612487521[65] = 0.0;
   out_7413758488612487521[66] = 0.0;
   out_7413758488612487521[67] = 0.0;
   out_7413758488612487521[68] = 0.0;
   out_7413758488612487521[69] = 0.0;
   out_7413758488612487521[70] = 0.0;
   out_7413758488612487521[71] = 0.0;
   out_7413758488612487521[72] = 0.0;
   out_7413758488612487521[73] = 0.0;
   out_7413758488612487521[74] = 0.0;
   out_7413758488612487521[75] = 0.0;
   out_7413758488612487521[76] = 1.0;
   out_7413758488612487521[77] = 0.0;
   out_7413758488612487521[78] = 0.0;
   out_7413758488612487521[79] = 0.0;
   out_7413758488612487521[80] = 0.0;
   out_7413758488612487521[81] = 0.0;
   out_7413758488612487521[82] = 0.0;
   out_7413758488612487521[83] = 0.0;
   out_7413758488612487521[84] = 0.0;
   out_7413758488612487521[85] = 0.0;
   out_7413758488612487521[86] = 0.0;
   out_7413758488612487521[87] = 0.0;
   out_7413758488612487521[88] = 0.0;
   out_7413758488612487521[89] = 0.0;
   out_7413758488612487521[90] = 0.0;
   out_7413758488612487521[91] = 0.0;
   out_7413758488612487521[92] = 0.0;
   out_7413758488612487521[93] = 0.0;
   out_7413758488612487521[94] = 0.0;
   out_7413758488612487521[95] = 1.0;
   out_7413758488612487521[96] = 0.0;
   out_7413758488612487521[97] = 0.0;
   out_7413758488612487521[98] = 0.0;
   out_7413758488612487521[99] = 0.0;
   out_7413758488612487521[100] = 0.0;
   out_7413758488612487521[101] = 0.0;
   out_7413758488612487521[102] = 0.0;
   out_7413758488612487521[103] = 0.0;
   out_7413758488612487521[104] = 0.0;
   out_7413758488612487521[105] = 0.0;
   out_7413758488612487521[106] = 0.0;
   out_7413758488612487521[107] = 0.0;
   out_7413758488612487521[108] = 0.0;
   out_7413758488612487521[109] = 0.0;
   out_7413758488612487521[110] = 0.0;
   out_7413758488612487521[111] = 0.0;
   out_7413758488612487521[112] = 0.0;
   out_7413758488612487521[113] = 0.0;
   out_7413758488612487521[114] = 1.0;
   out_7413758488612487521[115] = 0.0;
   out_7413758488612487521[116] = 0.0;
   out_7413758488612487521[117] = 0.0;
   out_7413758488612487521[118] = 0.0;
   out_7413758488612487521[119] = 0.0;
   out_7413758488612487521[120] = 0.0;
   out_7413758488612487521[121] = 0.0;
   out_7413758488612487521[122] = 0.0;
   out_7413758488612487521[123] = 0.0;
   out_7413758488612487521[124] = 0.0;
   out_7413758488612487521[125] = 0.0;
   out_7413758488612487521[126] = 0.0;
   out_7413758488612487521[127] = 0.0;
   out_7413758488612487521[128] = 0.0;
   out_7413758488612487521[129] = 0.0;
   out_7413758488612487521[130] = 0.0;
   out_7413758488612487521[131] = 0.0;
   out_7413758488612487521[132] = 0.0;
   out_7413758488612487521[133] = 1.0;
   out_7413758488612487521[134] = 0.0;
   out_7413758488612487521[135] = 0.0;
   out_7413758488612487521[136] = 0.0;
   out_7413758488612487521[137] = 0.0;
   out_7413758488612487521[138] = 0.0;
   out_7413758488612487521[139] = 0.0;
   out_7413758488612487521[140] = 0.0;
   out_7413758488612487521[141] = 0.0;
   out_7413758488612487521[142] = 0.0;
   out_7413758488612487521[143] = 0.0;
   out_7413758488612487521[144] = 0.0;
   out_7413758488612487521[145] = 0.0;
   out_7413758488612487521[146] = 0.0;
   out_7413758488612487521[147] = 0.0;
   out_7413758488612487521[148] = 0.0;
   out_7413758488612487521[149] = 0.0;
   out_7413758488612487521[150] = 0.0;
   out_7413758488612487521[151] = 0.0;
   out_7413758488612487521[152] = 1.0;
   out_7413758488612487521[153] = 0.0;
   out_7413758488612487521[154] = 0.0;
   out_7413758488612487521[155] = 0.0;
   out_7413758488612487521[156] = 0.0;
   out_7413758488612487521[157] = 0.0;
   out_7413758488612487521[158] = 0.0;
   out_7413758488612487521[159] = 0.0;
   out_7413758488612487521[160] = 0.0;
   out_7413758488612487521[161] = 0.0;
   out_7413758488612487521[162] = 0.0;
   out_7413758488612487521[163] = 0.0;
   out_7413758488612487521[164] = 0.0;
   out_7413758488612487521[165] = 0.0;
   out_7413758488612487521[166] = 0.0;
   out_7413758488612487521[167] = 0.0;
   out_7413758488612487521[168] = 0.0;
   out_7413758488612487521[169] = 0.0;
   out_7413758488612487521[170] = 0.0;
   out_7413758488612487521[171] = 1.0;
   out_7413758488612487521[172] = 0.0;
   out_7413758488612487521[173] = 0.0;
   out_7413758488612487521[174] = 0.0;
   out_7413758488612487521[175] = 0.0;
   out_7413758488612487521[176] = 0.0;
   out_7413758488612487521[177] = 0.0;
   out_7413758488612487521[178] = 0.0;
   out_7413758488612487521[179] = 0.0;
   out_7413758488612487521[180] = 0.0;
   out_7413758488612487521[181] = 0.0;
   out_7413758488612487521[182] = 0.0;
   out_7413758488612487521[183] = 0.0;
   out_7413758488612487521[184] = 0.0;
   out_7413758488612487521[185] = 0.0;
   out_7413758488612487521[186] = 0.0;
   out_7413758488612487521[187] = 0.0;
   out_7413758488612487521[188] = 0.0;
   out_7413758488612487521[189] = 0.0;
   out_7413758488612487521[190] = 1.0;
   out_7413758488612487521[191] = 0.0;
   out_7413758488612487521[192] = 0.0;
   out_7413758488612487521[193] = 0.0;
   out_7413758488612487521[194] = 0.0;
   out_7413758488612487521[195] = 0.0;
   out_7413758488612487521[196] = 0.0;
   out_7413758488612487521[197] = 0.0;
   out_7413758488612487521[198] = 0.0;
   out_7413758488612487521[199] = 0.0;
   out_7413758488612487521[200] = 0.0;
   out_7413758488612487521[201] = 0.0;
   out_7413758488612487521[202] = 0.0;
   out_7413758488612487521[203] = 0.0;
   out_7413758488612487521[204] = 0.0;
   out_7413758488612487521[205] = 0.0;
   out_7413758488612487521[206] = 0.0;
   out_7413758488612487521[207] = 0.0;
   out_7413758488612487521[208] = 0.0;
   out_7413758488612487521[209] = 1.0;
   out_7413758488612487521[210] = 0.0;
   out_7413758488612487521[211] = 0.0;
   out_7413758488612487521[212] = 0.0;
   out_7413758488612487521[213] = 0.0;
   out_7413758488612487521[214] = 0.0;
   out_7413758488612487521[215] = 0.0;
   out_7413758488612487521[216] = 0.0;
   out_7413758488612487521[217] = 0.0;
   out_7413758488612487521[218] = 0.0;
   out_7413758488612487521[219] = 0.0;
   out_7413758488612487521[220] = 0.0;
   out_7413758488612487521[221] = 0.0;
   out_7413758488612487521[222] = 0.0;
   out_7413758488612487521[223] = 0.0;
   out_7413758488612487521[224] = 0.0;
   out_7413758488612487521[225] = 0.0;
   out_7413758488612487521[226] = 0.0;
   out_7413758488612487521[227] = 0.0;
   out_7413758488612487521[228] = 1.0;
   out_7413758488612487521[229] = 0.0;
   out_7413758488612487521[230] = 0.0;
   out_7413758488612487521[231] = 0.0;
   out_7413758488612487521[232] = 0.0;
   out_7413758488612487521[233] = 0.0;
   out_7413758488612487521[234] = 0.0;
   out_7413758488612487521[235] = 0.0;
   out_7413758488612487521[236] = 0.0;
   out_7413758488612487521[237] = 0.0;
   out_7413758488612487521[238] = 0.0;
   out_7413758488612487521[239] = 0.0;
   out_7413758488612487521[240] = 0.0;
   out_7413758488612487521[241] = 0.0;
   out_7413758488612487521[242] = 0.0;
   out_7413758488612487521[243] = 0.0;
   out_7413758488612487521[244] = 0.0;
   out_7413758488612487521[245] = 0.0;
   out_7413758488612487521[246] = 0.0;
   out_7413758488612487521[247] = 1.0;
   out_7413758488612487521[248] = 0.0;
   out_7413758488612487521[249] = 0.0;
   out_7413758488612487521[250] = 0.0;
   out_7413758488612487521[251] = 0.0;
   out_7413758488612487521[252] = 0.0;
   out_7413758488612487521[253] = 0.0;
   out_7413758488612487521[254] = 0.0;
   out_7413758488612487521[255] = 0.0;
   out_7413758488612487521[256] = 0.0;
   out_7413758488612487521[257] = 0.0;
   out_7413758488612487521[258] = 0.0;
   out_7413758488612487521[259] = 0.0;
   out_7413758488612487521[260] = 0.0;
   out_7413758488612487521[261] = 0.0;
   out_7413758488612487521[262] = 0.0;
   out_7413758488612487521[263] = 0.0;
   out_7413758488612487521[264] = 0.0;
   out_7413758488612487521[265] = 0.0;
   out_7413758488612487521[266] = 1.0;
   out_7413758488612487521[267] = 0.0;
   out_7413758488612487521[268] = 0.0;
   out_7413758488612487521[269] = 0.0;
   out_7413758488612487521[270] = 0.0;
   out_7413758488612487521[271] = 0.0;
   out_7413758488612487521[272] = 0.0;
   out_7413758488612487521[273] = 0.0;
   out_7413758488612487521[274] = 0.0;
   out_7413758488612487521[275] = 0.0;
   out_7413758488612487521[276] = 0.0;
   out_7413758488612487521[277] = 0.0;
   out_7413758488612487521[278] = 0.0;
   out_7413758488612487521[279] = 0.0;
   out_7413758488612487521[280] = 0.0;
   out_7413758488612487521[281] = 0.0;
   out_7413758488612487521[282] = 0.0;
   out_7413758488612487521[283] = 0.0;
   out_7413758488612487521[284] = 0.0;
   out_7413758488612487521[285] = 1.0;
   out_7413758488612487521[286] = 0.0;
   out_7413758488612487521[287] = 0.0;
   out_7413758488612487521[288] = 0.0;
   out_7413758488612487521[289] = 0.0;
   out_7413758488612487521[290] = 0.0;
   out_7413758488612487521[291] = 0.0;
   out_7413758488612487521[292] = 0.0;
   out_7413758488612487521[293] = 0.0;
   out_7413758488612487521[294] = 0.0;
   out_7413758488612487521[295] = 0.0;
   out_7413758488612487521[296] = 0.0;
   out_7413758488612487521[297] = 0.0;
   out_7413758488612487521[298] = 0.0;
   out_7413758488612487521[299] = 0.0;
   out_7413758488612487521[300] = 0.0;
   out_7413758488612487521[301] = 0.0;
   out_7413758488612487521[302] = 0.0;
   out_7413758488612487521[303] = 0.0;
   out_7413758488612487521[304] = 1.0;
   out_7413758488612487521[305] = 0.0;
   out_7413758488612487521[306] = 0.0;
   out_7413758488612487521[307] = 0.0;
   out_7413758488612487521[308] = 0.0;
   out_7413758488612487521[309] = 0.0;
   out_7413758488612487521[310] = 0.0;
   out_7413758488612487521[311] = 0.0;
   out_7413758488612487521[312] = 0.0;
   out_7413758488612487521[313] = 0.0;
   out_7413758488612487521[314] = 0.0;
   out_7413758488612487521[315] = 0.0;
   out_7413758488612487521[316] = 0.0;
   out_7413758488612487521[317] = 0.0;
   out_7413758488612487521[318] = 0.0;
   out_7413758488612487521[319] = 0.0;
   out_7413758488612487521[320] = 0.0;
   out_7413758488612487521[321] = 0.0;
   out_7413758488612487521[322] = 0.0;
   out_7413758488612487521[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3232314017905079538) {
   out_3232314017905079538[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3232314017905079538[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3232314017905079538[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3232314017905079538[3] = dt*state[12] + state[3];
   out_3232314017905079538[4] = dt*state[13] + state[4];
   out_3232314017905079538[5] = dt*state[14] + state[5];
   out_3232314017905079538[6] = state[6];
   out_3232314017905079538[7] = state[7];
   out_3232314017905079538[8] = state[8];
   out_3232314017905079538[9] = state[9];
   out_3232314017905079538[10] = state[10];
   out_3232314017905079538[11] = state[11];
   out_3232314017905079538[12] = state[12];
   out_3232314017905079538[13] = state[13];
   out_3232314017905079538[14] = state[14];
   out_3232314017905079538[15] = state[15];
   out_3232314017905079538[16] = state[16];
   out_3232314017905079538[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5501644288694722521) {
   out_5501644288694722521[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5501644288694722521[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5501644288694722521[2] = 0;
   out_5501644288694722521[3] = 0;
   out_5501644288694722521[4] = 0;
   out_5501644288694722521[5] = 0;
   out_5501644288694722521[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5501644288694722521[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5501644288694722521[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5501644288694722521[9] = 0;
   out_5501644288694722521[10] = 0;
   out_5501644288694722521[11] = 0;
   out_5501644288694722521[12] = 0;
   out_5501644288694722521[13] = 0;
   out_5501644288694722521[14] = 0;
   out_5501644288694722521[15] = 0;
   out_5501644288694722521[16] = 0;
   out_5501644288694722521[17] = 0;
   out_5501644288694722521[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5501644288694722521[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5501644288694722521[20] = 0;
   out_5501644288694722521[21] = 0;
   out_5501644288694722521[22] = 0;
   out_5501644288694722521[23] = 0;
   out_5501644288694722521[24] = 0;
   out_5501644288694722521[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5501644288694722521[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5501644288694722521[27] = 0;
   out_5501644288694722521[28] = 0;
   out_5501644288694722521[29] = 0;
   out_5501644288694722521[30] = 0;
   out_5501644288694722521[31] = 0;
   out_5501644288694722521[32] = 0;
   out_5501644288694722521[33] = 0;
   out_5501644288694722521[34] = 0;
   out_5501644288694722521[35] = 0;
   out_5501644288694722521[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5501644288694722521[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5501644288694722521[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5501644288694722521[39] = 0;
   out_5501644288694722521[40] = 0;
   out_5501644288694722521[41] = 0;
   out_5501644288694722521[42] = 0;
   out_5501644288694722521[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5501644288694722521[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5501644288694722521[45] = 0;
   out_5501644288694722521[46] = 0;
   out_5501644288694722521[47] = 0;
   out_5501644288694722521[48] = 0;
   out_5501644288694722521[49] = 0;
   out_5501644288694722521[50] = 0;
   out_5501644288694722521[51] = 0;
   out_5501644288694722521[52] = 0;
   out_5501644288694722521[53] = 0;
   out_5501644288694722521[54] = 0;
   out_5501644288694722521[55] = 0;
   out_5501644288694722521[56] = 0;
   out_5501644288694722521[57] = 1;
   out_5501644288694722521[58] = 0;
   out_5501644288694722521[59] = 0;
   out_5501644288694722521[60] = 0;
   out_5501644288694722521[61] = 0;
   out_5501644288694722521[62] = 0;
   out_5501644288694722521[63] = 0;
   out_5501644288694722521[64] = 0;
   out_5501644288694722521[65] = 0;
   out_5501644288694722521[66] = dt;
   out_5501644288694722521[67] = 0;
   out_5501644288694722521[68] = 0;
   out_5501644288694722521[69] = 0;
   out_5501644288694722521[70] = 0;
   out_5501644288694722521[71] = 0;
   out_5501644288694722521[72] = 0;
   out_5501644288694722521[73] = 0;
   out_5501644288694722521[74] = 0;
   out_5501644288694722521[75] = 0;
   out_5501644288694722521[76] = 1;
   out_5501644288694722521[77] = 0;
   out_5501644288694722521[78] = 0;
   out_5501644288694722521[79] = 0;
   out_5501644288694722521[80] = 0;
   out_5501644288694722521[81] = 0;
   out_5501644288694722521[82] = 0;
   out_5501644288694722521[83] = 0;
   out_5501644288694722521[84] = 0;
   out_5501644288694722521[85] = dt;
   out_5501644288694722521[86] = 0;
   out_5501644288694722521[87] = 0;
   out_5501644288694722521[88] = 0;
   out_5501644288694722521[89] = 0;
   out_5501644288694722521[90] = 0;
   out_5501644288694722521[91] = 0;
   out_5501644288694722521[92] = 0;
   out_5501644288694722521[93] = 0;
   out_5501644288694722521[94] = 0;
   out_5501644288694722521[95] = 1;
   out_5501644288694722521[96] = 0;
   out_5501644288694722521[97] = 0;
   out_5501644288694722521[98] = 0;
   out_5501644288694722521[99] = 0;
   out_5501644288694722521[100] = 0;
   out_5501644288694722521[101] = 0;
   out_5501644288694722521[102] = 0;
   out_5501644288694722521[103] = 0;
   out_5501644288694722521[104] = dt;
   out_5501644288694722521[105] = 0;
   out_5501644288694722521[106] = 0;
   out_5501644288694722521[107] = 0;
   out_5501644288694722521[108] = 0;
   out_5501644288694722521[109] = 0;
   out_5501644288694722521[110] = 0;
   out_5501644288694722521[111] = 0;
   out_5501644288694722521[112] = 0;
   out_5501644288694722521[113] = 0;
   out_5501644288694722521[114] = 1;
   out_5501644288694722521[115] = 0;
   out_5501644288694722521[116] = 0;
   out_5501644288694722521[117] = 0;
   out_5501644288694722521[118] = 0;
   out_5501644288694722521[119] = 0;
   out_5501644288694722521[120] = 0;
   out_5501644288694722521[121] = 0;
   out_5501644288694722521[122] = 0;
   out_5501644288694722521[123] = 0;
   out_5501644288694722521[124] = 0;
   out_5501644288694722521[125] = 0;
   out_5501644288694722521[126] = 0;
   out_5501644288694722521[127] = 0;
   out_5501644288694722521[128] = 0;
   out_5501644288694722521[129] = 0;
   out_5501644288694722521[130] = 0;
   out_5501644288694722521[131] = 0;
   out_5501644288694722521[132] = 0;
   out_5501644288694722521[133] = 1;
   out_5501644288694722521[134] = 0;
   out_5501644288694722521[135] = 0;
   out_5501644288694722521[136] = 0;
   out_5501644288694722521[137] = 0;
   out_5501644288694722521[138] = 0;
   out_5501644288694722521[139] = 0;
   out_5501644288694722521[140] = 0;
   out_5501644288694722521[141] = 0;
   out_5501644288694722521[142] = 0;
   out_5501644288694722521[143] = 0;
   out_5501644288694722521[144] = 0;
   out_5501644288694722521[145] = 0;
   out_5501644288694722521[146] = 0;
   out_5501644288694722521[147] = 0;
   out_5501644288694722521[148] = 0;
   out_5501644288694722521[149] = 0;
   out_5501644288694722521[150] = 0;
   out_5501644288694722521[151] = 0;
   out_5501644288694722521[152] = 1;
   out_5501644288694722521[153] = 0;
   out_5501644288694722521[154] = 0;
   out_5501644288694722521[155] = 0;
   out_5501644288694722521[156] = 0;
   out_5501644288694722521[157] = 0;
   out_5501644288694722521[158] = 0;
   out_5501644288694722521[159] = 0;
   out_5501644288694722521[160] = 0;
   out_5501644288694722521[161] = 0;
   out_5501644288694722521[162] = 0;
   out_5501644288694722521[163] = 0;
   out_5501644288694722521[164] = 0;
   out_5501644288694722521[165] = 0;
   out_5501644288694722521[166] = 0;
   out_5501644288694722521[167] = 0;
   out_5501644288694722521[168] = 0;
   out_5501644288694722521[169] = 0;
   out_5501644288694722521[170] = 0;
   out_5501644288694722521[171] = 1;
   out_5501644288694722521[172] = 0;
   out_5501644288694722521[173] = 0;
   out_5501644288694722521[174] = 0;
   out_5501644288694722521[175] = 0;
   out_5501644288694722521[176] = 0;
   out_5501644288694722521[177] = 0;
   out_5501644288694722521[178] = 0;
   out_5501644288694722521[179] = 0;
   out_5501644288694722521[180] = 0;
   out_5501644288694722521[181] = 0;
   out_5501644288694722521[182] = 0;
   out_5501644288694722521[183] = 0;
   out_5501644288694722521[184] = 0;
   out_5501644288694722521[185] = 0;
   out_5501644288694722521[186] = 0;
   out_5501644288694722521[187] = 0;
   out_5501644288694722521[188] = 0;
   out_5501644288694722521[189] = 0;
   out_5501644288694722521[190] = 1;
   out_5501644288694722521[191] = 0;
   out_5501644288694722521[192] = 0;
   out_5501644288694722521[193] = 0;
   out_5501644288694722521[194] = 0;
   out_5501644288694722521[195] = 0;
   out_5501644288694722521[196] = 0;
   out_5501644288694722521[197] = 0;
   out_5501644288694722521[198] = 0;
   out_5501644288694722521[199] = 0;
   out_5501644288694722521[200] = 0;
   out_5501644288694722521[201] = 0;
   out_5501644288694722521[202] = 0;
   out_5501644288694722521[203] = 0;
   out_5501644288694722521[204] = 0;
   out_5501644288694722521[205] = 0;
   out_5501644288694722521[206] = 0;
   out_5501644288694722521[207] = 0;
   out_5501644288694722521[208] = 0;
   out_5501644288694722521[209] = 1;
   out_5501644288694722521[210] = 0;
   out_5501644288694722521[211] = 0;
   out_5501644288694722521[212] = 0;
   out_5501644288694722521[213] = 0;
   out_5501644288694722521[214] = 0;
   out_5501644288694722521[215] = 0;
   out_5501644288694722521[216] = 0;
   out_5501644288694722521[217] = 0;
   out_5501644288694722521[218] = 0;
   out_5501644288694722521[219] = 0;
   out_5501644288694722521[220] = 0;
   out_5501644288694722521[221] = 0;
   out_5501644288694722521[222] = 0;
   out_5501644288694722521[223] = 0;
   out_5501644288694722521[224] = 0;
   out_5501644288694722521[225] = 0;
   out_5501644288694722521[226] = 0;
   out_5501644288694722521[227] = 0;
   out_5501644288694722521[228] = 1;
   out_5501644288694722521[229] = 0;
   out_5501644288694722521[230] = 0;
   out_5501644288694722521[231] = 0;
   out_5501644288694722521[232] = 0;
   out_5501644288694722521[233] = 0;
   out_5501644288694722521[234] = 0;
   out_5501644288694722521[235] = 0;
   out_5501644288694722521[236] = 0;
   out_5501644288694722521[237] = 0;
   out_5501644288694722521[238] = 0;
   out_5501644288694722521[239] = 0;
   out_5501644288694722521[240] = 0;
   out_5501644288694722521[241] = 0;
   out_5501644288694722521[242] = 0;
   out_5501644288694722521[243] = 0;
   out_5501644288694722521[244] = 0;
   out_5501644288694722521[245] = 0;
   out_5501644288694722521[246] = 0;
   out_5501644288694722521[247] = 1;
   out_5501644288694722521[248] = 0;
   out_5501644288694722521[249] = 0;
   out_5501644288694722521[250] = 0;
   out_5501644288694722521[251] = 0;
   out_5501644288694722521[252] = 0;
   out_5501644288694722521[253] = 0;
   out_5501644288694722521[254] = 0;
   out_5501644288694722521[255] = 0;
   out_5501644288694722521[256] = 0;
   out_5501644288694722521[257] = 0;
   out_5501644288694722521[258] = 0;
   out_5501644288694722521[259] = 0;
   out_5501644288694722521[260] = 0;
   out_5501644288694722521[261] = 0;
   out_5501644288694722521[262] = 0;
   out_5501644288694722521[263] = 0;
   out_5501644288694722521[264] = 0;
   out_5501644288694722521[265] = 0;
   out_5501644288694722521[266] = 1;
   out_5501644288694722521[267] = 0;
   out_5501644288694722521[268] = 0;
   out_5501644288694722521[269] = 0;
   out_5501644288694722521[270] = 0;
   out_5501644288694722521[271] = 0;
   out_5501644288694722521[272] = 0;
   out_5501644288694722521[273] = 0;
   out_5501644288694722521[274] = 0;
   out_5501644288694722521[275] = 0;
   out_5501644288694722521[276] = 0;
   out_5501644288694722521[277] = 0;
   out_5501644288694722521[278] = 0;
   out_5501644288694722521[279] = 0;
   out_5501644288694722521[280] = 0;
   out_5501644288694722521[281] = 0;
   out_5501644288694722521[282] = 0;
   out_5501644288694722521[283] = 0;
   out_5501644288694722521[284] = 0;
   out_5501644288694722521[285] = 1;
   out_5501644288694722521[286] = 0;
   out_5501644288694722521[287] = 0;
   out_5501644288694722521[288] = 0;
   out_5501644288694722521[289] = 0;
   out_5501644288694722521[290] = 0;
   out_5501644288694722521[291] = 0;
   out_5501644288694722521[292] = 0;
   out_5501644288694722521[293] = 0;
   out_5501644288694722521[294] = 0;
   out_5501644288694722521[295] = 0;
   out_5501644288694722521[296] = 0;
   out_5501644288694722521[297] = 0;
   out_5501644288694722521[298] = 0;
   out_5501644288694722521[299] = 0;
   out_5501644288694722521[300] = 0;
   out_5501644288694722521[301] = 0;
   out_5501644288694722521[302] = 0;
   out_5501644288694722521[303] = 0;
   out_5501644288694722521[304] = 1;
   out_5501644288694722521[305] = 0;
   out_5501644288694722521[306] = 0;
   out_5501644288694722521[307] = 0;
   out_5501644288694722521[308] = 0;
   out_5501644288694722521[309] = 0;
   out_5501644288694722521[310] = 0;
   out_5501644288694722521[311] = 0;
   out_5501644288694722521[312] = 0;
   out_5501644288694722521[313] = 0;
   out_5501644288694722521[314] = 0;
   out_5501644288694722521[315] = 0;
   out_5501644288694722521[316] = 0;
   out_5501644288694722521[317] = 0;
   out_5501644288694722521[318] = 0;
   out_5501644288694722521[319] = 0;
   out_5501644288694722521[320] = 0;
   out_5501644288694722521[321] = 0;
   out_5501644288694722521[322] = 0;
   out_5501644288694722521[323] = 1;
}
void h_4(double *state, double *unused, double *out_2917410844925153063) {
   out_2917410844925153063[0] = state[6] + state[9];
   out_2917410844925153063[1] = state[7] + state[10];
   out_2917410844925153063[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7261178375253156523) {
   out_7261178375253156523[0] = 0;
   out_7261178375253156523[1] = 0;
   out_7261178375253156523[2] = 0;
   out_7261178375253156523[3] = 0;
   out_7261178375253156523[4] = 0;
   out_7261178375253156523[5] = 0;
   out_7261178375253156523[6] = 1;
   out_7261178375253156523[7] = 0;
   out_7261178375253156523[8] = 0;
   out_7261178375253156523[9] = 1;
   out_7261178375253156523[10] = 0;
   out_7261178375253156523[11] = 0;
   out_7261178375253156523[12] = 0;
   out_7261178375253156523[13] = 0;
   out_7261178375253156523[14] = 0;
   out_7261178375253156523[15] = 0;
   out_7261178375253156523[16] = 0;
   out_7261178375253156523[17] = 0;
   out_7261178375253156523[18] = 0;
   out_7261178375253156523[19] = 0;
   out_7261178375253156523[20] = 0;
   out_7261178375253156523[21] = 0;
   out_7261178375253156523[22] = 0;
   out_7261178375253156523[23] = 0;
   out_7261178375253156523[24] = 0;
   out_7261178375253156523[25] = 1;
   out_7261178375253156523[26] = 0;
   out_7261178375253156523[27] = 0;
   out_7261178375253156523[28] = 1;
   out_7261178375253156523[29] = 0;
   out_7261178375253156523[30] = 0;
   out_7261178375253156523[31] = 0;
   out_7261178375253156523[32] = 0;
   out_7261178375253156523[33] = 0;
   out_7261178375253156523[34] = 0;
   out_7261178375253156523[35] = 0;
   out_7261178375253156523[36] = 0;
   out_7261178375253156523[37] = 0;
   out_7261178375253156523[38] = 0;
   out_7261178375253156523[39] = 0;
   out_7261178375253156523[40] = 0;
   out_7261178375253156523[41] = 0;
   out_7261178375253156523[42] = 0;
   out_7261178375253156523[43] = 0;
   out_7261178375253156523[44] = 1;
   out_7261178375253156523[45] = 0;
   out_7261178375253156523[46] = 0;
   out_7261178375253156523[47] = 1;
   out_7261178375253156523[48] = 0;
   out_7261178375253156523[49] = 0;
   out_7261178375253156523[50] = 0;
   out_7261178375253156523[51] = 0;
   out_7261178375253156523[52] = 0;
   out_7261178375253156523[53] = 0;
}
void h_10(double *state, double *unused, double *out_2425004257133495675) {
   out_2425004257133495675[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2425004257133495675[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2425004257133495675[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_735877534270755889) {
   out_735877534270755889[0] = 0;
   out_735877534270755889[1] = 9.8100000000000005*cos(state[1]);
   out_735877534270755889[2] = 0;
   out_735877534270755889[3] = 0;
   out_735877534270755889[4] = -state[8];
   out_735877534270755889[5] = state[7];
   out_735877534270755889[6] = 0;
   out_735877534270755889[7] = state[5];
   out_735877534270755889[8] = -state[4];
   out_735877534270755889[9] = 0;
   out_735877534270755889[10] = 0;
   out_735877534270755889[11] = 0;
   out_735877534270755889[12] = 1;
   out_735877534270755889[13] = 0;
   out_735877534270755889[14] = 0;
   out_735877534270755889[15] = 1;
   out_735877534270755889[16] = 0;
   out_735877534270755889[17] = 0;
   out_735877534270755889[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_735877534270755889[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_735877534270755889[20] = 0;
   out_735877534270755889[21] = state[8];
   out_735877534270755889[22] = 0;
   out_735877534270755889[23] = -state[6];
   out_735877534270755889[24] = -state[5];
   out_735877534270755889[25] = 0;
   out_735877534270755889[26] = state[3];
   out_735877534270755889[27] = 0;
   out_735877534270755889[28] = 0;
   out_735877534270755889[29] = 0;
   out_735877534270755889[30] = 0;
   out_735877534270755889[31] = 1;
   out_735877534270755889[32] = 0;
   out_735877534270755889[33] = 0;
   out_735877534270755889[34] = 1;
   out_735877534270755889[35] = 0;
   out_735877534270755889[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_735877534270755889[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_735877534270755889[38] = 0;
   out_735877534270755889[39] = -state[7];
   out_735877534270755889[40] = state[6];
   out_735877534270755889[41] = 0;
   out_735877534270755889[42] = state[4];
   out_735877534270755889[43] = -state[3];
   out_735877534270755889[44] = 0;
   out_735877534270755889[45] = 0;
   out_735877534270755889[46] = 0;
   out_735877534270755889[47] = 0;
   out_735877534270755889[48] = 0;
   out_735877534270755889[49] = 0;
   out_735877534270755889[50] = 1;
   out_735877534270755889[51] = 0;
   out_735877534270755889[52] = 0;
   out_735877534270755889[53] = 1;
}
void h_13(double *state, double *unused, double *out_2858741791526306440) {
   out_2858741791526306440[0] = state[3];
   out_2858741791526306440[1] = state[4];
   out_2858741791526306440[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4048904549920823722) {
   out_4048904549920823722[0] = 0;
   out_4048904549920823722[1] = 0;
   out_4048904549920823722[2] = 0;
   out_4048904549920823722[3] = 1;
   out_4048904549920823722[4] = 0;
   out_4048904549920823722[5] = 0;
   out_4048904549920823722[6] = 0;
   out_4048904549920823722[7] = 0;
   out_4048904549920823722[8] = 0;
   out_4048904549920823722[9] = 0;
   out_4048904549920823722[10] = 0;
   out_4048904549920823722[11] = 0;
   out_4048904549920823722[12] = 0;
   out_4048904549920823722[13] = 0;
   out_4048904549920823722[14] = 0;
   out_4048904549920823722[15] = 0;
   out_4048904549920823722[16] = 0;
   out_4048904549920823722[17] = 0;
   out_4048904549920823722[18] = 0;
   out_4048904549920823722[19] = 0;
   out_4048904549920823722[20] = 0;
   out_4048904549920823722[21] = 0;
   out_4048904549920823722[22] = 1;
   out_4048904549920823722[23] = 0;
   out_4048904549920823722[24] = 0;
   out_4048904549920823722[25] = 0;
   out_4048904549920823722[26] = 0;
   out_4048904549920823722[27] = 0;
   out_4048904549920823722[28] = 0;
   out_4048904549920823722[29] = 0;
   out_4048904549920823722[30] = 0;
   out_4048904549920823722[31] = 0;
   out_4048904549920823722[32] = 0;
   out_4048904549920823722[33] = 0;
   out_4048904549920823722[34] = 0;
   out_4048904549920823722[35] = 0;
   out_4048904549920823722[36] = 0;
   out_4048904549920823722[37] = 0;
   out_4048904549920823722[38] = 0;
   out_4048904549920823722[39] = 0;
   out_4048904549920823722[40] = 0;
   out_4048904549920823722[41] = 1;
   out_4048904549920823722[42] = 0;
   out_4048904549920823722[43] = 0;
   out_4048904549920823722[44] = 0;
   out_4048904549920823722[45] = 0;
   out_4048904549920823722[46] = 0;
   out_4048904549920823722[47] = 0;
   out_4048904549920823722[48] = 0;
   out_4048904549920823722[49] = 0;
   out_4048904549920823722[50] = 0;
   out_4048904549920823722[51] = 0;
   out_4048904549920823722[52] = 0;
   out_4048904549920823722[53] = 0;
}
void h_14(double *state, double *unused, double *out_7352000875437629003) {
   out_7352000875437629003[0] = state[6];
   out_7352000875437629003[1] = state[7];
   out_7352000875437629003[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8102777266161022797) {
   out_8102777266161022797[0] = 0;
   out_8102777266161022797[1] = 0;
   out_8102777266161022797[2] = 0;
   out_8102777266161022797[3] = 0;
   out_8102777266161022797[4] = 0;
   out_8102777266161022797[5] = 0;
   out_8102777266161022797[6] = 1;
   out_8102777266161022797[7] = 0;
   out_8102777266161022797[8] = 0;
   out_8102777266161022797[9] = 0;
   out_8102777266161022797[10] = 0;
   out_8102777266161022797[11] = 0;
   out_8102777266161022797[12] = 0;
   out_8102777266161022797[13] = 0;
   out_8102777266161022797[14] = 0;
   out_8102777266161022797[15] = 0;
   out_8102777266161022797[16] = 0;
   out_8102777266161022797[17] = 0;
   out_8102777266161022797[18] = 0;
   out_8102777266161022797[19] = 0;
   out_8102777266161022797[20] = 0;
   out_8102777266161022797[21] = 0;
   out_8102777266161022797[22] = 0;
   out_8102777266161022797[23] = 0;
   out_8102777266161022797[24] = 0;
   out_8102777266161022797[25] = 1;
   out_8102777266161022797[26] = 0;
   out_8102777266161022797[27] = 0;
   out_8102777266161022797[28] = 0;
   out_8102777266161022797[29] = 0;
   out_8102777266161022797[30] = 0;
   out_8102777266161022797[31] = 0;
   out_8102777266161022797[32] = 0;
   out_8102777266161022797[33] = 0;
   out_8102777266161022797[34] = 0;
   out_8102777266161022797[35] = 0;
   out_8102777266161022797[36] = 0;
   out_8102777266161022797[37] = 0;
   out_8102777266161022797[38] = 0;
   out_8102777266161022797[39] = 0;
   out_8102777266161022797[40] = 0;
   out_8102777266161022797[41] = 0;
   out_8102777266161022797[42] = 0;
   out_8102777266161022797[43] = 0;
   out_8102777266161022797[44] = 1;
   out_8102777266161022797[45] = 0;
   out_8102777266161022797[46] = 0;
   out_8102777266161022797[47] = 0;
   out_8102777266161022797[48] = 0;
   out_8102777266161022797[49] = 0;
   out_8102777266161022797[50] = 0;
   out_8102777266161022797[51] = 0;
   out_8102777266161022797[52] = 0;
   out_8102777266161022797[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_9033090375826759827) {
  err_fun(nom_x, delta_x, out_9033090375826759827);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9158915157634621903) {
  inv_err_fun(nom_x, true_x, out_9158915157634621903);
}
void pose_H_mod_fun(double *state, double *out_7413758488612487521) {
  H_mod_fun(state, out_7413758488612487521);
}
void pose_f_fun(double *state, double dt, double *out_3232314017905079538) {
  f_fun(state,  dt, out_3232314017905079538);
}
void pose_F_fun(double *state, double dt, double *out_5501644288694722521) {
  F_fun(state,  dt, out_5501644288694722521);
}
void pose_h_4(double *state, double *unused, double *out_2917410844925153063) {
  h_4(state, unused, out_2917410844925153063);
}
void pose_H_4(double *state, double *unused, double *out_7261178375253156523) {
  H_4(state, unused, out_7261178375253156523);
}
void pose_h_10(double *state, double *unused, double *out_2425004257133495675) {
  h_10(state, unused, out_2425004257133495675);
}
void pose_H_10(double *state, double *unused, double *out_735877534270755889) {
  H_10(state, unused, out_735877534270755889);
}
void pose_h_13(double *state, double *unused, double *out_2858741791526306440) {
  h_13(state, unused, out_2858741791526306440);
}
void pose_H_13(double *state, double *unused, double *out_4048904549920823722) {
  H_13(state, unused, out_4048904549920823722);
}
void pose_h_14(double *state, double *unused, double *out_7352000875437629003) {
  h_14(state, unused, out_7352000875437629003);
}
void pose_H_14(double *state, double *unused, double *out_8102777266161022797) {
  H_14(state, unused, out_8102777266161022797);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
