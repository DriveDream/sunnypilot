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
void err_fun(double *nom_x, double *delta_x, double *out_552322323456612592) {
   out_552322323456612592[0] = delta_x[0] + nom_x[0];
   out_552322323456612592[1] = delta_x[1] + nom_x[1];
   out_552322323456612592[2] = delta_x[2] + nom_x[2];
   out_552322323456612592[3] = delta_x[3] + nom_x[3];
   out_552322323456612592[4] = delta_x[4] + nom_x[4];
   out_552322323456612592[5] = delta_x[5] + nom_x[5];
   out_552322323456612592[6] = delta_x[6] + nom_x[6];
   out_552322323456612592[7] = delta_x[7] + nom_x[7];
   out_552322323456612592[8] = delta_x[8] + nom_x[8];
   out_552322323456612592[9] = delta_x[9] + nom_x[9];
   out_552322323456612592[10] = delta_x[10] + nom_x[10];
   out_552322323456612592[11] = delta_x[11] + nom_x[11];
   out_552322323456612592[12] = delta_x[12] + nom_x[12];
   out_552322323456612592[13] = delta_x[13] + nom_x[13];
   out_552322323456612592[14] = delta_x[14] + nom_x[14];
   out_552322323456612592[15] = delta_x[15] + nom_x[15];
   out_552322323456612592[16] = delta_x[16] + nom_x[16];
   out_552322323456612592[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4941387461979504541) {
   out_4941387461979504541[0] = -nom_x[0] + true_x[0];
   out_4941387461979504541[1] = -nom_x[1] + true_x[1];
   out_4941387461979504541[2] = -nom_x[2] + true_x[2];
   out_4941387461979504541[3] = -nom_x[3] + true_x[3];
   out_4941387461979504541[4] = -nom_x[4] + true_x[4];
   out_4941387461979504541[5] = -nom_x[5] + true_x[5];
   out_4941387461979504541[6] = -nom_x[6] + true_x[6];
   out_4941387461979504541[7] = -nom_x[7] + true_x[7];
   out_4941387461979504541[8] = -nom_x[8] + true_x[8];
   out_4941387461979504541[9] = -nom_x[9] + true_x[9];
   out_4941387461979504541[10] = -nom_x[10] + true_x[10];
   out_4941387461979504541[11] = -nom_x[11] + true_x[11];
   out_4941387461979504541[12] = -nom_x[12] + true_x[12];
   out_4941387461979504541[13] = -nom_x[13] + true_x[13];
   out_4941387461979504541[14] = -nom_x[14] + true_x[14];
   out_4941387461979504541[15] = -nom_x[15] + true_x[15];
   out_4941387461979504541[16] = -nom_x[16] + true_x[16];
   out_4941387461979504541[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1196212354931887211) {
   out_1196212354931887211[0] = 1.0;
   out_1196212354931887211[1] = 0.0;
   out_1196212354931887211[2] = 0.0;
   out_1196212354931887211[3] = 0.0;
   out_1196212354931887211[4] = 0.0;
   out_1196212354931887211[5] = 0.0;
   out_1196212354931887211[6] = 0.0;
   out_1196212354931887211[7] = 0.0;
   out_1196212354931887211[8] = 0.0;
   out_1196212354931887211[9] = 0.0;
   out_1196212354931887211[10] = 0.0;
   out_1196212354931887211[11] = 0.0;
   out_1196212354931887211[12] = 0.0;
   out_1196212354931887211[13] = 0.0;
   out_1196212354931887211[14] = 0.0;
   out_1196212354931887211[15] = 0.0;
   out_1196212354931887211[16] = 0.0;
   out_1196212354931887211[17] = 0.0;
   out_1196212354931887211[18] = 0.0;
   out_1196212354931887211[19] = 1.0;
   out_1196212354931887211[20] = 0.0;
   out_1196212354931887211[21] = 0.0;
   out_1196212354931887211[22] = 0.0;
   out_1196212354931887211[23] = 0.0;
   out_1196212354931887211[24] = 0.0;
   out_1196212354931887211[25] = 0.0;
   out_1196212354931887211[26] = 0.0;
   out_1196212354931887211[27] = 0.0;
   out_1196212354931887211[28] = 0.0;
   out_1196212354931887211[29] = 0.0;
   out_1196212354931887211[30] = 0.0;
   out_1196212354931887211[31] = 0.0;
   out_1196212354931887211[32] = 0.0;
   out_1196212354931887211[33] = 0.0;
   out_1196212354931887211[34] = 0.0;
   out_1196212354931887211[35] = 0.0;
   out_1196212354931887211[36] = 0.0;
   out_1196212354931887211[37] = 0.0;
   out_1196212354931887211[38] = 1.0;
   out_1196212354931887211[39] = 0.0;
   out_1196212354931887211[40] = 0.0;
   out_1196212354931887211[41] = 0.0;
   out_1196212354931887211[42] = 0.0;
   out_1196212354931887211[43] = 0.0;
   out_1196212354931887211[44] = 0.0;
   out_1196212354931887211[45] = 0.0;
   out_1196212354931887211[46] = 0.0;
   out_1196212354931887211[47] = 0.0;
   out_1196212354931887211[48] = 0.0;
   out_1196212354931887211[49] = 0.0;
   out_1196212354931887211[50] = 0.0;
   out_1196212354931887211[51] = 0.0;
   out_1196212354931887211[52] = 0.0;
   out_1196212354931887211[53] = 0.0;
   out_1196212354931887211[54] = 0.0;
   out_1196212354931887211[55] = 0.0;
   out_1196212354931887211[56] = 0.0;
   out_1196212354931887211[57] = 1.0;
   out_1196212354931887211[58] = 0.0;
   out_1196212354931887211[59] = 0.0;
   out_1196212354931887211[60] = 0.0;
   out_1196212354931887211[61] = 0.0;
   out_1196212354931887211[62] = 0.0;
   out_1196212354931887211[63] = 0.0;
   out_1196212354931887211[64] = 0.0;
   out_1196212354931887211[65] = 0.0;
   out_1196212354931887211[66] = 0.0;
   out_1196212354931887211[67] = 0.0;
   out_1196212354931887211[68] = 0.0;
   out_1196212354931887211[69] = 0.0;
   out_1196212354931887211[70] = 0.0;
   out_1196212354931887211[71] = 0.0;
   out_1196212354931887211[72] = 0.0;
   out_1196212354931887211[73] = 0.0;
   out_1196212354931887211[74] = 0.0;
   out_1196212354931887211[75] = 0.0;
   out_1196212354931887211[76] = 1.0;
   out_1196212354931887211[77] = 0.0;
   out_1196212354931887211[78] = 0.0;
   out_1196212354931887211[79] = 0.0;
   out_1196212354931887211[80] = 0.0;
   out_1196212354931887211[81] = 0.0;
   out_1196212354931887211[82] = 0.0;
   out_1196212354931887211[83] = 0.0;
   out_1196212354931887211[84] = 0.0;
   out_1196212354931887211[85] = 0.0;
   out_1196212354931887211[86] = 0.0;
   out_1196212354931887211[87] = 0.0;
   out_1196212354931887211[88] = 0.0;
   out_1196212354931887211[89] = 0.0;
   out_1196212354931887211[90] = 0.0;
   out_1196212354931887211[91] = 0.0;
   out_1196212354931887211[92] = 0.0;
   out_1196212354931887211[93] = 0.0;
   out_1196212354931887211[94] = 0.0;
   out_1196212354931887211[95] = 1.0;
   out_1196212354931887211[96] = 0.0;
   out_1196212354931887211[97] = 0.0;
   out_1196212354931887211[98] = 0.0;
   out_1196212354931887211[99] = 0.0;
   out_1196212354931887211[100] = 0.0;
   out_1196212354931887211[101] = 0.0;
   out_1196212354931887211[102] = 0.0;
   out_1196212354931887211[103] = 0.0;
   out_1196212354931887211[104] = 0.0;
   out_1196212354931887211[105] = 0.0;
   out_1196212354931887211[106] = 0.0;
   out_1196212354931887211[107] = 0.0;
   out_1196212354931887211[108] = 0.0;
   out_1196212354931887211[109] = 0.0;
   out_1196212354931887211[110] = 0.0;
   out_1196212354931887211[111] = 0.0;
   out_1196212354931887211[112] = 0.0;
   out_1196212354931887211[113] = 0.0;
   out_1196212354931887211[114] = 1.0;
   out_1196212354931887211[115] = 0.0;
   out_1196212354931887211[116] = 0.0;
   out_1196212354931887211[117] = 0.0;
   out_1196212354931887211[118] = 0.0;
   out_1196212354931887211[119] = 0.0;
   out_1196212354931887211[120] = 0.0;
   out_1196212354931887211[121] = 0.0;
   out_1196212354931887211[122] = 0.0;
   out_1196212354931887211[123] = 0.0;
   out_1196212354931887211[124] = 0.0;
   out_1196212354931887211[125] = 0.0;
   out_1196212354931887211[126] = 0.0;
   out_1196212354931887211[127] = 0.0;
   out_1196212354931887211[128] = 0.0;
   out_1196212354931887211[129] = 0.0;
   out_1196212354931887211[130] = 0.0;
   out_1196212354931887211[131] = 0.0;
   out_1196212354931887211[132] = 0.0;
   out_1196212354931887211[133] = 1.0;
   out_1196212354931887211[134] = 0.0;
   out_1196212354931887211[135] = 0.0;
   out_1196212354931887211[136] = 0.0;
   out_1196212354931887211[137] = 0.0;
   out_1196212354931887211[138] = 0.0;
   out_1196212354931887211[139] = 0.0;
   out_1196212354931887211[140] = 0.0;
   out_1196212354931887211[141] = 0.0;
   out_1196212354931887211[142] = 0.0;
   out_1196212354931887211[143] = 0.0;
   out_1196212354931887211[144] = 0.0;
   out_1196212354931887211[145] = 0.0;
   out_1196212354931887211[146] = 0.0;
   out_1196212354931887211[147] = 0.0;
   out_1196212354931887211[148] = 0.0;
   out_1196212354931887211[149] = 0.0;
   out_1196212354931887211[150] = 0.0;
   out_1196212354931887211[151] = 0.0;
   out_1196212354931887211[152] = 1.0;
   out_1196212354931887211[153] = 0.0;
   out_1196212354931887211[154] = 0.0;
   out_1196212354931887211[155] = 0.0;
   out_1196212354931887211[156] = 0.0;
   out_1196212354931887211[157] = 0.0;
   out_1196212354931887211[158] = 0.0;
   out_1196212354931887211[159] = 0.0;
   out_1196212354931887211[160] = 0.0;
   out_1196212354931887211[161] = 0.0;
   out_1196212354931887211[162] = 0.0;
   out_1196212354931887211[163] = 0.0;
   out_1196212354931887211[164] = 0.0;
   out_1196212354931887211[165] = 0.0;
   out_1196212354931887211[166] = 0.0;
   out_1196212354931887211[167] = 0.0;
   out_1196212354931887211[168] = 0.0;
   out_1196212354931887211[169] = 0.0;
   out_1196212354931887211[170] = 0.0;
   out_1196212354931887211[171] = 1.0;
   out_1196212354931887211[172] = 0.0;
   out_1196212354931887211[173] = 0.0;
   out_1196212354931887211[174] = 0.0;
   out_1196212354931887211[175] = 0.0;
   out_1196212354931887211[176] = 0.0;
   out_1196212354931887211[177] = 0.0;
   out_1196212354931887211[178] = 0.0;
   out_1196212354931887211[179] = 0.0;
   out_1196212354931887211[180] = 0.0;
   out_1196212354931887211[181] = 0.0;
   out_1196212354931887211[182] = 0.0;
   out_1196212354931887211[183] = 0.0;
   out_1196212354931887211[184] = 0.0;
   out_1196212354931887211[185] = 0.0;
   out_1196212354931887211[186] = 0.0;
   out_1196212354931887211[187] = 0.0;
   out_1196212354931887211[188] = 0.0;
   out_1196212354931887211[189] = 0.0;
   out_1196212354931887211[190] = 1.0;
   out_1196212354931887211[191] = 0.0;
   out_1196212354931887211[192] = 0.0;
   out_1196212354931887211[193] = 0.0;
   out_1196212354931887211[194] = 0.0;
   out_1196212354931887211[195] = 0.0;
   out_1196212354931887211[196] = 0.0;
   out_1196212354931887211[197] = 0.0;
   out_1196212354931887211[198] = 0.0;
   out_1196212354931887211[199] = 0.0;
   out_1196212354931887211[200] = 0.0;
   out_1196212354931887211[201] = 0.0;
   out_1196212354931887211[202] = 0.0;
   out_1196212354931887211[203] = 0.0;
   out_1196212354931887211[204] = 0.0;
   out_1196212354931887211[205] = 0.0;
   out_1196212354931887211[206] = 0.0;
   out_1196212354931887211[207] = 0.0;
   out_1196212354931887211[208] = 0.0;
   out_1196212354931887211[209] = 1.0;
   out_1196212354931887211[210] = 0.0;
   out_1196212354931887211[211] = 0.0;
   out_1196212354931887211[212] = 0.0;
   out_1196212354931887211[213] = 0.0;
   out_1196212354931887211[214] = 0.0;
   out_1196212354931887211[215] = 0.0;
   out_1196212354931887211[216] = 0.0;
   out_1196212354931887211[217] = 0.0;
   out_1196212354931887211[218] = 0.0;
   out_1196212354931887211[219] = 0.0;
   out_1196212354931887211[220] = 0.0;
   out_1196212354931887211[221] = 0.0;
   out_1196212354931887211[222] = 0.0;
   out_1196212354931887211[223] = 0.0;
   out_1196212354931887211[224] = 0.0;
   out_1196212354931887211[225] = 0.0;
   out_1196212354931887211[226] = 0.0;
   out_1196212354931887211[227] = 0.0;
   out_1196212354931887211[228] = 1.0;
   out_1196212354931887211[229] = 0.0;
   out_1196212354931887211[230] = 0.0;
   out_1196212354931887211[231] = 0.0;
   out_1196212354931887211[232] = 0.0;
   out_1196212354931887211[233] = 0.0;
   out_1196212354931887211[234] = 0.0;
   out_1196212354931887211[235] = 0.0;
   out_1196212354931887211[236] = 0.0;
   out_1196212354931887211[237] = 0.0;
   out_1196212354931887211[238] = 0.0;
   out_1196212354931887211[239] = 0.0;
   out_1196212354931887211[240] = 0.0;
   out_1196212354931887211[241] = 0.0;
   out_1196212354931887211[242] = 0.0;
   out_1196212354931887211[243] = 0.0;
   out_1196212354931887211[244] = 0.0;
   out_1196212354931887211[245] = 0.0;
   out_1196212354931887211[246] = 0.0;
   out_1196212354931887211[247] = 1.0;
   out_1196212354931887211[248] = 0.0;
   out_1196212354931887211[249] = 0.0;
   out_1196212354931887211[250] = 0.0;
   out_1196212354931887211[251] = 0.0;
   out_1196212354931887211[252] = 0.0;
   out_1196212354931887211[253] = 0.0;
   out_1196212354931887211[254] = 0.0;
   out_1196212354931887211[255] = 0.0;
   out_1196212354931887211[256] = 0.0;
   out_1196212354931887211[257] = 0.0;
   out_1196212354931887211[258] = 0.0;
   out_1196212354931887211[259] = 0.0;
   out_1196212354931887211[260] = 0.0;
   out_1196212354931887211[261] = 0.0;
   out_1196212354931887211[262] = 0.0;
   out_1196212354931887211[263] = 0.0;
   out_1196212354931887211[264] = 0.0;
   out_1196212354931887211[265] = 0.0;
   out_1196212354931887211[266] = 1.0;
   out_1196212354931887211[267] = 0.0;
   out_1196212354931887211[268] = 0.0;
   out_1196212354931887211[269] = 0.0;
   out_1196212354931887211[270] = 0.0;
   out_1196212354931887211[271] = 0.0;
   out_1196212354931887211[272] = 0.0;
   out_1196212354931887211[273] = 0.0;
   out_1196212354931887211[274] = 0.0;
   out_1196212354931887211[275] = 0.0;
   out_1196212354931887211[276] = 0.0;
   out_1196212354931887211[277] = 0.0;
   out_1196212354931887211[278] = 0.0;
   out_1196212354931887211[279] = 0.0;
   out_1196212354931887211[280] = 0.0;
   out_1196212354931887211[281] = 0.0;
   out_1196212354931887211[282] = 0.0;
   out_1196212354931887211[283] = 0.0;
   out_1196212354931887211[284] = 0.0;
   out_1196212354931887211[285] = 1.0;
   out_1196212354931887211[286] = 0.0;
   out_1196212354931887211[287] = 0.0;
   out_1196212354931887211[288] = 0.0;
   out_1196212354931887211[289] = 0.0;
   out_1196212354931887211[290] = 0.0;
   out_1196212354931887211[291] = 0.0;
   out_1196212354931887211[292] = 0.0;
   out_1196212354931887211[293] = 0.0;
   out_1196212354931887211[294] = 0.0;
   out_1196212354931887211[295] = 0.0;
   out_1196212354931887211[296] = 0.0;
   out_1196212354931887211[297] = 0.0;
   out_1196212354931887211[298] = 0.0;
   out_1196212354931887211[299] = 0.0;
   out_1196212354931887211[300] = 0.0;
   out_1196212354931887211[301] = 0.0;
   out_1196212354931887211[302] = 0.0;
   out_1196212354931887211[303] = 0.0;
   out_1196212354931887211[304] = 1.0;
   out_1196212354931887211[305] = 0.0;
   out_1196212354931887211[306] = 0.0;
   out_1196212354931887211[307] = 0.0;
   out_1196212354931887211[308] = 0.0;
   out_1196212354931887211[309] = 0.0;
   out_1196212354931887211[310] = 0.0;
   out_1196212354931887211[311] = 0.0;
   out_1196212354931887211[312] = 0.0;
   out_1196212354931887211[313] = 0.0;
   out_1196212354931887211[314] = 0.0;
   out_1196212354931887211[315] = 0.0;
   out_1196212354931887211[316] = 0.0;
   out_1196212354931887211[317] = 0.0;
   out_1196212354931887211[318] = 0.0;
   out_1196212354931887211[319] = 0.0;
   out_1196212354931887211[320] = 0.0;
   out_1196212354931887211[321] = 0.0;
   out_1196212354931887211[322] = 0.0;
   out_1196212354931887211[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8376554467190809622) {
   out_8376554467190809622[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8376554467190809622[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8376554467190809622[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8376554467190809622[3] = dt*state[12] + state[3];
   out_8376554467190809622[4] = dt*state[13] + state[4];
   out_8376554467190809622[5] = dt*state[14] + state[5];
   out_8376554467190809622[6] = state[6];
   out_8376554467190809622[7] = state[7];
   out_8376554467190809622[8] = state[8];
   out_8376554467190809622[9] = state[9];
   out_8376554467190809622[10] = state[10];
   out_8376554467190809622[11] = state[11];
   out_8376554467190809622[12] = state[12];
   out_8376554467190809622[13] = state[13];
   out_8376554467190809622[14] = state[14];
   out_8376554467190809622[15] = state[15];
   out_8376554467190809622[16] = state[16];
   out_8376554467190809622[17] = state[17];
}
void F_fun(double *state, double dt, double *out_9160386404678547461) {
   out_9160386404678547461[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9160386404678547461[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9160386404678547461[2] = 0;
   out_9160386404678547461[3] = 0;
   out_9160386404678547461[4] = 0;
   out_9160386404678547461[5] = 0;
   out_9160386404678547461[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9160386404678547461[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9160386404678547461[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_9160386404678547461[9] = 0;
   out_9160386404678547461[10] = 0;
   out_9160386404678547461[11] = 0;
   out_9160386404678547461[12] = 0;
   out_9160386404678547461[13] = 0;
   out_9160386404678547461[14] = 0;
   out_9160386404678547461[15] = 0;
   out_9160386404678547461[16] = 0;
   out_9160386404678547461[17] = 0;
   out_9160386404678547461[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9160386404678547461[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9160386404678547461[20] = 0;
   out_9160386404678547461[21] = 0;
   out_9160386404678547461[22] = 0;
   out_9160386404678547461[23] = 0;
   out_9160386404678547461[24] = 0;
   out_9160386404678547461[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9160386404678547461[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_9160386404678547461[27] = 0;
   out_9160386404678547461[28] = 0;
   out_9160386404678547461[29] = 0;
   out_9160386404678547461[30] = 0;
   out_9160386404678547461[31] = 0;
   out_9160386404678547461[32] = 0;
   out_9160386404678547461[33] = 0;
   out_9160386404678547461[34] = 0;
   out_9160386404678547461[35] = 0;
   out_9160386404678547461[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9160386404678547461[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9160386404678547461[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9160386404678547461[39] = 0;
   out_9160386404678547461[40] = 0;
   out_9160386404678547461[41] = 0;
   out_9160386404678547461[42] = 0;
   out_9160386404678547461[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9160386404678547461[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_9160386404678547461[45] = 0;
   out_9160386404678547461[46] = 0;
   out_9160386404678547461[47] = 0;
   out_9160386404678547461[48] = 0;
   out_9160386404678547461[49] = 0;
   out_9160386404678547461[50] = 0;
   out_9160386404678547461[51] = 0;
   out_9160386404678547461[52] = 0;
   out_9160386404678547461[53] = 0;
   out_9160386404678547461[54] = 0;
   out_9160386404678547461[55] = 0;
   out_9160386404678547461[56] = 0;
   out_9160386404678547461[57] = 1;
   out_9160386404678547461[58] = 0;
   out_9160386404678547461[59] = 0;
   out_9160386404678547461[60] = 0;
   out_9160386404678547461[61] = 0;
   out_9160386404678547461[62] = 0;
   out_9160386404678547461[63] = 0;
   out_9160386404678547461[64] = 0;
   out_9160386404678547461[65] = 0;
   out_9160386404678547461[66] = dt;
   out_9160386404678547461[67] = 0;
   out_9160386404678547461[68] = 0;
   out_9160386404678547461[69] = 0;
   out_9160386404678547461[70] = 0;
   out_9160386404678547461[71] = 0;
   out_9160386404678547461[72] = 0;
   out_9160386404678547461[73] = 0;
   out_9160386404678547461[74] = 0;
   out_9160386404678547461[75] = 0;
   out_9160386404678547461[76] = 1;
   out_9160386404678547461[77] = 0;
   out_9160386404678547461[78] = 0;
   out_9160386404678547461[79] = 0;
   out_9160386404678547461[80] = 0;
   out_9160386404678547461[81] = 0;
   out_9160386404678547461[82] = 0;
   out_9160386404678547461[83] = 0;
   out_9160386404678547461[84] = 0;
   out_9160386404678547461[85] = dt;
   out_9160386404678547461[86] = 0;
   out_9160386404678547461[87] = 0;
   out_9160386404678547461[88] = 0;
   out_9160386404678547461[89] = 0;
   out_9160386404678547461[90] = 0;
   out_9160386404678547461[91] = 0;
   out_9160386404678547461[92] = 0;
   out_9160386404678547461[93] = 0;
   out_9160386404678547461[94] = 0;
   out_9160386404678547461[95] = 1;
   out_9160386404678547461[96] = 0;
   out_9160386404678547461[97] = 0;
   out_9160386404678547461[98] = 0;
   out_9160386404678547461[99] = 0;
   out_9160386404678547461[100] = 0;
   out_9160386404678547461[101] = 0;
   out_9160386404678547461[102] = 0;
   out_9160386404678547461[103] = 0;
   out_9160386404678547461[104] = dt;
   out_9160386404678547461[105] = 0;
   out_9160386404678547461[106] = 0;
   out_9160386404678547461[107] = 0;
   out_9160386404678547461[108] = 0;
   out_9160386404678547461[109] = 0;
   out_9160386404678547461[110] = 0;
   out_9160386404678547461[111] = 0;
   out_9160386404678547461[112] = 0;
   out_9160386404678547461[113] = 0;
   out_9160386404678547461[114] = 1;
   out_9160386404678547461[115] = 0;
   out_9160386404678547461[116] = 0;
   out_9160386404678547461[117] = 0;
   out_9160386404678547461[118] = 0;
   out_9160386404678547461[119] = 0;
   out_9160386404678547461[120] = 0;
   out_9160386404678547461[121] = 0;
   out_9160386404678547461[122] = 0;
   out_9160386404678547461[123] = 0;
   out_9160386404678547461[124] = 0;
   out_9160386404678547461[125] = 0;
   out_9160386404678547461[126] = 0;
   out_9160386404678547461[127] = 0;
   out_9160386404678547461[128] = 0;
   out_9160386404678547461[129] = 0;
   out_9160386404678547461[130] = 0;
   out_9160386404678547461[131] = 0;
   out_9160386404678547461[132] = 0;
   out_9160386404678547461[133] = 1;
   out_9160386404678547461[134] = 0;
   out_9160386404678547461[135] = 0;
   out_9160386404678547461[136] = 0;
   out_9160386404678547461[137] = 0;
   out_9160386404678547461[138] = 0;
   out_9160386404678547461[139] = 0;
   out_9160386404678547461[140] = 0;
   out_9160386404678547461[141] = 0;
   out_9160386404678547461[142] = 0;
   out_9160386404678547461[143] = 0;
   out_9160386404678547461[144] = 0;
   out_9160386404678547461[145] = 0;
   out_9160386404678547461[146] = 0;
   out_9160386404678547461[147] = 0;
   out_9160386404678547461[148] = 0;
   out_9160386404678547461[149] = 0;
   out_9160386404678547461[150] = 0;
   out_9160386404678547461[151] = 0;
   out_9160386404678547461[152] = 1;
   out_9160386404678547461[153] = 0;
   out_9160386404678547461[154] = 0;
   out_9160386404678547461[155] = 0;
   out_9160386404678547461[156] = 0;
   out_9160386404678547461[157] = 0;
   out_9160386404678547461[158] = 0;
   out_9160386404678547461[159] = 0;
   out_9160386404678547461[160] = 0;
   out_9160386404678547461[161] = 0;
   out_9160386404678547461[162] = 0;
   out_9160386404678547461[163] = 0;
   out_9160386404678547461[164] = 0;
   out_9160386404678547461[165] = 0;
   out_9160386404678547461[166] = 0;
   out_9160386404678547461[167] = 0;
   out_9160386404678547461[168] = 0;
   out_9160386404678547461[169] = 0;
   out_9160386404678547461[170] = 0;
   out_9160386404678547461[171] = 1;
   out_9160386404678547461[172] = 0;
   out_9160386404678547461[173] = 0;
   out_9160386404678547461[174] = 0;
   out_9160386404678547461[175] = 0;
   out_9160386404678547461[176] = 0;
   out_9160386404678547461[177] = 0;
   out_9160386404678547461[178] = 0;
   out_9160386404678547461[179] = 0;
   out_9160386404678547461[180] = 0;
   out_9160386404678547461[181] = 0;
   out_9160386404678547461[182] = 0;
   out_9160386404678547461[183] = 0;
   out_9160386404678547461[184] = 0;
   out_9160386404678547461[185] = 0;
   out_9160386404678547461[186] = 0;
   out_9160386404678547461[187] = 0;
   out_9160386404678547461[188] = 0;
   out_9160386404678547461[189] = 0;
   out_9160386404678547461[190] = 1;
   out_9160386404678547461[191] = 0;
   out_9160386404678547461[192] = 0;
   out_9160386404678547461[193] = 0;
   out_9160386404678547461[194] = 0;
   out_9160386404678547461[195] = 0;
   out_9160386404678547461[196] = 0;
   out_9160386404678547461[197] = 0;
   out_9160386404678547461[198] = 0;
   out_9160386404678547461[199] = 0;
   out_9160386404678547461[200] = 0;
   out_9160386404678547461[201] = 0;
   out_9160386404678547461[202] = 0;
   out_9160386404678547461[203] = 0;
   out_9160386404678547461[204] = 0;
   out_9160386404678547461[205] = 0;
   out_9160386404678547461[206] = 0;
   out_9160386404678547461[207] = 0;
   out_9160386404678547461[208] = 0;
   out_9160386404678547461[209] = 1;
   out_9160386404678547461[210] = 0;
   out_9160386404678547461[211] = 0;
   out_9160386404678547461[212] = 0;
   out_9160386404678547461[213] = 0;
   out_9160386404678547461[214] = 0;
   out_9160386404678547461[215] = 0;
   out_9160386404678547461[216] = 0;
   out_9160386404678547461[217] = 0;
   out_9160386404678547461[218] = 0;
   out_9160386404678547461[219] = 0;
   out_9160386404678547461[220] = 0;
   out_9160386404678547461[221] = 0;
   out_9160386404678547461[222] = 0;
   out_9160386404678547461[223] = 0;
   out_9160386404678547461[224] = 0;
   out_9160386404678547461[225] = 0;
   out_9160386404678547461[226] = 0;
   out_9160386404678547461[227] = 0;
   out_9160386404678547461[228] = 1;
   out_9160386404678547461[229] = 0;
   out_9160386404678547461[230] = 0;
   out_9160386404678547461[231] = 0;
   out_9160386404678547461[232] = 0;
   out_9160386404678547461[233] = 0;
   out_9160386404678547461[234] = 0;
   out_9160386404678547461[235] = 0;
   out_9160386404678547461[236] = 0;
   out_9160386404678547461[237] = 0;
   out_9160386404678547461[238] = 0;
   out_9160386404678547461[239] = 0;
   out_9160386404678547461[240] = 0;
   out_9160386404678547461[241] = 0;
   out_9160386404678547461[242] = 0;
   out_9160386404678547461[243] = 0;
   out_9160386404678547461[244] = 0;
   out_9160386404678547461[245] = 0;
   out_9160386404678547461[246] = 0;
   out_9160386404678547461[247] = 1;
   out_9160386404678547461[248] = 0;
   out_9160386404678547461[249] = 0;
   out_9160386404678547461[250] = 0;
   out_9160386404678547461[251] = 0;
   out_9160386404678547461[252] = 0;
   out_9160386404678547461[253] = 0;
   out_9160386404678547461[254] = 0;
   out_9160386404678547461[255] = 0;
   out_9160386404678547461[256] = 0;
   out_9160386404678547461[257] = 0;
   out_9160386404678547461[258] = 0;
   out_9160386404678547461[259] = 0;
   out_9160386404678547461[260] = 0;
   out_9160386404678547461[261] = 0;
   out_9160386404678547461[262] = 0;
   out_9160386404678547461[263] = 0;
   out_9160386404678547461[264] = 0;
   out_9160386404678547461[265] = 0;
   out_9160386404678547461[266] = 1;
   out_9160386404678547461[267] = 0;
   out_9160386404678547461[268] = 0;
   out_9160386404678547461[269] = 0;
   out_9160386404678547461[270] = 0;
   out_9160386404678547461[271] = 0;
   out_9160386404678547461[272] = 0;
   out_9160386404678547461[273] = 0;
   out_9160386404678547461[274] = 0;
   out_9160386404678547461[275] = 0;
   out_9160386404678547461[276] = 0;
   out_9160386404678547461[277] = 0;
   out_9160386404678547461[278] = 0;
   out_9160386404678547461[279] = 0;
   out_9160386404678547461[280] = 0;
   out_9160386404678547461[281] = 0;
   out_9160386404678547461[282] = 0;
   out_9160386404678547461[283] = 0;
   out_9160386404678547461[284] = 0;
   out_9160386404678547461[285] = 1;
   out_9160386404678547461[286] = 0;
   out_9160386404678547461[287] = 0;
   out_9160386404678547461[288] = 0;
   out_9160386404678547461[289] = 0;
   out_9160386404678547461[290] = 0;
   out_9160386404678547461[291] = 0;
   out_9160386404678547461[292] = 0;
   out_9160386404678547461[293] = 0;
   out_9160386404678547461[294] = 0;
   out_9160386404678547461[295] = 0;
   out_9160386404678547461[296] = 0;
   out_9160386404678547461[297] = 0;
   out_9160386404678547461[298] = 0;
   out_9160386404678547461[299] = 0;
   out_9160386404678547461[300] = 0;
   out_9160386404678547461[301] = 0;
   out_9160386404678547461[302] = 0;
   out_9160386404678547461[303] = 0;
   out_9160386404678547461[304] = 1;
   out_9160386404678547461[305] = 0;
   out_9160386404678547461[306] = 0;
   out_9160386404678547461[307] = 0;
   out_9160386404678547461[308] = 0;
   out_9160386404678547461[309] = 0;
   out_9160386404678547461[310] = 0;
   out_9160386404678547461[311] = 0;
   out_9160386404678547461[312] = 0;
   out_9160386404678547461[313] = 0;
   out_9160386404678547461[314] = 0;
   out_9160386404678547461[315] = 0;
   out_9160386404678547461[316] = 0;
   out_9160386404678547461[317] = 0;
   out_9160386404678547461[318] = 0;
   out_9160386404678547461[319] = 0;
   out_9160386404678547461[320] = 0;
   out_9160386404678547461[321] = 0;
   out_9160386404678547461[322] = 0;
   out_9160386404678547461[323] = 1;
}
void h_4(double *state, double *unused, double *out_7457733264180481157) {
   out_7457733264180481157[0] = state[6] + state[9];
   out_7457733264180481157[1] = state[7] + state[10];
   out_7457733264180481157[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_442051352877174604) {
   out_442051352877174604[0] = 0;
   out_442051352877174604[1] = 0;
   out_442051352877174604[2] = 0;
   out_442051352877174604[3] = 0;
   out_442051352877174604[4] = 0;
   out_442051352877174604[5] = 0;
   out_442051352877174604[6] = 1;
   out_442051352877174604[7] = 0;
   out_442051352877174604[8] = 0;
   out_442051352877174604[9] = 1;
   out_442051352877174604[10] = 0;
   out_442051352877174604[11] = 0;
   out_442051352877174604[12] = 0;
   out_442051352877174604[13] = 0;
   out_442051352877174604[14] = 0;
   out_442051352877174604[15] = 0;
   out_442051352877174604[16] = 0;
   out_442051352877174604[17] = 0;
   out_442051352877174604[18] = 0;
   out_442051352877174604[19] = 0;
   out_442051352877174604[20] = 0;
   out_442051352877174604[21] = 0;
   out_442051352877174604[22] = 0;
   out_442051352877174604[23] = 0;
   out_442051352877174604[24] = 0;
   out_442051352877174604[25] = 1;
   out_442051352877174604[26] = 0;
   out_442051352877174604[27] = 0;
   out_442051352877174604[28] = 1;
   out_442051352877174604[29] = 0;
   out_442051352877174604[30] = 0;
   out_442051352877174604[31] = 0;
   out_442051352877174604[32] = 0;
   out_442051352877174604[33] = 0;
   out_442051352877174604[34] = 0;
   out_442051352877174604[35] = 0;
   out_442051352877174604[36] = 0;
   out_442051352877174604[37] = 0;
   out_442051352877174604[38] = 0;
   out_442051352877174604[39] = 0;
   out_442051352877174604[40] = 0;
   out_442051352877174604[41] = 0;
   out_442051352877174604[42] = 0;
   out_442051352877174604[43] = 0;
   out_442051352877174604[44] = 1;
   out_442051352877174604[45] = 0;
   out_442051352877174604[46] = 0;
   out_442051352877174604[47] = 1;
   out_442051352877174604[48] = 0;
   out_442051352877174604[49] = 0;
   out_442051352877174604[50] = 0;
   out_442051352877174604[51] = 0;
   out_442051352877174604[52] = 0;
   out_442051352877174604[53] = 0;
}
void h_10(double *state, double *unused, double *out_7956640001151253329) {
   out_7956640001151253329[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_7956640001151253329[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_7956640001151253329[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2661350816086051052) {
   out_2661350816086051052[0] = 0;
   out_2661350816086051052[1] = 9.8100000000000005*cos(state[1]);
   out_2661350816086051052[2] = 0;
   out_2661350816086051052[3] = 0;
   out_2661350816086051052[4] = -state[8];
   out_2661350816086051052[5] = state[7];
   out_2661350816086051052[6] = 0;
   out_2661350816086051052[7] = state[5];
   out_2661350816086051052[8] = -state[4];
   out_2661350816086051052[9] = 0;
   out_2661350816086051052[10] = 0;
   out_2661350816086051052[11] = 0;
   out_2661350816086051052[12] = 1;
   out_2661350816086051052[13] = 0;
   out_2661350816086051052[14] = 0;
   out_2661350816086051052[15] = 1;
   out_2661350816086051052[16] = 0;
   out_2661350816086051052[17] = 0;
   out_2661350816086051052[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2661350816086051052[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2661350816086051052[20] = 0;
   out_2661350816086051052[21] = state[8];
   out_2661350816086051052[22] = 0;
   out_2661350816086051052[23] = -state[6];
   out_2661350816086051052[24] = -state[5];
   out_2661350816086051052[25] = 0;
   out_2661350816086051052[26] = state[3];
   out_2661350816086051052[27] = 0;
   out_2661350816086051052[28] = 0;
   out_2661350816086051052[29] = 0;
   out_2661350816086051052[30] = 0;
   out_2661350816086051052[31] = 1;
   out_2661350816086051052[32] = 0;
   out_2661350816086051052[33] = 0;
   out_2661350816086051052[34] = 1;
   out_2661350816086051052[35] = 0;
   out_2661350816086051052[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2661350816086051052[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2661350816086051052[38] = 0;
   out_2661350816086051052[39] = -state[7];
   out_2661350816086051052[40] = state[6];
   out_2661350816086051052[41] = 0;
   out_2661350816086051052[42] = state[4];
   out_2661350816086051052[43] = -state[3];
   out_2661350816086051052[44] = 0;
   out_2661350816086051052[45] = 0;
   out_2661350816086051052[46] = 0;
   out_2661350816086051052[47] = 0;
   out_2661350816086051052[48] = 0;
   out_2661350816086051052[49] = 0;
   out_2661350816086051052[50] = 1;
   out_2661350816086051052[51] = 0;
   out_2661350816086051052[52] = 0;
   out_2661350816086051052[53] = 1;
}
void h_13(double *state, double *unused, double *out_5574642289715389555) {
   out_5574642289715389555[0] = state[3];
   out_5574642289715389555[1] = state[4];
   out_5574642289715389555[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7168579855439526325) {
   out_7168579855439526325[0] = 0;
   out_7168579855439526325[1] = 0;
   out_7168579855439526325[2] = 0;
   out_7168579855439526325[3] = 1;
   out_7168579855439526325[4] = 0;
   out_7168579855439526325[5] = 0;
   out_7168579855439526325[6] = 0;
   out_7168579855439526325[7] = 0;
   out_7168579855439526325[8] = 0;
   out_7168579855439526325[9] = 0;
   out_7168579855439526325[10] = 0;
   out_7168579855439526325[11] = 0;
   out_7168579855439526325[12] = 0;
   out_7168579855439526325[13] = 0;
   out_7168579855439526325[14] = 0;
   out_7168579855439526325[15] = 0;
   out_7168579855439526325[16] = 0;
   out_7168579855439526325[17] = 0;
   out_7168579855439526325[18] = 0;
   out_7168579855439526325[19] = 0;
   out_7168579855439526325[20] = 0;
   out_7168579855439526325[21] = 0;
   out_7168579855439526325[22] = 1;
   out_7168579855439526325[23] = 0;
   out_7168579855439526325[24] = 0;
   out_7168579855439526325[25] = 0;
   out_7168579855439526325[26] = 0;
   out_7168579855439526325[27] = 0;
   out_7168579855439526325[28] = 0;
   out_7168579855439526325[29] = 0;
   out_7168579855439526325[30] = 0;
   out_7168579855439526325[31] = 0;
   out_7168579855439526325[32] = 0;
   out_7168579855439526325[33] = 0;
   out_7168579855439526325[34] = 0;
   out_7168579855439526325[35] = 0;
   out_7168579855439526325[36] = 0;
   out_7168579855439526325[37] = 0;
   out_7168579855439526325[38] = 0;
   out_7168579855439526325[39] = 0;
   out_7168579855439526325[40] = 0;
   out_7168579855439526325[41] = 1;
   out_7168579855439526325[42] = 0;
   out_7168579855439526325[43] = 0;
   out_7168579855439526325[44] = 0;
   out_7168579855439526325[45] = 0;
   out_7168579855439526325[46] = 0;
   out_7168579855439526325[47] = 0;
   out_7168579855439526325[48] = 0;
   out_7168579855439526325[49] = 0;
   out_7168579855439526325[50] = 0;
   out_7168579855439526325[51] = 0;
   out_7168579855439526325[52] = 0;
   out_7168579855439526325[53] = 0;
}
void h_14(double *state, double *unused, double *out_3993226962874543760) {
   out_3993226962874543760[0] = state[6];
   out_3993226962874543760[1] = state[7];
   out_3993226962874543760[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3521189503462309925) {
   out_3521189503462309925[0] = 0;
   out_3521189503462309925[1] = 0;
   out_3521189503462309925[2] = 0;
   out_3521189503462309925[3] = 0;
   out_3521189503462309925[4] = 0;
   out_3521189503462309925[5] = 0;
   out_3521189503462309925[6] = 1;
   out_3521189503462309925[7] = 0;
   out_3521189503462309925[8] = 0;
   out_3521189503462309925[9] = 0;
   out_3521189503462309925[10] = 0;
   out_3521189503462309925[11] = 0;
   out_3521189503462309925[12] = 0;
   out_3521189503462309925[13] = 0;
   out_3521189503462309925[14] = 0;
   out_3521189503462309925[15] = 0;
   out_3521189503462309925[16] = 0;
   out_3521189503462309925[17] = 0;
   out_3521189503462309925[18] = 0;
   out_3521189503462309925[19] = 0;
   out_3521189503462309925[20] = 0;
   out_3521189503462309925[21] = 0;
   out_3521189503462309925[22] = 0;
   out_3521189503462309925[23] = 0;
   out_3521189503462309925[24] = 0;
   out_3521189503462309925[25] = 1;
   out_3521189503462309925[26] = 0;
   out_3521189503462309925[27] = 0;
   out_3521189503462309925[28] = 0;
   out_3521189503462309925[29] = 0;
   out_3521189503462309925[30] = 0;
   out_3521189503462309925[31] = 0;
   out_3521189503462309925[32] = 0;
   out_3521189503462309925[33] = 0;
   out_3521189503462309925[34] = 0;
   out_3521189503462309925[35] = 0;
   out_3521189503462309925[36] = 0;
   out_3521189503462309925[37] = 0;
   out_3521189503462309925[38] = 0;
   out_3521189503462309925[39] = 0;
   out_3521189503462309925[40] = 0;
   out_3521189503462309925[41] = 0;
   out_3521189503462309925[42] = 0;
   out_3521189503462309925[43] = 0;
   out_3521189503462309925[44] = 1;
   out_3521189503462309925[45] = 0;
   out_3521189503462309925[46] = 0;
   out_3521189503462309925[47] = 0;
   out_3521189503462309925[48] = 0;
   out_3521189503462309925[49] = 0;
   out_3521189503462309925[50] = 0;
   out_3521189503462309925[51] = 0;
   out_3521189503462309925[52] = 0;
   out_3521189503462309925[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_552322323456612592) {
  err_fun(nom_x, delta_x, out_552322323456612592);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4941387461979504541) {
  inv_err_fun(nom_x, true_x, out_4941387461979504541);
}
void pose_H_mod_fun(double *state, double *out_1196212354931887211) {
  H_mod_fun(state, out_1196212354931887211);
}
void pose_f_fun(double *state, double dt, double *out_8376554467190809622) {
  f_fun(state,  dt, out_8376554467190809622);
}
void pose_F_fun(double *state, double dt, double *out_9160386404678547461) {
  F_fun(state,  dt, out_9160386404678547461);
}
void pose_h_4(double *state, double *unused, double *out_7457733264180481157) {
  h_4(state, unused, out_7457733264180481157);
}
void pose_H_4(double *state, double *unused, double *out_442051352877174604) {
  H_4(state, unused, out_442051352877174604);
}
void pose_h_10(double *state, double *unused, double *out_7956640001151253329) {
  h_10(state, unused, out_7956640001151253329);
}
void pose_H_10(double *state, double *unused, double *out_2661350816086051052) {
  H_10(state, unused, out_2661350816086051052);
}
void pose_h_13(double *state, double *unused, double *out_5574642289715389555) {
  h_13(state, unused, out_5574642289715389555);
}
void pose_H_13(double *state, double *unused, double *out_7168579855439526325) {
  H_13(state, unused, out_7168579855439526325);
}
void pose_h_14(double *state, double *unused, double *out_3993226962874543760) {
  h_14(state, unused, out_3993226962874543760);
}
void pose_H_14(double *state, double *unused, double *out_3521189503462309925) {
  H_14(state, unused, out_3521189503462309925);
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
