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
void err_fun(double *nom_x, double *delta_x, double *out_6480965603127692382) {
   out_6480965603127692382[0] = delta_x[0] + nom_x[0];
   out_6480965603127692382[1] = delta_x[1] + nom_x[1];
   out_6480965603127692382[2] = delta_x[2] + nom_x[2];
   out_6480965603127692382[3] = delta_x[3] + nom_x[3];
   out_6480965603127692382[4] = delta_x[4] + nom_x[4];
   out_6480965603127692382[5] = delta_x[5] + nom_x[5];
   out_6480965603127692382[6] = delta_x[6] + nom_x[6];
   out_6480965603127692382[7] = delta_x[7] + nom_x[7];
   out_6480965603127692382[8] = delta_x[8] + nom_x[8];
   out_6480965603127692382[9] = delta_x[9] + nom_x[9];
   out_6480965603127692382[10] = delta_x[10] + nom_x[10];
   out_6480965603127692382[11] = delta_x[11] + nom_x[11];
   out_6480965603127692382[12] = delta_x[12] + nom_x[12];
   out_6480965603127692382[13] = delta_x[13] + nom_x[13];
   out_6480965603127692382[14] = delta_x[14] + nom_x[14];
   out_6480965603127692382[15] = delta_x[15] + nom_x[15];
   out_6480965603127692382[16] = delta_x[16] + nom_x[16];
   out_6480965603127692382[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7794393530327101272) {
   out_7794393530327101272[0] = -nom_x[0] + true_x[0];
   out_7794393530327101272[1] = -nom_x[1] + true_x[1];
   out_7794393530327101272[2] = -nom_x[2] + true_x[2];
   out_7794393530327101272[3] = -nom_x[3] + true_x[3];
   out_7794393530327101272[4] = -nom_x[4] + true_x[4];
   out_7794393530327101272[5] = -nom_x[5] + true_x[5];
   out_7794393530327101272[6] = -nom_x[6] + true_x[6];
   out_7794393530327101272[7] = -nom_x[7] + true_x[7];
   out_7794393530327101272[8] = -nom_x[8] + true_x[8];
   out_7794393530327101272[9] = -nom_x[9] + true_x[9];
   out_7794393530327101272[10] = -nom_x[10] + true_x[10];
   out_7794393530327101272[11] = -nom_x[11] + true_x[11];
   out_7794393530327101272[12] = -nom_x[12] + true_x[12];
   out_7794393530327101272[13] = -nom_x[13] + true_x[13];
   out_7794393530327101272[14] = -nom_x[14] + true_x[14];
   out_7794393530327101272[15] = -nom_x[15] + true_x[15];
   out_7794393530327101272[16] = -nom_x[16] + true_x[16];
   out_7794393530327101272[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6847342441623025206) {
   out_6847342441623025206[0] = 1.0;
   out_6847342441623025206[1] = 0.0;
   out_6847342441623025206[2] = 0.0;
   out_6847342441623025206[3] = 0.0;
   out_6847342441623025206[4] = 0.0;
   out_6847342441623025206[5] = 0.0;
   out_6847342441623025206[6] = 0.0;
   out_6847342441623025206[7] = 0.0;
   out_6847342441623025206[8] = 0.0;
   out_6847342441623025206[9] = 0.0;
   out_6847342441623025206[10] = 0.0;
   out_6847342441623025206[11] = 0.0;
   out_6847342441623025206[12] = 0.0;
   out_6847342441623025206[13] = 0.0;
   out_6847342441623025206[14] = 0.0;
   out_6847342441623025206[15] = 0.0;
   out_6847342441623025206[16] = 0.0;
   out_6847342441623025206[17] = 0.0;
   out_6847342441623025206[18] = 0.0;
   out_6847342441623025206[19] = 1.0;
   out_6847342441623025206[20] = 0.0;
   out_6847342441623025206[21] = 0.0;
   out_6847342441623025206[22] = 0.0;
   out_6847342441623025206[23] = 0.0;
   out_6847342441623025206[24] = 0.0;
   out_6847342441623025206[25] = 0.0;
   out_6847342441623025206[26] = 0.0;
   out_6847342441623025206[27] = 0.0;
   out_6847342441623025206[28] = 0.0;
   out_6847342441623025206[29] = 0.0;
   out_6847342441623025206[30] = 0.0;
   out_6847342441623025206[31] = 0.0;
   out_6847342441623025206[32] = 0.0;
   out_6847342441623025206[33] = 0.0;
   out_6847342441623025206[34] = 0.0;
   out_6847342441623025206[35] = 0.0;
   out_6847342441623025206[36] = 0.0;
   out_6847342441623025206[37] = 0.0;
   out_6847342441623025206[38] = 1.0;
   out_6847342441623025206[39] = 0.0;
   out_6847342441623025206[40] = 0.0;
   out_6847342441623025206[41] = 0.0;
   out_6847342441623025206[42] = 0.0;
   out_6847342441623025206[43] = 0.0;
   out_6847342441623025206[44] = 0.0;
   out_6847342441623025206[45] = 0.0;
   out_6847342441623025206[46] = 0.0;
   out_6847342441623025206[47] = 0.0;
   out_6847342441623025206[48] = 0.0;
   out_6847342441623025206[49] = 0.0;
   out_6847342441623025206[50] = 0.0;
   out_6847342441623025206[51] = 0.0;
   out_6847342441623025206[52] = 0.0;
   out_6847342441623025206[53] = 0.0;
   out_6847342441623025206[54] = 0.0;
   out_6847342441623025206[55] = 0.0;
   out_6847342441623025206[56] = 0.0;
   out_6847342441623025206[57] = 1.0;
   out_6847342441623025206[58] = 0.0;
   out_6847342441623025206[59] = 0.0;
   out_6847342441623025206[60] = 0.0;
   out_6847342441623025206[61] = 0.0;
   out_6847342441623025206[62] = 0.0;
   out_6847342441623025206[63] = 0.0;
   out_6847342441623025206[64] = 0.0;
   out_6847342441623025206[65] = 0.0;
   out_6847342441623025206[66] = 0.0;
   out_6847342441623025206[67] = 0.0;
   out_6847342441623025206[68] = 0.0;
   out_6847342441623025206[69] = 0.0;
   out_6847342441623025206[70] = 0.0;
   out_6847342441623025206[71] = 0.0;
   out_6847342441623025206[72] = 0.0;
   out_6847342441623025206[73] = 0.0;
   out_6847342441623025206[74] = 0.0;
   out_6847342441623025206[75] = 0.0;
   out_6847342441623025206[76] = 1.0;
   out_6847342441623025206[77] = 0.0;
   out_6847342441623025206[78] = 0.0;
   out_6847342441623025206[79] = 0.0;
   out_6847342441623025206[80] = 0.0;
   out_6847342441623025206[81] = 0.0;
   out_6847342441623025206[82] = 0.0;
   out_6847342441623025206[83] = 0.0;
   out_6847342441623025206[84] = 0.0;
   out_6847342441623025206[85] = 0.0;
   out_6847342441623025206[86] = 0.0;
   out_6847342441623025206[87] = 0.0;
   out_6847342441623025206[88] = 0.0;
   out_6847342441623025206[89] = 0.0;
   out_6847342441623025206[90] = 0.0;
   out_6847342441623025206[91] = 0.0;
   out_6847342441623025206[92] = 0.0;
   out_6847342441623025206[93] = 0.0;
   out_6847342441623025206[94] = 0.0;
   out_6847342441623025206[95] = 1.0;
   out_6847342441623025206[96] = 0.0;
   out_6847342441623025206[97] = 0.0;
   out_6847342441623025206[98] = 0.0;
   out_6847342441623025206[99] = 0.0;
   out_6847342441623025206[100] = 0.0;
   out_6847342441623025206[101] = 0.0;
   out_6847342441623025206[102] = 0.0;
   out_6847342441623025206[103] = 0.0;
   out_6847342441623025206[104] = 0.0;
   out_6847342441623025206[105] = 0.0;
   out_6847342441623025206[106] = 0.0;
   out_6847342441623025206[107] = 0.0;
   out_6847342441623025206[108] = 0.0;
   out_6847342441623025206[109] = 0.0;
   out_6847342441623025206[110] = 0.0;
   out_6847342441623025206[111] = 0.0;
   out_6847342441623025206[112] = 0.0;
   out_6847342441623025206[113] = 0.0;
   out_6847342441623025206[114] = 1.0;
   out_6847342441623025206[115] = 0.0;
   out_6847342441623025206[116] = 0.0;
   out_6847342441623025206[117] = 0.0;
   out_6847342441623025206[118] = 0.0;
   out_6847342441623025206[119] = 0.0;
   out_6847342441623025206[120] = 0.0;
   out_6847342441623025206[121] = 0.0;
   out_6847342441623025206[122] = 0.0;
   out_6847342441623025206[123] = 0.0;
   out_6847342441623025206[124] = 0.0;
   out_6847342441623025206[125] = 0.0;
   out_6847342441623025206[126] = 0.0;
   out_6847342441623025206[127] = 0.0;
   out_6847342441623025206[128] = 0.0;
   out_6847342441623025206[129] = 0.0;
   out_6847342441623025206[130] = 0.0;
   out_6847342441623025206[131] = 0.0;
   out_6847342441623025206[132] = 0.0;
   out_6847342441623025206[133] = 1.0;
   out_6847342441623025206[134] = 0.0;
   out_6847342441623025206[135] = 0.0;
   out_6847342441623025206[136] = 0.0;
   out_6847342441623025206[137] = 0.0;
   out_6847342441623025206[138] = 0.0;
   out_6847342441623025206[139] = 0.0;
   out_6847342441623025206[140] = 0.0;
   out_6847342441623025206[141] = 0.0;
   out_6847342441623025206[142] = 0.0;
   out_6847342441623025206[143] = 0.0;
   out_6847342441623025206[144] = 0.0;
   out_6847342441623025206[145] = 0.0;
   out_6847342441623025206[146] = 0.0;
   out_6847342441623025206[147] = 0.0;
   out_6847342441623025206[148] = 0.0;
   out_6847342441623025206[149] = 0.0;
   out_6847342441623025206[150] = 0.0;
   out_6847342441623025206[151] = 0.0;
   out_6847342441623025206[152] = 1.0;
   out_6847342441623025206[153] = 0.0;
   out_6847342441623025206[154] = 0.0;
   out_6847342441623025206[155] = 0.0;
   out_6847342441623025206[156] = 0.0;
   out_6847342441623025206[157] = 0.0;
   out_6847342441623025206[158] = 0.0;
   out_6847342441623025206[159] = 0.0;
   out_6847342441623025206[160] = 0.0;
   out_6847342441623025206[161] = 0.0;
   out_6847342441623025206[162] = 0.0;
   out_6847342441623025206[163] = 0.0;
   out_6847342441623025206[164] = 0.0;
   out_6847342441623025206[165] = 0.0;
   out_6847342441623025206[166] = 0.0;
   out_6847342441623025206[167] = 0.0;
   out_6847342441623025206[168] = 0.0;
   out_6847342441623025206[169] = 0.0;
   out_6847342441623025206[170] = 0.0;
   out_6847342441623025206[171] = 1.0;
   out_6847342441623025206[172] = 0.0;
   out_6847342441623025206[173] = 0.0;
   out_6847342441623025206[174] = 0.0;
   out_6847342441623025206[175] = 0.0;
   out_6847342441623025206[176] = 0.0;
   out_6847342441623025206[177] = 0.0;
   out_6847342441623025206[178] = 0.0;
   out_6847342441623025206[179] = 0.0;
   out_6847342441623025206[180] = 0.0;
   out_6847342441623025206[181] = 0.0;
   out_6847342441623025206[182] = 0.0;
   out_6847342441623025206[183] = 0.0;
   out_6847342441623025206[184] = 0.0;
   out_6847342441623025206[185] = 0.0;
   out_6847342441623025206[186] = 0.0;
   out_6847342441623025206[187] = 0.0;
   out_6847342441623025206[188] = 0.0;
   out_6847342441623025206[189] = 0.0;
   out_6847342441623025206[190] = 1.0;
   out_6847342441623025206[191] = 0.0;
   out_6847342441623025206[192] = 0.0;
   out_6847342441623025206[193] = 0.0;
   out_6847342441623025206[194] = 0.0;
   out_6847342441623025206[195] = 0.0;
   out_6847342441623025206[196] = 0.0;
   out_6847342441623025206[197] = 0.0;
   out_6847342441623025206[198] = 0.0;
   out_6847342441623025206[199] = 0.0;
   out_6847342441623025206[200] = 0.0;
   out_6847342441623025206[201] = 0.0;
   out_6847342441623025206[202] = 0.0;
   out_6847342441623025206[203] = 0.0;
   out_6847342441623025206[204] = 0.0;
   out_6847342441623025206[205] = 0.0;
   out_6847342441623025206[206] = 0.0;
   out_6847342441623025206[207] = 0.0;
   out_6847342441623025206[208] = 0.0;
   out_6847342441623025206[209] = 1.0;
   out_6847342441623025206[210] = 0.0;
   out_6847342441623025206[211] = 0.0;
   out_6847342441623025206[212] = 0.0;
   out_6847342441623025206[213] = 0.0;
   out_6847342441623025206[214] = 0.0;
   out_6847342441623025206[215] = 0.0;
   out_6847342441623025206[216] = 0.0;
   out_6847342441623025206[217] = 0.0;
   out_6847342441623025206[218] = 0.0;
   out_6847342441623025206[219] = 0.0;
   out_6847342441623025206[220] = 0.0;
   out_6847342441623025206[221] = 0.0;
   out_6847342441623025206[222] = 0.0;
   out_6847342441623025206[223] = 0.0;
   out_6847342441623025206[224] = 0.0;
   out_6847342441623025206[225] = 0.0;
   out_6847342441623025206[226] = 0.0;
   out_6847342441623025206[227] = 0.0;
   out_6847342441623025206[228] = 1.0;
   out_6847342441623025206[229] = 0.0;
   out_6847342441623025206[230] = 0.0;
   out_6847342441623025206[231] = 0.0;
   out_6847342441623025206[232] = 0.0;
   out_6847342441623025206[233] = 0.0;
   out_6847342441623025206[234] = 0.0;
   out_6847342441623025206[235] = 0.0;
   out_6847342441623025206[236] = 0.0;
   out_6847342441623025206[237] = 0.0;
   out_6847342441623025206[238] = 0.0;
   out_6847342441623025206[239] = 0.0;
   out_6847342441623025206[240] = 0.0;
   out_6847342441623025206[241] = 0.0;
   out_6847342441623025206[242] = 0.0;
   out_6847342441623025206[243] = 0.0;
   out_6847342441623025206[244] = 0.0;
   out_6847342441623025206[245] = 0.0;
   out_6847342441623025206[246] = 0.0;
   out_6847342441623025206[247] = 1.0;
   out_6847342441623025206[248] = 0.0;
   out_6847342441623025206[249] = 0.0;
   out_6847342441623025206[250] = 0.0;
   out_6847342441623025206[251] = 0.0;
   out_6847342441623025206[252] = 0.0;
   out_6847342441623025206[253] = 0.0;
   out_6847342441623025206[254] = 0.0;
   out_6847342441623025206[255] = 0.0;
   out_6847342441623025206[256] = 0.0;
   out_6847342441623025206[257] = 0.0;
   out_6847342441623025206[258] = 0.0;
   out_6847342441623025206[259] = 0.0;
   out_6847342441623025206[260] = 0.0;
   out_6847342441623025206[261] = 0.0;
   out_6847342441623025206[262] = 0.0;
   out_6847342441623025206[263] = 0.0;
   out_6847342441623025206[264] = 0.0;
   out_6847342441623025206[265] = 0.0;
   out_6847342441623025206[266] = 1.0;
   out_6847342441623025206[267] = 0.0;
   out_6847342441623025206[268] = 0.0;
   out_6847342441623025206[269] = 0.0;
   out_6847342441623025206[270] = 0.0;
   out_6847342441623025206[271] = 0.0;
   out_6847342441623025206[272] = 0.0;
   out_6847342441623025206[273] = 0.0;
   out_6847342441623025206[274] = 0.0;
   out_6847342441623025206[275] = 0.0;
   out_6847342441623025206[276] = 0.0;
   out_6847342441623025206[277] = 0.0;
   out_6847342441623025206[278] = 0.0;
   out_6847342441623025206[279] = 0.0;
   out_6847342441623025206[280] = 0.0;
   out_6847342441623025206[281] = 0.0;
   out_6847342441623025206[282] = 0.0;
   out_6847342441623025206[283] = 0.0;
   out_6847342441623025206[284] = 0.0;
   out_6847342441623025206[285] = 1.0;
   out_6847342441623025206[286] = 0.0;
   out_6847342441623025206[287] = 0.0;
   out_6847342441623025206[288] = 0.0;
   out_6847342441623025206[289] = 0.0;
   out_6847342441623025206[290] = 0.0;
   out_6847342441623025206[291] = 0.0;
   out_6847342441623025206[292] = 0.0;
   out_6847342441623025206[293] = 0.0;
   out_6847342441623025206[294] = 0.0;
   out_6847342441623025206[295] = 0.0;
   out_6847342441623025206[296] = 0.0;
   out_6847342441623025206[297] = 0.0;
   out_6847342441623025206[298] = 0.0;
   out_6847342441623025206[299] = 0.0;
   out_6847342441623025206[300] = 0.0;
   out_6847342441623025206[301] = 0.0;
   out_6847342441623025206[302] = 0.0;
   out_6847342441623025206[303] = 0.0;
   out_6847342441623025206[304] = 1.0;
   out_6847342441623025206[305] = 0.0;
   out_6847342441623025206[306] = 0.0;
   out_6847342441623025206[307] = 0.0;
   out_6847342441623025206[308] = 0.0;
   out_6847342441623025206[309] = 0.0;
   out_6847342441623025206[310] = 0.0;
   out_6847342441623025206[311] = 0.0;
   out_6847342441623025206[312] = 0.0;
   out_6847342441623025206[313] = 0.0;
   out_6847342441623025206[314] = 0.0;
   out_6847342441623025206[315] = 0.0;
   out_6847342441623025206[316] = 0.0;
   out_6847342441623025206[317] = 0.0;
   out_6847342441623025206[318] = 0.0;
   out_6847342441623025206[319] = 0.0;
   out_6847342441623025206[320] = 0.0;
   out_6847342441623025206[321] = 0.0;
   out_6847342441623025206[322] = 0.0;
   out_6847342441623025206[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7448658717317011616) {
   out_7448658717317011616[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7448658717317011616[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7448658717317011616[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7448658717317011616[3] = dt*state[12] + state[3];
   out_7448658717317011616[4] = dt*state[13] + state[4];
   out_7448658717317011616[5] = dt*state[14] + state[5];
   out_7448658717317011616[6] = state[6];
   out_7448658717317011616[7] = state[7];
   out_7448658717317011616[8] = state[8];
   out_7448658717317011616[9] = state[9];
   out_7448658717317011616[10] = state[10];
   out_7448658717317011616[11] = state[11];
   out_7448658717317011616[12] = state[12];
   out_7448658717317011616[13] = state[13];
   out_7448658717317011616[14] = state[14];
   out_7448658717317011616[15] = state[15];
   out_7448658717317011616[16] = state[16];
   out_7448658717317011616[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3051819978456506628) {
   out_3051819978456506628[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3051819978456506628[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3051819978456506628[2] = 0;
   out_3051819978456506628[3] = 0;
   out_3051819978456506628[4] = 0;
   out_3051819978456506628[5] = 0;
   out_3051819978456506628[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3051819978456506628[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3051819978456506628[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3051819978456506628[9] = 0;
   out_3051819978456506628[10] = 0;
   out_3051819978456506628[11] = 0;
   out_3051819978456506628[12] = 0;
   out_3051819978456506628[13] = 0;
   out_3051819978456506628[14] = 0;
   out_3051819978456506628[15] = 0;
   out_3051819978456506628[16] = 0;
   out_3051819978456506628[17] = 0;
   out_3051819978456506628[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3051819978456506628[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3051819978456506628[20] = 0;
   out_3051819978456506628[21] = 0;
   out_3051819978456506628[22] = 0;
   out_3051819978456506628[23] = 0;
   out_3051819978456506628[24] = 0;
   out_3051819978456506628[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3051819978456506628[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3051819978456506628[27] = 0;
   out_3051819978456506628[28] = 0;
   out_3051819978456506628[29] = 0;
   out_3051819978456506628[30] = 0;
   out_3051819978456506628[31] = 0;
   out_3051819978456506628[32] = 0;
   out_3051819978456506628[33] = 0;
   out_3051819978456506628[34] = 0;
   out_3051819978456506628[35] = 0;
   out_3051819978456506628[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3051819978456506628[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3051819978456506628[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3051819978456506628[39] = 0;
   out_3051819978456506628[40] = 0;
   out_3051819978456506628[41] = 0;
   out_3051819978456506628[42] = 0;
   out_3051819978456506628[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3051819978456506628[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3051819978456506628[45] = 0;
   out_3051819978456506628[46] = 0;
   out_3051819978456506628[47] = 0;
   out_3051819978456506628[48] = 0;
   out_3051819978456506628[49] = 0;
   out_3051819978456506628[50] = 0;
   out_3051819978456506628[51] = 0;
   out_3051819978456506628[52] = 0;
   out_3051819978456506628[53] = 0;
   out_3051819978456506628[54] = 0;
   out_3051819978456506628[55] = 0;
   out_3051819978456506628[56] = 0;
   out_3051819978456506628[57] = 1;
   out_3051819978456506628[58] = 0;
   out_3051819978456506628[59] = 0;
   out_3051819978456506628[60] = 0;
   out_3051819978456506628[61] = 0;
   out_3051819978456506628[62] = 0;
   out_3051819978456506628[63] = 0;
   out_3051819978456506628[64] = 0;
   out_3051819978456506628[65] = 0;
   out_3051819978456506628[66] = dt;
   out_3051819978456506628[67] = 0;
   out_3051819978456506628[68] = 0;
   out_3051819978456506628[69] = 0;
   out_3051819978456506628[70] = 0;
   out_3051819978456506628[71] = 0;
   out_3051819978456506628[72] = 0;
   out_3051819978456506628[73] = 0;
   out_3051819978456506628[74] = 0;
   out_3051819978456506628[75] = 0;
   out_3051819978456506628[76] = 1;
   out_3051819978456506628[77] = 0;
   out_3051819978456506628[78] = 0;
   out_3051819978456506628[79] = 0;
   out_3051819978456506628[80] = 0;
   out_3051819978456506628[81] = 0;
   out_3051819978456506628[82] = 0;
   out_3051819978456506628[83] = 0;
   out_3051819978456506628[84] = 0;
   out_3051819978456506628[85] = dt;
   out_3051819978456506628[86] = 0;
   out_3051819978456506628[87] = 0;
   out_3051819978456506628[88] = 0;
   out_3051819978456506628[89] = 0;
   out_3051819978456506628[90] = 0;
   out_3051819978456506628[91] = 0;
   out_3051819978456506628[92] = 0;
   out_3051819978456506628[93] = 0;
   out_3051819978456506628[94] = 0;
   out_3051819978456506628[95] = 1;
   out_3051819978456506628[96] = 0;
   out_3051819978456506628[97] = 0;
   out_3051819978456506628[98] = 0;
   out_3051819978456506628[99] = 0;
   out_3051819978456506628[100] = 0;
   out_3051819978456506628[101] = 0;
   out_3051819978456506628[102] = 0;
   out_3051819978456506628[103] = 0;
   out_3051819978456506628[104] = dt;
   out_3051819978456506628[105] = 0;
   out_3051819978456506628[106] = 0;
   out_3051819978456506628[107] = 0;
   out_3051819978456506628[108] = 0;
   out_3051819978456506628[109] = 0;
   out_3051819978456506628[110] = 0;
   out_3051819978456506628[111] = 0;
   out_3051819978456506628[112] = 0;
   out_3051819978456506628[113] = 0;
   out_3051819978456506628[114] = 1;
   out_3051819978456506628[115] = 0;
   out_3051819978456506628[116] = 0;
   out_3051819978456506628[117] = 0;
   out_3051819978456506628[118] = 0;
   out_3051819978456506628[119] = 0;
   out_3051819978456506628[120] = 0;
   out_3051819978456506628[121] = 0;
   out_3051819978456506628[122] = 0;
   out_3051819978456506628[123] = 0;
   out_3051819978456506628[124] = 0;
   out_3051819978456506628[125] = 0;
   out_3051819978456506628[126] = 0;
   out_3051819978456506628[127] = 0;
   out_3051819978456506628[128] = 0;
   out_3051819978456506628[129] = 0;
   out_3051819978456506628[130] = 0;
   out_3051819978456506628[131] = 0;
   out_3051819978456506628[132] = 0;
   out_3051819978456506628[133] = 1;
   out_3051819978456506628[134] = 0;
   out_3051819978456506628[135] = 0;
   out_3051819978456506628[136] = 0;
   out_3051819978456506628[137] = 0;
   out_3051819978456506628[138] = 0;
   out_3051819978456506628[139] = 0;
   out_3051819978456506628[140] = 0;
   out_3051819978456506628[141] = 0;
   out_3051819978456506628[142] = 0;
   out_3051819978456506628[143] = 0;
   out_3051819978456506628[144] = 0;
   out_3051819978456506628[145] = 0;
   out_3051819978456506628[146] = 0;
   out_3051819978456506628[147] = 0;
   out_3051819978456506628[148] = 0;
   out_3051819978456506628[149] = 0;
   out_3051819978456506628[150] = 0;
   out_3051819978456506628[151] = 0;
   out_3051819978456506628[152] = 1;
   out_3051819978456506628[153] = 0;
   out_3051819978456506628[154] = 0;
   out_3051819978456506628[155] = 0;
   out_3051819978456506628[156] = 0;
   out_3051819978456506628[157] = 0;
   out_3051819978456506628[158] = 0;
   out_3051819978456506628[159] = 0;
   out_3051819978456506628[160] = 0;
   out_3051819978456506628[161] = 0;
   out_3051819978456506628[162] = 0;
   out_3051819978456506628[163] = 0;
   out_3051819978456506628[164] = 0;
   out_3051819978456506628[165] = 0;
   out_3051819978456506628[166] = 0;
   out_3051819978456506628[167] = 0;
   out_3051819978456506628[168] = 0;
   out_3051819978456506628[169] = 0;
   out_3051819978456506628[170] = 0;
   out_3051819978456506628[171] = 1;
   out_3051819978456506628[172] = 0;
   out_3051819978456506628[173] = 0;
   out_3051819978456506628[174] = 0;
   out_3051819978456506628[175] = 0;
   out_3051819978456506628[176] = 0;
   out_3051819978456506628[177] = 0;
   out_3051819978456506628[178] = 0;
   out_3051819978456506628[179] = 0;
   out_3051819978456506628[180] = 0;
   out_3051819978456506628[181] = 0;
   out_3051819978456506628[182] = 0;
   out_3051819978456506628[183] = 0;
   out_3051819978456506628[184] = 0;
   out_3051819978456506628[185] = 0;
   out_3051819978456506628[186] = 0;
   out_3051819978456506628[187] = 0;
   out_3051819978456506628[188] = 0;
   out_3051819978456506628[189] = 0;
   out_3051819978456506628[190] = 1;
   out_3051819978456506628[191] = 0;
   out_3051819978456506628[192] = 0;
   out_3051819978456506628[193] = 0;
   out_3051819978456506628[194] = 0;
   out_3051819978456506628[195] = 0;
   out_3051819978456506628[196] = 0;
   out_3051819978456506628[197] = 0;
   out_3051819978456506628[198] = 0;
   out_3051819978456506628[199] = 0;
   out_3051819978456506628[200] = 0;
   out_3051819978456506628[201] = 0;
   out_3051819978456506628[202] = 0;
   out_3051819978456506628[203] = 0;
   out_3051819978456506628[204] = 0;
   out_3051819978456506628[205] = 0;
   out_3051819978456506628[206] = 0;
   out_3051819978456506628[207] = 0;
   out_3051819978456506628[208] = 0;
   out_3051819978456506628[209] = 1;
   out_3051819978456506628[210] = 0;
   out_3051819978456506628[211] = 0;
   out_3051819978456506628[212] = 0;
   out_3051819978456506628[213] = 0;
   out_3051819978456506628[214] = 0;
   out_3051819978456506628[215] = 0;
   out_3051819978456506628[216] = 0;
   out_3051819978456506628[217] = 0;
   out_3051819978456506628[218] = 0;
   out_3051819978456506628[219] = 0;
   out_3051819978456506628[220] = 0;
   out_3051819978456506628[221] = 0;
   out_3051819978456506628[222] = 0;
   out_3051819978456506628[223] = 0;
   out_3051819978456506628[224] = 0;
   out_3051819978456506628[225] = 0;
   out_3051819978456506628[226] = 0;
   out_3051819978456506628[227] = 0;
   out_3051819978456506628[228] = 1;
   out_3051819978456506628[229] = 0;
   out_3051819978456506628[230] = 0;
   out_3051819978456506628[231] = 0;
   out_3051819978456506628[232] = 0;
   out_3051819978456506628[233] = 0;
   out_3051819978456506628[234] = 0;
   out_3051819978456506628[235] = 0;
   out_3051819978456506628[236] = 0;
   out_3051819978456506628[237] = 0;
   out_3051819978456506628[238] = 0;
   out_3051819978456506628[239] = 0;
   out_3051819978456506628[240] = 0;
   out_3051819978456506628[241] = 0;
   out_3051819978456506628[242] = 0;
   out_3051819978456506628[243] = 0;
   out_3051819978456506628[244] = 0;
   out_3051819978456506628[245] = 0;
   out_3051819978456506628[246] = 0;
   out_3051819978456506628[247] = 1;
   out_3051819978456506628[248] = 0;
   out_3051819978456506628[249] = 0;
   out_3051819978456506628[250] = 0;
   out_3051819978456506628[251] = 0;
   out_3051819978456506628[252] = 0;
   out_3051819978456506628[253] = 0;
   out_3051819978456506628[254] = 0;
   out_3051819978456506628[255] = 0;
   out_3051819978456506628[256] = 0;
   out_3051819978456506628[257] = 0;
   out_3051819978456506628[258] = 0;
   out_3051819978456506628[259] = 0;
   out_3051819978456506628[260] = 0;
   out_3051819978456506628[261] = 0;
   out_3051819978456506628[262] = 0;
   out_3051819978456506628[263] = 0;
   out_3051819978456506628[264] = 0;
   out_3051819978456506628[265] = 0;
   out_3051819978456506628[266] = 1;
   out_3051819978456506628[267] = 0;
   out_3051819978456506628[268] = 0;
   out_3051819978456506628[269] = 0;
   out_3051819978456506628[270] = 0;
   out_3051819978456506628[271] = 0;
   out_3051819978456506628[272] = 0;
   out_3051819978456506628[273] = 0;
   out_3051819978456506628[274] = 0;
   out_3051819978456506628[275] = 0;
   out_3051819978456506628[276] = 0;
   out_3051819978456506628[277] = 0;
   out_3051819978456506628[278] = 0;
   out_3051819978456506628[279] = 0;
   out_3051819978456506628[280] = 0;
   out_3051819978456506628[281] = 0;
   out_3051819978456506628[282] = 0;
   out_3051819978456506628[283] = 0;
   out_3051819978456506628[284] = 0;
   out_3051819978456506628[285] = 1;
   out_3051819978456506628[286] = 0;
   out_3051819978456506628[287] = 0;
   out_3051819978456506628[288] = 0;
   out_3051819978456506628[289] = 0;
   out_3051819978456506628[290] = 0;
   out_3051819978456506628[291] = 0;
   out_3051819978456506628[292] = 0;
   out_3051819978456506628[293] = 0;
   out_3051819978456506628[294] = 0;
   out_3051819978456506628[295] = 0;
   out_3051819978456506628[296] = 0;
   out_3051819978456506628[297] = 0;
   out_3051819978456506628[298] = 0;
   out_3051819978456506628[299] = 0;
   out_3051819978456506628[300] = 0;
   out_3051819978456506628[301] = 0;
   out_3051819978456506628[302] = 0;
   out_3051819978456506628[303] = 0;
   out_3051819978456506628[304] = 1;
   out_3051819978456506628[305] = 0;
   out_3051819978456506628[306] = 0;
   out_3051819978456506628[307] = 0;
   out_3051819978456506628[308] = 0;
   out_3051819978456506628[309] = 0;
   out_3051819978456506628[310] = 0;
   out_3051819978456506628[311] = 0;
   out_3051819978456506628[312] = 0;
   out_3051819978456506628[313] = 0;
   out_3051819978456506628[314] = 0;
   out_3051819978456506628[315] = 0;
   out_3051819978456506628[316] = 0;
   out_3051819978456506628[317] = 0;
   out_3051819978456506628[318] = 0;
   out_3051819978456506628[319] = 0;
   out_3051819978456506628[320] = 0;
   out_3051819978456506628[321] = 0;
   out_3051819978456506628[322] = 0;
   out_3051819978456506628[323] = 1;
}
void h_4(double *state, double *unused, double *out_4242734118973278604) {
   out_4242734118973278604[0] = state[6] + state[9];
   out_4242734118973278604[1] = state[7] + state[10];
   out_4242734118973278604[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2097019224434796136) {
   out_2097019224434796136[0] = 0;
   out_2097019224434796136[1] = 0;
   out_2097019224434796136[2] = 0;
   out_2097019224434796136[3] = 0;
   out_2097019224434796136[4] = 0;
   out_2097019224434796136[5] = 0;
   out_2097019224434796136[6] = 1;
   out_2097019224434796136[7] = 0;
   out_2097019224434796136[8] = 0;
   out_2097019224434796136[9] = 1;
   out_2097019224434796136[10] = 0;
   out_2097019224434796136[11] = 0;
   out_2097019224434796136[12] = 0;
   out_2097019224434796136[13] = 0;
   out_2097019224434796136[14] = 0;
   out_2097019224434796136[15] = 0;
   out_2097019224434796136[16] = 0;
   out_2097019224434796136[17] = 0;
   out_2097019224434796136[18] = 0;
   out_2097019224434796136[19] = 0;
   out_2097019224434796136[20] = 0;
   out_2097019224434796136[21] = 0;
   out_2097019224434796136[22] = 0;
   out_2097019224434796136[23] = 0;
   out_2097019224434796136[24] = 0;
   out_2097019224434796136[25] = 1;
   out_2097019224434796136[26] = 0;
   out_2097019224434796136[27] = 0;
   out_2097019224434796136[28] = 1;
   out_2097019224434796136[29] = 0;
   out_2097019224434796136[30] = 0;
   out_2097019224434796136[31] = 0;
   out_2097019224434796136[32] = 0;
   out_2097019224434796136[33] = 0;
   out_2097019224434796136[34] = 0;
   out_2097019224434796136[35] = 0;
   out_2097019224434796136[36] = 0;
   out_2097019224434796136[37] = 0;
   out_2097019224434796136[38] = 0;
   out_2097019224434796136[39] = 0;
   out_2097019224434796136[40] = 0;
   out_2097019224434796136[41] = 0;
   out_2097019224434796136[42] = 0;
   out_2097019224434796136[43] = 0;
   out_2097019224434796136[44] = 1;
   out_2097019224434796136[45] = 0;
   out_2097019224434796136[46] = 0;
   out_2097019224434796136[47] = 1;
   out_2097019224434796136[48] = 0;
   out_2097019224434796136[49] = 0;
   out_2097019224434796136[50] = 0;
   out_2097019224434796136[51] = 0;
   out_2097019224434796136[52] = 0;
   out_2097019224434796136[53] = 0;
}
void h_10(double *state, double *unused, double *out_2858621772777210172) {
   out_2858621772777210172[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2858621772777210172[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2858621772777210172[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3733794250216138204) {
   out_3733794250216138204[0] = 0;
   out_3733794250216138204[1] = 9.8100000000000005*cos(state[1]);
   out_3733794250216138204[2] = 0;
   out_3733794250216138204[3] = 0;
   out_3733794250216138204[4] = -state[8];
   out_3733794250216138204[5] = state[7];
   out_3733794250216138204[6] = 0;
   out_3733794250216138204[7] = state[5];
   out_3733794250216138204[8] = -state[4];
   out_3733794250216138204[9] = 0;
   out_3733794250216138204[10] = 0;
   out_3733794250216138204[11] = 0;
   out_3733794250216138204[12] = 1;
   out_3733794250216138204[13] = 0;
   out_3733794250216138204[14] = 0;
   out_3733794250216138204[15] = 1;
   out_3733794250216138204[16] = 0;
   out_3733794250216138204[17] = 0;
   out_3733794250216138204[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3733794250216138204[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3733794250216138204[20] = 0;
   out_3733794250216138204[21] = state[8];
   out_3733794250216138204[22] = 0;
   out_3733794250216138204[23] = -state[6];
   out_3733794250216138204[24] = -state[5];
   out_3733794250216138204[25] = 0;
   out_3733794250216138204[26] = state[3];
   out_3733794250216138204[27] = 0;
   out_3733794250216138204[28] = 0;
   out_3733794250216138204[29] = 0;
   out_3733794250216138204[30] = 0;
   out_3733794250216138204[31] = 1;
   out_3733794250216138204[32] = 0;
   out_3733794250216138204[33] = 0;
   out_3733794250216138204[34] = 1;
   out_3733794250216138204[35] = 0;
   out_3733794250216138204[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3733794250216138204[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3733794250216138204[38] = 0;
   out_3733794250216138204[39] = -state[7];
   out_3733794250216138204[40] = state[6];
   out_3733794250216138204[41] = 0;
   out_3733794250216138204[42] = state[4];
   out_3733794250216138204[43] = -state[3];
   out_3733794250216138204[44] = 0;
   out_3733794250216138204[45] = 0;
   out_3733794250216138204[46] = 0;
   out_3733794250216138204[47] = 0;
   out_3733794250216138204[48] = 0;
   out_3733794250216138204[49] = 0;
   out_3733794250216138204[50] = 1;
   out_3733794250216138204[51] = 0;
   out_3733794250216138204[52] = 0;
   out_3733794250216138204[53] = 1;
}
void h_13(double *state, double *unused, double *out_2147303471254468456) {
   out_2147303471254468456[0] = state[3];
   out_2147303471254468456[1] = state[4];
   out_2147303471254468456[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1532417304752952032) {
   out_1532417304752952032[0] = 0;
   out_1532417304752952032[1] = 0;
   out_1532417304752952032[2] = 0;
   out_1532417304752952032[3] = 1;
   out_1532417304752952032[4] = 0;
   out_1532417304752952032[5] = 0;
   out_1532417304752952032[6] = 0;
   out_1532417304752952032[7] = 0;
   out_1532417304752952032[8] = 0;
   out_1532417304752952032[9] = 0;
   out_1532417304752952032[10] = 0;
   out_1532417304752952032[11] = 0;
   out_1532417304752952032[12] = 0;
   out_1532417304752952032[13] = 0;
   out_1532417304752952032[14] = 0;
   out_1532417304752952032[15] = 0;
   out_1532417304752952032[16] = 0;
   out_1532417304752952032[17] = 0;
   out_1532417304752952032[18] = 0;
   out_1532417304752952032[19] = 0;
   out_1532417304752952032[20] = 0;
   out_1532417304752952032[21] = 0;
   out_1532417304752952032[22] = 1;
   out_1532417304752952032[23] = 0;
   out_1532417304752952032[24] = 0;
   out_1532417304752952032[25] = 0;
   out_1532417304752952032[26] = 0;
   out_1532417304752952032[27] = 0;
   out_1532417304752952032[28] = 0;
   out_1532417304752952032[29] = 0;
   out_1532417304752952032[30] = 0;
   out_1532417304752952032[31] = 0;
   out_1532417304752952032[32] = 0;
   out_1532417304752952032[33] = 0;
   out_1532417304752952032[34] = 0;
   out_1532417304752952032[35] = 0;
   out_1532417304752952032[36] = 0;
   out_1532417304752952032[37] = 0;
   out_1532417304752952032[38] = 0;
   out_1532417304752952032[39] = 0;
   out_1532417304752952032[40] = 0;
   out_1532417304752952032[41] = 1;
   out_1532417304752952032[42] = 0;
   out_1532417304752952032[43] = 0;
   out_1532417304752952032[44] = 0;
   out_1532417304752952032[45] = 0;
   out_1532417304752952032[46] = 0;
   out_1532417304752952032[47] = 0;
   out_1532417304752952032[48] = 0;
   out_1532417304752952032[49] = 0;
   out_1532417304752952032[50] = 0;
   out_1532417304752952032[51] = 0;
   out_1532417304752952032[52] = 0;
   out_1532417304752952032[53] = 0;
}
void h_14(double *state, double *unused, double *out_7424563568260947704) {
   out_7424563568260947704[0] = state[6];
   out_7424563568260947704[1] = state[7];
   out_7424563568260947704[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5179807656730168432) {
   out_5179807656730168432[0] = 0;
   out_5179807656730168432[1] = 0;
   out_5179807656730168432[2] = 0;
   out_5179807656730168432[3] = 0;
   out_5179807656730168432[4] = 0;
   out_5179807656730168432[5] = 0;
   out_5179807656730168432[6] = 1;
   out_5179807656730168432[7] = 0;
   out_5179807656730168432[8] = 0;
   out_5179807656730168432[9] = 0;
   out_5179807656730168432[10] = 0;
   out_5179807656730168432[11] = 0;
   out_5179807656730168432[12] = 0;
   out_5179807656730168432[13] = 0;
   out_5179807656730168432[14] = 0;
   out_5179807656730168432[15] = 0;
   out_5179807656730168432[16] = 0;
   out_5179807656730168432[17] = 0;
   out_5179807656730168432[18] = 0;
   out_5179807656730168432[19] = 0;
   out_5179807656730168432[20] = 0;
   out_5179807656730168432[21] = 0;
   out_5179807656730168432[22] = 0;
   out_5179807656730168432[23] = 0;
   out_5179807656730168432[24] = 0;
   out_5179807656730168432[25] = 1;
   out_5179807656730168432[26] = 0;
   out_5179807656730168432[27] = 0;
   out_5179807656730168432[28] = 0;
   out_5179807656730168432[29] = 0;
   out_5179807656730168432[30] = 0;
   out_5179807656730168432[31] = 0;
   out_5179807656730168432[32] = 0;
   out_5179807656730168432[33] = 0;
   out_5179807656730168432[34] = 0;
   out_5179807656730168432[35] = 0;
   out_5179807656730168432[36] = 0;
   out_5179807656730168432[37] = 0;
   out_5179807656730168432[38] = 0;
   out_5179807656730168432[39] = 0;
   out_5179807656730168432[40] = 0;
   out_5179807656730168432[41] = 0;
   out_5179807656730168432[42] = 0;
   out_5179807656730168432[43] = 0;
   out_5179807656730168432[44] = 1;
   out_5179807656730168432[45] = 0;
   out_5179807656730168432[46] = 0;
   out_5179807656730168432[47] = 0;
   out_5179807656730168432[48] = 0;
   out_5179807656730168432[49] = 0;
   out_5179807656730168432[50] = 0;
   out_5179807656730168432[51] = 0;
   out_5179807656730168432[52] = 0;
   out_5179807656730168432[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6480965603127692382) {
  err_fun(nom_x, delta_x, out_6480965603127692382);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7794393530327101272) {
  inv_err_fun(nom_x, true_x, out_7794393530327101272);
}
void pose_H_mod_fun(double *state, double *out_6847342441623025206) {
  H_mod_fun(state, out_6847342441623025206);
}
void pose_f_fun(double *state, double dt, double *out_7448658717317011616) {
  f_fun(state,  dt, out_7448658717317011616);
}
void pose_F_fun(double *state, double dt, double *out_3051819978456506628) {
  F_fun(state,  dt, out_3051819978456506628);
}
void pose_h_4(double *state, double *unused, double *out_4242734118973278604) {
  h_4(state, unused, out_4242734118973278604);
}
void pose_H_4(double *state, double *unused, double *out_2097019224434796136) {
  H_4(state, unused, out_2097019224434796136);
}
void pose_h_10(double *state, double *unused, double *out_2858621772777210172) {
  h_10(state, unused, out_2858621772777210172);
}
void pose_H_10(double *state, double *unused, double *out_3733794250216138204) {
  H_10(state, unused, out_3733794250216138204);
}
void pose_h_13(double *state, double *unused, double *out_2147303471254468456) {
  h_13(state, unused, out_2147303471254468456);
}
void pose_H_13(double *state, double *unused, double *out_1532417304752952032) {
  H_13(state, unused, out_1532417304752952032);
}
void pose_h_14(double *state, double *unused, double *out_7424563568260947704) {
  h_14(state, unused, out_7424563568260947704);
}
void pose_H_14(double *state, double *unused, double *out_5179807656730168432) {
  H_14(state, unused, out_5179807656730168432);
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
