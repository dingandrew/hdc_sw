//*#*********************************************************************************************************************/
//*# Software       : TSMC MEMORY COMPILER tsn28hpcpuhddpsram_2012.02.00.d.170a						*/
//*# Technology     : TSMC 28nm CMOS LOGIC High Performance Compact Mobile 1P10M HKMG CU_ELK 0.9V				*/
//*# Memory Type    : TSMC 28nm High Performance Compact Mobile Ultra High Density Dual Port SRAM with d127 bit cell SVT Periphery */
//*# Library Name   : tsdn28hpcpuhdb512x64m4mwa (user specify : TSDN28HPCPUHDB512X64M4MWA)				*/
//*# Library Version: 170a												*/
//*# Generated Time : 2023/11/20, 20:23:53										*/
//*#*********************************************************************************************************************/
//*#															*/
//*# STATEMENT OF USE													*/
//*#															*/
//*# This information contains confidential and proprietary information of TSMC.					*/
//*# No part of this information may be reproduced, transmitted, transcribed,						*/
//*# stored in a retrieval system, or translated into any human or computer						*/
//*# language, in any form or by any means, electronic, mechanical, magnetic,						*/
//*# optical, chemical, manual, or otherwise, without the prior written permission					*/
//*# of TSMC. This information was prepared for informational purpose and is for					*/
//*# use by TSMC's customers only. TSMC reserves the right to make changes in the					*/
//*# information at any time and without notice.									*/
//*#															*/
//*#*********************************************************************************************************************/
///*******************************************************************************/
//*      Usage Limitation: PLEASE READ CAREFULLY FOR CORRECT USAGE               */
//* The model doesn't support the control enable, data, address signals          */
//* transition at positive clock edge.                                           */
//* Please have some timing delays between control/data/address and clock signals*/
//* to ensure the correct behavior.                                              */
//*                                                                              */
//* Please be careful when using non 2^n  memory.                                */
//* In a non-fully decoded array, a write cycle to a nonexistent address location*/
//* does not change the memory array contents and output remains the same.       */
//* In a non-fully decoded array, a read cycle to a nonexistent address location */
//* does not change the memory array contents but output becomes unknown.        */
//*                                                                              */
//* In the verilog model, the behavior of unknown clock will corrupt the         */
//* memory data and make output unknown regardless of CEB signal.  But in the    */
//* silicon, the unknown clock at CEB high, the memory and output data will be   */
//* held. The verilog model behavior is more conservative in this condition.     */
//*                                                                              */
//* The model doesn't identify physical column and row address                   */
//*                                                                              */
//* The verilog model provides TSMC_CM_UNIT_DELAY mode for the fast function     */
//* simulation.                                                                  */
//* All timing values in the specification are not checked in the                */
//* TSMC_CM_UNIT_DELAY mode simulation.                                          */
//*                                                                              */
//*                                                                              */
//*                                                                              */
//* Please use the verilog simulator version with $recrem timing check support.  */
//* Some earlier simulator versions might support $recovery only, not $recrem.   */
//*                                                                              */
//* Template Version : S_01_61101                                       */
//****************************************************************************** */
//*      Macro Usage       : (+define[MACRO] for Verilog compiliers)             */
//* +TSMC_CM_UNIT_DELAY : Enable fast function simulation.                       */
//* +no_warning : Disable all runtime warnings message from this model.          */
//* +TSMC_INITIALIZE_MEM : Initialize the memory data in verilog format.         */
//* +TSMC_INITIALIZE_FAULT : Initialize the memory fault data in verilog format. */
//* +TSMC_NO_TESTPINS_WARNING : Disable the wrong test pins connection error     */
//*                             message if necessary.                            */
//****************************************************************************** */

`resetall
`celldefine

`timescale 1ns/1ps
`delay_mode_path
`suppress_faults
`enable_portfaults

module TSDN28HPCPUHDB512X64M4MWA
    (
           RTSEL,
           WTSEL,
           PTSEL,
    AA,
    DA,
    BWEBA,
    WEBA,CEBA,CLK,
    AB,
    DB,
    BWEBB,
    WEBB,CEBB,
    AWT,
    QA,
    QB
  );

// Parameter declarations
parameter  N = 32;
parameter  W = 262144/(32/8);  // modified to match vproc_tb
parameter  M = 16;       // modfied to match vproc_tb
parameter  RA = 14;

    wire SLP=1'b0;
    wire DSLP=1'b0;
    wire SD=1'b0;
    input [1:0] RTSEL;
    input [1:0] WTSEL;
    input [1:0] PTSEL;

// Input-Output declarations

    input [M-1:0] AA;
    input [N-1:0] DA;
    input [N-1:0] BWEBA;

    input WEBA;
    input CEBA;
    input CLK;
    input [M-1:0] AB;
    input [N-1:0] DB;
    input [N-1:0] BWEBB;
    input WEBB;
    input CEBB;
    input AWT;
    output [N-1:0] QA;
    output [N-1:0] QB;

`ifdef no_warning
parameter MES_ALL = "OFF";
`else
parameter MES_ALL = "ON";
`endif

`ifdef TSMC_CM_UNIT_DELAY
parameter  SRAM_DELAY = 0.010;
`endif
`ifdef TSMC_INITIALIZE_MEM
parameter INITIAL_MEM_DELAY = 0.01;
`else
  `ifdef TSMC_INITIALIZE_MEM_USING_DEFAULT_TASKS
parameter INITIAL_MEM_DELAY = 0.01;
  `endif
`endif
`ifdef TSMC_INITIALIZE_FAULT
parameter INITIAL_FAULT_DELAY = 0.01;
`endif

`ifdef TSMC_INITIALIZE_MEM
parameter cdeFileInit  = "TSDN28HPCPUHDB512X64M4MWA_initial.cde";
`endif
`ifdef TSMC_INITIALIZE_FAULT
parameter cdeFileFault = "TSDN28HPCPUHDB512X64M4MWA_fault.cde";
`endif

// Registers
reg invalid_aslp;
reg invalid_bslp;
reg invalid_adslp;
reg invalid_bdslp;
reg invalid_sdwk_dslp;

reg [N-1:0] DAL;
reg [N-1:0] DBL;
reg [N-1:0] bDBL;
 
reg [N-1:0] BWEBAL;
reg [N-1:0] BWEBBL;
reg [N-1:0] bBWEBBL;
 
reg [M-1:0] AAL;
reg [M-1:0] ABL;
 
reg WEBAL,CEBAL;
reg WEBBL,CEBBL;
 
wire [N-1:0] QAL;
wire [N-1:0] QBL;

reg valid_testpin;


reg valid_ck,valid_cka,valid_ckb;
reg valid_cea, valid_ceb;
reg valid_wea, valid_web;
reg valid_aa;
reg valid_ab;
reg valid_contentiona,valid_contentionb,valid_contentionc;
reg valid_da63, valid_da62, valid_da61, valid_da60, valid_da59, valid_da58, valid_da57, valid_da56, valid_da55, valid_da54, valid_da53, valid_da52, valid_da51, valid_da50, valid_da49, valid_da48, valid_da47, valid_da46, valid_da45, valid_da44, valid_da43, valid_da42, valid_da41, valid_da40, valid_da39, valid_da38, valid_da37, valid_da36, valid_da35, valid_da34, valid_da33, valid_da32, valid_da31, valid_da30, valid_da29, valid_da28, valid_da27, valid_da26, valid_da25, valid_da24, valid_da23, valid_da22, valid_da21, valid_da20, valid_da19, valid_da18, valid_da17, valid_da16, valid_da15, valid_da14, valid_da13, valid_da12, valid_da11, valid_da10, valid_da9, valid_da8, valid_da7, valid_da6, valid_da5, valid_da4, valid_da3, valid_da2, valid_da1, valid_da0;
reg valid_db63, valid_db62, valid_db61, valid_db60, valid_db59, valid_db58, valid_db57, valid_db56, valid_db55, valid_db54, valid_db53, valid_db52, valid_db51, valid_db50, valid_db49, valid_db48, valid_db47, valid_db46, valid_db45, valid_db44, valid_db43, valid_db42, valid_db41, valid_db40, valid_db39, valid_db38, valid_db37, valid_db36, valid_db35, valid_db34, valid_db33, valid_db32, valid_db31, valid_db30, valid_db29, valid_db28, valid_db27, valid_db26, valid_db25, valid_db24, valid_db23, valid_db22, valid_db21, valid_db20, valid_db19, valid_db18, valid_db17, valid_db16, valid_db15, valid_db14, valid_db13, valid_db12, valid_db11, valid_db10, valid_db9, valid_db8, valid_db7, valid_db6, valid_db5, valid_db4, valid_db3, valid_db2, valid_db1, valid_db0;
reg valid_bwa63, valid_bwa62, valid_bwa61, valid_bwa60, valid_bwa59, valid_bwa58, valid_bwa57, valid_bwa56, valid_bwa55, valid_bwa54, valid_bwa53, valid_bwa52, valid_bwa51, valid_bwa50, valid_bwa49, valid_bwa48, valid_bwa47, valid_bwa46, valid_bwa45, valid_bwa44, valid_bwa43, valid_bwa42, valid_bwa41, valid_bwa40, valid_bwa39, valid_bwa38, valid_bwa37, valid_bwa36, valid_bwa35, valid_bwa34, valid_bwa33, valid_bwa32, valid_bwa31, valid_bwa30, valid_bwa29, valid_bwa28, valid_bwa27, valid_bwa26, valid_bwa25, valid_bwa24, valid_bwa23, valid_bwa22, valid_bwa21, valid_bwa20, valid_bwa19, valid_bwa18, valid_bwa17, valid_bwa16, valid_bwa15, valid_bwa14, valid_bwa13, valid_bwa12, valid_bwa11, valid_bwa10, valid_bwa9, valid_bwa8, valid_bwa7, valid_bwa6, valid_bwa5, valid_bwa4, valid_bwa3, valid_bwa2, valid_bwa1, valid_bwa0;
reg valid_bwb63, valid_bwb62, valid_bwb61, valid_bwb60, valid_bwb59, valid_bwb58, valid_bwb57, valid_bwb56, valid_bwb55, valid_bwb54, valid_bwb53, valid_bwb52, valid_bwb51, valid_bwb50, valid_bwb49, valid_bwb48, valid_bwb47, valid_bwb46, valid_bwb45, valid_bwb44, valid_bwb43, valid_bwb42, valid_bwb41, valid_bwb40, valid_bwb39, valid_bwb38, valid_bwb37, valid_bwb36, valid_bwb35, valid_bwb34, valid_bwb33, valid_bwb32, valid_bwb31, valid_bwb30, valid_bwb29, valid_bwb28, valid_bwb27, valid_bwb26, valid_bwb25, valid_bwb24, valid_bwb23, valid_bwb22, valid_bwb21, valid_bwb20, valid_bwb19, valid_bwb18, valid_bwb17, valid_bwb16, valid_bwb15, valid_bwb14, valid_bwb13, valid_bwb12, valid_bwb11, valid_bwb10, valid_bwb9, valid_bwb8, valid_bwb7, valid_bwb6, valid_bwb5, valid_bwb4, valid_bwb3, valid_bwb2, valid_bwb1, valid_bwb0;
 
reg EN;
reg RDA, RDB;

reg RCLKA,RCLKB;


wire [1:0] bRTSEL;
wire [1:0] bWTSEL;
wire [1:0] bPTSEL;


wire [N-1:0] bBWEBA;
wire [N-1:0] bBWEBB;
 
wire [N-1:0] bDA;
wire [N-1:0] bDB;
 
wire [M-1:0] bAA;
wire [M-1:0] bAB;
wire [RA-1:0] rowAA;
wire [RA-1:0] rowAB;
 
wire bWEBA,bWEBB;
wire bCEBA,bCEBB;
wire bCLKA,bCLKB;
 
reg [N-1:0] bQA;
reg [N-1:0] bQB;

wire bBIST;
wire WEA,WEB,CSA,CSB;
wire bAWT;
reg valid_awt_wk;
wire iCEBA = bCEBA;
wire iCEBB = bCEBB;
wire iCLKA = bCLKA;
wire iCLKB = bCLKB;
wire [N-1:0] iBWEBA = bBWEBA;
wire [N-1:0] iBWEBB = bBWEBB;

wire [N-1:0] bbQA;
wire [N-1:0] bbQB;
 
integer i;
integer clk_count;
integer sd_mode;




// Address Inputs
buf sAA0 (bAA[0], AA[0]);
buf sAB0 (bAB[0], AB[0]);
buf sAA1 (bAA[1], AA[1]);
buf sAB1 (bAB[1], AB[1]);
buf sAA2 (bAA[2], AA[2]);
buf sAB2 (bAB[2], AB[2]);
buf sAA3 (bAA[3], AA[3]);
buf sAB3 (bAB[3], AB[3]);
buf sAA4 (bAA[4], AA[4]);
buf sAB4 (bAB[4], AB[4]);
buf sAA5 (bAA[5], AA[5]);
buf sAB5 (bAB[5], AB[5]);
buf sAA6 (bAA[6], AA[6]);
buf sAB6 (bAB[6], AB[6]);
buf sAA7 (bAA[7], AA[7]);
buf sAB7 (bAB[7], AB[7]);
buf sAA8 (bAA[8], AA[8]);
buf sAB8 (bAB[8], AB[8]);
buf sAA9 (bAA[9], AA[9]);
buf sAB9 (bAB[9], AB[9]);
buf sAA10 (bAA[10], AA[10]);
buf sAB10 (bAB[10], AB[10]);
buf sAA11 (bAA[11], AA[11]);
buf sAB11 (bAB[11], AB[11]);
buf sAA12 (bAA[12], AA[12]);
buf sAB12 (bAB[12], AB[12]);
buf sAA13 (bAA[13], AA[13]);
buf sAB13 (bAB[13], AB[13]);
buf sAA14 (bAA[14], AA[14]);
buf sAB14 (bAB[14], AB[14]);
buf sAA15 (bAA[15], AA[15]);
buf sAB15 (bAB[15], AB[15]);
buf srAA0 (rowAA[0], AA[2]);
buf srAB0 (rowAB[0], AB[2]);
buf srAA1 (rowAA[1], AA[3]);
buf srAB1 (rowAB[1], AB[3]);
buf srAA2 (rowAA[2], AA[4]);
buf srAB2 (rowAB[2], AB[4]);
buf srAA3 (rowAA[3], AA[5]);
buf srAB3 (rowAB[3], AB[5]);
buf srAA4 (rowAA[4], AA[6]);
buf srAB4 (rowAB[4], AB[6]);
buf srAA5 (rowAA[5], AA[7]);
buf srAB5 (rowAB[5], AB[7]);
buf srAA6 (rowAA[6], AA[8]);
buf srAB6 (rowAB[6], AB[8]);
buf srAA7 (rowAA[7], AA[9]);
buf srAB7 (rowAB[7], AB[9]);
buf srAA8 (rowAA[8], AA[10]);
buf srAB8 (rowAB[8], AB[10]);
buf srAA9 (rowAA[9], AA[11]);
buf srAB9 (rowAB[9], AB[11]);
buf srAA10 (rowAA[10], AA[12]);
buf srAB10 (rowAB[10], AB[12]);
buf srAA11 (rowAA[11], AA[13]);
buf srAB11 (rowAB[11], AB[13]);
buf srAA12 (rowAA[12], AA[14]);
buf srAB12 (rowAB[12], AB[14]);
buf srAA13 (rowAA[13], AA[15]);
buf srAB13 (rowAB[13], AB[15]);


// Bit Write/Data Inputs 
buf sDA0 (bDA[0], DA[0]);
buf sDB0 (bDB[0], DB[0]);
buf sDA1 (bDA[1], DA[1]);
buf sDB1 (bDB[1], DB[1]);
buf sDA2 (bDA[2], DA[2]);
buf sDB2 (bDB[2], DB[2]);
buf sDA3 (bDA[3], DA[3]);
buf sDB3 (bDB[3], DB[3]);
buf sDA4 (bDA[4], DA[4]);
buf sDB4 (bDB[4], DB[4]);
buf sDA5 (bDA[5], DA[5]);
buf sDB5 (bDB[5], DB[5]);
buf sDA6 (bDA[6], DA[6]);
buf sDB6 (bDB[6], DB[6]);
buf sDA7 (bDA[7], DA[7]);
buf sDB7 (bDB[7], DB[7]);
buf sDA8 (bDA[8], DA[8]);
buf sDB8 (bDB[8], DB[8]);
buf sDA9 (bDA[9], DA[9]);
buf sDB9 (bDB[9], DB[9]);
buf sDA10 (bDA[10], DA[10]);
buf sDB10 (bDB[10], DB[10]);
buf sDA11 (bDA[11], DA[11]);
buf sDB11 (bDB[11], DB[11]);
buf sDA12 (bDA[12], DA[12]);
buf sDB12 (bDB[12], DB[12]);
buf sDA13 (bDA[13], DA[13]);
buf sDB13 (bDB[13], DB[13]);
buf sDA14 (bDA[14], DA[14]);
buf sDB14 (bDB[14], DB[14]);
buf sDA15 (bDA[15], DA[15]);
buf sDB15 (bDB[15], DB[15]);
buf sDA16 (bDA[16], DA[16]);
buf sDB16 (bDB[16], DB[16]);
buf sDA17 (bDA[17], DA[17]);
buf sDB17 (bDB[17], DB[17]);
buf sDA18 (bDA[18], DA[18]);
buf sDB18 (bDB[18], DB[18]);
buf sDA19 (bDA[19], DA[19]);
buf sDB19 (bDB[19], DB[19]);
buf sDA20 (bDA[20], DA[20]);
buf sDB20 (bDB[20], DB[20]);
buf sDA21 (bDA[21], DA[21]);
buf sDB21 (bDB[21], DB[21]);
buf sDA22 (bDA[22], DA[22]);
buf sDB22 (bDB[22], DB[22]);
buf sDA23 (bDA[23], DA[23]);
buf sDB23 (bDB[23], DB[23]);
buf sDA24 (bDA[24], DA[24]);
buf sDB24 (bDB[24], DB[24]);
buf sDA25 (bDA[25], DA[25]);
buf sDB25 (bDB[25], DB[25]);
buf sDA26 (bDA[26], DA[26]);
buf sDB26 (bDB[26], DB[26]);
buf sDA27 (bDA[27], DA[27]);
buf sDB27 (bDB[27], DB[27]);
buf sDA28 (bDA[28], DA[28]);
buf sDB28 (bDB[28], DB[28]);
buf sDA29 (bDA[29], DA[29]);
buf sDB29 (bDB[29], DB[29]);
buf sDA30 (bDA[30], DA[30]);
buf sDB30 (bDB[30], DB[30]);
buf sDA31 (bDA[31], DA[31]);
buf sDB31 (bDB[31], DB[31]);
// buf sDA32 (bDA[32], DA[32]);
// buf sDB32 (bDB[32], DB[32]);
// buf sDA33 (bDA[33], DA[33]);
// buf sDB33 (bDB[33], DB[33]);
// buf sDA34 (bDA[34], DA[34]);
// buf sDB34 (bDB[34], DB[34]);
// buf sDA35 (bDA[35], DA[35]);
// buf sDB35 (bDB[35], DB[35]);
// buf sDA36 (bDA[36], DA[36]);
// buf sDB36 (bDB[36], DB[36]);
// buf sDA37 (bDA[37], DA[37]);
// buf sDB37 (bDB[37], DB[37]);
// buf sDA38 (bDA[38], DA[38]);
// buf sDB38 (bDB[38], DB[38]);
// buf sDA39 (bDA[39], DA[39]);
// buf sDB39 (bDB[39], DB[39]);
// buf sDA40 (bDA[40], DA[40]);
// buf sDB40 (bDB[40], DB[40]);
// buf sDA41 (bDA[41], DA[41]);
// buf sDB41 (bDB[41], DB[41]);
// buf sDA42 (bDA[42], DA[42]);
// buf sDB42 (bDB[42], DB[42]);
// buf sDA43 (bDA[43], DA[43]);
// buf sDB43 (bDB[43], DB[43]);
// buf sDA44 (bDA[44], DA[44]);
// buf sDB44 (bDB[44], DB[44]);
// buf sDA45 (bDA[45], DA[45]);
// buf sDB45 (bDB[45], DB[45]);
// buf sDA46 (bDA[46], DA[46]);
// buf sDB46 (bDB[46], DB[46]);
// buf sDA47 (bDA[47], DA[47]);
// buf sDB47 (bDB[47], DB[47]);
// buf sDA48 (bDA[48], DA[48]);
// buf sDB48 (bDB[48], DB[48]);
// buf sDA49 (bDA[49], DA[49]);
// buf sDB49 (bDB[49], DB[49]);
// buf sDA50 (bDA[50], DA[50]);
// buf sDB50 (bDB[50], DB[50]);
// buf sDA51 (bDA[51], DA[51]);
// buf sDB51 (bDB[51], DB[51]);
// buf sDA52 (bDA[52], DA[52]);
// buf sDB52 (bDB[52], DB[52]);
// buf sDA53 (bDA[53], DA[53]);
// buf sDB53 (bDB[53], DB[53]);
// buf sDA54 (bDA[54], DA[54]);
// buf sDB54 (bDB[54], DB[54]);
// buf sDA55 (bDA[55], DA[55]);
// buf sDB55 (bDB[55], DB[55]);
// buf sDA56 (bDA[56], DA[56]);
// buf sDB56 (bDB[56], DB[56]);
// buf sDA57 (bDA[57], DA[57]);
// buf sDB57 (bDB[57], DB[57]);
// buf sDA58 (bDA[58], DA[58]);
// buf sDB58 (bDB[58], DB[58]);
// buf sDA59 (bDA[59], DA[59]);
// buf sDB59 (bDB[59], DB[59]);
// buf sDA60 (bDA[60], DA[60]);
// buf sDB60 (bDB[60], DB[60]);
// buf sDA61 (bDA[61], DA[61]);
// buf sDB61 (bDB[61], DB[61]);
// buf sDA62 (bDA[62], DA[62]);
// buf sDB62 (bDB[62], DB[62]);
// buf sDA63 (bDA[63], DA[63]);
// buf sDB63 (bDB[63], DB[63]);


buf sBWEBA0 (bBWEBA[0], BWEBA[0]);
buf sBWEBB0 (bBWEBB[0], BWEBB[0]);
buf sBWEBA1 (bBWEBA[1], BWEBA[1]);
buf sBWEBB1 (bBWEBB[1], BWEBB[1]);
buf sBWEBA2 (bBWEBA[2], BWEBA[2]);
buf sBWEBB2 (bBWEBB[2], BWEBB[2]);
buf sBWEBA3 (bBWEBA[3], BWEBA[3]);
buf sBWEBB3 (bBWEBB[3], BWEBB[3]);
buf sBWEBA4 (bBWEBA[4], BWEBA[4]);
buf sBWEBB4 (bBWEBB[4], BWEBB[4]);
buf sBWEBA5 (bBWEBA[5], BWEBA[5]);
buf sBWEBB5 (bBWEBB[5], BWEBB[5]);
buf sBWEBA6 (bBWEBA[6], BWEBA[6]);
buf sBWEBB6 (bBWEBB[6], BWEBB[6]);
buf sBWEBA7 (bBWEBA[7], BWEBA[7]);
buf sBWEBB7 (bBWEBB[7], BWEBB[7]);
buf sBWEBA8 (bBWEBA[8], BWEBA[8]);
buf sBWEBB8 (bBWEBB[8], BWEBB[8]);
buf sBWEBA9 (bBWEBA[9], BWEBA[9]);
buf sBWEBB9 (bBWEBB[9], BWEBB[9]);
buf sBWEBA10 (bBWEBA[10], BWEBA[10]);
buf sBWEBB10 (bBWEBB[10], BWEBB[10]);
buf sBWEBA11 (bBWEBA[11], BWEBA[11]);
buf sBWEBB11 (bBWEBB[11], BWEBB[11]);
buf sBWEBA12 (bBWEBA[12], BWEBA[12]);
buf sBWEBB12 (bBWEBB[12], BWEBB[12]);
buf sBWEBA13 (bBWEBA[13], BWEBA[13]);
buf sBWEBB13 (bBWEBB[13], BWEBB[13]);
buf sBWEBA14 (bBWEBA[14], BWEBA[14]);
buf sBWEBB14 (bBWEBB[14], BWEBB[14]);
buf sBWEBA15 (bBWEBA[15], BWEBA[15]);
buf sBWEBB15 (bBWEBB[15], BWEBB[15]);
buf sBWEBA16 (bBWEBA[16], BWEBA[16]);
buf sBWEBB16 (bBWEBB[16], BWEBB[16]);
buf sBWEBA17 (bBWEBA[17], BWEBA[17]);
buf sBWEBB17 (bBWEBB[17], BWEBB[17]);
buf sBWEBA18 (bBWEBA[18], BWEBA[18]);
buf sBWEBB18 (bBWEBB[18], BWEBB[18]);
buf sBWEBA19 (bBWEBA[19], BWEBA[19]);
buf sBWEBB19 (bBWEBB[19], BWEBB[19]);
buf sBWEBA20 (bBWEBA[20], BWEBA[20]);
buf sBWEBB20 (bBWEBB[20], BWEBB[20]);
buf sBWEBA21 (bBWEBA[21], BWEBA[21]);
buf sBWEBB21 (bBWEBB[21], BWEBB[21]);
buf sBWEBA22 (bBWEBA[22], BWEBA[22]);
buf sBWEBB22 (bBWEBB[22], BWEBB[22]);
buf sBWEBA23 (bBWEBA[23], BWEBA[23]);
buf sBWEBB23 (bBWEBB[23], BWEBB[23]);
buf sBWEBA24 (bBWEBA[24], BWEBA[24]);
buf sBWEBB24 (bBWEBB[24], BWEBB[24]);
buf sBWEBA25 (bBWEBA[25], BWEBA[25]);
buf sBWEBB25 (bBWEBB[25], BWEBB[25]);
buf sBWEBA26 (bBWEBA[26], BWEBA[26]);
buf sBWEBB26 (bBWEBB[26], BWEBB[26]);
buf sBWEBA27 (bBWEBA[27], BWEBA[27]);
buf sBWEBB27 (bBWEBB[27], BWEBB[27]);
buf sBWEBA28 (bBWEBA[28], BWEBA[28]);
buf sBWEBB28 (bBWEBB[28], BWEBB[28]);
buf sBWEBA29 (bBWEBA[29], BWEBA[29]);
buf sBWEBB29 (bBWEBB[29], BWEBB[29]);
buf sBWEBA30 (bBWEBA[30], BWEBA[30]);
buf sBWEBB30 (bBWEBB[30], BWEBB[30]);
buf sBWEBA31 (bBWEBA[31], BWEBA[31]);
buf sBWEBB31 (bBWEBB[31], BWEBB[31]);
// buf sBWEBA32 (bBWEBA[32], BWEBA[32]);
// buf sBWEBB32 (bBWEBB[32], BWEBB[32]);
// buf sBWEBA33 (bBWEBA[33], BWEBA[33]);
// buf sBWEBB33 (bBWEBB[33], BWEBB[33]);
// buf sBWEBA34 (bBWEBA[34], BWEBA[34]);
// buf sBWEBB34 (bBWEBB[34], BWEBB[34]);
// buf sBWEBA35 (bBWEBA[35], BWEBA[35]);
// buf sBWEBB35 (bBWEBB[35], BWEBB[35]);
// buf sBWEBA36 (bBWEBA[36], BWEBA[36]);
// buf sBWEBB36 (bBWEBB[36], BWEBB[36]);
// buf sBWEBA37 (bBWEBA[37], BWEBA[37]);
// buf sBWEBB37 (bBWEBB[37], BWEBB[37]);
// buf sBWEBA38 (bBWEBA[38], BWEBA[38]);
// buf sBWEBB38 (bBWEBB[38], BWEBB[38]);
// buf sBWEBA39 (bBWEBA[39], BWEBA[39]);
// buf sBWEBB39 (bBWEBB[39], BWEBB[39]);
// buf sBWEBA40 (bBWEBA[40], BWEBA[40]);
// buf sBWEBB40 (bBWEBB[40], BWEBB[40]);
// buf sBWEBA41 (bBWEBA[41], BWEBA[41]);
// buf sBWEBB41 (bBWEBB[41], BWEBB[41]);
// buf sBWEBA42 (bBWEBA[42], BWEBA[42]);
// buf sBWEBB42 (bBWEBB[42], BWEBB[42]);
// buf sBWEBA43 (bBWEBA[43], BWEBA[43]);
// buf sBWEBB43 (bBWEBB[43], BWEBB[43]);
// buf sBWEBA44 (bBWEBA[44], BWEBA[44]);
// buf sBWEBB44 (bBWEBB[44], BWEBB[44]);
// buf sBWEBA45 (bBWEBA[45], BWEBA[45]);
// buf sBWEBB45 (bBWEBB[45], BWEBB[45]);
// buf sBWEBA46 (bBWEBA[46], BWEBA[46]);
// buf sBWEBB46 (bBWEBB[46], BWEBB[46]);
// buf sBWEBA47 (bBWEBA[47], BWEBA[47]);
// buf sBWEBB47 (bBWEBB[47], BWEBB[47]);
// buf sBWEBA48 (bBWEBA[48], BWEBA[48]);
// buf sBWEBB48 (bBWEBB[48], BWEBB[48]);
// buf sBWEBA49 (bBWEBA[49], BWEBA[49]);
// buf sBWEBB49 (bBWEBB[49], BWEBB[49]);
// buf sBWEBA50 (bBWEBA[50], BWEBA[50]);
// buf sBWEBB50 (bBWEBB[50], BWEBB[50]);
// buf sBWEBA51 (bBWEBA[51], BWEBA[51]);
// buf sBWEBB51 (bBWEBB[51], BWEBB[51]);
// buf sBWEBA52 (bBWEBA[52], BWEBA[52]);
// buf sBWEBB52 (bBWEBB[52], BWEBB[52]);
// buf sBWEBA53 (bBWEBA[53], BWEBA[53]);
// buf sBWEBB53 (bBWEBB[53], BWEBB[53]);
// buf sBWEBA54 (bBWEBA[54], BWEBA[54]);
// buf sBWEBB54 (bBWEBB[54], BWEBB[54]);
// buf sBWEBA55 (bBWEBA[55], BWEBA[55]);
// buf sBWEBB55 (bBWEBB[55], BWEBB[55]);
// buf sBWEBA56 (bBWEBA[56], BWEBA[56]);
// buf sBWEBB56 (bBWEBB[56], BWEBB[56]);
// buf sBWEBA57 (bBWEBA[57], BWEBA[57]);
// buf sBWEBB57 (bBWEBB[57], BWEBB[57]);
// buf sBWEBA58 (bBWEBA[58], BWEBA[58]);
// buf sBWEBB58 (bBWEBB[58], BWEBB[58]);
// buf sBWEBA59 (bBWEBA[59], BWEBA[59]);
// buf sBWEBB59 (bBWEBB[59], BWEBB[59]);
// buf sBWEBA60 (bBWEBA[60], BWEBA[60]);
// buf sBWEBB60 (bBWEBB[60], BWEBB[60]);
// buf sBWEBA61 (bBWEBA[61], BWEBA[61]);
// buf sBWEBB61 (bBWEBB[61], BWEBB[61]);
// buf sBWEBA62 (bBWEBA[62], BWEBA[62]);
// buf sBWEBB62 (bBWEBB[62], BWEBB[62]);
// buf sBWEBA63 (bBWEBA[63], BWEBA[63]);
// buf sBWEBB63 (bBWEBB[63], BWEBB[63]);


// Input Controls
buf sWEBA (bWEBA, WEBA);
buf sWEBB (bWEBB, WEBB);
wire bSLP = 1'b0;
wire bDSLP = 1'b0;
wire bSD = 1'b0;
 
buf sCEBA (bCEBA, CEBA);
buf sCEBB (bCEBB, CEBB);
 
buf sCLKA (bCLKA, CLK);
buf sCLKB (bCLKB, CLK);
buf sAWT (bAWT, AWT);
assign bBIST = 1'b0;

buf sRTSEL0 (bRTSEL[0], RTSEL[0]);
buf sRTSEL1 (bRTSEL[1], RTSEL[1]);
buf sWTSEL0 (bWTSEL[0], WTSEL[0]);
buf sWTSEL1 (bWTSEL[1], WTSEL[1]);
buf sPTSEL0 (bPTSEL[0], PTSEL[0]);
buf sPTSEL1 (bPTSEL[1], PTSEL[1]);

// Output Data
buf sQA0 (QA[0], bbQA[0]);
buf sQA1 (QA[1], bbQA[1]);
buf sQA2 (QA[2], bbQA[2]);
buf sQA3 (QA[3], bbQA[3]);
buf sQA4 (QA[4], bbQA[4]);
buf sQA5 (QA[5], bbQA[5]);
buf sQA6 (QA[6], bbQA[6]);
buf sQA7 (QA[7], bbQA[7]);
buf sQA8 (QA[8], bbQA[8]);
buf sQA9 (QA[9], bbQA[9]);
buf sQA10 (QA[10], bbQA[10]);
buf sQA11 (QA[11], bbQA[11]);
buf sQA12 (QA[12], bbQA[12]);
buf sQA13 (QA[13], bbQA[13]);
buf sQA14 (QA[14], bbQA[14]);
buf sQA15 (QA[15], bbQA[15]);
buf sQA16 (QA[16], bbQA[16]);
buf sQA17 (QA[17], bbQA[17]);
buf sQA18 (QA[18], bbQA[18]);
buf sQA19 (QA[19], bbQA[19]);
buf sQA20 (QA[20], bbQA[20]);
buf sQA21 (QA[21], bbQA[21]);
buf sQA22 (QA[22], bbQA[22]);
buf sQA23 (QA[23], bbQA[23]);
buf sQA24 (QA[24], bbQA[24]);
buf sQA25 (QA[25], bbQA[25]);
buf sQA26 (QA[26], bbQA[26]);
buf sQA27 (QA[27], bbQA[27]);
buf sQA28 (QA[28], bbQA[28]);
buf sQA29 (QA[29], bbQA[29]);
buf sQA30 (QA[30], bbQA[30]);
buf sQA31 (QA[31], bbQA[31]);
// buf sQA32 (QA[32], bbQA[32]);
// buf sQA33 (QA[33], bbQA[33]);
// buf sQA34 (QA[34], bbQA[34]);
// buf sQA35 (QA[35], bbQA[35]);
// buf sQA36 (QA[36], bbQA[36]);
// buf sQA37 (QA[37], bbQA[37]);
// buf sQA38 (QA[38], bbQA[38]);
// buf sQA39 (QA[39], bbQA[39]);
// buf sQA40 (QA[40], bbQA[40]);
// buf sQA41 (QA[41], bbQA[41]);
// buf sQA42 (QA[42], bbQA[42]);
// buf sQA43 (QA[43], bbQA[43]);
// buf sQA44 (QA[44], bbQA[44]);
// buf sQA45 (QA[45], bbQA[45]);
// buf sQA46 (QA[46], bbQA[46]);
// buf sQA47 (QA[47], bbQA[47]);
// buf sQA48 (QA[48], bbQA[48]);
// buf sQA49 (QA[49], bbQA[49]);
// buf sQA50 (QA[50], bbQA[50]);
// buf sQA51 (QA[51], bbQA[51]);
// buf sQA52 (QA[52], bbQA[52]);
// buf sQA53 (QA[53], bbQA[53]);
// buf sQA54 (QA[54], bbQA[54]);
// buf sQA55 (QA[55], bbQA[55]);
// buf sQA56 (QA[56], bbQA[56]);
// buf sQA57 (QA[57], bbQA[57]);
// buf sQA58 (QA[58], bbQA[58]);
// buf sQA59 (QA[59], bbQA[59]);
// buf sQA60 (QA[60], bbQA[60]);
// buf sQA61 (QA[61], bbQA[61]);
// buf sQA62 (QA[62], bbQA[62]);
// buf sQA63 (QA[63], bbQA[63]);

buf sQB0 (QB[0], bbQB[0]);
buf sQB1 (QB[1], bbQB[1]);
buf sQB2 (QB[2], bbQB[2]);
buf sQB3 (QB[3], bbQB[3]);
buf sQB4 (QB[4], bbQB[4]);
buf sQB5 (QB[5], bbQB[5]);
buf sQB6 (QB[6], bbQB[6]);
buf sQB7 (QB[7], bbQB[7]);
buf sQB8 (QB[8], bbQB[8]);
buf sQB9 (QB[9], bbQB[9]);
buf sQB10 (QB[10], bbQB[10]);
buf sQB11 (QB[11], bbQB[11]);
buf sQB12 (QB[12], bbQB[12]);
buf sQB13 (QB[13], bbQB[13]);
buf sQB14 (QB[14], bbQB[14]);
buf sQB15 (QB[15], bbQB[15]);
buf sQB16 (QB[16], bbQB[16]);
buf sQB17 (QB[17], bbQB[17]);
buf sQB18 (QB[18], bbQB[18]);
buf sQB19 (QB[19], bbQB[19]);
buf sQB20 (QB[20], bbQB[20]);
buf sQB21 (QB[21], bbQB[21]);
buf sQB22 (QB[22], bbQB[22]);
buf sQB23 (QB[23], bbQB[23]);
buf sQB24 (QB[24], bbQB[24]);
buf sQB25 (QB[25], bbQB[25]);
buf sQB26 (QB[26], bbQB[26]);
buf sQB27 (QB[27], bbQB[27]);
buf sQB28 (QB[28], bbQB[28]);
buf sQB29 (QB[29], bbQB[29]);
buf sQB30 (QB[30], bbQB[30]);
buf sQB31 (QB[31], bbQB[31]);
// buf sQB32 (QB[32], bbQB[32]);
// buf sQB33 (QB[33], bbQB[33]);
// buf sQB34 (QB[34], bbQB[34]);
// buf sQB35 (QB[35], bbQB[35]);
// buf sQB36 (QB[36], bbQB[36]);
// buf sQB37 (QB[37], bbQB[37]);
// buf sQB38 (QB[38], bbQB[38]);
// buf sQB39 (QB[39], bbQB[39]);
// buf sQB40 (QB[40], bbQB[40]);
// buf sQB41 (QB[41], bbQB[41]);
// buf sQB42 (QB[42], bbQB[42]);
// buf sQB43 (QB[43], bbQB[43]);
// buf sQB44 (QB[44], bbQB[44]);
// buf sQB45 (QB[45], bbQB[45]);
// buf sQB46 (QB[46], bbQB[46]);
// buf sQB47 (QB[47], bbQB[47]);
// buf sQB48 (QB[48], bbQB[48]);
// buf sQB49 (QB[49], bbQB[49]);
// buf sQB50 (QB[50], bbQB[50]);
// buf sQB51 (QB[51], bbQB[51]);
// buf sQB52 (QB[52], bbQB[52]);
// buf sQB53 (QB[53], bbQB[53]);
// buf sQB54 (QB[54], bbQB[54]);
// buf sQB55 (QB[55], bbQB[55]);
// buf sQB56 (QB[56], bbQB[56]);
// buf sQB57 (QB[57], bbQB[57]);
// buf sQB58 (QB[58], bbQB[58]);
// buf sQB59 (QB[59], bbQB[59]);
// buf sQB60 (QB[60], bbQB[60]);
// buf sQB61 (QB[61], bbQB[61]);
// buf sQB62 (QB[62], bbQB[62]);
// buf sQB63 (QB[63], bbQB[63]);

assign bbQA=bQA;
assign bbQB=bQB;

//and sWEA (WEA, !bWEBA, !bCEBA);
//and sWEB (WEB, !bWEBB, !bCEBB);
assign WEA = !bSLP & !bDSLP & !bSD & !bCEBA & !bWEBA;
assign WEB = !bSLP & !bDSLP & !bSD & !bCEBB & !bWEBB;

//buf sCSA (CSA, !bCEBA);
//buf sCSB (CSB, !bCEBB);
assign CSA = !bSLP & !bDSLP & !bSD & !bCEBA;
assign CSB = !bSLP & !bDSLP & !bSD & !bCEBB;

wire check_noidle_b = ~CEBBL & ~bSD & ~bDSLP & ~bSLP;
wire check_idle_b = CEBBL & ~bSD & ~bDSLP & ~bSLP;
wire check_noidle_a = ~CEBAL & ~bSD & ~bDSLP & ~bSLP;
wire check_idle_a = CEBAL & ~bSD & ~bDSLP & ~bSLP;
wire check_noidle_norm_b = check_noidle_b & ~bBIST;
wire check_noidle_bist_b = check_noidle_b & bBIST;
wire check_idle_norm_b = check_idle_b & ~bBIST;
wire check_idle_bist_b = check_idle_b & bBIST;
wire check_noidle_norm_a = check_noidle_a & ~bBIST;
wire check_noidle_bist_a = check_noidle_a & bBIST;
wire check_idle_norm_a = check_idle_a & !bBIST;
wire check_idle_bist_a = check_idle_a & bBIST;

wire check_ceb  = (~iCEBA | ~iCEBB) & ~bSD & ~bDSLP & ~bSLP;
wire check_ceba = ~iCEBA & ~bSD & ~bDSLP & ~bSLP;
wire check_cebb = ~iCEBB & ~bSD & ~bDSLP & ~bSLP;
wire check_cebm = (~iCEBA | ~iCEBB) & ~bSD & ~bDSLP & ~bSLP;
wire check_ceb_a  = ~iCEBA & iCEBB & ~bSD & ~bDSLP & ~bSLP;
wire check_ceb_b  = iCEBA & ~iCEBB & ~bSD & ~bDSLP & ~bSLP;
wire check_ceb_ab = ~iCEBA & ~iCEBB & ~bSD & ~bDSLP & ~bSLP;





wire check_slp = !bSD & !bDSLP;
wire check_dslp = !bSD & !bSLP;


`ifdef TSMC_CM_UNIT_DELAY
`else
specify
    specparam PATHPULSE$ = ( 0, 0.001 );

specparam
tckl = 0.1070300,
tckh = 0.1070300,
tcyc  = 0.7600950,


taas = 0.0668700,
taah = 0.0705000,
tdas = 0.0124200,
tdah = 0.0855700,
tcas = 0.0943700,
tcah = 0.0874400,
twas = 0.0929400,
twah = 0.0695100,
tbwas = 0.0145100,
tbwah = 0.0851300,

tabs = 0.0100000,
tabh = 0.0874400,
tdbs = 0.0100000,
tdbh = 0.1226400,
tcbs = 0.0943700,
tcbh = 0.0874400,
twbs = 0.0100000,
twbh = 0.0874400,
tbwbs = 0.0100000,
tbwbh = 0.1225300,

ttests = 0.760,
ttesth = 0.760,
tawtq = 0.1963050,
tawtqh = 0.1063350,
tdq = 0.1542150,
tdqh = 0.0810050,
tbwebq = 0.1498450,
tbwebqh = 0.0872100,
tcda   = 0.3654880,
tcdb   = 0.7442400,
`ifdef TSMC_CM_READ_X_SQUASHING
tholda    = 0.3654880,
tholdb    = 0.7442400;
`else
tholda    = 0.2497300,
tholdb    = 0.5610850;
`endif

// tckl = 0,
// tckh = 0,
// tcyc  = 0,


// taas = 0,
// taah = 0,
// tdas = 0,
// tdah = 0,
// tcas = 0,
// tcah = 0,
// twas = 0,
// twah = 0,
// tbwas = 0,
// tbwah = 0,

// tabs = 0,
// tabh = 0,
// tdbs = 0,
// tdbh = 0,
// tcbs = 0,
// tcbh = 0,
// twbs = 0,
// twbh = 0,
// tbwbs = 0,
// tbwbh = 0,

// ttests = 0,
// ttesth = 0,
// tawtq = 0,
// tawtqh = 0,
// tdq = 0,
// tdqh = 0,
// tbwebq = 0,
// tbwebqh = 0,
// tcda   = 0,
// tcdb   = 0,
// `ifdef TSMC_CM_READ_X_SQUASHING
// tholda    = 0,
// tholdb    = 0;
// `else
// tholda    = 0,
// tholdb    = 0;
// `endif

    $setuphold (posedge CLK &&& check_noidle_a, posedge RTSEL[0], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_a, negedge RTSEL[0], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_a, posedge RTSEL[0], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_a, negedge RTSEL[0], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_a, posedge RTSEL[1], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_a, negedge RTSEL[1], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_a, posedge RTSEL[1], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_a, negedge RTSEL[1], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_a, posedge WTSEL[0], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_a, negedge WTSEL[0], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_a, posedge WTSEL[0], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_a, negedge WTSEL[0], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_a, posedge WTSEL[1], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_a, negedge WTSEL[1], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_a, posedge WTSEL[1], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_a, negedge WTSEL[1], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_a, posedge PTSEL[0], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_a, negedge PTSEL[0], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_a, posedge PTSEL[0], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_a, negedge PTSEL[0], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_a, posedge PTSEL[1], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_a, negedge PTSEL[1], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_a, posedge PTSEL[1], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_a, negedge PTSEL[1], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_b, posedge RTSEL[0], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_b, negedge RTSEL[0], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_b, posedge RTSEL[0], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_b, negedge RTSEL[0], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_b, posedge RTSEL[1], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_b, negedge RTSEL[1], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_b, posedge RTSEL[1], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_b, negedge RTSEL[1], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_b, posedge WTSEL[0], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_b, negedge WTSEL[0], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_b, posedge WTSEL[0], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_b, negedge WTSEL[0], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_b, posedge WTSEL[1], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_b, negedge WTSEL[1], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_b, posedge WTSEL[1], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_b, negedge WTSEL[1], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_b, posedge PTSEL[0], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_b, negedge PTSEL[0], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_b, posedge PTSEL[0], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_b, negedge PTSEL[0], 0, ttesth, valid_testpin);
    $setuphold (posedge CLK &&& check_noidle_b, posedge PTSEL[1], ttests, 0, valid_testpin); 
    $setuphold (posedge CLK &&& check_noidle_b, negedge PTSEL[1], ttests, 0, valid_testpin);
    $setuphold (posedge CLK &&& check_idle_b, posedge PTSEL[1], 0, ttesth, valid_testpin); 
    $setuphold (posedge CLK &&& check_idle_b, negedge PTSEL[1], 0, ttesth, valid_testpin);










    $setuphold (posedge CLK &&& CSA, posedge AA[0], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[0], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[0], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[0], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[1], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[1], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[1], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[1], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[2], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[2], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[2], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[2], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[3], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[3], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[3], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[3], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[4], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[4], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[4], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[4], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[5], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[5], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[5], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[5], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[6], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[6], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[6], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[6], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[7], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[7], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[7], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[7], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSA, posedge AA[8], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSA, negedge AA[8], taas, taah, valid_aa);
    $setuphold (posedge CLK &&& CSB, posedge AB[8], tabs, tabh, valid_ab);
    $setuphold (posedge CLK &&& CSB, negedge AB[8], tabs, tabh, valid_ab);

    $setuphold (posedge CLK &&& WEA, posedge DA[0], tdas, tdah, valid_da0);
    $setuphold (posedge CLK &&& WEA, negedge DA[0], tdas, tdah, valid_da0);
    $setuphold (posedge CLK &&& WEB, posedge DB[0], tdbs, tdbh, valid_db0);
    $setuphold (posedge CLK &&& WEB, negedge DB[0], tdbs, tdbh, valid_db0);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[0], tbwas, tbwah, valid_bwa0);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[0], tbwas, tbwah, valid_bwa0);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[0], tbwbs, tbwbh, valid_bwb0);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[0], tbwbs, tbwbh, valid_bwb0);
    $setuphold (posedge CLK &&& WEA, posedge DA[1], tdas, tdah, valid_da1);
    $setuphold (posedge CLK &&& WEA, negedge DA[1], tdas, tdah, valid_da1);
    $setuphold (posedge CLK &&& WEB, posedge DB[1], tdbs, tdbh, valid_db1);
    $setuphold (posedge CLK &&& WEB, negedge DB[1], tdbs, tdbh, valid_db1);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[1], tbwas, tbwah, valid_bwa1);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[1], tbwas, tbwah, valid_bwa1);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[1], tbwbs, tbwbh, valid_bwb1);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[1], tbwbs, tbwbh, valid_bwb1);
    $setuphold (posedge CLK &&& WEA, posedge DA[2], tdas, tdah, valid_da2);
    $setuphold (posedge CLK &&& WEA, negedge DA[2], tdas, tdah, valid_da2);
    $setuphold (posedge CLK &&& WEB, posedge DB[2], tdbs, tdbh, valid_db2);
    $setuphold (posedge CLK &&& WEB, negedge DB[2], tdbs, tdbh, valid_db2);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[2], tbwas, tbwah, valid_bwa2);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[2], tbwas, tbwah, valid_bwa2);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[2], tbwbs, tbwbh, valid_bwb2);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[2], tbwbs, tbwbh, valid_bwb2);
    $setuphold (posedge CLK &&& WEA, posedge DA[3], tdas, tdah, valid_da3);
    $setuphold (posedge CLK &&& WEA, negedge DA[3], tdas, tdah, valid_da3);
    $setuphold (posedge CLK &&& WEB, posedge DB[3], tdbs, tdbh, valid_db3);
    $setuphold (posedge CLK &&& WEB, negedge DB[3], tdbs, tdbh, valid_db3);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[3], tbwas, tbwah, valid_bwa3);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[3], tbwas, tbwah, valid_bwa3);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[3], tbwbs, tbwbh, valid_bwb3);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[3], tbwbs, tbwbh, valid_bwb3);
    $setuphold (posedge CLK &&& WEA, posedge DA[4], tdas, tdah, valid_da4);
    $setuphold (posedge CLK &&& WEA, negedge DA[4], tdas, tdah, valid_da4);
    $setuphold (posedge CLK &&& WEB, posedge DB[4], tdbs, tdbh, valid_db4);
    $setuphold (posedge CLK &&& WEB, negedge DB[4], tdbs, tdbh, valid_db4);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[4], tbwas, tbwah, valid_bwa4);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[4], tbwas, tbwah, valid_bwa4);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[4], tbwbs, tbwbh, valid_bwb4);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[4], tbwbs, tbwbh, valid_bwb4);
    $setuphold (posedge CLK &&& WEA, posedge DA[5], tdas, tdah, valid_da5);
    $setuphold (posedge CLK &&& WEA, negedge DA[5], tdas, tdah, valid_da5);
    $setuphold (posedge CLK &&& WEB, posedge DB[5], tdbs, tdbh, valid_db5);
    $setuphold (posedge CLK &&& WEB, negedge DB[5], tdbs, tdbh, valid_db5);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[5], tbwas, tbwah, valid_bwa5);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[5], tbwas, tbwah, valid_bwa5);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[5], tbwbs, tbwbh, valid_bwb5);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[5], tbwbs, tbwbh, valid_bwb5);
    $setuphold (posedge CLK &&& WEA, posedge DA[6], tdas, tdah, valid_da6);
    $setuphold (posedge CLK &&& WEA, negedge DA[6], tdas, tdah, valid_da6);
    $setuphold (posedge CLK &&& WEB, posedge DB[6], tdbs, tdbh, valid_db6);
    $setuphold (posedge CLK &&& WEB, negedge DB[6], tdbs, tdbh, valid_db6);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[6], tbwas, tbwah, valid_bwa6);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[6], tbwas, tbwah, valid_bwa6);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[6], tbwbs, tbwbh, valid_bwb6);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[6], tbwbs, tbwbh, valid_bwb6);
    $setuphold (posedge CLK &&& WEA, posedge DA[7], tdas, tdah, valid_da7);
    $setuphold (posedge CLK &&& WEA, negedge DA[7], tdas, tdah, valid_da7);
    $setuphold (posedge CLK &&& WEB, posedge DB[7], tdbs, tdbh, valid_db7);
    $setuphold (posedge CLK &&& WEB, negedge DB[7], tdbs, tdbh, valid_db7);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[7], tbwas, tbwah, valid_bwa7);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[7], tbwas, tbwah, valid_bwa7);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[7], tbwbs, tbwbh, valid_bwb7);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[7], tbwbs, tbwbh, valid_bwb7);
    $setuphold (posedge CLK &&& WEA, posedge DA[8], tdas, tdah, valid_da8);
    $setuphold (posedge CLK &&& WEA, negedge DA[8], tdas, tdah, valid_da8);
    $setuphold (posedge CLK &&& WEB, posedge DB[8], tdbs, tdbh, valid_db8);
    $setuphold (posedge CLK &&& WEB, negedge DB[8], tdbs, tdbh, valid_db8);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[8], tbwas, tbwah, valid_bwa8);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[8], tbwas, tbwah, valid_bwa8);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[8], tbwbs, tbwbh, valid_bwb8);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[8], tbwbs, tbwbh, valid_bwb8);
    $setuphold (posedge CLK &&& WEA, posedge DA[9], tdas, tdah, valid_da9);
    $setuphold (posedge CLK &&& WEA, negedge DA[9], tdas, tdah, valid_da9);
    $setuphold (posedge CLK &&& WEB, posedge DB[9], tdbs, tdbh, valid_db9);
    $setuphold (posedge CLK &&& WEB, negedge DB[9], tdbs, tdbh, valid_db9);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[9], tbwas, tbwah, valid_bwa9);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[9], tbwas, tbwah, valid_bwa9);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[9], tbwbs, tbwbh, valid_bwb9);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[9], tbwbs, tbwbh, valid_bwb9);
    $setuphold (posedge CLK &&& WEA, posedge DA[10], tdas, tdah, valid_da10);
    $setuphold (posedge CLK &&& WEA, negedge DA[10], tdas, tdah, valid_da10);
    $setuphold (posedge CLK &&& WEB, posedge DB[10], tdbs, tdbh, valid_db10);
    $setuphold (posedge CLK &&& WEB, negedge DB[10], tdbs, tdbh, valid_db10);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[10], tbwas, tbwah, valid_bwa10);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[10], tbwas, tbwah, valid_bwa10);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[10], tbwbs, tbwbh, valid_bwb10);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[10], tbwbs, tbwbh, valid_bwb10);
    $setuphold (posedge CLK &&& WEA, posedge DA[11], tdas, tdah, valid_da11);
    $setuphold (posedge CLK &&& WEA, negedge DA[11], tdas, tdah, valid_da11);
    $setuphold (posedge CLK &&& WEB, posedge DB[11], tdbs, tdbh, valid_db11);
    $setuphold (posedge CLK &&& WEB, negedge DB[11], tdbs, tdbh, valid_db11);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[11], tbwas, tbwah, valid_bwa11);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[11], tbwas, tbwah, valid_bwa11);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[11], tbwbs, tbwbh, valid_bwb11);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[11], tbwbs, tbwbh, valid_bwb11);
    $setuphold (posedge CLK &&& WEA, posedge DA[12], tdas, tdah, valid_da12);
    $setuphold (posedge CLK &&& WEA, negedge DA[12], tdas, tdah, valid_da12);
    $setuphold (posedge CLK &&& WEB, posedge DB[12], tdbs, tdbh, valid_db12);
    $setuphold (posedge CLK &&& WEB, negedge DB[12], tdbs, tdbh, valid_db12);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[12], tbwas, tbwah, valid_bwa12);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[12], tbwas, tbwah, valid_bwa12);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[12], tbwbs, tbwbh, valid_bwb12);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[12], tbwbs, tbwbh, valid_bwb12);
    $setuphold (posedge CLK &&& WEA, posedge DA[13], tdas, tdah, valid_da13);
    $setuphold (posedge CLK &&& WEA, negedge DA[13], tdas, tdah, valid_da13);
    $setuphold (posedge CLK &&& WEB, posedge DB[13], tdbs, tdbh, valid_db13);
    $setuphold (posedge CLK &&& WEB, negedge DB[13], tdbs, tdbh, valid_db13);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[13], tbwas, tbwah, valid_bwa13);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[13], tbwas, tbwah, valid_bwa13);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[13], tbwbs, tbwbh, valid_bwb13);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[13], tbwbs, tbwbh, valid_bwb13);
    $setuphold (posedge CLK &&& WEA, posedge DA[14], tdas, tdah, valid_da14);
    $setuphold (posedge CLK &&& WEA, negedge DA[14], tdas, tdah, valid_da14);
    $setuphold (posedge CLK &&& WEB, posedge DB[14], tdbs, tdbh, valid_db14);
    $setuphold (posedge CLK &&& WEB, negedge DB[14], tdbs, tdbh, valid_db14);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[14], tbwas, tbwah, valid_bwa14);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[14], tbwas, tbwah, valid_bwa14);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[14], tbwbs, tbwbh, valid_bwb14);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[14], tbwbs, tbwbh, valid_bwb14);
    $setuphold (posedge CLK &&& WEA, posedge DA[15], tdas, tdah, valid_da15);
    $setuphold (posedge CLK &&& WEA, negedge DA[15], tdas, tdah, valid_da15);
    $setuphold (posedge CLK &&& WEB, posedge DB[15], tdbs, tdbh, valid_db15);
    $setuphold (posedge CLK &&& WEB, negedge DB[15], tdbs, tdbh, valid_db15);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[15], tbwas, tbwah, valid_bwa15);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[15], tbwas, tbwah, valid_bwa15);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[15], tbwbs, tbwbh, valid_bwb15);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[15], tbwbs, tbwbh, valid_bwb15);
    $setuphold (posedge CLK &&& WEA, posedge DA[16], tdas, tdah, valid_da16);
    $setuphold (posedge CLK &&& WEA, negedge DA[16], tdas, tdah, valid_da16);
    $setuphold (posedge CLK &&& WEB, posedge DB[16], tdbs, tdbh, valid_db16);
    $setuphold (posedge CLK &&& WEB, negedge DB[16], tdbs, tdbh, valid_db16);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[16], tbwas, tbwah, valid_bwa16);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[16], tbwas, tbwah, valid_bwa16);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[16], tbwbs, tbwbh, valid_bwb16);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[16], tbwbs, tbwbh, valid_bwb16);
    $setuphold (posedge CLK &&& WEA, posedge DA[17], tdas, tdah, valid_da17);
    $setuphold (posedge CLK &&& WEA, negedge DA[17], tdas, tdah, valid_da17);
    $setuphold (posedge CLK &&& WEB, posedge DB[17], tdbs, tdbh, valid_db17);
    $setuphold (posedge CLK &&& WEB, negedge DB[17], tdbs, tdbh, valid_db17);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[17], tbwas, tbwah, valid_bwa17);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[17], tbwas, tbwah, valid_bwa17);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[17], tbwbs, tbwbh, valid_bwb17);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[17], tbwbs, tbwbh, valid_bwb17);
    $setuphold (posedge CLK &&& WEA, posedge DA[18], tdas, tdah, valid_da18);
    $setuphold (posedge CLK &&& WEA, negedge DA[18], tdas, tdah, valid_da18);
    $setuphold (posedge CLK &&& WEB, posedge DB[18], tdbs, tdbh, valid_db18);
    $setuphold (posedge CLK &&& WEB, negedge DB[18], tdbs, tdbh, valid_db18);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[18], tbwas, tbwah, valid_bwa18);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[18], tbwas, tbwah, valid_bwa18);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[18], tbwbs, tbwbh, valid_bwb18);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[18], tbwbs, tbwbh, valid_bwb18);
    $setuphold (posedge CLK &&& WEA, posedge DA[19], tdas, tdah, valid_da19);
    $setuphold (posedge CLK &&& WEA, negedge DA[19], tdas, tdah, valid_da19);
    $setuphold (posedge CLK &&& WEB, posedge DB[19], tdbs, tdbh, valid_db19);
    $setuphold (posedge CLK &&& WEB, negedge DB[19], tdbs, tdbh, valid_db19);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[19], tbwas, tbwah, valid_bwa19);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[19], tbwas, tbwah, valid_bwa19);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[19], tbwbs, tbwbh, valid_bwb19);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[19], tbwbs, tbwbh, valid_bwb19);
    $setuphold (posedge CLK &&& WEA, posedge DA[20], tdas, tdah, valid_da20);
    $setuphold (posedge CLK &&& WEA, negedge DA[20], tdas, tdah, valid_da20);
    $setuphold (posedge CLK &&& WEB, posedge DB[20], tdbs, tdbh, valid_db20);
    $setuphold (posedge CLK &&& WEB, negedge DB[20], tdbs, tdbh, valid_db20);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[20], tbwas, tbwah, valid_bwa20);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[20], tbwas, tbwah, valid_bwa20);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[20], tbwbs, tbwbh, valid_bwb20);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[20], tbwbs, tbwbh, valid_bwb20);
    $setuphold (posedge CLK &&& WEA, posedge DA[21], tdas, tdah, valid_da21);
    $setuphold (posedge CLK &&& WEA, negedge DA[21], tdas, tdah, valid_da21);
    $setuphold (posedge CLK &&& WEB, posedge DB[21], tdbs, tdbh, valid_db21);
    $setuphold (posedge CLK &&& WEB, negedge DB[21], tdbs, tdbh, valid_db21);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[21], tbwas, tbwah, valid_bwa21);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[21], tbwas, tbwah, valid_bwa21);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[21], tbwbs, tbwbh, valid_bwb21);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[21], tbwbs, tbwbh, valid_bwb21);
    $setuphold (posedge CLK &&& WEA, posedge DA[22], tdas, tdah, valid_da22);
    $setuphold (posedge CLK &&& WEA, negedge DA[22], tdas, tdah, valid_da22);
    $setuphold (posedge CLK &&& WEB, posedge DB[22], tdbs, tdbh, valid_db22);
    $setuphold (posedge CLK &&& WEB, negedge DB[22], tdbs, tdbh, valid_db22);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[22], tbwas, tbwah, valid_bwa22);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[22], tbwas, tbwah, valid_bwa22);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[22], tbwbs, tbwbh, valid_bwb22);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[22], tbwbs, tbwbh, valid_bwb22);
    $setuphold (posedge CLK &&& WEA, posedge DA[23], tdas, tdah, valid_da23);
    $setuphold (posedge CLK &&& WEA, negedge DA[23], tdas, tdah, valid_da23);
    $setuphold (posedge CLK &&& WEB, posedge DB[23], tdbs, tdbh, valid_db23);
    $setuphold (posedge CLK &&& WEB, negedge DB[23], tdbs, tdbh, valid_db23);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[23], tbwas, tbwah, valid_bwa23);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[23], tbwas, tbwah, valid_bwa23);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[23], tbwbs, tbwbh, valid_bwb23);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[23], tbwbs, tbwbh, valid_bwb23);
    $setuphold (posedge CLK &&& WEA, posedge DA[24], tdas, tdah, valid_da24);
    $setuphold (posedge CLK &&& WEA, negedge DA[24], tdas, tdah, valid_da24);
    $setuphold (posedge CLK &&& WEB, posedge DB[24], tdbs, tdbh, valid_db24);
    $setuphold (posedge CLK &&& WEB, negedge DB[24], tdbs, tdbh, valid_db24);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[24], tbwas, tbwah, valid_bwa24);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[24], tbwas, tbwah, valid_bwa24);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[24], tbwbs, tbwbh, valid_bwb24);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[24], tbwbs, tbwbh, valid_bwb24);
    $setuphold (posedge CLK &&& WEA, posedge DA[25], tdas, tdah, valid_da25);
    $setuphold (posedge CLK &&& WEA, negedge DA[25], tdas, tdah, valid_da25);
    $setuphold (posedge CLK &&& WEB, posedge DB[25], tdbs, tdbh, valid_db25);
    $setuphold (posedge CLK &&& WEB, negedge DB[25], tdbs, tdbh, valid_db25);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[25], tbwas, tbwah, valid_bwa25);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[25], tbwas, tbwah, valid_bwa25);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[25], tbwbs, tbwbh, valid_bwb25);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[25], tbwbs, tbwbh, valid_bwb25);
    $setuphold (posedge CLK &&& WEA, posedge DA[26], tdas, tdah, valid_da26);
    $setuphold (posedge CLK &&& WEA, negedge DA[26], tdas, tdah, valid_da26);
    $setuphold (posedge CLK &&& WEB, posedge DB[26], tdbs, tdbh, valid_db26);
    $setuphold (posedge CLK &&& WEB, negedge DB[26], tdbs, tdbh, valid_db26);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[26], tbwas, tbwah, valid_bwa26);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[26], tbwas, tbwah, valid_bwa26);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[26], tbwbs, tbwbh, valid_bwb26);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[26], tbwbs, tbwbh, valid_bwb26);
    $setuphold (posedge CLK &&& WEA, posedge DA[27], tdas, tdah, valid_da27);
    $setuphold (posedge CLK &&& WEA, negedge DA[27], tdas, tdah, valid_da27);
    $setuphold (posedge CLK &&& WEB, posedge DB[27], tdbs, tdbh, valid_db27);
    $setuphold (posedge CLK &&& WEB, negedge DB[27], tdbs, tdbh, valid_db27);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[27], tbwas, tbwah, valid_bwa27);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[27], tbwas, tbwah, valid_bwa27);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[27], tbwbs, tbwbh, valid_bwb27);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[27], tbwbs, tbwbh, valid_bwb27);
    $setuphold (posedge CLK &&& WEA, posedge DA[28], tdas, tdah, valid_da28);
    $setuphold (posedge CLK &&& WEA, negedge DA[28], tdas, tdah, valid_da28);
    $setuphold (posedge CLK &&& WEB, posedge DB[28], tdbs, tdbh, valid_db28);
    $setuphold (posedge CLK &&& WEB, negedge DB[28], tdbs, tdbh, valid_db28);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[28], tbwas, tbwah, valid_bwa28);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[28], tbwas, tbwah, valid_bwa28);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[28], tbwbs, tbwbh, valid_bwb28);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[28], tbwbs, tbwbh, valid_bwb28);
    $setuphold (posedge CLK &&& WEA, posedge DA[29], tdas, tdah, valid_da29);
    $setuphold (posedge CLK &&& WEA, negedge DA[29], tdas, tdah, valid_da29);
    $setuphold (posedge CLK &&& WEB, posedge DB[29], tdbs, tdbh, valid_db29);
    $setuphold (posedge CLK &&& WEB, negedge DB[29], tdbs, tdbh, valid_db29);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[29], tbwas, tbwah, valid_bwa29);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[29], tbwas, tbwah, valid_bwa29);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[29], tbwbs, tbwbh, valid_bwb29);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[29], tbwbs, tbwbh, valid_bwb29);
    $setuphold (posedge CLK &&& WEA, posedge DA[30], tdas, tdah, valid_da30);
    $setuphold (posedge CLK &&& WEA, negedge DA[30], tdas, tdah, valid_da30);
    $setuphold (posedge CLK &&& WEB, posedge DB[30], tdbs, tdbh, valid_db30);
    $setuphold (posedge CLK &&& WEB, negedge DB[30], tdbs, tdbh, valid_db30);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[30], tbwas, tbwah, valid_bwa30);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[30], tbwas, tbwah, valid_bwa30);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[30], tbwbs, tbwbh, valid_bwb30);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[30], tbwbs, tbwbh, valid_bwb30);
    $setuphold (posedge CLK &&& WEA, posedge DA[31], tdas, tdah, valid_da31);
    $setuphold (posedge CLK &&& WEA, negedge DA[31], tdas, tdah, valid_da31);
    $setuphold (posedge CLK &&& WEB, posedge DB[31], tdbs, tdbh, valid_db31);
    $setuphold (posedge CLK &&& WEB, negedge DB[31], tdbs, tdbh, valid_db31);
 
    $setuphold (posedge CLK &&& WEA, posedge BWEBA[31], tbwas, tbwah, valid_bwa31);
    $setuphold (posedge CLK &&& WEA, negedge BWEBA[31], tbwas, tbwah, valid_bwa31);
    $setuphold (posedge CLK &&& WEB, posedge BWEBB[31], tbwbs, tbwbh, valid_bwb31);
    $setuphold (posedge CLK &&& WEB, negedge BWEBB[31], tbwbs, tbwbh, valid_bwb31);
    // $setuphold (posedge CLK &&& WEA, posedge DA[32], tdas, tdah, valid_da32);
    // $setuphold (posedge CLK &&& WEA, negedge DA[32], tdas, tdah, valid_da32);
    // $setuphold (posedge CLK &&& WEB, posedge DB[32], tdbs, tdbh, valid_db32);
    // $setuphold (posedge CLK &&& WEB, negedge DB[32], tdbs, tdbh, valid_db32);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[32], tbwas, tbwah, valid_bwa32);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[32], tbwas, tbwah, valid_bwa32);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[32], tbwbs, tbwbh, valid_bwb32);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[32], tbwbs, tbwbh, valid_bwb32);
    // $setuphold (posedge CLK &&& WEA, posedge DA[33], tdas, tdah, valid_da33);
    // $setuphold (posedge CLK &&& WEA, negedge DA[33], tdas, tdah, valid_da33);
    // $setuphold (posedge CLK &&& WEB, posedge DB[33], tdbs, tdbh, valid_db33);
    // $setuphold (posedge CLK &&& WEB, negedge DB[33], tdbs, tdbh, valid_db33);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[33], tbwas, tbwah, valid_bwa33);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[33], tbwas, tbwah, valid_bwa33);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[33], tbwbs, tbwbh, valid_bwb33);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[33], tbwbs, tbwbh, valid_bwb33);
    // $setuphold (posedge CLK &&& WEA, posedge DA[34], tdas, tdah, valid_da34);
    // $setuphold (posedge CLK &&& WEA, negedge DA[34], tdas, tdah, valid_da34);
    // $setuphold (posedge CLK &&& WEB, posedge DB[34], tdbs, tdbh, valid_db34);
    // $setuphold (posedge CLK &&& WEB, negedge DB[34], tdbs, tdbh, valid_db34);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[34], tbwas, tbwah, valid_bwa34);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[34], tbwas, tbwah, valid_bwa34);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[34], tbwbs, tbwbh, valid_bwb34);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[34], tbwbs, tbwbh, valid_bwb34);
    // $setuphold (posedge CLK &&& WEA, posedge DA[35], tdas, tdah, valid_da35);
    // $setuphold (posedge CLK &&& WEA, negedge DA[35], tdas, tdah, valid_da35);
    // $setuphold (posedge CLK &&& WEB, posedge DB[35], tdbs, tdbh, valid_db35);
    // $setuphold (posedge CLK &&& WEB, negedge DB[35], tdbs, tdbh, valid_db35);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[35], tbwas, tbwah, valid_bwa35);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[35], tbwas, tbwah, valid_bwa35);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[35], tbwbs, tbwbh, valid_bwb35);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[35], tbwbs, tbwbh, valid_bwb35);
    // $setuphold (posedge CLK &&& WEA, posedge DA[36], tdas, tdah, valid_da36);
    // $setuphold (posedge CLK &&& WEA, negedge DA[36], tdas, tdah, valid_da36);
    // $setuphold (posedge CLK &&& WEB, posedge DB[36], tdbs, tdbh, valid_db36);
    // $setuphold (posedge CLK &&& WEB, negedge DB[36], tdbs, tdbh, valid_db36);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[36], tbwas, tbwah, valid_bwa36);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[36], tbwas, tbwah, valid_bwa36);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[36], tbwbs, tbwbh, valid_bwb36);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[36], tbwbs, tbwbh, valid_bwb36);
    // $setuphold (posedge CLK &&& WEA, posedge DA[37], tdas, tdah, valid_da37);
    // $setuphold (posedge CLK &&& WEA, negedge DA[37], tdas, tdah, valid_da37);
    // $setuphold (posedge CLK &&& WEB, posedge DB[37], tdbs, tdbh, valid_db37);
    // $setuphold (posedge CLK &&& WEB, negedge DB[37], tdbs, tdbh, valid_db37);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[37], tbwas, tbwah, valid_bwa37);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[37], tbwas, tbwah, valid_bwa37);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[37], tbwbs, tbwbh, valid_bwb37);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[37], tbwbs, tbwbh, valid_bwb37);
    // $setuphold (posedge CLK &&& WEA, posedge DA[38], tdas, tdah, valid_da38);
    // $setuphold (posedge CLK &&& WEA, negedge DA[38], tdas, tdah, valid_da38);
    // $setuphold (posedge CLK &&& WEB, posedge DB[38], tdbs, tdbh, valid_db38);
    // $setuphold (posedge CLK &&& WEB, negedge DB[38], tdbs, tdbh, valid_db38);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[38], tbwas, tbwah, valid_bwa38);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[38], tbwas, tbwah, valid_bwa38);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[38], tbwbs, tbwbh, valid_bwb38);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[38], tbwbs, tbwbh, valid_bwb38);
    // $setuphold (posedge CLK &&& WEA, posedge DA[39], tdas, tdah, valid_da39);
    // $setuphold (posedge CLK &&& WEA, negedge DA[39], tdas, tdah, valid_da39);
    // $setuphold (posedge CLK &&& WEB, posedge DB[39], tdbs, tdbh, valid_db39);
    // $setuphold (posedge CLK &&& WEB, negedge DB[39], tdbs, tdbh, valid_db39);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[39], tbwas, tbwah, valid_bwa39);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[39], tbwas, tbwah, valid_bwa39);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[39], tbwbs, tbwbh, valid_bwb39);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[39], tbwbs, tbwbh, valid_bwb39);
    // $setuphold (posedge CLK &&& WEA, posedge DA[40], tdas, tdah, valid_da40);
    // $setuphold (posedge CLK &&& WEA, negedge DA[40], tdas, tdah, valid_da40);
    // $setuphold (posedge CLK &&& WEB, posedge DB[40], tdbs, tdbh, valid_db40);
    // $setuphold (posedge CLK &&& WEB, negedge DB[40], tdbs, tdbh, valid_db40);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[40], tbwas, tbwah, valid_bwa40);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[40], tbwas, tbwah, valid_bwa40);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[40], tbwbs, tbwbh, valid_bwb40);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[40], tbwbs, tbwbh, valid_bwb40);
    // $setuphold (posedge CLK &&& WEA, posedge DA[41], tdas, tdah, valid_da41);
    // $setuphold (posedge CLK &&& WEA, negedge DA[41], tdas, tdah, valid_da41);
    // $setuphold (posedge CLK &&& WEB, posedge DB[41], tdbs, tdbh, valid_db41);
    // $setuphold (posedge CLK &&& WEB, negedge DB[41], tdbs, tdbh, valid_db41);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[41], tbwas, tbwah, valid_bwa41);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[41], tbwas, tbwah, valid_bwa41);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[41], tbwbs, tbwbh, valid_bwb41);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[41], tbwbs, tbwbh, valid_bwb41);
    // $setuphold (posedge CLK &&& WEA, posedge DA[42], tdas, tdah, valid_da42);
    // $setuphold (posedge CLK &&& WEA, negedge DA[42], tdas, tdah, valid_da42);
    // $setuphold (posedge CLK &&& WEB, posedge DB[42], tdbs, tdbh, valid_db42);
    // $setuphold (posedge CLK &&& WEB, negedge DB[42], tdbs, tdbh, valid_db42);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[42], tbwas, tbwah, valid_bwa42);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[42], tbwas, tbwah, valid_bwa42);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[42], tbwbs, tbwbh, valid_bwb42);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[42], tbwbs, tbwbh, valid_bwb42);
    // $setuphold (posedge CLK &&& WEA, posedge DA[43], tdas, tdah, valid_da43);
    // $setuphold (posedge CLK &&& WEA, negedge DA[43], tdas, tdah, valid_da43);
    // $setuphold (posedge CLK &&& WEB, posedge DB[43], tdbs, tdbh, valid_db43);
    // $setuphold (posedge CLK &&& WEB, negedge DB[43], tdbs, tdbh, valid_db43);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[43], tbwas, tbwah, valid_bwa43);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[43], tbwas, tbwah, valid_bwa43);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[43], tbwbs, tbwbh, valid_bwb43);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[43], tbwbs, tbwbh, valid_bwb43);
    // $setuphold (posedge CLK &&& WEA, posedge DA[44], tdas, tdah, valid_da44);
    // $setuphold (posedge CLK &&& WEA, negedge DA[44], tdas, tdah, valid_da44);
    // $setuphold (posedge CLK &&& WEB, posedge DB[44], tdbs, tdbh, valid_db44);
    // $setuphold (posedge CLK &&& WEB, negedge DB[44], tdbs, tdbh, valid_db44);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[44], tbwas, tbwah, valid_bwa44);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[44], tbwas, tbwah, valid_bwa44);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[44], tbwbs, tbwbh, valid_bwb44);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[44], tbwbs, tbwbh, valid_bwb44);
    // $setuphold (posedge CLK &&& WEA, posedge DA[45], tdas, tdah, valid_da45);
    // $setuphold (posedge CLK &&& WEA, negedge DA[45], tdas, tdah, valid_da45);
    // $setuphold (posedge CLK &&& WEB, posedge DB[45], tdbs, tdbh, valid_db45);
    // $setuphold (posedge CLK &&& WEB, negedge DB[45], tdbs, tdbh, valid_db45);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[45], tbwas, tbwah, valid_bwa45);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[45], tbwas, tbwah, valid_bwa45);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[45], tbwbs, tbwbh, valid_bwb45);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[45], tbwbs, tbwbh, valid_bwb45);
    // $setuphold (posedge CLK &&& WEA, posedge DA[46], tdas, tdah, valid_da46);
    // $setuphold (posedge CLK &&& WEA, negedge DA[46], tdas, tdah, valid_da46);
    // $setuphold (posedge CLK &&& WEB, posedge DB[46], tdbs, tdbh, valid_db46);
    // $setuphold (posedge CLK &&& WEB, negedge DB[46], tdbs, tdbh, valid_db46);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[46], tbwas, tbwah, valid_bwa46);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[46], tbwas, tbwah, valid_bwa46);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[46], tbwbs, tbwbh, valid_bwb46);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[46], tbwbs, tbwbh, valid_bwb46);
    // $setuphold (posedge CLK &&& WEA, posedge DA[47], tdas, tdah, valid_da47);
    // $setuphold (posedge CLK &&& WEA, negedge DA[47], tdas, tdah, valid_da47);
    // $setuphold (posedge CLK &&& WEB, posedge DB[47], tdbs, tdbh, valid_db47);
    // $setuphold (posedge CLK &&& WEB, negedge DB[47], tdbs, tdbh, valid_db47);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[47], tbwas, tbwah, valid_bwa47);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[47], tbwas, tbwah, valid_bwa47);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[47], tbwbs, tbwbh, valid_bwb47);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[47], tbwbs, tbwbh, valid_bwb47);
    // $setuphold (posedge CLK &&& WEA, posedge DA[48], tdas, tdah, valid_da48);
    // $setuphold (posedge CLK &&& WEA, negedge DA[48], tdas, tdah, valid_da48);
    // $setuphold (posedge CLK &&& WEB, posedge DB[48], tdbs, tdbh, valid_db48);
    // $setuphold (posedge CLK &&& WEB, negedge DB[48], tdbs, tdbh, valid_db48);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[48], tbwas, tbwah, valid_bwa48);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[48], tbwas, tbwah, valid_bwa48);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[48], tbwbs, tbwbh, valid_bwb48);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[48], tbwbs, tbwbh, valid_bwb48);
    // $setuphold (posedge CLK &&& WEA, posedge DA[49], tdas, tdah, valid_da49);
    // $setuphold (posedge CLK &&& WEA, negedge DA[49], tdas, tdah, valid_da49);
    // $setuphold (posedge CLK &&& WEB, posedge DB[49], tdbs, tdbh, valid_db49);
    // $setuphold (posedge CLK &&& WEB, negedge DB[49], tdbs, tdbh, valid_db49);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[49], tbwas, tbwah, valid_bwa49);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[49], tbwas, tbwah, valid_bwa49);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[49], tbwbs, tbwbh, valid_bwb49);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[49], tbwbs, tbwbh, valid_bwb49);
    // $setuphold (posedge CLK &&& WEA, posedge DA[50], tdas, tdah, valid_da50);
    // $setuphold (posedge CLK &&& WEA, negedge DA[50], tdas, tdah, valid_da50);
    // $setuphold (posedge CLK &&& WEB, posedge DB[50], tdbs, tdbh, valid_db50);
    // $setuphold (posedge CLK &&& WEB, negedge DB[50], tdbs, tdbh, valid_db50);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[50], tbwas, tbwah, valid_bwa50);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[50], tbwas, tbwah, valid_bwa50);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[50], tbwbs, tbwbh, valid_bwb50);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[50], tbwbs, tbwbh, valid_bwb50);
    // $setuphold (posedge CLK &&& WEA, posedge DA[51], tdas, tdah, valid_da51);
    // $setuphold (posedge CLK &&& WEA, negedge DA[51], tdas, tdah, valid_da51);
    // $setuphold (posedge CLK &&& WEB, posedge DB[51], tdbs, tdbh, valid_db51);
    // $setuphold (posedge CLK &&& WEB, negedge DB[51], tdbs, tdbh, valid_db51);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[51], tbwas, tbwah, valid_bwa51);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[51], tbwas, tbwah, valid_bwa51);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[51], tbwbs, tbwbh, valid_bwb51);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[51], tbwbs, tbwbh, valid_bwb51);
    // $setuphold (posedge CLK &&& WEA, posedge DA[52], tdas, tdah, valid_da52);
    // $setuphold (posedge CLK &&& WEA, negedge DA[52], tdas, tdah, valid_da52);
    // $setuphold (posedge CLK &&& WEB, posedge DB[52], tdbs, tdbh, valid_db52);
    // $setuphold (posedge CLK &&& WEB, negedge DB[52], tdbs, tdbh, valid_db52);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[52], tbwas, tbwah, valid_bwa52);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[52], tbwas, tbwah, valid_bwa52);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[52], tbwbs, tbwbh, valid_bwb52);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[52], tbwbs, tbwbh, valid_bwb52);
    // $setuphold (posedge CLK &&& WEA, posedge DA[53], tdas, tdah, valid_da53);
    // $setuphold (posedge CLK &&& WEA, negedge DA[53], tdas, tdah, valid_da53);
    // $setuphold (posedge CLK &&& WEB, posedge DB[53], tdbs, tdbh, valid_db53);
    // $setuphold (posedge CLK &&& WEB, negedge DB[53], tdbs, tdbh, valid_db53);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[53], tbwas, tbwah, valid_bwa53);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[53], tbwas, tbwah, valid_bwa53);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[53], tbwbs, tbwbh, valid_bwb53);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[53], tbwbs, tbwbh, valid_bwb53);
    // $setuphold (posedge CLK &&& WEA, posedge DA[54], tdas, tdah, valid_da54);
    // $setuphold (posedge CLK &&& WEA, negedge DA[54], tdas, tdah, valid_da54);
    // $setuphold (posedge CLK &&& WEB, posedge DB[54], tdbs, tdbh, valid_db54);
    // $setuphold (posedge CLK &&& WEB, negedge DB[54], tdbs, tdbh, valid_db54);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[54], tbwas, tbwah, valid_bwa54);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[54], tbwas, tbwah, valid_bwa54);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[54], tbwbs, tbwbh, valid_bwb54);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[54], tbwbs, tbwbh, valid_bwb54);
    // $setuphold (posedge CLK &&& WEA, posedge DA[55], tdas, tdah, valid_da55);
    // $setuphold (posedge CLK &&& WEA, negedge DA[55], tdas, tdah, valid_da55);
    // $setuphold (posedge CLK &&& WEB, posedge DB[55], tdbs, tdbh, valid_db55);
    // $setuphold (posedge CLK &&& WEB, negedge DB[55], tdbs, tdbh, valid_db55);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[55], tbwas, tbwah, valid_bwa55);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[55], tbwas, tbwah, valid_bwa55);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[55], tbwbs, tbwbh, valid_bwb55);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[55], tbwbs, tbwbh, valid_bwb55);
    // $setuphold (posedge CLK &&& WEA, posedge DA[56], tdas, tdah, valid_da56);
    // $setuphold (posedge CLK &&& WEA, negedge DA[56], tdas, tdah, valid_da56);
    // $setuphold (posedge CLK &&& WEB, posedge DB[56], tdbs, tdbh, valid_db56);
    // $setuphold (posedge CLK &&& WEB, negedge DB[56], tdbs, tdbh, valid_db56);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[56], tbwas, tbwah, valid_bwa56);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[56], tbwas, tbwah, valid_bwa56);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[56], tbwbs, tbwbh, valid_bwb56);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[56], tbwbs, tbwbh, valid_bwb56);
    // $setuphold (posedge CLK &&& WEA, posedge DA[57], tdas, tdah, valid_da57);
    // $setuphold (posedge CLK &&& WEA, negedge DA[57], tdas, tdah, valid_da57);
    // $setuphold (posedge CLK &&& WEB, posedge DB[57], tdbs, tdbh, valid_db57);
    // $setuphold (posedge CLK &&& WEB, negedge DB[57], tdbs, tdbh, valid_db57);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[57], tbwas, tbwah, valid_bwa57);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[57], tbwas, tbwah, valid_bwa57);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[57], tbwbs, tbwbh, valid_bwb57);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[57], tbwbs, tbwbh, valid_bwb57);
    // $setuphold (posedge CLK &&& WEA, posedge DA[58], tdas, tdah, valid_da58);
    // $setuphold (posedge CLK &&& WEA, negedge DA[58], tdas, tdah, valid_da58);
    // $setuphold (posedge CLK &&& WEB, posedge DB[58], tdbs, tdbh, valid_db58);
    // $setuphold (posedge CLK &&& WEB, negedge DB[58], tdbs, tdbh, valid_db58);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[58], tbwas, tbwah, valid_bwa58);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[58], tbwas, tbwah, valid_bwa58);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[58], tbwbs, tbwbh, valid_bwb58);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[58], tbwbs, tbwbh, valid_bwb58);
    // $setuphold (posedge CLK &&& WEA, posedge DA[59], tdas, tdah, valid_da59);
    // $setuphold (posedge CLK &&& WEA, negedge DA[59], tdas, tdah, valid_da59);
    // $setuphold (posedge CLK &&& WEB, posedge DB[59], tdbs, tdbh, valid_db59);
    // $setuphold (posedge CLK &&& WEB, negedge DB[59], tdbs, tdbh, valid_db59);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[59], tbwas, tbwah, valid_bwa59);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[59], tbwas, tbwah, valid_bwa59);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[59], tbwbs, tbwbh, valid_bwb59);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[59], tbwbs, tbwbh, valid_bwb59);
    // $setuphold (posedge CLK &&& WEA, posedge DA[60], tdas, tdah, valid_da60);
    // $setuphold (posedge CLK &&& WEA, negedge DA[60], tdas, tdah, valid_da60);
    // $setuphold (posedge CLK &&& WEB, posedge DB[60], tdbs, tdbh, valid_db60);
    // $setuphold (posedge CLK &&& WEB, negedge DB[60], tdbs, tdbh, valid_db60);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[60], tbwas, tbwah, valid_bwa60);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[60], tbwas, tbwah, valid_bwa60);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[60], tbwbs, tbwbh, valid_bwb60);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[60], tbwbs, tbwbh, valid_bwb60);
    // $setuphold (posedge CLK &&& WEA, posedge DA[61], tdas, tdah, valid_da61);
    // $setuphold (posedge CLK &&& WEA, negedge DA[61], tdas, tdah, valid_da61);
    // $setuphold (posedge CLK &&& WEB, posedge DB[61], tdbs, tdbh, valid_db61);
    // $setuphold (posedge CLK &&& WEB, negedge DB[61], tdbs, tdbh, valid_db61);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[61], tbwas, tbwah, valid_bwa61);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[61], tbwas, tbwah, valid_bwa61);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[61], tbwbs, tbwbh, valid_bwb61);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[61], tbwbs, tbwbh, valid_bwb61);
    // $setuphold (posedge CLK &&& WEA, posedge DA[62], tdas, tdah, valid_da62);
    // $setuphold (posedge CLK &&& WEA, negedge DA[62], tdas, tdah, valid_da62);
    // $setuphold (posedge CLK &&& WEB, posedge DB[62], tdbs, tdbh, valid_db62);
    // $setuphold (posedge CLK &&& WEB, negedge DB[62], tdbs, tdbh, valid_db62);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[62], tbwas, tbwah, valid_bwa62);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[62], tbwas, tbwah, valid_bwa62);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[62], tbwbs, tbwbh, valid_bwb62);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[62], tbwbs, tbwbh, valid_bwb62);
    // $setuphold (posedge CLK &&& WEA, posedge DA[63], tdas, tdah, valid_da63);
    // $setuphold (posedge CLK &&& WEA, negedge DA[63], tdas, tdah, valid_da63);
    // $setuphold (posedge CLK &&& WEB, posedge DB[63], tdbs, tdbh, valid_db63);
    // $setuphold (posedge CLK &&& WEB, negedge DB[63], tdbs, tdbh, valid_db63);
 
    // $setuphold (posedge CLK &&& WEA, posedge BWEBA[63], tbwas, tbwah, valid_bwa63);
    // $setuphold (posedge CLK &&& WEA, negedge BWEBA[63], tbwas, tbwah, valid_bwa63);
    // $setuphold (posedge CLK &&& WEB, posedge BWEBB[63], tbwbs, tbwbh, valid_bwb63);
    // $setuphold (posedge CLK &&& WEB, negedge BWEBB[63], tbwbs, tbwbh, valid_bwb63);
    $setuphold (posedge CLK &&& CSA, posedge WEBA, twas, twah, valid_wea);
    $setuphold (posedge CLK &&& CSA, negedge WEBA, twas, twah, valid_wea);
    $setuphold (posedge CLK &&& CSB, posedge WEBB, twbs, twbh, valid_web);
    $setuphold (posedge CLK &&& CSB, negedge WEBB, twbs, twbh, valid_web);

    $setuphold (posedge CLK, posedge CEBA, tcas, tcah, valid_cea);
    $setuphold (posedge CLK, negedge CEBA, tcas, tcah, valid_cea);
    $setuphold (posedge CLK, posedge CEBB, tcbs, tcbh, valid_ceb);
    $setuphold (posedge CLK, negedge CEBB, tcbs, tcbh, valid_ceb);

    $width (negedge CLK &&& check_ceb, tckl, 0, valid_ck);
    $width (posedge CLK &&& check_ceb, tckh, 0, valid_ck);
    $period (posedge CLK &&& check_ceb, tcyc, valid_ck);
    $period (negedge CLK &&& check_ceb, tcyc, valid_ck);


if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[0] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[0] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[0] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[0] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[0] => (QA[0] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[0] => (QA[0] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[0] => (QB[0] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[0] => (QB[0] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[0] => (QA[0] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[0] => (QA[0] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[0] => (QB[0] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[0] => (QB[0] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[1] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[1] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[1] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[1] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[1] => (QA[1] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[1] => (QA[1] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[1] => (QB[1] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[1] => (QB[1] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[1] => (QA[1] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[1] => (QA[1] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[1] => (QB[1] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[1] => (QB[1] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[2] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[2] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[2] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[2] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[2] => (QA[2] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[2] => (QA[2] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[2] => (QB[2] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[2] => (QB[2] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[2] => (QA[2] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[2] => (QA[2] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[2] => (QB[2] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[2] => (QB[2] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[3] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[3] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[3] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[3] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[3] => (QA[3] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[3] => (QA[3] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[3] => (QB[3] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[3] => (QB[3] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[3] => (QA[3] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[3] => (QA[3] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[3] => (QB[3] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[3] => (QB[3] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[4] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[4] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[4] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[4] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[4] => (QA[4] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[4] => (QA[4] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[4] => (QB[4] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[4] => (QB[4] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[4] => (QA[4] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[4] => (QA[4] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[4] => (QB[4] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[4] => (QB[4] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[5] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[5] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[5] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[5] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[5] => (QA[5] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[5] => (QA[5] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[5] => (QB[5] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[5] => (QB[5] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[5] => (QA[5] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[5] => (QA[5] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[5] => (QB[5] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[5] => (QB[5] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[6] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[6] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[6] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[6] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[6] => (QA[6] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[6] => (QA[6] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[6] => (QB[6] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[6] => (QB[6] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[6] => (QA[6] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[6] => (QA[6] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[6] => (QB[6] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[6] => (QB[6] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[7] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[7] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[7] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[7] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[7] => (QA[7] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[7] => (QA[7] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[7] => (QB[7] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[7] => (QB[7] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[7] => (QA[7] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[7] => (QA[7] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[7] => (QB[7] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[7] => (QB[7] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[8] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[8] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[8] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[8] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[8] => (QA[8] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[8] => (QA[8] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[8] => (QB[8] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[8] => (QB[8] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[8] => (QA[8] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[8] => (QA[8] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[8] => (QB[8] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[8] => (QB[8] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[9] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[9] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[9] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[9] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[9] => (QA[9] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[9] => (QA[9] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[9] => (QB[9] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[9] => (QB[9] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[9] => (QA[9] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[9] => (QA[9] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[9] => (QB[9] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[9] => (QB[9] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[10] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[10] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[10] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[10] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[10] => (QA[10] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[10] => (QA[10] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[10] => (QB[10] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[10] => (QB[10] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[10] => (QA[10] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[10] => (QA[10] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[10] => (QB[10] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[10] => (QB[10] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[11] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[11] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[11] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[11] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[11] => (QA[11] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[11] => (QA[11] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[11] => (QB[11] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[11] => (QB[11] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[11] => (QA[11] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[11] => (QA[11] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[11] => (QB[11] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[11] => (QB[11] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[12] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[12] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[12] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[12] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[12] => (QA[12] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[12] => (QA[12] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[12] => (QB[12] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[12] => (QB[12] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[12] => (QA[12] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[12] => (QA[12] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[12] => (QB[12] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[12] => (QB[12] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[13] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[13] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[13] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[13] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[13] => (QA[13] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[13] => (QA[13] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[13] => (QB[13] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[13] => (QB[13] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[13] => (QA[13] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[13] => (QA[13] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[13] => (QB[13] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[13] => (QB[13] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[14] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[14] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[14] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[14] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[14] => (QA[14] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[14] => (QA[14] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[14] => (QB[14] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[14] => (QB[14] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[14] => (QA[14] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[14] => (QA[14] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[14] => (QB[14] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[14] => (QB[14] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[15] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[15] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[15] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[15] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[15] => (QA[15] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[15] => (QA[15] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[15] => (QB[15] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[15] => (QB[15] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[15] => (QA[15] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[15] => (QA[15] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[15] => (QB[15] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[15] => (QB[15] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[16] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[16] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[16] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[16] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[16] => (QA[16] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[16] => (QA[16] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[16] => (QB[16] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[16] => (QB[16] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[16] => (QA[16] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[16] => (QA[16] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[16] => (QB[16] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[16] => (QB[16] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[17] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[17] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[17] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[17] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[17] => (QA[17] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[17] => (QA[17] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[17] => (QB[17] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[17] => (QB[17] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[17] => (QA[17] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[17] => (QA[17] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[17] => (QB[17] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[17] => (QB[17] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[18] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[18] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[18] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[18] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[18] => (QA[18] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[18] => (QA[18] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[18] => (QB[18] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[18] => (QB[18] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[18] => (QA[18] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[18] => (QA[18] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[18] => (QB[18] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[18] => (QB[18] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[19] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[19] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[19] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[19] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[19] => (QA[19] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[19] => (QA[19] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[19] => (QB[19] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[19] => (QB[19] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[19] => (QA[19] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[19] => (QA[19] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[19] => (QB[19] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[19] => (QB[19] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[20] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[20] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[20] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[20] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[20] => (QA[20] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[20] => (QA[20] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[20] => (QB[20] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[20] => (QB[20] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[20] => (QA[20] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[20] => (QA[20] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[20] => (QB[20] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[20] => (QB[20] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[21] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[21] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[21] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[21] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[21] => (QA[21] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[21] => (QA[21] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[21] => (QB[21] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[21] => (QB[21] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[21] => (QA[21] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[21] => (QA[21] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[21] => (QB[21] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[21] => (QB[21] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[22] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[22] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[22] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[22] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[22] => (QA[22] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[22] => (QA[22] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[22] => (QB[22] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[22] => (QB[22] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[22] => (QA[22] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[22] => (QA[22] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[22] => (QB[22] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[22] => (QB[22] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[23] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[23] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[23] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[23] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[23] => (QA[23] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[23] => (QA[23] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[23] => (QB[23] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[23] => (QB[23] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[23] => (QA[23] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[23] => (QA[23] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[23] => (QB[23] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[23] => (QB[23] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[24] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[24] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[24] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[24] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[24] => (QA[24] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[24] => (QA[24] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[24] => (QB[24] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[24] => (QB[24] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[24] => (QA[24] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[24] => (QA[24] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[24] => (QB[24] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[24] => (QB[24] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[25] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[25] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[25] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[25] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[25] => (QA[25] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[25] => (QA[25] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[25] => (QB[25] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[25] => (QB[25] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[25] => (QA[25] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[25] => (QA[25] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[25] => (QB[25] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[25] => (QB[25] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[26] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[26] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[26] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[26] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[26] => (QA[26] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[26] => (QA[26] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[26] => (QB[26] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[26] => (QB[26] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[26] => (QA[26] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[26] => (QA[26] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[26] => (QB[26] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[26] => (QB[26] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[27] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[27] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[27] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[27] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[27] => (QA[27] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[27] => (QA[27] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[27] => (QB[27] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[27] => (QB[27] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[27] => (QA[27] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[27] => (QA[27] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[27] => (QB[27] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[27] => (QB[27] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[28] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[28] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[28] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[28] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[28] => (QA[28] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[28] => (QA[28] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[28] => (QB[28] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[28] => (QB[28] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[28] => (QA[28] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[28] => (QA[28] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[28] => (QB[28] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[28] => (QB[28] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[29] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[29] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[29] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[29] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[29] => (QA[29] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[29] => (QA[29] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[29] => (QB[29] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[29] => (QB[29] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[29] => (QA[29] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[29] => (QA[29] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[29] => (QB[29] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[29] => (QB[29] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[30] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[30] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[30] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[30] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[30] => (QA[30] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[30] => (QA[30] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[30] => (QB[30] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[30] => (QB[30] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[30] => (QA[30] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[30] => (QA[30] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[30] => (QB[30] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[30] => (QB[30] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[31] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[31] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
(posedge AWT => (QA[31] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
(posedge AWT => (QB[31] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




if(AWT) (posedge DA[31] => (QA[31] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DA[31] => (QA[31] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge DB[31] => (QB[31] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (negedge DB[31] => (QB[31] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
if(AWT) (posedge BWEBA[31] => (QA[31] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBA[31] => (QA[31] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (posedge BWEBB[31] => (QB[31] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
if(AWT) (negedge BWEBB[31] => (QB[31] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[32] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[32] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[32] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[32] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[32] => (QA[32] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[32] => (QA[32] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[32] => (QB[32] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[32] => (QB[32] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[32] => (QA[32] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[32] => (QA[32] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[32] => (QB[32] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[32] => (QB[32] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[33] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[33] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[33] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[33] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[33] => (QA[33] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[33] => (QA[33] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[33] => (QB[33] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[33] => (QB[33] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[33] => (QA[33] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[33] => (QA[33] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[33] => (QB[33] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[33] => (QB[33] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[34] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[34] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[34] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[34] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[34] => (QA[34] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[34] => (QA[34] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[34] => (QB[34] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[34] => (QB[34] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[34] => (QA[34] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[34] => (QA[34] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[34] => (QB[34] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[34] => (QB[34] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[35] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[35] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[35] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[35] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[35] => (QA[35] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[35] => (QA[35] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[35] => (QB[35] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[35] => (QB[35] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[35] => (QA[35] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[35] => (QA[35] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[35] => (QB[35] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[35] => (QB[35] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[36] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[36] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[36] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[36] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[36] => (QA[36] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[36] => (QA[36] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[36] => (QB[36] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[36] => (QB[36] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[36] => (QA[36] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[36] => (QA[36] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[36] => (QB[36] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[36] => (QB[36] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[37] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[37] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[37] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[37] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[37] => (QA[37] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[37] => (QA[37] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[37] => (QB[37] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[37] => (QB[37] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[37] => (QA[37] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[37] => (QA[37] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[37] => (QB[37] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[37] => (QB[37] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[38] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[38] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[38] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[38] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[38] => (QA[38] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[38] => (QA[38] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[38] => (QB[38] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[38] => (QB[38] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[38] => (QA[38] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[38] => (QA[38] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[38] => (QB[38] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[38] => (QB[38] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[39] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[39] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[39] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[39] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[39] => (QA[39] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[39] => (QA[39] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[39] => (QB[39] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[39] => (QB[39] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[39] => (QA[39] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[39] => (QA[39] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[39] => (QB[39] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[39] => (QB[39] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[40] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[40] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[40] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[40] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[40] => (QA[40] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[40] => (QA[40] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[40] => (QB[40] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[40] => (QB[40] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[40] => (QA[40] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[40] => (QA[40] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[40] => (QB[40] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[40] => (QB[40] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[41] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[41] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[41] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[41] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[41] => (QA[41] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[41] => (QA[41] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[41] => (QB[41] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[41] => (QB[41] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[41] => (QA[41] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[41] => (QA[41] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[41] => (QB[41] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[41] => (QB[41] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[42] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[42] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[42] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[42] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[42] => (QA[42] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[42] => (QA[42] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[42] => (QB[42] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[42] => (QB[42] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[42] => (QA[42] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[42] => (QA[42] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[42] => (QB[42] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[42] => (QB[42] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[43] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[43] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[43] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[43] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[43] => (QA[43] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[43] => (QA[43] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[43] => (QB[43] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[43] => (QB[43] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[43] => (QA[43] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[43] => (QA[43] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[43] => (QB[43] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[43] => (QB[43] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[44] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[44] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[44] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[44] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[44] => (QA[44] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[44] => (QA[44] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[44] => (QB[44] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[44] => (QB[44] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[44] => (QA[44] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[44] => (QA[44] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[44] => (QB[44] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[44] => (QB[44] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[45] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[45] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[45] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[45] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[45] => (QA[45] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[45] => (QA[45] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[45] => (QB[45] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[45] => (QB[45] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[45] => (QA[45] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[45] => (QA[45] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[45] => (QB[45] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[45] => (QB[45] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[46] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[46] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[46] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[46] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[46] => (QA[46] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[46] => (QA[46] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[46] => (QB[46] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[46] => (QB[46] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[46] => (QA[46] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[46] => (QA[46] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[46] => (QB[46] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[46] => (QB[46] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[47] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[47] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[47] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[47] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[47] => (QA[47] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[47] => (QA[47] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[47] => (QB[47] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[47] => (QB[47] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[47] => (QA[47] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[47] => (QA[47] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[47] => (QB[47] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[47] => (QB[47] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[48] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[48] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[48] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[48] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[48] => (QA[48] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[48] => (QA[48] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[48] => (QB[48] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[48] => (QB[48] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[48] => (QA[48] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[48] => (QA[48] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[48] => (QB[48] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[48] => (QB[48] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[49] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[49] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[49] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[49] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[49] => (QA[49] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[49] => (QA[49] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[49] => (QB[49] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[49] => (QB[49] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[49] => (QA[49] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[49] => (QA[49] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[49] => (QB[49] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[49] => (QB[49] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[50] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[50] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[50] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[50] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[50] => (QA[50] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[50] => (QA[50] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[50] => (QB[50] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[50] => (QB[50] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[50] => (QA[50] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[50] => (QA[50] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[50] => (QB[50] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[50] => (QB[50] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[51] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[51] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[51] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[51] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[51] => (QA[51] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[51] => (QA[51] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[51] => (QB[51] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[51] => (QB[51] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[51] => (QA[51] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[51] => (QA[51] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[51] => (QB[51] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[51] => (QB[51] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[52] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[52] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[52] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[52] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[52] => (QA[52] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[52] => (QA[52] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[52] => (QB[52] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[52] => (QB[52] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[52] => (QA[52] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[52] => (QA[52] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[52] => (QB[52] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[52] => (QB[52] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[53] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[53] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[53] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[53] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[53] => (QA[53] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[53] => (QA[53] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[53] => (QB[53] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[53] => (QB[53] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[53] => (QA[53] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[53] => (QA[53] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[53] => (QB[53] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[53] => (QB[53] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[54] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[54] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[54] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[54] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[54] => (QA[54] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[54] => (QA[54] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[54] => (QB[54] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[54] => (QB[54] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[54] => (QA[54] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[54] => (QA[54] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[54] => (QB[54] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[54] => (QB[54] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[55] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[55] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[55] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[55] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[55] => (QA[55] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[55] => (QA[55] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[55] => (QB[55] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[55] => (QB[55] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[55] => (QA[55] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[55] => (QA[55] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[55] => (QB[55] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[55] => (QB[55] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[56] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[56] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[56] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[56] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[56] => (QA[56] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[56] => (QA[56] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[56] => (QB[56] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[56] => (QB[56] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[56] => (QA[56] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[56] => (QA[56] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[56] => (QB[56] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[56] => (QB[56] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[57] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[57] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[57] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[57] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[57] => (QA[57] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[57] => (QA[57] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[57] => (QB[57] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[57] => (QB[57] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[57] => (QA[57] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[57] => (QA[57] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[57] => (QB[57] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[57] => (QB[57] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[58] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[58] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[58] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[58] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[58] => (QA[58] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[58] => (QA[58] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[58] => (QB[58] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[58] => (QB[58] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[58] => (QA[58] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[58] => (QA[58] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[58] => (QB[58] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[58] => (QB[58] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[59] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[59] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[59] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[59] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[59] => (QA[59] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[59] => (QA[59] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[59] => (QB[59] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[59] => (QB[59] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[59] => (QA[59] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[59] => (QA[59] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[59] => (QB[59] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[59] => (QB[59] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[60] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[60] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[60] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[60] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[60] => (QA[60] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[60] => (QA[60] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[60] => (QB[60] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[60] => (QB[60] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[60] => (QA[60] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[60] => (QA[60] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[60] => (QB[60] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[60] => (QB[60] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[61] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[61] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[61] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[61] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[61] => (QA[61] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[61] => (QA[61] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[61] => (QB[61] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[61] => (QB[61] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[61] => (QA[61] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[61] => (QA[61] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[61] => (QB[61] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[61] => (QB[61] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[62] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[62] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[62] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[62] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[62] => (QA[62] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[62] => (QA[62] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[62] => (QB[62] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[62] => (QB[62] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[62] => (QA[62] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[62] => (QA[62] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[62] => (QB[62] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[62] => (QB[62] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(!AWT & !CEBA & WEBA) (posedge CLK => (QA[63] : 1'bx)) = (tcda,tcda,tholda,tcda,tholda,tcda);
// if(!AWT & !CEBB & WEBB) (posedge CLK => (QB[63] : 1'bx)) = (tcdb,tcdb,tholdb,tcdb,tholdb,tcdb);
// (posedge AWT => (QA[63] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);
// (posedge AWT => (QB[63] : 1'bx)) = (tawtq,tawtq,tawtqh,tawtq,tawtqh,tawtq);




// if(AWT) (posedge DA[63] => (QA[63] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DA[63] => (QA[63] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge DB[63] => (QB[63] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (negedge DB[63] => (QB[63] : 1'bx)) = (tdq,tdq,tdqh,tdq,tdqh,tdq);
// if(AWT) (posedge BWEBA[63] => (QA[63] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBA[63] => (QA[63] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (posedge BWEBB[63] => (QB[63] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
// if(AWT) (negedge BWEBB[63] => (QB[63] : 1'bx)) = (tbwebq,tbwebq,tbwebqh,tbwebq,tbwebqh,tbwebq);
endspecify
`endif

initial begin
    assign EN = 1;
    RDA = 1;
    RDB = 1;
    ABL = 1'b1;
    AAL = {M{1'b0}};
    BWEBAL = {N{1'b1}};
    BWEBBL = {N{1'b1}};
    CEBAL = 1'b1;
    CEBBL = 1'b1;
    clk_count = 0;
    sd_mode = 0;
    invalid_aslp = 1'b0;
    invalid_bslp = 1'b0;    
    invalid_adslp = 1'b0;
    invalid_bdslp = 1'b0;   
    invalid_sdwk_dslp = 1'b0;    
end

`ifdef TSMC_INITIALIZE_MEM
initial
   begin 
`ifdef TSMC_INITIALIZE_FORMAT_BINARY
     #(INITIAL_MEM_DELAY)  $readmemb(cdeFileInit, MX.mem, 0, W-1);
`else
     #(INITIAL_MEM_DELAY)  $readmemh(cdeFileInit, MX.mem, 0, W-1);
`endif
   end
`endif //  `ifdef TSMC_INITIALIZE_MEM
   
`ifdef TSMC_INITIALIZE_FAULT
initial
   begin
`ifdef TSMC_INITIALIZE_FORMAT_BINARY
     #(INITIAL_FAULT_DELAY) $readmemb(cdeFileFault, MX.mem_fault, 0, W-1);
`else
     #(INITIAL_FAULT_DELAY) $readmemh(cdeFileFault, MX.mem_fault, 0, W-1);
`endif
   end
`endif //  `ifdef TSMC_INITIALIZE_FAULT


always @(bRTSEL) begin
    if (bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
        if(($realtime > 0) && (!CEBAL || !CEBBL) ) begin
`ifdef no_warning
`else        
            $display("\tWarning %m : input RTSEL should not be toggled when CEBA/CEBB is low at simulation time %t\n", $realtime);
`endif            
    `ifdef TSMC_CM_UNIT_DELAY
            #(SRAM_DELAY);
    `endif
            bQA = {N{1'bx}};
            bQB = {N{1'bx}};
            xMemoryAll;
        end
    end
end
always @(bWTSEL) begin
    if (bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
        if(($realtime > 0) && (!CEBAL || !CEBBL) ) begin
`ifdef no_warning
`else        
            $display("\tWarning %m : input WTSEL should not be toggled when CEBA/CEBB is low at simulation time %t\n", $realtime);
`endif            
    `ifdef TSMC_CM_UNIT_DELAY
            #(SRAM_DELAY);
    `endif
            bQA = {N{1'bx}};
            bQB = {N{1'bx}};
            xMemoryAll;
        end
    end
end
always @(bPTSEL) begin
    if (bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
        if(($realtime > 0) && (!CEBAL || !CEBBL) ) begin
`ifdef no_warning
`else        
            $display("\tWarning %m : input PTSEL should not be toggled when CEBA/CEBB is low at simulation time %t\n", $realtime);
`endif            
    `ifdef TSMC_CM_UNIT_DELAY
            #(SRAM_DELAY);
    `endif
            bQA = {N{1'bx}};
            bQB = {N{1'bx}};
            xMemoryAll;
        end
    end
end

`ifdef TSMC_NO_TESTPINS_WARNING
`else
always @(bCLKA or bCLKB or bRTSEL) 
begin
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
        if((bRTSEL !== 2'b00) && ($realtime > 0)) 
        begin
            $display("\tError %m : input RTSEL should be set to 2'b00 at simulation time %t\n", $realtime);
            $display("\tError %m : Please refer the datasheet for the RTSEL setting in the different segment and mux configuration\n");
            bQA <= #0.01 {N{1'bx}};
            bQB <= #0.01 {N{1'bx}};
            AAL <= {M{1'bx}};
            BWEBAL <= {N{1'b0}};
        end
    end
end

always @(bCLKA or bCLKB or bWTSEL) 
begin
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
        if((bWTSEL !== 2'b00) && ($realtime > 0)) 
        begin
            $display("\tError %m : input WTSEL should be set to 2'b00 at simulation time %t\n", $realtime);
            $display("\tError %m : Please refer the datasheet for the WTSEL setting in the different segment and mux configuration\n");
            bQA <= #0.01 {N{1'bx}};
            bQB <= #0.01 {N{1'bx}};
            AAL <= {M{1'bx}};
            BWEBAL <= {N{1'b0}};
        end
    end
end

always @(bCLKA or bCLKB or bPTSEL) 
begin
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
        if((bPTSEL !== 2'b00) && ($realtime > 0)) 
        begin
            $display("\tError %m : input PTSEL should be set to 2'b00 at simulation time %t\n", $realtime);
            $display("\tError %m : Please refer the datasheet for the PTSEL setting in the different segment and mux configuration\n");
            bQA <= #0.01 {N{1'bx}};
            bQB <= #0.01 {N{1'bx}};
            AAL <= {M{1'bx}};
            BWEBAL <= {N{1'b0}};
        end
    end
end

`endif

//always @(bTMA or bTMB) begin
//    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0 && bTMA === 1'b1 && bTMB === 1'b1) begin
//        if( MES_ALL=="ON" && $realtime != 0)
//        begin
//            $display("\nWarning %m : TMA and TMB cannot both be 1 at the same time, at %t. >>", $realtime);
//        end
//        xMemoryAll;
//`ifdef TSMC_CM_UNIT_DELAY
//        bQA <= #(SRAM_DELAY + 0.001) {N{1'bx}}; 
//        bQB <= #(SRAM_DELAY + 0.001) {N{1'bx}};
//`else
//        bQA <= #0.01 {N{1'bx}}; 
//        bQB <= #0.01 {N{1'bx}};
//`endif
//    end
//end

always @(bCLKA) 
begin : CLKAOP
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0 && invalid_sdwk_dslp === 1'b0) begin
    if(bCLKA === 1'bx) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\nWarning %m : CLK unknown at %t. >>", $realtime);
        end
        xMemoryAll;
        if(bAWT === 1'b0) begin
            bQA <= #0.01 {N{1'bx}};
        end
    end
    else if(bCLKA === 1'b1 && RCLKA === 1'b0) 
    begin
        if(bCEBA === 1'bx) 
        begin
            if( MES_ALL=="ON" && $realtime != 0)
            begin
                $display("\nWarning %m CEBA unknown at %t. >>", $realtime);
            end
            xMemoryAll;
            if(bAWT === 1'b0) begin
                bQA <= #0.01 {N{1'bx}}; 
            end
        end
        else if(bWEBA === 1'bx && bCEBA === 1'b0) 
        begin
            if( MES_ALL=="ON" && $realtime != 0)
            begin
                $display("\nWarning %m WEBA unknown at %t. >>", $realtime);
            end
            xMemoryAll;
            if(bAWT === 1'b0) begin
                bQA <= #0.01 {N{1'bx}}; 
            end
        end
        else begin                                
            WEBAL = bWEBA;
            CEBAL = bCEBA;
            if(^bAA === 1'bx && bWEBA === 1'b0 && bCEBA === 1'b0) 
            begin
                if( MES_ALL=="ON" && $realtime != 0)
                begin
                    $display("\nWarning %m WRITE AA unknown at %t. >>", $realtime);
                end
                xMemoryAll;
            end
            else if(^bAA === 1'bx && bWEBA === 1'b1 && bCEBA === 1'b0) 
            begin
                if( MES_ALL=="ON" && $realtime != 0)
                begin
                    $display("\nWarning %m READ AA unknown at %t. >>", $realtime);
                end
                xMemoryAll;
                if(bAWT === 1'b0) begin
                    bQA <= #0.01 {N{1'bx}}; 
                end
            end
            else 
            begin
                if(!bCEBA) 
                begin    // begin if(bCEBA)
                    AAL = bAA;
                    DAL = bDA;
                    if(bWEBA === 1'b1 && clk_count == 0)
                    begin
                        RDA = ~RDA;
                    end
                    if(bWEBA === 1'b0) 
                    begin
                        for (i = 0; i < N; i = i + 1) 
                        begin
                            if(!bBWEBA[i] && !bWEBA) 
                            begin
                                BWEBAL[i] = 1'b0;
                            end
                            if(bWEBA === 1'bx || bBWEBA[i] === 1'bx)
                            begin
                                BWEBAL[i] = 1'b0;
                                DAL[i] = 1'bx;
                            end
                        end
                        if(^bBWEBA === 1'bx) 
                        begin
                            if( MES_ALL=="ON" && $realtime != 0)
                            begin
                                $display("\nWarning %m BWEBA unknown at %t. >>", $realtime);
                            end
                        end
                    end
                    else if(bWEBA === 1'b1 && bAWT === 1'b1) 
                    begin
                        if( MES_ALL=="ON" && $realtime != 0) 
                            $display("\nInfo %m Read operation failed during AWT mode at %t. >>", $realtime);
                    end
                end
            end
        end                                

        CEBBL = bCEBB;
        if(bCEBB === 1'b0) begin
            WEBBL = bWEBB;
            ABL   = bAB;
            bBWEBBL = bBWEBB;
            bDBL    = bDB;
        end
        #0.001;

        if(CEBBL === 1'bx) 
        begin
            if( MES_ALL=="ON" && $realtime != 0)
            begin
                $display("\nWarning %m CEBB unknown at %t. >>", $realtime);
            end
            xMemoryAll;
            if(bAWT === 1'b0) begin
                bQB <= #0.01 {N{1'bx}}; 
            end
        end
        else if(WEBBL === 1'bx && CEBBL === 1'b0) 
        begin
            if( MES_ALL=="ON" && $realtime != 0)
            begin
                $display("\nWarning %m WEBB unknown at %t. >>", $realtime);
            end
            xMemoryAll;
            if(bAWT === 1'b0) begin
                bQB <= #0.01 {N{1'bx}}; 
            end
        end
        else 
        begin                               
            if(^ABL === 1'bx && WEBBL === 1'b0 && CEBBL === 1'b0) 
            begin
                if( MES_ALL=="ON" && $realtime != 0)
                begin
                    $display("\nWarning %m WRITE AB unknown at %t. >>", $realtime);
                end
                xMemoryAll;
            end
            else if(^ABL === 1'bx && WEBBL === 1'b1 && CEBBL === 1'b0) 
            begin
                if( MES_ALL=="ON" && $realtime != 0)
                begin
                    $display("\nWarning %m READ AB unknown at %t. >>", $realtime);
                end
                xMemoryAll;
                if(bAWT === 1'b0) begin
                    bQB <= #0.01 {N{1'bx}}; 
                end
            end
            else begin
                if(!CEBBL) 
                begin    // begin if(CEBBL)
                    DBL = bDBL;                    
                    if(WEBBL === 1'b1 && clk_count == 0)
                    begin
                        RDB = ~RDB;
                    end
                    if(WEBBL !== 1'b1) 
                    begin
                        for (i = 0; i < N; i = i + 1) 
                        begin
                            if(!bBWEBBL[i] && !WEBBL) 
                            begin
                                BWEBBL[i] = 1'b0;
                            end
                            if(WEBBL === 1'bx || bBWEBBL[i] === 1'bx)
                            begin
                                BWEBBL[i] = 1'b0;
                                DBL[i] = 1'bx;
                            end
                        end
                        if(^bBWEBBL === 1'bx) 
                        begin
                            if( MES_ALL=="ON" && $realtime != 0)
                            begin
                                $display("\nWarning %m BWEBB unknown at %t. >>", $realtime);
                            end
                        end
                    end
                    else if(WEBBL === 1'b1 && bAWT === 1'b1) 
                    begin
                        if( MES_ALL=="ON" && $realtime != 0) 
                            $display("\nInfo %m Read operation failed during AWT mode at %t. >>", $realtime);
                    end
                end
            end
        end                       
    end
    end
    #0.001 RCLKA = bCLKA;

end

always @(bAWT or bDA[0] or bBWEBA[0]) begin : AWTQA0
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[0] = 1'bx; 
        #0.01;
       `endif
        bQA[0] = bDA[0] ^ bBWEBA[0];
    end
    end
end
always @(bAWT or bDB[0] or bBWEBB[0]) begin : AWTQB0
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[0] = 1'bx;
        #0.01;
        `endif
        bQB[0] = bDB[0] ^ bBWEBB[0];
    end
    end
end
always @(bAWT or bDA[1] or bBWEBA[1]) begin : AWTQA1
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[1] = 1'bx; 
        #0.01;
       `endif
        bQA[1] = bDA[1] ^ bBWEBA[1];
    end
    end
end
always @(bAWT or bDB[1] or bBWEBB[1]) begin : AWTQB1
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[1] = 1'bx;
        #0.01;
        `endif
        bQB[1] = bDB[1] ^ bBWEBB[1];
    end
    end
end
always @(bAWT or bDA[2] or bBWEBA[2]) begin : AWTQA2
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[2] = 1'bx; 
        #0.01;
       `endif
        bQA[2] = bDA[2] ^ bBWEBA[2];
    end
    end
end
always @(bAWT or bDB[2] or bBWEBB[2]) begin : AWTQB2
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[2] = 1'bx;
        #0.01;
        `endif
        bQB[2] = bDB[2] ^ bBWEBB[2];
    end
    end
end
always @(bAWT or bDA[3] or bBWEBA[3]) begin : AWTQA3
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[3] = 1'bx; 
        #0.01;
       `endif
        bQA[3] = bDA[3] ^ bBWEBA[3];
    end
    end
end
always @(bAWT or bDB[3] or bBWEBB[3]) begin : AWTQB3
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[3] = 1'bx;
        #0.01;
        `endif
        bQB[3] = bDB[3] ^ bBWEBB[3];
    end
    end
end
always @(bAWT or bDA[4] or bBWEBA[4]) begin : AWTQA4
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[4] = 1'bx; 
        #0.01;
       `endif
        bQA[4] = bDA[4] ^ bBWEBA[4];
    end
    end
end
always @(bAWT or bDB[4] or bBWEBB[4]) begin : AWTQB4
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[4] = 1'bx;
        #0.01;
        `endif
        bQB[4] = bDB[4] ^ bBWEBB[4];
    end
    end
end
always @(bAWT or bDA[5] or bBWEBA[5]) begin : AWTQA5
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[5] = 1'bx; 
        #0.01;
       `endif
        bQA[5] = bDA[5] ^ bBWEBA[5];
    end
    end
end
always @(bAWT or bDB[5] or bBWEBB[5]) begin : AWTQB5
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[5] = 1'bx;
        #0.01;
        `endif
        bQB[5] = bDB[5] ^ bBWEBB[5];
    end
    end
end
always @(bAWT or bDA[6] or bBWEBA[6]) begin : AWTQA6
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[6] = 1'bx; 
        #0.01;
       `endif
        bQA[6] = bDA[6] ^ bBWEBA[6];
    end
    end
end
always @(bAWT or bDB[6] or bBWEBB[6]) begin : AWTQB6
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[6] = 1'bx;
        #0.01;
        `endif
        bQB[6] = bDB[6] ^ bBWEBB[6];
    end
    end
end
always @(bAWT or bDA[7] or bBWEBA[7]) begin : AWTQA7
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[7] = 1'bx; 
        #0.01;
       `endif
        bQA[7] = bDA[7] ^ bBWEBA[7];
    end
    end
end
always @(bAWT or bDB[7] or bBWEBB[7]) begin : AWTQB7
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[7] = 1'bx;
        #0.01;
        `endif
        bQB[7] = bDB[7] ^ bBWEBB[7];
    end
    end
end
always @(bAWT or bDA[8] or bBWEBA[8]) begin : AWTQA8
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[8] = 1'bx; 
        #0.01;
       `endif
        bQA[8] = bDA[8] ^ bBWEBA[8];
    end
    end
end
always @(bAWT or bDB[8] or bBWEBB[8]) begin : AWTQB8
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[8] = 1'bx;
        #0.01;
        `endif
        bQB[8] = bDB[8] ^ bBWEBB[8];
    end
    end
end
always @(bAWT or bDA[9] or bBWEBA[9]) begin : AWTQA9
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[9] = 1'bx; 
        #0.01;
       `endif
        bQA[9] = bDA[9] ^ bBWEBA[9];
    end
    end
end
always @(bAWT or bDB[9] or bBWEBB[9]) begin : AWTQB9
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[9] = 1'bx;
        #0.01;
        `endif
        bQB[9] = bDB[9] ^ bBWEBB[9];
    end
    end
end
always @(bAWT or bDA[10] or bBWEBA[10]) begin : AWTQA10
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[10] = 1'bx; 
        #0.01;
       `endif
        bQA[10] = bDA[10] ^ bBWEBA[10];
    end
    end
end
always @(bAWT or bDB[10] or bBWEBB[10]) begin : AWTQB10
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[10] = 1'bx;
        #0.01;
        `endif
        bQB[10] = bDB[10] ^ bBWEBB[10];
    end
    end
end
always @(bAWT or bDA[11] or bBWEBA[11]) begin : AWTQA11
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[11] = 1'bx; 
        #0.01;
       `endif
        bQA[11] = bDA[11] ^ bBWEBA[11];
    end
    end
end
always @(bAWT or bDB[11] or bBWEBB[11]) begin : AWTQB11
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[11] = 1'bx;
        #0.01;
        `endif
        bQB[11] = bDB[11] ^ bBWEBB[11];
    end
    end
end
always @(bAWT or bDA[12] or bBWEBA[12]) begin : AWTQA12
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[12] = 1'bx; 
        #0.01;
       `endif
        bQA[12] = bDA[12] ^ bBWEBA[12];
    end
    end
end
always @(bAWT or bDB[12] or bBWEBB[12]) begin : AWTQB12
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[12] = 1'bx;
        #0.01;
        `endif
        bQB[12] = bDB[12] ^ bBWEBB[12];
    end
    end
end
always @(bAWT or bDA[13] or bBWEBA[13]) begin : AWTQA13
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[13] = 1'bx; 
        #0.01;
       `endif
        bQA[13] = bDA[13] ^ bBWEBA[13];
    end
    end
end
always @(bAWT or bDB[13] or bBWEBB[13]) begin : AWTQB13
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[13] = 1'bx;
        #0.01;
        `endif
        bQB[13] = bDB[13] ^ bBWEBB[13];
    end
    end
end
always @(bAWT or bDA[14] or bBWEBA[14]) begin : AWTQA14
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[14] = 1'bx; 
        #0.01;
       `endif
        bQA[14] = bDA[14] ^ bBWEBA[14];
    end
    end
end
always @(bAWT or bDB[14] or bBWEBB[14]) begin : AWTQB14
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[14] = 1'bx;
        #0.01;
        `endif
        bQB[14] = bDB[14] ^ bBWEBB[14];
    end
    end
end
always @(bAWT or bDA[15] or bBWEBA[15]) begin : AWTQA15
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[15] = 1'bx; 
        #0.01;
       `endif
        bQA[15] = bDA[15] ^ bBWEBA[15];
    end
    end
end
always @(bAWT or bDB[15] or bBWEBB[15]) begin : AWTQB15
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[15] = 1'bx;
        #0.01;
        `endif
        bQB[15] = bDB[15] ^ bBWEBB[15];
    end
    end
end
always @(bAWT or bDA[16] or bBWEBA[16]) begin : AWTQA16
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[16] = 1'bx; 
        #0.01;
       `endif
        bQA[16] = bDA[16] ^ bBWEBA[16];
    end
    end
end
always @(bAWT or bDB[16] or bBWEBB[16]) begin : AWTQB16
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[16] = 1'bx;
        #0.01;
        `endif
        bQB[16] = bDB[16] ^ bBWEBB[16];
    end
    end
end
always @(bAWT or bDA[17] or bBWEBA[17]) begin : AWTQA17
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[17] = 1'bx; 
        #0.01;
       `endif
        bQA[17] = bDA[17] ^ bBWEBA[17];
    end
    end
end
always @(bAWT or bDB[17] or bBWEBB[17]) begin : AWTQB17
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[17] = 1'bx;
        #0.01;
        `endif
        bQB[17] = bDB[17] ^ bBWEBB[17];
    end
    end
end
always @(bAWT or bDA[18] or bBWEBA[18]) begin : AWTQA18
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[18] = 1'bx; 
        #0.01;
       `endif
        bQA[18] = bDA[18] ^ bBWEBA[18];
    end
    end
end
always @(bAWT or bDB[18] or bBWEBB[18]) begin : AWTQB18
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[18] = 1'bx;
        #0.01;
        `endif
        bQB[18] = bDB[18] ^ bBWEBB[18];
    end
    end
end
always @(bAWT or bDA[19] or bBWEBA[19]) begin : AWTQA19
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[19] = 1'bx; 
        #0.01;
       `endif
        bQA[19] = bDA[19] ^ bBWEBA[19];
    end
    end
end
always @(bAWT or bDB[19] or bBWEBB[19]) begin : AWTQB19
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[19] = 1'bx;
        #0.01;
        `endif
        bQB[19] = bDB[19] ^ bBWEBB[19];
    end
    end
end
always @(bAWT or bDA[20] or bBWEBA[20]) begin : AWTQA20
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[20] = 1'bx; 
        #0.01;
       `endif
        bQA[20] = bDA[20] ^ bBWEBA[20];
    end
    end
end
always @(bAWT or bDB[20] or bBWEBB[20]) begin : AWTQB20
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[20] = 1'bx;
        #0.01;
        `endif
        bQB[20] = bDB[20] ^ bBWEBB[20];
    end
    end
end
always @(bAWT or bDA[21] or bBWEBA[21]) begin : AWTQA21
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[21] = 1'bx; 
        #0.01;
       `endif
        bQA[21] = bDA[21] ^ bBWEBA[21];
    end
    end
end
always @(bAWT or bDB[21] or bBWEBB[21]) begin : AWTQB21
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[21] = 1'bx;
        #0.01;
        `endif
        bQB[21] = bDB[21] ^ bBWEBB[21];
    end
    end
end
always @(bAWT or bDA[22] or bBWEBA[22]) begin : AWTQA22
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[22] = 1'bx; 
        #0.01;
       `endif
        bQA[22] = bDA[22] ^ bBWEBA[22];
    end
    end
end
always @(bAWT or bDB[22] or bBWEBB[22]) begin : AWTQB22
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[22] = 1'bx;
        #0.01;
        `endif
        bQB[22] = bDB[22] ^ bBWEBB[22];
    end
    end
end
always @(bAWT or bDA[23] or bBWEBA[23]) begin : AWTQA23
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[23] = 1'bx; 
        #0.01;
       `endif
        bQA[23] = bDA[23] ^ bBWEBA[23];
    end
    end
end
always @(bAWT or bDB[23] or bBWEBB[23]) begin : AWTQB23
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[23] = 1'bx;
        #0.01;
        `endif
        bQB[23] = bDB[23] ^ bBWEBB[23];
    end
    end
end
always @(bAWT or bDA[24] or bBWEBA[24]) begin : AWTQA24
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[24] = 1'bx; 
        #0.01;
       `endif
        bQA[24] = bDA[24] ^ bBWEBA[24];
    end
    end
end
always @(bAWT or bDB[24] or bBWEBB[24]) begin : AWTQB24
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[24] = 1'bx;
        #0.01;
        `endif
        bQB[24] = bDB[24] ^ bBWEBB[24];
    end
    end
end
always @(bAWT or bDA[25] or bBWEBA[25]) begin : AWTQA25
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[25] = 1'bx; 
        #0.01;
       `endif
        bQA[25] = bDA[25] ^ bBWEBA[25];
    end
    end
end
always @(bAWT or bDB[25] or bBWEBB[25]) begin : AWTQB25
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[25] = 1'bx;
        #0.01;
        `endif
        bQB[25] = bDB[25] ^ bBWEBB[25];
    end
    end
end
always @(bAWT or bDA[26] or bBWEBA[26]) begin : AWTQA26
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[26] = 1'bx; 
        #0.01;
       `endif
        bQA[26] = bDA[26] ^ bBWEBA[26];
    end
    end
end
always @(bAWT or bDB[26] or bBWEBB[26]) begin : AWTQB26
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[26] = 1'bx;
        #0.01;
        `endif
        bQB[26] = bDB[26] ^ bBWEBB[26];
    end
    end
end
always @(bAWT or bDA[27] or bBWEBA[27]) begin : AWTQA27
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[27] = 1'bx; 
        #0.01;
       `endif
        bQA[27] = bDA[27] ^ bBWEBA[27];
    end
    end
end
always @(bAWT or bDB[27] or bBWEBB[27]) begin : AWTQB27
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[27] = 1'bx;
        #0.01;
        `endif
        bQB[27] = bDB[27] ^ bBWEBB[27];
    end
    end
end
always @(bAWT or bDA[28] or bBWEBA[28]) begin : AWTQA28
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[28] = 1'bx; 
        #0.01;
       `endif
        bQA[28] = bDA[28] ^ bBWEBA[28];
    end
    end
end
always @(bAWT or bDB[28] or bBWEBB[28]) begin : AWTQB28
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[28] = 1'bx;
        #0.01;
        `endif
        bQB[28] = bDB[28] ^ bBWEBB[28];
    end
    end
end
always @(bAWT or bDA[29] or bBWEBA[29]) begin : AWTQA29
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[29] = 1'bx; 
        #0.01;
       `endif
        bQA[29] = bDA[29] ^ bBWEBA[29];
    end
    end
end
always @(bAWT or bDB[29] or bBWEBB[29]) begin : AWTQB29
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[29] = 1'bx;
        #0.01;
        `endif
        bQB[29] = bDB[29] ^ bBWEBB[29];
    end
    end
end
always @(bAWT or bDA[30] or bBWEBA[30]) begin : AWTQA30
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[30] = 1'bx; 
        #0.01;
       `endif
        bQA[30] = bDA[30] ^ bBWEBA[30];
    end
    end
end
always @(bAWT or bDB[30] or bBWEBB[30]) begin : AWTQB30
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[30] = 1'bx;
        #0.01;
        `endif
        bQB[30] = bDB[30] ^ bBWEBB[30];
    end
    end
end
always @(bAWT or bDA[31] or bBWEBA[31]) begin : AWTQA31
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[31] = 1'bx; 
        #0.01;
       `endif
        bQA[31] = bDA[31] ^ bBWEBA[31];
    end
    end
end
always @(bAWT or bDB[31] or bBWEBB[31]) begin : AWTQB31
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[31] = 1'bx;
        #0.01;
        `endif
        bQB[31] = bDB[31] ^ bBWEBB[31];
    end
    end
end
always @(bAWT or bDA[32] or bBWEBA[32]) begin : AWTQA32
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[32] = 1'bx; 
        #0.01;
       `endif
        bQA[32] = bDA[32] ^ bBWEBA[32];
    end
    end
end
always @(bAWT or bDB[32] or bBWEBB[32]) begin : AWTQB32
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[32] = 1'bx;
        #0.01;
        `endif
        bQB[32] = bDB[32] ^ bBWEBB[32];
    end
    end
end
always @(bAWT or bDA[33] or bBWEBA[33]) begin : AWTQA33
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[33] = 1'bx; 
        #0.01;
       `endif
        bQA[33] = bDA[33] ^ bBWEBA[33];
    end
    end
end
always @(bAWT or bDB[33] or bBWEBB[33]) begin : AWTQB33
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[33] = 1'bx;
        #0.01;
        `endif
        bQB[33] = bDB[33] ^ bBWEBB[33];
    end
    end
end
always @(bAWT or bDA[34] or bBWEBA[34]) begin : AWTQA34
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[34] = 1'bx; 
        #0.01;
       `endif
        bQA[34] = bDA[34] ^ bBWEBA[34];
    end
    end
end
always @(bAWT or bDB[34] or bBWEBB[34]) begin : AWTQB34
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[34] = 1'bx;
        #0.01;
        `endif
        bQB[34] = bDB[34] ^ bBWEBB[34];
    end
    end
end
always @(bAWT or bDA[35] or bBWEBA[35]) begin : AWTQA35
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[35] = 1'bx; 
        #0.01;
       `endif
        bQA[35] = bDA[35] ^ bBWEBA[35];
    end
    end
end
always @(bAWT or bDB[35] or bBWEBB[35]) begin : AWTQB35
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[35] = 1'bx;
        #0.01;
        `endif
        bQB[35] = bDB[35] ^ bBWEBB[35];
    end
    end
end
always @(bAWT or bDA[36] or bBWEBA[36]) begin : AWTQA36
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[36] = 1'bx; 
        #0.01;
       `endif
        bQA[36] = bDA[36] ^ bBWEBA[36];
    end
    end
end
always @(bAWT or bDB[36] or bBWEBB[36]) begin : AWTQB36
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[36] = 1'bx;
        #0.01;
        `endif
        bQB[36] = bDB[36] ^ bBWEBB[36];
    end
    end
end
always @(bAWT or bDA[37] or bBWEBA[37]) begin : AWTQA37
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[37] = 1'bx; 
        #0.01;
       `endif
        bQA[37] = bDA[37] ^ bBWEBA[37];
    end
    end
end
always @(bAWT or bDB[37] or bBWEBB[37]) begin : AWTQB37
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[37] = 1'bx;
        #0.01;
        `endif
        bQB[37] = bDB[37] ^ bBWEBB[37];
    end
    end
end
always @(bAWT or bDA[38] or bBWEBA[38]) begin : AWTQA38
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[38] = 1'bx; 
        #0.01;
       `endif
        bQA[38] = bDA[38] ^ bBWEBA[38];
    end
    end
end
always @(bAWT or bDB[38] or bBWEBB[38]) begin : AWTQB38
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[38] = 1'bx;
        #0.01;
        `endif
        bQB[38] = bDB[38] ^ bBWEBB[38];
    end
    end
end
always @(bAWT or bDA[39] or bBWEBA[39]) begin : AWTQA39
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[39] = 1'bx; 
        #0.01;
       `endif
        bQA[39] = bDA[39] ^ bBWEBA[39];
    end
    end
end
always @(bAWT or bDB[39] or bBWEBB[39]) begin : AWTQB39
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[39] = 1'bx;
        #0.01;
        `endif
        bQB[39] = bDB[39] ^ bBWEBB[39];
    end
    end
end
always @(bAWT or bDA[40] or bBWEBA[40]) begin : AWTQA40
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[40] = 1'bx; 
        #0.01;
       `endif
        bQA[40] = bDA[40] ^ bBWEBA[40];
    end
    end
end
always @(bAWT or bDB[40] or bBWEBB[40]) begin : AWTQB40
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[40] = 1'bx;
        #0.01;
        `endif
        bQB[40] = bDB[40] ^ bBWEBB[40];
    end
    end
end
always @(bAWT or bDA[41] or bBWEBA[41]) begin : AWTQA41
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[41] = 1'bx; 
        #0.01;
       `endif
        bQA[41] = bDA[41] ^ bBWEBA[41];
    end
    end
end
always @(bAWT or bDB[41] or bBWEBB[41]) begin : AWTQB41
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[41] = 1'bx;
        #0.01;
        `endif
        bQB[41] = bDB[41] ^ bBWEBB[41];
    end
    end
end
always @(bAWT or bDA[42] or bBWEBA[42]) begin : AWTQA42
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[42] = 1'bx; 
        #0.01;
       `endif
        bQA[42] = bDA[42] ^ bBWEBA[42];
    end
    end
end
always @(bAWT or bDB[42] or bBWEBB[42]) begin : AWTQB42
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[42] = 1'bx;
        #0.01;
        `endif
        bQB[42] = bDB[42] ^ bBWEBB[42];
    end
    end
end
always @(bAWT or bDA[43] or bBWEBA[43]) begin : AWTQA43
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[43] = 1'bx; 
        #0.01;
       `endif
        bQA[43] = bDA[43] ^ bBWEBA[43];
    end
    end
end
always @(bAWT or bDB[43] or bBWEBB[43]) begin : AWTQB43
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[43] = 1'bx;
        #0.01;
        `endif
        bQB[43] = bDB[43] ^ bBWEBB[43];
    end
    end
end
always @(bAWT or bDA[44] or bBWEBA[44]) begin : AWTQA44
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[44] = 1'bx; 
        #0.01;
       `endif
        bQA[44] = bDA[44] ^ bBWEBA[44];
    end
    end
end
always @(bAWT or bDB[44] or bBWEBB[44]) begin : AWTQB44
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[44] = 1'bx;
        #0.01;
        `endif
        bQB[44] = bDB[44] ^ bBWEBB[44];
    end
    end
end
always @(bAWT or bDA[45] or bBWEBA[45]) begin : AWTQA45
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[45] = 1'bx; 
        #0.01;
       `endif
        bQA[45] = bDA[45] ^ bBWEBA[45];
    end
    end
end
always @(bAWT or bDB[45] or bBWEBB[45]) begin : AWTQB45
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[45] = 1'bx;
        #0.01;
        `endif
        bQB[45] = bDB[45] ^ bBWEBB[45];
    end
    end
end
always @(bAWT or bDA[46] or bBWEBA[46]) begin : AWTQA46
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[46] = 1'bx; 
        #0.01;
       `endif
        bQA[46] = bDA[46] ^ bBWEBA[46];
    end
    end
end
always @(bAWT or bDB[46] or bBWEBB[46]) begin : AWTQB46
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[46] = 1'bx;
        #0.01;
        `endif
        bQB[46] = bDB[46] ^ bBWEBB[46];
    end
    end
end
always @(bAWT or bDA[47] or bBWEBA[47]) begin : AWTQA47
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[47] = 1'bx; 
        #0.01;
       `endif
        bQA[47] = bDA[47] ^ bBWEBA[47];
    end
    end
end
always @(bAWT or bDB[47] or bBWEBB[47]) begin : AWTQB47
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[47] = 1'bx;
        #0.01;
        `endif
        bQB[47] = bDB[47] ^ bBWEBB[47];
    end
    end
end
always @(bAWT or bDA[48] or bBWEBA[48]) begin : AWTQA48
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[48] = 1'bx; 
        #0.01;
       `endif
        bQA[48] = bDA[48] ^ bBWEBA[48];
    end
    end
end
always @(bAWT or bDB[48] or bBWEBB[48]) begin : AWTQB48
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[48] = 1'bx;
        #0.01;
        `endif
        bQB[48] = bDB[48] ^ bBWEBB[48];
    end
    end
end
always @(bAWT or bDA[49] or bBWEBA[49]) begin : AWTQA49
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[49] = 1'bx; 
        #0.01;
       `endif
        bQA[49] = bDA[49] ^ bBWEBA[49];
    end
    end
end
always @(bAWT or bDB[49] or bBWEBB[49]) begin : AWTQB49
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[49] = 1'bx;
        #0.01;
        `endif
        bQB[49] = bDB[49] ^ bBWEBB[49];
    end
    end
end
always @(bAWT or bDA[50] or bBWEBA[50]) begin : AWTQA50
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[50] = 1'bx; 
        #0.01;
       `endif
        bQA[50] = bDA[50] ^ bBWEBA[50];
    end
    end
end
always @(bAWT or bDB[50] or bBWEBB[50]) begin : AWTQB50
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[50] = 1'bx;
        #0.01;
        `endif
        bQB[50] = bDB[50] ^ bBWEBB[50];
    end
    end
end
always @(bAWT or bDA[51] or bBWEBA[51]) begin : AWTQA51
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[51] = 1'bx; 
        #0.01;
       `endif
        bQA[51] = bDA[51] ^ bBWEBA[51];
    end
    end
end
always @(bAWT or bDB[51] or bBWEBB[51]) begin : AWTQB51
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[51] = 1'bx;
        #0.01;
        `endif
        bQB[51] = bDB[51] ^ bBWEBB[51];
    end
    end
end
always @(bAWT or bDA[52] or bBWEBA[52]) begin : AWTQA52
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[52] = 1'bx; 
        #0.01;
       `endif
        bQA[52] = bDA[52] ^ bBWEBA[52];
    end
    end
end
always @(bAWT or bDB[52] or bBWEBB[52]) begin : AWTQB52
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[52] = 1'bx;
        #0.01;
        `endif
        bQB[52] = bDB[52] ^ bBWEBB[52];
    end
    end
end
always @(bAWT or bDA[53] or bBWEBA[53]) begin : AWTQA53
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[53] = 1'bx; 
        #0.01;
       `endif
        bQA[53] = bDA[53] ^ bBWEBA[53];
    end
    end
end
always @(bAWT or bDB[53] or bBWEBB[53]) begin : AWTQB53
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[53] = 1'bx;
        #0.01;
        `endif
        bQB[53] = bDB[53] ^ bBWEBB[53];
    end
    end
end
always @(bAWT or bDA[54] or bBWEBA[54]) begin : AWTQA54
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[54] = 1'bx; 
        #0.01;
       `endif
        bQA[54] = bDA[54] ^ bBWEBA[54];
    end
    end
end
always @(bAWT or bDB[54] or bBWEBB[54]) begin : AWTQB54
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[54] = 1'bx;
        #0.01;
        `endif
        bQB[54] = bDB[54] ^ bBWEBB[54];
    end
    end
end
always @(bAWT or bDA[55] or bBWEBA[55]) begin : AWTQA55
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[55] = 1'bx; 
        #0.01;
       `endif
        bQA[55] = bDA[55] ^ bBWEBA[55];
    end
    end
end
always @(bAWT or bDB[55] or bBWEBB[55]) begin : AWTQB55
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[55] = 1'bx;
        #0.01;
        `endif
        bQB[55] = bDB[55] ^ bBWEBB[55];
    end
    end
end
always @(bAWT or bDA[56] or bBWEBA[56]) begin : AWTQA56
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[56] = 1'bx; 
        #0.01;
       `endif
        bQA[56] = bDA[56] ^ bBWEBA[56];
    end
    end
end
always @(bAWT or bDB[56] or bBWEBB[56]) begin : AWTQB56
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[56] = 1'bx;
        #0.01;
        `endif
        bQB[56] = bDB[56] ^ bBWEBB[56];
    end
    end
end
always @(bAWT or bDA[57] or bBWEBA[57]) begin : AWTQA57
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[57] = 1'bx; 
        #0.01;
       `endif
        bQA[57] = bDA[57] ^ bBWEBA[57];
    end
    end
end
always @(bAWT or bDB[57] or bBWEBB[57]) begin : AWTQB57
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[57] = 1'bx;
        #0.01;
        `endif
        bQB[57] = bDB[57] ^ bBWEBB[57];
    end
    end
end
always @(bAWT or bDA[58] or bBWEBA[58]) begin : AWTQA58
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[58] = 1'bx; 
        #0.01;
       `endif
        bQA[58] = bDA[58] ^ bBWEBA[58];
    end
    end
end
always @(bAWT or bDB[58] or bBWEBB[58]) begin : AWTQB58
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[58] = 1'bx;
        #0.01;
        `endif
        bQB[58] = bDB[58] ^ bBWEBB[58];
    end
    end
end
always @(bAWT or bDA[59] or bBWEBA[59]) begin : AWTQA59
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[59] = 1'bx; 
        #0.01;
       `endif
        bQA[59] = bDA[59] ^ bBWEBA[59];
    end
    end
end
always @(bAWT or bDB[59] or bBWEBB[59]) begin : AWTQB59
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[59] = 1'bx;
        #0.01;
        `endif
        bQB[59] = bDB[59] ^ bBWEBB[59];
    end
    end
end
always @(bAWT or bDA[60] or bBWEBA[60]) begin : AWTQA60
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[60] = 1'bx; 
        #0.01;
       `endif
        bQA[60] = bDA[60] ^ bBWEBA[60];
    end
    end
end
always @(bAWT or bDB[60] or bBWEBB[60]) begin : AWTQB60
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[60] = 1'bx;
        #0.01;
        `endif
        bQB[60] = bDB[60] ^ bBWEBB[60];
    end
    end
end
always @(bAWT or bDA[61] or bBWEBA[61]) begin : AWTQA61
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[61] = 1'bx; 
        #0.01;
       `endif
        bQA[61] = bDA[61] ^ bBWEBA[61];
    end
    end
end
always @(bAWT or bDB[61] or bBWEBB[61]) begin : AWTQB61
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[61] = 1'bx;
        #0.01;
        `endif
        bQB[61] = bDB[61] ^ bBWEBB[61];
    end
    end
end
always @(bAWT or bDA[62] or bBWEBA[62]) begin : AWTQA62
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[62] = 1'bx; 
        #0.01;
       `endif
        bQA[62] = bDA[62] ^ bBWEBA[62];
    end
    end
end
always @(bAWT or bDB[62] or bBWEBB[62]) begin : AWTQB62
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[62] = 1'bx;
        #0.01;
        `endif
        bQB[62] = bDB[62] ^ bBWEBB[62];
    end
    end
end
always @(bAWT or bDA[63] or bBWEBA[63]) begin : AWTQA63
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBAL === 1'b0 && WEBAL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
        end
    end
    if(bAWT && clk_count == 0) 
    begin
       `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQA[63] = 1'bx; 
        #0.01;
       `endif
        bQA[63] = bDA[63] ^ bBWEBA[63];
    end
    end
end
always @(bAWT or bDB[63] or bBWEBB[63]) begin : AWTQB63
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
    if(bAWT && clk_count == 0 && CEBBL === 1'b0 && WEBBL === 1'b1) 
    begin
        if( MES_ALL=="ON" && $realtime != 0)
        begin
            $display("\tInfo %m : Read operation failed when AWT is asserted at simulation time %t\n", $realtime);
         end
    end
    if(bAWT && clk_count == 0) 
    begin
        `ifdef TSMC_CM_UNIT_DELAY
        #(SRAM_DELAY);
        `else
        bQB[63] = 1'bx;
        #0.01;
        `endif
        bQB[63] = bDB[63] ^ bBWEBB[63];
    end
    end
end


always @(RDA or QAL) 
begin : CLKAROP
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0 && invalid_sdwk_dslp === 1'b0 && bAWT === 1'b0) begin
    if(!CEBAL && WEBAL && clk_count == 0) 
    begin
        begin
            if(bAWT !== 1'b1) begin
`ifdef TSMC_CM_UNIT_DELAY
            #(SRAM_DELAY);
`else
            bQA = {N{1'bx}};
            #0.01;
`endif
            bQA <= QAL;
            end
        end
    end // if(!CEBAL && WEBAL && clk_count == 0)
    end
end // always @ (RDA or QAL)

always @(RDB or QBL) 
begin : CLKBROP
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0 && invalid_sdwk_dslp === 1'b0 && bAWT === 1'b0) begin
    if(!CEBBL && WEBBL && clk_count == 0) 
    begin
        begin
            if(bAWT !== 1'b1) begin
`ifdef TSMC_CM_UNIT_DELAY
            #(SRAM_DELAY);
`else
            bQB = {N{1'bx}};
            #0.01;
`endif
            bQB <= QBL;
            end
        end
    end // if(!bAWT && !CEBBL && WEBBL && clk_count == 0)
    end
end // always @ (RDB or QBL)




always @(bAWT) 
begin
    if(bSLP === 1'b0 && bDSLP === 1'b0 && bSD === 1'b0) begin
        if(bAWT === 1'bx) 
        begin
            if( MES_ALL=="ON" && $realtime != 0)
            begin
                $display("\nWarning %m AWT unknown, Outputs unknown at %t. >>", $realtime);
            end
            //xMemoryAll;
`ifdef TSMC_CM_UNIT_DELAY
            bQA <= #(SRAM_DELAY + 0.001) {N{1'bx}}; 
            bQB <= #(SRAM_DELAY + 0.001) {N{1'bx}};
`else  
            bQA <= #0.01 {N{1'bx}}; 
            bQB <= #0.01 {N{1'bx}};
`endif
        end
        else if(bAWT === 1'b1) begin
            disable CLKAROP;
            disable CLKBROP;
        end
        else if(bAWT === 1'b0) begin
            disable AWTQA0;
            disable AWTQB0;
            disable AWTQA1;
            disable AWTQB1;
            disable AWTQA2;
            disable AWTQB2;
            disable AWTQA3;
            disable AWTQB3;
            disable AWTQA4;
            disable AWTQB4;
            disable AWTQA5;
            disable AWTQB5;
            disable AWTQA6;
            disable AWTQB6;
            disable AWTQA7;
            disable AWTQB7;
            disable AWTQA8;
            disable AWTQB8;
            disable AWTQA9;
            disable AWTQB9;
            disable AWTQA10;
            disable AWTQB10;
            disable AWTQA11;
            disable AWTQB11;
            disable AWTQA12;
            disable AWTQB12;
            disable AWTQA13;
            disable AWTQB13;
            disable AWTQA14;
            disable AWTQB14;
            disable AWTQA15;
            disable AWTQB15;
            disable AWTQA16;
            disable AWTQB16;
            disable AWTQA17;
            disable AWTQB17;
            disable AWTQA18;
            disable AWTQB18;
            disable AWTQA19;
            disable AWTQB19;
            disable AWTQA20;
            disable AWTQB20;
            disable AWTQA21;
            disable AWTQB21;
            disable AWTQA22;
            disable AWTQB22;
            disable AWTQA23;
            disable AWTQB23;
            disable AWTQA24;
            disable AWTQB24;
            disable AWTQA25;
            disable AWTQB25;
            disable AWTQA26;
            disable AWTQB26;
            disable AWTQA27;
            disable AWTQB27;
            disable AWTQA28;
            disable AWTQB28;
            disable AWTQA29;
            disable AWTQB29;
            disable AWTQA30;
            disable AWTQB30;
            disable AWTQA31;
            disable AWTQB31;
            disable AWTQA32;
            disable AWTQB32;
            disable AWTQA33;
            disable AWTQB33;
            disable AWTQA34;
            disable AWTQB34;
            disable AWTQA35;
            disable AWTQB35;
            disable AWTQA36;
            disable AWTQB36;
            disable AWTQA37;
            disable AWTQB37;
            disable AWTQA38;
            disable AWTQB38;
            disable AWTQA39;
            disable AWTQB39;
            disable AWTQA40;
            disable AWTQB40;
            disable AWTQA41;
            disable AWTQB41;
            disable AWTQA42;
            disable AWTQB42;
            disable AWTQA43;
            disable AWTQB43;
            disable AWTQA44;
            disable AWTQB44;
            disable AWTQA45;
            disable AWTQB45;
            disable AWTQA46;
            disable AWTQB46;
            disable AWTQA47;
            disable AWTQB47;
            disable AWTQA48;
            disable AWTQB48;
            disable AWTQA49;
            disable AWTQB49;
            disable AWTQA50;
            disable AWTQB50;
            disable AWTQA51;
            disable AWTQB51;
            disable AWTQA52;
            disable AWTQB52;
            disable AWTQA53;
            disable AWTQB53;
            disable AWTQA54;
            disable AWTQB54;
            disable AWTQA55;
            disable AWTQB55;
            disable AWTQA56;
            disable AWTQB56;
            disable AWTQA57;
            disable AWTQB57;
            disable AWTQA58;
            disable AWTQB58;
            disable AWTQA59;
            disable AWTQB59;
            disable AWTQA60;
            disable AWTQB60;
            disable AWTQA61;
            disable AWTQB61;
            disable AWTQA62;
            disable AWTQB62;
            disable AWTQA63;
            disable AWTQB63;
            bQA <= {N{1'bx}}; 
            bQB <= {N{1'bx}};    
        end
    end
end

always @(BWEBAL) 
begin
    BWEBAL = #0.01 {N{1'b1}};
end

always @(BWEBBL) 
begin
    BWEBBL = #0.01 {N{1'b1}};
end

 
`ifdef TSMC_CM_UNIT_DELAY
`else 
always @(valid_testpin) begin
    AAL <= {M{1'bx}};
    BWEBAL <= {N{1'b0}};
    BWEBBL <= {N{1'b0}};
    if(bAWT !== 1'b1) begin
      bQA = #0.01 {N{1'bx}};
      bQB = #0.01 {N{1'bx}};
    end
end


always @(valid_ck) 
begin
    if (iCEBA === 1'b0) begin
        #0.002;
        AAL = {M{1'bx}};
        BWEBAL = {N{1'b0}};
        if(bAWT !== 1'b1) begin
          bQA = #0.01 {N{1'bx}};
        end
    end      

    if (iCEBB === 1'b0) begin
        #0.002;
        ABL = {M{1'bx}};
        BWEBBL = {N{1'b0}};
        if(bAWT !== 1'b1) begin
        bQB = #0.01 {N{1'bx}};
        end
    end      
end
 

always @(valid_cka) 
begin
    
    #0.002;
    AAL = {M{1'bx}};
    BWEBAL = {N{1'b0}};
    if(bAWT !== 1'b1) begin
      bQA = #0.01 {N{1'bx}};
    end
end
 
always @(valid_ckb) 
begin
    
    #0.002;
    ABL = {M{1'bx}};
    BWEBBL = {N{1'b0}};
    if(bAWT !== 1'b1) begin
      bQB = #0.01 {N{1'bx}};
    end
end


always @(valid_aa) 
begin
    
    if(!WEBAL) 
    begin
        #0.002;
        BWEBAL = {N{1'b0}};
        AAL = {M{1'bx}};
    end
    else 
    begin
        #0.002;
        BWEBAL = {N{1'b0}};
        AAL = {M{1'bx}};
        if(bAWT !== 1'b1) begin
            bQA = #0.01 {N{1'bx}};
        end
    end
end

always @(valid_ab) 
begin
    
    if(!WEBBL) 
    begin
        BWEBBL = {N{1'b0}};
        ABL = {M{1'bx}};
    end
    else 
    begin
        #0.002;
        BWEBBL = {N{1'b0}};
        ABL = {M{1'bx}};
        if(bAWT !== 1'b1) begin
            bQB = #0.01 {N{1'bx}};
        end
    end
end

always @(valid_da0) 
begin
    
    DAL[0] = 1'bx;
    BWEBAL[0] = 1'b0;
end

always @(valid_db0) 
begin
    disable CLKAOP;
    DBL[0] = 1'bx;
    BWEBBL[0] = 1'b0;
end

always @(valid_bwa0) 
begin
    
    DAL[0] = 1'bx;
    BWEBAL[0] = 1'b0;
end

always @(valid_bwb0) 
begin
    disable CLKAOP;
    DBL[0] = 1'bx;
    BWEBBL[0] = 1'b0;
end
always @(valid_da1) 
begin
    
    DAL[1] = 1'bx;
    BWEBAL[1] = 1'b0;
end

always @(valid_db1) 
begin
    disable CLKAOP;
    DBL[1] = 1'bx;
    BWEBBL[1] = 1'b0;
end

always @(valid_bwa1) 
begin
    
    DAL[1] = 1'bx;
    BWEBAL[1] = 1'b0;
end

always @(valid_bwb1) 
begin
    disable CLKAOP;
    DBL[1] = 1'bx;
    BWEBBL[1] = 1'b0;
end
always @(valid_da2) 
begin
    
    DAL[2] = 1'bx;
    BWEBAL[2] = 1'b0;
end

always @(valid_db2) 
begin
    disable CLKAOP;
    DBL[2] = 1'bx;
    BWEBBL[2] = 1'b0;
end

always @(valid_bwa2) 
begin
    
    DAL[2] = 1'bx;
    BWEBAL[2] = 1'b0;
end

always @(valid_bwb2) 
begin
    disable CLKAOP;
    DBL[2] = 1'bx;
    BWEBBL[2] = 1'b0;
end
always @(valid_da3) 
begin
    
    DAL[3] = 1'bx;
    BWEBAL[3] = 1'b0;
end

always @(valid_db3) 
begin
    disable CLKAOP;
    DBL[3] = 1'bx;
    BWEBBL[3] = 1'b0;
end

always @(valid_bwa3) 
begin
    
    DAL[3] = 1'bx;
    BWEBAL[3] = 1'b0;
end

always @(valid_bwb3) 
begin
    disable CLKAOP;
    DBL[3] = 1'bx;
    BWEBBL[3] = 1'b0;
end
always @(valid_da4) 
begin
    
    DAL[4] = 1'bx;
    BWEBAL[4] = 1'b0;
end

always @(valid_db4) 
begin
    disable CLKAOP;
    DBL[4] = 1'bx;
    BWEBBL[4] = 1'b0;
end

always @(valid_bwa4) 
begin
    
    DAL[4] = 1'bx;
    BWEBAL[4] = 1'b0;
end

always @(valid_bwb4) 
begin
    disable CLKAOP;
    DBL[4] = 1'bx;
    BWEBBL[4] = 1'b0;
end
always @(valid_da5) 
begin
    
    DAL[5] = 1'bx;
    BWEBAL[5] = 1'b0;
end

always @(valid_db5) 
begin
    disable CLKAOP;
    DBL[5] = 1'bx;
    BWEBBL[5] = 1'b0;
end

always @(valid_bwa5) 
begin
    
    DAL[5] = 1'bx;
    BWEBAL[5] = 1'b0;
end

always @(valid_bwb5) 
begin
    disable CLKAOP;
    DBL[5] = 1'bx;
    BWEBBL[5] = 1'b0;
end
always @(valid_da6) 
begin
    
    DAL[6] = 1'bx;
    BWEBAL[6] = 1'b0;
end

always @(valid_db6) 
begin
    disable CLKAOP;
    DBL[6] = 1'bx;
    BWEBBL[6] = 1'b0;
end

always @(valid_bwa6) 
begin
    
    DAL[6] = 1'bx;
    BWEBAL[6] = 1'b0;
end

always @(valid_bwb6) 
begin
    disable CLKAOP;
    DBL[6] = 1'bx;
    BWEBBL[6] = 1'b0;
end
always @(valid_da7) 
begin
    
    DAL[7] = 1'bx;
    BWEBAL[7] = 1'b0;
end

always @(valid_db7) 
begin
    disable CLKAOP;
    DBL[7] = 1'bx;
    BWEBBL[7] = 1'b0;
end

always @(valid_bwa7) 
begin
    
    DAL[7] = 1'bx;
    BWEBAL[7] = 1'b0;
end

always @(valid_bwb7) 
begin
    disable CLKAOP;
    DBL[7] = 1'bx;
    BWEBBL[7] = 1'b0;
end
always @(valid_da8) 
begin
    
    DAL[8] = 1'bx;
    BWEBAL[8] = 1'b0;
end

always @(valid_db8) 
begin
    disable CLKAOP;
    DBL[8] = 1'bx;
    BWEBBL[8] = 1'b0;
end

always @(valid_bwa8) 
begin
    
    DAL[8] = 1'bx;
    BWEBAL[8] = 1'b0;
end

always @(valid_bwb8) 
begin
    disable CLKAOP;
    DBL[8] = 1'bx;
    BWEBBL[8] = 1'b0;
end
always @(valid_da9) 
begin
    
    DAL[9] = 1'bx;
    BWEBAL[9] = 1'b0;
end

always @(valid_db9) 
begin
    disable CLKAOP;
    DBL[9] = 1'bx;
    BWEBBL[9] = 1'b0;
end

always @(valid_bwa9) 
begin
    
    DAL[9] = 1'bx;
    BWEBAL[9] = 1'b0;
end

always @(valid_bwb9) 
begin
    disable CLKAOP;
    DBL[9] = 1'bx;
    BWEBBL[9] = 1'b0;
end
always @(valid_da10) 
begin
    
    DAL[10] = 1'bx;
    BWEBAL[10] = 1'b0;
end

always @(valid_db10) 
begin
    disable CLKAOP;
    DBL[10] = 1'bx;
    BWEBBL[10] = 1'b0;
end

always @(valid_bwa10) 
begin
    
    DAL[10] = 1'bx;
    BWEBAL[10] = 1'b0;
end

always @(valid_bwb10) 
begin
    disable CLKAOP;
    DBL[10] = 1'bx;
    BWEBBL[10] = 1'b0;
end
always @(valid_da11) 
begin
    
    DAL[11] = 1'bx;
    BWEBAL[11] = 1'b0;
end

always @(valid_db11) 
begin
    disable CLKAOP;
    DBL[11] = 1'bx;
    BWEBBL[11] = 1'b0;
end

always @(valid_bwa11) 
begin
    
    DAL[11] = 1'bx;
    BWEBAL[11] = 1'b0;
end

always @(valid_bwb11) 
begin
    disable CLKAOP;
    DBL[11] = 1'bx;
    BWEBBL[11] = 1'b0;
end
always @(valid_da12) 
begin
    
    DAL[12] = 1'bx;
    BWEBAL[12] = 1'b0;
end

always @(valid_db12) 
begin
    disable CLKAOP;
    DBL[12] = 1'bx;
    BWEBBL[12] = 1'b0;
end

always @(valid_bwa12) 
begin
    
    DAL[12] = 1'bx;
    BWEBAL[12] = 1'b0;
end

always @(valid_bwb12) 
begin
    disable CLKAOP;
    DBL[12] = 1'bx;
    BWEBBL[12] = 1'b0;
end
always @(valid_da13) 
begin
    
    DAL[13] = 1'bx;
    BWEBAL[13] = 1'b0;
end

always @(valid_db13) 
begin
    disable CLKAOP;
    DBL[13] = 1'bx;
    BWEBBL[13] = 1'b0;
end

always @(valid_bwa13) 
begin
    
    DAL[13] = 1'bx;
    BWEBAL[13] = 1'b0;
end

always @(valid_bwb13) 
begin
    disable CLKAOP;
    DBL[13] = 1'bx;
    BWEBBL[13] = 1'b0;
end
always @(valid_da14) 
begin
    
    DAL[14] = 1'bx;
    BWEBAL[14] = 1'b0;
end

always @(valid_db14) 
begin
    disable CLKAOP;
    DBL[14] = 1'bx;
    BWEBBL[14] = 1'b0;
end

always @(valid_bwa14) 
begin
    
    DAL[14] = 1'bx;
    BWEBAL[14] = 1'b0;
end

always @(valid_bwb14) 
begin
    disable CLKAOP;
    DBL[14] = 1'bx;
    BWEBBL[14] = 1'b0;
end
always @(valid_da15) 
begin
    
    DAL[15] = 1'bx;
    BWEBAL[15] = 1'b0;
end

always @(valid_db15) 
begin
    disable CLKAOP;
    DBL[15] = 1'bx;
    BWEBBL[15] = 1'b0;
end

always @(valid_bwa15) 
begin
    
    DAL[15] = 1'bx;
    BWEBAL[15] = 1'b0;
end

always @(valid_bwb15) 
begin
    disable CLKAOP;
    DBL[15] = 1'bx;
    BWEBBL[15] = 1'b0;
end
always @(valid_da16) 
begin
    
    DAL[16] = 1'bx;
    BWEBAL[16] = 1'b0;
end

always @(valid_db16) 
begin
    disable CLKAOP;
    DBL[16] = 1'bx;
    BWEBBL[16] = 1'b0;
end

always @(valid_bwa16) 
begin
    
    DAL[16] = 1'bx;
    BWEBAL[16] = 1'b0;
end

always @(valid_bwb16) 
begin
    disable CLKAOP;
    DBL[16] = 1'bx;
    BWEBBL[16] = 1'b0;
end
always @(valid_da17) 
begin
    
    DAL[17] = 1'bx;
    BWEBAL[17] = 1'b0;
end

always @(valid_db17) 
begin
    disable CLKAOP;
    DBL[17] = 1'bx;
    BWEBBL[17] = 1'b0;
end

always @(valid_bwa17) 
begin
    
    DAL[17] = 1'bx;
    BWEBAL[17] = 1'b0;
end

always @(valid_bwb17) 
begin
    disable CLKAOP;
    DBL[17] = 1'bx;
    BWEBBL[17] = 1'b0;
end
always @(valid_da18) 
begin
    
    DAL[18] = 1'bx;
    BWEBAL[18] = 1'b0;
end

always @(valid_db18) 
begin
    disable CLKAOP;
    DBL[18] = 1'bx;
    BWEBBL[18] = 1'b0;
end

always @(valid_bwa18) 
begin
    
    DAL[18] = 1'bx;
    BWEBAL[18] = 1'b0;
end

always @(valid_bwb18) 
begin
    disable CLKAOP;
    DBL[18] = 1'bx;
    BWEBBL[18] = 1'b0;
end
always @(valid_da19) 
begin
    
    DAL[19] = 1'bx;
    BWEBAL[19] = 1'b0;
end

always @(valid_db19) 
begin
    disable CLKAOP;
    DBL[19] = 1'bx;
    BWEBBL[19] = 1'b0;
end

always @(valid_bwa19) 
begin
    
    DAL[19] = 1'bx;
    BWEBAL[19] = 1'b0;
end

always @(valid_bwb19) 
begin
    disable CLKAOP;
    DBL[19] = 1'bx;
    BWEBBL[19] = 1'b0;
end
always @(valid_da20) 
begin
    
    DAL[20] = 1'bx;
    BWEBAL[20] = 1'b0;
end

always @(valid_db20) 
begin
    disable CLKAOP;
    DBL[20] = 1'bx;
    BWEBBL[20] = 1'b0;
end

always @(valid_bwa20) 
begin
    
    DAL[20] = 1'bx;
    BWEBAL[20] = 1'b0;
end

always @(valid_bwb20) 
begin
    disable CLKAOP;
    DBL[20] = 1'bx;
    BWEBBL[20] = 1'b0;
end
always @(valid_da21) 
begin
    
    DAL[21] = 1'bx;
    BWEBAL[21] = 1'b0;
end

always @(valid_db21) 
begin
    disable CLKAOP;
    DBL[21] = 1'bx;
    BWEBBL[21] = 1'b0;
end

always @(valid_bwa21) 
begin
    
    DAL[21] = 1'bx;
    BWEBAL[21] = 1'b0;
end

always @(valid_bwb21) 
begin
    disable CLKAOP;
    DBL[21] = 1'bx;
    BWEBBL[21] = 1'b0;
end
always @(valid_da22) 
begin
    
    DAL[22] = 1'bx;
    BWEBAL[22] = 1'b0;
end

always @(valid_db22) 
begin
    disable CLKAOP;
    DBL[22] = 1'bx;
    BWEBBL[22] = 1'b0;
end

always @(valid_bwa22) 
begin
    
    DAL[22] = 1'bx;
    BWEBAL[22] = 1'b0;
end

always @(valid_bwb22) 
begin
    disable CLKAOP;
    DBL[22] = 1'bx;
    BWEBBL[22] = 1'b0;
end
always @(valid_da23) 
begin
    
    DAL[23] = 1'bx;
    BWEBAL[23] = 1'b0;
end

always @(valid_db23) 
begin
    disable CLKAOP;
    DBL[23] = 1'bx;
    BWEBBL[23] = 1'b0;
end

always @(valid_bwa23) 
begin
    
    DAL[23] = 1'bx;
    BWEBAL[23] = 1'b0;
end

always @(valid_bwb23) 
begin
    disable CLKAOP;
    DBL[23] = 1'bx;
    BWEBBL[23] = 1'b0;
end
always @(valid_da24) 
begin
    
    DAL[24] = 1'bx;
    BWEBAL[24] = 1'b0;
end

always @(valid_db24) 
begin
    disable CLKAOP;
    DBL[24] = 1'bx;
    BWEBBL[24] = 1'b0;
end

always @(valid_bwa24) 
begin
    
    DAL[24] = 1'bx;
    BWEBAL[24] = 1'b0;
end

always @(valid_bwb24) 
begin
    disable CLKAOP;
    DBL[24] = 1'bx;
    BWEBBL[24] = 1'b0;
end
always @(valid_da25) 
begin
    
    DAL[25] = 1'bx;
    BWEBAL[25] = 1'b0;
end

always @(valid_db25) 
begin
    disable CLKAOP;
    DBL[25] = 1'bx;
    BWEBBL[25] = 1'b0;
end

always @(valid_bwa25) 
begin
    
    DAL[25] = 1'bx;
    BWEBAL[25] = 1'b0;
end

always @(valid_bwb25) 
begin
    disable CLKAOP;
    DBL[25] = 1'bx;
    BWEBBL[25] = 1'b0;
end
always @(valid_da26) 
begin
    
    DAL[26] = 1'bx;
    BWEBAL[26] = 1'b0;
end

always @(valid_db26) 
begin
    disable CLKAOP;
    DBL[26] = 1'bx;
    BWEBBL[26] = 1'b0;
end

always @(valid_bwa26) 
begin
    
    DAL[26] = 1'bx;
    BWEBAL[26] = 1'b0;
end

always @(valid_bwb26) 
begin
    disable CLKAOP;
    DBL[26] = 1'bx;
    BWEBBL[26] = 1'b0;
end
always @(valid_da27) 
begin
    
    DAL[27] = 1'bx;
    BWEBAL[27] = 1'b0;
end

always @(valid_db27) 
begin
    disable CLKAOP;
    DBL[27] = 1'bx;
    BWEBBL[27] = 1'b0;
end

always @(valid_bwa27) 
begin
    
    DAL[27] = 1'bx;
    BWEBAL[27] = 1'b0;
end

always @(valid_bwb27) 
begin
    disable CLKAOP;
    DBL[27] = 1'bx;
    BWEBBL[27] = 1'b0;
end
always @(valid_da28) 
begin
    
    DAL[28] = 1'bx;
    BWEBAL[28] = 1'b0;
end

always @(valid_db28) 
begin
    disable CLKAOP;
    DBL[28] = 1'bx;
    BWEBBL[28] = 1'b0;
end

always @(valid_bwa28) 
begin
    
    DAL[28] = 1'bx;
    BWEBAL[28] = 1'b0;
end

always @(valid_bwb28) 
begin
    disable CLKAOP;
    DBL[28] = 1'bx;
    BWEBBL[28] = 1'b0;
end
always @(valid_da29) 
begin
    
    DAL[29] = 1'bx;
    BWEBAL[29] = 1'b0;
end

always @(valid_db29) 
begin
    disable CLKAOP;
    DBL[29] = 1'bx;
    BWEBBL[29] = 1'b0;
end

always @(valid_bwa29) 
begin
    
    DAL[29] = 1'bx;
    BWEBAL[29] = 1'b0;
end

always @(valid_bwb29) 
begin
    disable CLKAOP;
    DBL[29] = 1'bx;
    BWEBBL[29] = 1'b0;
end
always @(valid_da30) 
begin
    
    DAL[30] = 1'bx;
    BWEBAL[30] = 1'b0;
end

always @(valid_db30) 
begin
    disable CLKAOP;
    DBL[30] = 1'bx;
    BWEBBL[30] = 1'b0;
end

always @(valid_bwa30) 
begin
    
    DAL[30] = 1'bx;
    BWEBAL[30] = 1'b0;
end

always @(valid_bwb30) 
begin
    disable CLKAOP;
    DBL[30] = 1'bx;
    BWEBBL[30] = 1'b0;
end
always @(valid_da31) 
begin
    
    DAL[31] = 1'bx;
    BWEBAL[31] = 1'b0;
end

always @(valid_db31) 
begin
    disable CLKAOP;
    DBL[31] = 1'bx;
    BWEBBL[31] = 1'b0;
end

always @(valid_bwa31) 
begin
    
    DAL[31] = 1'bx;
    BWEBAL[31] = 1'b0;
end

always @(valid_bwb31) 
begin
    disable CLKAOP;
    DBL[31] = 1'bx;
    BWEBBL[31] = 1'b0;
end
always @(valid_da32) 
begin
    
    DAL[32] = 1'bx;
    BWEBAL[32] = 1'b0;
end

always @(valid_db32) 
begin
    disable CLKAOP;
    DBL[32] = 1'bx;
    BWEBBL[32] = 1'b0;
end

always @(valid_bwa32) 
begin
    
    DAL[32] = 1'bx;
    BWEBAL[32] = 1'b0;
end

always @(valid_bwb32) 
begin
    disable CLKAOP;
    DBL[32] = 1'bx;
    BWEBBL[32] = 1'b0;
end
always @(valid_da33) 
begin
    
    DAL[33] = 1'bx;
    BWEBAL[33] = 1'b0;
end

always @(valid_db33) 
begin
    disable CLKAOP;
    DBL[33] = 1'bx;
    BWEBBL[33] = 1'b0;
end

always @(valid_bwa33) 
begin
    
    DAL[33] = 1'bx;
    BWEBAL[33] = 1'b0;
end

always @(valid_bwb33) 
begin
    disable CLKAOP;
    DBL[33] = 1'bx;
    BWEBBL[33] = 1'b0;
end
always @(valid_da34) 
begin
    
    DAL[34] = 1'bx;
    BWEBAL[34] = 1'b0;
end

always @(valid_db34) 
begin
    disable CLKAOP;
    DBL[34] = 1'bx;
    BWEBBL[34] = 1'b0;
end

always @(valid_bwa34) 
begin
    
    DAL[34] = 1'bx;
    BWEBAL[34] = 1'b0;
end

always @(valid_bwb34) 
begin
    disable CLKAOP;
    DBL[34] = 1'bx;
    BWEBBL[34] = 1'b0;
end
always @(valid_da35) 
begin
    
    DAL[35] = 1'bx;
    BWEBAL[35] = 1'b0;
end

always @(valid_db35) 
begin
    disable CLKAOP;
    DBL[35] = 1'bx;
    BWEBBL[35] = 1'b0;
end

always @(valid_bwa35) 
begin
    
    DAL[35] = 1'bx;
    BWEBAL[35] = 1'b0;
end

always @(valid_bwb35) 
begin
    disable CLKAOP;
    DBL[35] = 1'bx;
    BWEBBL[35] = 1'b0;
end
always @(valid_da36) 
begin
    
    DAL[36] = 1'bx;
    BWEBAL[36] = 1'b0;
end

always @(valid_db36) 
begin
    disable CLKAOP;
    DBL[36] = 1'bx;
    BWEBBL[36] = 1'b0;
end

always @(valid_bwa36) 
begin
    
    DAL[36] = 1'bx;
    BWEBAL[36] = 1'b0;
end

always @(valid_bwb36) 
begin
    disable CLKAOP;
    DBL[36] = 1'bx;
    BWEBBL[36] = 1'b0;
end
always @(valid_da37) 
begin
    
    DAL[37] = 1'bx;
    BWEBAL[37] = 1'b0;
end

always @(valid_db37) 
begin
    disable CLKAOP;
    DBL[37] = 1'bx;
    BWEBBL[37] = 1'b0;
end

always @(valid_bwa37) 
begin
    
    DAL[37] = 1'bx;
    BWEBAL[37] = 1'b0;
end

always @(valid_bwb37) 
begin
    disable CLKAOP;
    DBL[37] = 1'bx;
    BWEBBL[37] = 1'b0;
end
always @(valid_da38) 
begin
    
    DAL[38] = 1'bx;
    BWEBAL[38] = 1'b0;
end

always @(valid_db38) 
begin
    disable CLKAOP;
    DBL[38] = 1'bx;
    BWEBBL[38] = 1'b0;
end

always @(valid_bwa38) 
begin
    
    DAL[38] = 1'bx;
    BWEBAL[38] = 1'b0;
end

always @(valid_bwb38) 
begin
    disable CLKAOP;
    DBL[38] = 1'bx;
    BWEBBL[38] = 1'b0;
end
always @(valid_da39) 
begin
    
    DAL[39] = 1'bx;
    BWEBAL[39] = 1'b0;
end

always @(valid_db39) 
begin
    disable CLKAOP;
    DBL[39] = 1'bx;
    BWEBBL[39] = 1'b0;
end

always @(valid_bwa39) 
begin
    
    DAL[39] = 1'bx;
    BWEBAL[39] = 1'b0;
end

always @(valid_bwb39) 
begin
    disable CLKAOP;
    DBL[39] = 1'bx;
    BWEBBL[39] = 1'b0;
end
always @(valid_da40) 
begin
    
    DAL[40] = 1'bx;
    BWEBAL[40] = 1'b0;
end

always @(valid_db40) 
begin
    disable CLKAOP;
    DBL[40] = 1'bx;
    BWEBBL[40] = 1'b0;
end

always @(valid_bwa40) 
begin
    
    DAL[40] = 1'bx;
    BWEBAL[40] = 1'b0;
end

always @(valid_bwb40) 
begin
    disable CLKAOP;
    DBL[40] = 1'bx;
    BWEBBL[40] = 1'b0;
end
always @(valid_da41) 
begin
    
    DAL[41] = 1'bx;
    BWEBAL[41] = 1'b0;
end

always @(valid_db41) 
begin
    disable CLKAOP;
    DBL[41] = 1'bx;
    BWEBBL[41] = 1'b0;
end

always @(valid_bwa41) 
begin
    
    DAL[41] = 1'bx;
    BWEBAL[41] = 1'b0;
end

always @(valid_bwb41) 
begin
    disable CLKAOP;
    DBL[41] = 1'bx;
    BWEBBL[41] = 1'b0;
end
always @(valid_da42) 
begin
    
    DAL[42] = 1'bx;
    BWEBAL[42] = 1'b0;
end

always @(valid_db42) 
begin
    disable CLKAOP;
    DBL[42] = 1'bx;
    BWEBBL[42] = 1'b0;
end

always @(valid_bwa42) 
begin
    
    DAL[42] = 1'bx;
    BWEBAL[42] = 1'b0;
end

always @(valid_bwb42) 
begin
    disable CLKAOP;
    DBL[42] = 1'bx;
    BWEBBL[42] = 1'b0;
end
always @(valid_da43) 
begin
    
    DAL[43] = 1'bx;
    BWEBAL[43] = 1'b0;
end

always @(valid_db43) 
begin
    disable CLKAOP;
    DBL[43] = 1'bx;
    BWEBBL[43] = 1'b0;
end

always @(valid_bwa43) 
begin
    
    DAL[43] = 1'bx;
    BWEBAL[43] = 1'b0;
end

always @(valid_bwb43) 
begin
    disable CLKAOP;
    DBL[43] = 1'bx;
    BWEBBL[43] = 1'b0;
end
always @(valid_da44) 
begin
    
    DAL[44] = 1'bx;
    BWEBAL[44] = 1'b0;
end

always @(valid_db44) 
begin
    disable CLKAOP;
    DBL[44] = 1'bx;
    BWEBBL[44] = 1'b0;
end

always @(valid_bwa44) 
begin
    
    DAL[44] = 1'bx;
    BWEBAL[44] = 1'b0;
end

always @(valid_bwb44) 
begin
    disable CLKAOP;
    DBL[44] = 1'bx;
    BWEBBL[44] = 1'b0;
end
always @(valid_da45) 
begin
    
    DAL[45] = 1'bx;
    BWEBAL[45] = 1'b0;
end

always @(valid_db45) 
begin
    disable CLKAOP;
    DBL[45] = 1'bx;
    BWEBBL[45] = 1'b0;
end

always @(valid_bwa45) 
begin
    
    DAL[45] = 1'bx;
    BWEBAL[45] = 1'b0;
end

always @(valid_bwb45) 
begin
    disable CLKAOP;
    DBL[45] = 1'bx;
    BWEBBL[45] = 1'b0;
end
always @(valid_da46) 
begin
    
    DAL[46] = 1'bx;
    BWEBAL[46] = 1'b0;
end

always @(valid_db46) 
begin
    disable CLKAOP;
    DBL[46] = 1'bx;
    BWEBBL[46] = 1'b0;
end

always @(valid_bwa46) 
begin
    
    DAL[46] = 1'bx;
    BWEBAL[46] = 1'b0;
end

always @(valid_bwb46) 
begin
    disable CLKAOP;
    DBL[46] = 1'bx;
    BWEBBL[46] = 1'b0;
end
always @(valid_da47) 
begin
    
    DAL[47] = 1'bx;
    BWEBAL[47] = 1'b0;
end

always @(valid_db47) 
begin
    disable CLKAOP;
    DBL[47] = 1'bx;
    BWEBBL[47] = 1'b0;
end

always @(valid_bwa47) 
begin
    
    DAL[47] = 1'bx;
    BWEBAL[47] = 1'b0;
end

always @(valid_bwb47) 
begin
    disable CLKAOP;
    DBL[47] = 1'bx;
    BWEBBL[47] = 1'b0;
end
always @(valid_da48) 
begin
    
    DAL[48] = 1'bx;
    BWEBAL[48] = 1'b0;
end

always @(valid_db48) 
begin
    disable CLKAOP;
    DBL[48] = 1'bx;
    BWEBBL[48] = 1'b0;
end

always @(valid_bwa48) 
begin
    
    DAL[48] = 1'bx;
    BWEBAL[48] = 1'b0;
end

always @(valid_bwb48) 
begin
    disable CLKAOP;
    DBL[48] = 1'bx;
    BWEBBL[48] = 1'b0;
end
always @(valid_da49) 
begin
    
    DAL[49] = 1'bx;
    BWEBAL[49] = 1'b0;
end

always @(valid_db49) 
begin
    disable CLKAOP;
    DBL[49] = 1'bx;
    BWEBBL[49] = 1'b0;
end

always @(valid_bwa49) 
begin
    
    DAL[49] = 1'bx;
    BWEBAL[49] = 1'b0;
end

always @(valid_bwb49) 
begin
    disable CLKAOP;
    DBL[49] = 1'bx;
    BWEBBL[49] = 1'b0;
end
always @(valid_da50) 
begin
    
    DAL[50] = 1'bx;
    BWEBAL[50] = 1'b0;
end

always @(valid_db50) 
begin
    disable CLKAOP;
    DBL[50] = 1'bx;
    BWEBBL[50] = 1'b0;
end

always @(valid_bwa50) 
begin
    
    DAL[50] = 1'bx;
    BWEBAL[50] = 1'b0;
end

always @(valid_bwb50) 
begin
    disable CLKAOP;
    DBL[50] = 1'bx;
    BWEBBL[50] = 1'b0;
end
always @(valid_da51) 
begin
    
    DAL[51] = 1'bx;
    BWEBAL[51] = 1'b0;
end

always @(valid_db51) 
begin
    disable CLKAOP;
    DBL[51] = 1'bx;
    BWEBBL[51] = 1'b0;
end

always @(valid_bwa51) 
begin
    
    DAL[51] = 1'bx;
    BWEBAL[51] = 1'b0;
end

always @(valid_bwb51) 
begin
    disable CLKAOP;
    DBL[51] = 1'bx;
    BWEBBL[51] = 1'b0;
end
always @(valid_da52) 
begin
    
    DAL[52] = 1'bx;
    BWEBAL[52] = 1'b0;
end

always @(valid_db52) 
begin
    disable CLKAOP;
    DBL[52] = 1'bx;
    BWEBBL[52] = 1'b0;
end

always @(valid_bwa52) 
begin
    
    DAL[52] = 1'bx;
    BWEBAL[52] = 1'b0;
end

always @(valid_bwb52) 
begin
    disable CLKAOP;
    DBL[52] = 1'bx;
    BWEBBL[52] = 1'b0;
end
always @(valid_da53) 
begin
    
    DAL[53] = 1'bx;
    BWEBAL[53] = 1'b0;
end

always @(valid_db53) 
begin
    disable CLKAOP;
    DBL[53] = 1'bx;
    BWEBBL[53] = 1'b0;
end

always @(valid_bwa53) 
begin
    
    DAL[53] = 1'bx;
    BWEBAL[53] = 1'b0;
end

always @(valid_bwb53) 
begin
    disable CLKAOP;
    DBL[53] = 1'bx;
    BWEBBL[53] = 1'b0;
end
always @(valid_da54) 
begin
    
    DAL[54] = 1'bx;
    BWEBAL[54] = 1'b0;
end

always @(valid_db54) 
begin
    disable CLKAOP;
    DBL[54] = 1'bx;
    BWEBBL[54] = 1'b0;
end

always @(valid_bwa54) 
begin
    
    DAL[54] = 1'bx;
    BWEBAL[54] = 1'b0;
end

always @(valid_bwb54) 
begin
    disable CLKAOP;
    DBL[54] = 1'bx;
    BWEBBL[54] = 1'b0;
end
always @(valid_da55) 
begin
    
    DAL[55] = 1'bx;
    BWEBAL[55] = 1'b0;
end

always @(valid_db55) 
begin
    disable CLKAOP;
    DBL[55] = 1'bx;
    BWEBBL[55] = 1'b0;
end

always @(valid_bwa55) 
begin
    
    DAL[55] = 1'bx;
    BWEBAL[55] = 1'b0;
end

always @(valid_bwb55) 
begin
    disable CLKAOP;
    DBL[55] = 1'bx;
    BWEBBL[55] = 1'b0;
end
always @(valid_da56) 
begin
    
    DAL[56] = 1'bx;
    BWEBAL[56] = 1'b0;
end

always @(valid_db56) 
begin
    disable CLKAOP;
    DBL[56] = 1'bx;
    BWEBBL[56] = 1'b0;
end

always @(valid_bwa56) 
begin
    
    DAL[56] = 1'bx;
    BWEBAL[56] = 1'b0;
end

always @(valid_bwb56) 
begin
    disable CLKAOP;
    DBL[56] = 1'bx;
    BWEBBL[56] = 1'b0;
end
always @(valid_da57) 
begin
    
    DAL[57] = 1'bx;
    BWEBAL[57] = 1'b0;
end

always @(valid_db57) 
begin
    disable CLKAOP;
    DBL[57] = 1'bx;
    BWEBBL[57] = 1'b0;
end

always @(valid_bwa57) 
begin
    
    DAL[57] = 1'bx;
    BWEBAL[57] = 1'b0;
end

always @(valid_bwb57) 
begin
    disable CLKAOP;
    DBL[57] = 1'bx;
    BWEBBL[57] = 1'b0;
end
always @(valid_da58) 
begin
    
    DAL[58] = 1'bx;
    BWEBAL[58] = 1'b0;
end

always @(valid_db58) 
begin
    disable CLKAOP;
    DBL[58] = 1'bx;
    BWEBBL[58] = 1'b0;
end

always @(valid_bwa58) 
begin
    
    DAL[58] = 1'bx;
    BWEBAL[58] = 1'b0;
end

always @(valid_bwb58) 
begin
    disable CLKAOP;
    DBL[58] = 1'bx;
    BWEBBL[58] = 1'b0;
end
always @(valid_da59) 
begin
    
    DAL[59] = 1'bx;
    BWEBAL[59] = 1'b0;
end

always @(valid_db59) 
begin
    disable CLKAOP;
    DBL[59] = 1'bx;
    BWEBBL[59] = 1'b0;
end

always @(valid_bwa59) 
begin
    
    DAL[59] = 1'bx;
    BWEBAL[59] = 1'b0;
end

always @(valid_bwb59) 
begin
    disable CLKAOP;
    DBL[59] = 1'bx;
    BWEBBL[59] = 1'b0;
end
always @(valid_da60) 
begin
    
    DAL[60] = 1'bx;
    BWEBAL[60] = 1'b0;
end

always @(valid_db60) 
begin
    disable CLKAOP;
    DBL[60] = 1'bx;
    BWEBBL[60] = 1'b0;
end

always @(valid_bwa60) 
begin
    
    DAL[60] = 1'bx;
    BWEBAL[60] = 1'b0;
end

always @(valid_bwb60) 
begin
    disable CLKAOP;
    DBL[60] = 1'bx;
    BWEBBL[60] = 1'b0;
end
always @(valid_da61) 
begin
    
    DAL[61] = 1'bx;
    BWEBAL[61] = 1'b0;
end

always @(valid_db61) 
begin
    disable CLKAOP;
    DBL[61] = 1'bx;
    BWEBBL[61] = 1'b0;
end

always @(valid_bwa61) 
begin
    
    DAL[61] = 1'bx;
    BWEBAL[61] = 1'b0;
end

always @(valid_bwb61) 
begin
    disable CLKAOP;
    DBL[61] = 1'bx;
    BWEBBL[61] = 1'b0;
end
always @(valid_da62) 
begin
    
    DAL[62] = 1'bx;
    BWEBAL[62] = 1'b0;
end

always @(valid_db62) 
begin
    disable CLKAOP;
    DBL[62] = 1'bx;
    BWEBBL[62] = 1'b0;
end

always @(valid_bwa62) 
begin
    
    DAL[62] = 1'bx;
    BWEBAL[62] = 1'b0;
end

always @(valid_bwb62) 
begin
    disable CLKAOP;
    DBL[62] = 1'bx;
    BWEBBL[62] = 1'b0;
end
always @(valid_da63) 
begin
    
    DAL[63] = 1'bx;
    BWEBAL[63] = 1'b0;
end

always @(valid_db63) 
begin
    disable CLKAOP;
    DBL[63] = 1'bx;
    BWEBBL[63] = 1'b0;
end

always @(valid_bwa63) 
begin
    
    DAL[63] = 1'bx;
    BWEBAL[63] = 1'b0;
end

always @(valid_bwb63) 
begin
    disable CLKAOP;
    DBL[63] = 1'bx;
    BWEBBL[63] = 1'b0;
end

always @(valid_cea) 
begin
    
    #0.002;
    BWEBAL = {N{1'b0}};
    AAL = {M{1'bx}};
    if(bAWT !== 1'b1) begin
      bQA = #0.01 {N{1'bx}};
    end
end

always @(valid_ceb) 
begin
    
    #0.002;
    BWEBBL = {N{1'b0}};
    ABL = {M{1'bx}};
    if(bAWT !== 1'b1) begin
      bQB = #0.01 {N{1'bx}};
    end
end

always @(valid_wea) 
begin
    #0.002;
    BWEBAL = {N{1'b0}};
    AAL = {M{1'bx}};
    if(bAWT !== 1'b1) begin
      bQA = #0.01 {N{1'bx}};
    end
end
 
always @(valid_web) 
begin
    #0.002;
    BWEBBL = {N{1'b0}};
    ABL = {M{1'bx}};
    if(bAWT !== 1'b1) begin
      bQB = #0.01 {N{1'bx}};
    end
end

`endif

// Task for printing the memory between specified addresses..
task printMemoryFromTo;     
    input [M - 1:0] from;   // memory content are printed, start from this address.
    input [M - 1:0] to;     // memory content are printed, end at this address.
    begin 
        MX.printMemoryFromTo(from, to);
    end 
endtask

// Task for printing entire memory, including normal array and redundancy array.
task printMemory;   
    begin
        MX.printMemory;
    end
endtask

task xMemoryAll;   
    begin
       MX.xMemoryAll;  
    end
endtask

task zeroMemoryAll;   
    begin
       MX.zeroMemoryAll;   
    end
endtask

// Task for Loading a perdefined set of data from an external file.
task preloadData;   
    input [256*8:1] infile;  // Max 256 character File Name
    begin
        MX.preloadData(infile);  
    end
endtask

TSDN28HPCPUHDB512X64M4MWA_Int_Array #(2,2,W,N,M,MES_ALL) MX (.D({DAL,DBL}),.BW({BWEBAL,BWEBBL}),
         .AW({AAL,ABL}),.EN(EN),.AAR(AAL),.ABR(ABL),.RDA(RDA),.RDB(RDB),.QA(QAL),.QB(QBL));
 
endmodule

    `disable_portfaults
    `nosuppress_faults
    `endcelldefine

    /*
       The module ports are parameterizable vectors.
    */
    module TSDN28HPCPUHDB512X64M4MWA_Int_Array (D, BW, AW, EN, AAR, ABR, RDA, RDB, QA, QB);
    parameter Nread = 2;   // Number of Read Ports
    parameter Nwrite = 2;  // Number of Write Ports
    parameter Nword = 2;   // Number of Words
    parameter Ndata = 1;   // Number of Data Bits / Word
    parameter Naddr = 1;   // Number of Address Bits / Word
    parameter MES_ALL = "ON";
    parameter dly = 0.000;
    // Cannot define inputs/outputs as memories
    input  [Ndata*Nwrite-1:0] D;  // Data Word(s)
    input  [Ndata*Nwrite-1:0] BW; // Negative Bit Write Enable
    input  [Naddr*Nwrite-1:0] AW; // Write Address(es)
    input  EN;                    // Positive Write Enable
    input  RDA;                    // Positive Write Enable
    input  RDB;                    // Positive Write Enable
    input  [Naddr-1:0] AAR;  // Read Address(es)
    input  [Naddr-1:0] ABR;  // Read Address(es)
    output [Ndata-1:0] QA;   // Output Data Word(s)
    output [Ndata-1:0] QB;   // Output Data Word(s)
    reg    [Ndata-1:0] QA;
    reg    [Ndata-1:0] QB;
    reg [Ndata-1:0] mem [Nword-1:0];
    reg [Ndata-1:0] mem_fault [Nword-1:0];
    reg chgmem;            // Toggled when write to mem
    reg [Nwrite-1:0] wwe;  // Positive Word Write Enable for each Port
    reg we;                // Positive Write Enable for all Ports
    integer waddr[Nwrite-1:0]; // Write Address for each Enabled Port
    integer address;       // Current address
    reg [Naddr-1:0] abuf;  // Address of current port
    reg [Ndata-1:0] dbuf;  // Data for current port
    reg [Naddr-1:0] abuf_ra;  // Address of current port
    reg [Ndata-1:0] dbuf_ra;  // Data for current port
    reg [Naddr-1:0] abuf_rb;  // Address of current port
    reg [Ndata-1:0] dbuf_rb;  // Data for current port
    reg [Ndata-1:0] bwbuf; // Bit Write enable for current port
    reg dup;               // Is the address a duplicate?
    integer log;           // Log file descriptor
    integer ip, ip2, ib, iba_r, ibb_r, iw, iwb, i; // Vector indices


    initial 
    begin
        if(log[0] === 1'bx)
            log = 1;
        chgmem = 1'b0;
    end


    always @(D or BW or AW or EN) 
    begin: WRITE //{
        if(EN !== 1'b0) 
        begin //{ Possible write
            we = 1'b0;
            // Mark any write enabled ports & get write addresses
            for (ip = 0 ; ip < Nwrite ; ip = ip + 1) 
            begin //{
                ib = ip * Ndata;
                iw = ib + Ndata;
                while (ib < iw && BW[ib] === 1'b1)
                begin
                    ib = ib + 1;
                end
                if(ib == iw)
                begin
                    wwe[ip] = 1'b0;
                end
                else 
                begin //{ ip write enabled
                    iw = ip * Naddr;
                    for (ib = 0 ; ib < Naddr ; ib = ib + 1) 
                    begin //{
                        abuf[ib] = AW[iw+ib];
                        if(abuf[ib] !== 1'b0 && abuf[ib] !== 1'b1)
                        begin
                            ib = Naddr;
                        end
                    end //}
                    if(ib == Naddr) 
                    begin //{
                        if(abuf < Nword) 
                        begin //{ Valid address
                            waddr[ip] = abuf;
                            wwe[ip] = 1'b1;
                            if(we == 1'b0) 
                            begin
                                chgmem = ~chgmem;
                                we = EN;
                            end
                        end //}
                        else 
                        begin //{ Out of range address
                             wwe[ip] = 1'b0;
                             if( MES_ALL=="ON" && $realtime != 0)
                                  $fdisplay (log,
                                             "\nWarning! Int_Array instance, %m:",
                                             "\n\t Port %0d", ip,
                                             " write address x'%0h'", abuf,
                                             " out of range at time %t.", $realtime,
                                             "\n\t Port %0d data not written to memory.", ip);
                        end //}
                    end //}
                    else 
                    begin //{ unknown write address
                        for (ib = 0 ; ib < Ndata ; ib = ib + 1)
                        begin
                            dbuf[ib] = 1'bx;
                        end
                        for (iw = 0 ; iw < Nword ; iw = iw + 1)
                        begin
                            mem[iw] = dbuf;
                        end
                        chgmem = ~chgmem;
                        disable WRITE;
                    end //}
                end //} ip write enabled
            end //} for ip
            if(we === 1'b1) 
            begin //{ active write enable
                for (ip = 0 ; ip < Nwrite ; ip = ip + 1) 
                begin //{
                    if(wwe[ip]) 
                    begin //{ write enabled bits of write port ip
                        address = waddr[ip];
                        dbuf = mem[address];
                        iw = ip * Ndata;
                        for (ib = 0 ; ib < Ndata ; ib = ib + 1) 
                        begin //{
                            iwb = iw + ib;
                            if(BW[iwb] === 1'b0)
                            begin
                                dbuf[ib] = D[iwb];
                            end
                            else
                            if(BW[iwb] !== 1'b1)
                            begin
                                dbuf[ib] = 1'bx;
                            end
                        end //}
                        // Check other ports for same address &
                        // common write enable bits active
                        dup = 0;
                        for (ip2 = ip + 1 ; ip2 < Nwrite ; ip2 = ip2 + 1) 
                        begin //{
                            if(wwe[ip2] && address == waddr[ip2]) 
                            begin //{
                                // initialize bwbuf if first dup
                                if(!dup) 
                                begin
                                    for (ib = 0 ; ib < Ndata ; ib = ib + 1)
                                    begin
                                        bwbuf[ib] = BW[iw+ib];
                                    end
                                    dup = 1;
                                end
                                iw = ip2 * Ndata;
                                for (ib = 0 ; ib < Ndata ; ib = ib + 1) 
                                begin //{
                                    iwb = iw + ib;
                                    // New: Always set X if BW X
                                    if(BW[iwb] === 1'b0) 
                                    begin //{
                                        if(bwbuf[ib] !== 1'b1) 
                                        begin
                                            if(D[iwb] !== dbuf[ib])
                                            begin
                                                dbuf[ib] = 1'bx;
                                            end
                                        end
                                        else 
                                        begin
                                            dbuf[ib] = D[iwb];
                                            bwbuf[ib] = 1'b0;
                                        end
                                    end //}
                                    else if(BW[iwb] !== 1'b1) 
                                    begin
                                        dbuf[ib] = 1'bx;
                                        bwbuf[ib] = 1'bx;
                                    end
                                end //} for each bit
                                wwe[ip2] = 1'b0;
                            end //} Port ip2 address matches port ip
                        end //} for each port beyond ip (ip2=ip+1)
                        // Write dbuf to memory
                        mem[address] = dbuf;
                    end //} wwe[ip] - write port ip enabled
                end //} for each write port ip
            end //} active write enable
            else if(we !== 1'b0) 
            begin //{ unknown write enable
                for (ip = 0 ; ip < Nwrite ; ip = ip + 1) 
                begin //{
                    if(wwe[ip]) 
                    begin //{ write X to enabled bits of write port ip
                        address = waddr[ip];
                        dbuf = mem[address];
                        iw = ip * Ndata;
                        for (ib = 0 ; ib < Ndata ; ib = ib + 1) 
                        begin //{ 
                            if(BW[iw+ib] !== 1'b1)
                            begin
                                dbuf[ib] = 1'bx;
                            end
                        end //} 
                        mem[address] = dbuf;
                        if( MES_ALL=="ON" && $realtime != 0)
                            $fdisplay (log,
                                       "\nWarning! Int_Array instance, %m:",
                                       "\n\t Enable pin unknown at time %t.", $realtime,
                                       "\n\t Enabled bits at port %0d", ip,
                                       " write address x'%0h' set unknown.", address);
                    end //} wwe[ip] - write port ip enabled
                end //} for each write port ip
            end //} unknown write enable
        end //} possible write (EN != 0)
    end //} always @(D or BW or AW or EN)


    // Read memory
    always @(AAR or RDA) 
    begin //{
        for (iba_r = 0 ; iba_r < Naddr ; iba_r = iba_r + 1) 
        begin
            abuf_ra[iba_r] = AAR[iba_r];
            if(abuf_ra[iba_r] !== 0 && abuf_ra[iba_r] !== 1)
            begin
                iba_r = Naddr;
            end
        end
        if(iba_r == Naddr && abuf_ra < Nword) 
        begin //{ Read valid address
    `ifdef TSMC_INITIALIZE_FAULT
            dbuf_ra = mem[abuf_ra] ^ mem_fault[abuf_ra];
    `else
            dbuf_ra = mem[abuf_ra];
    `endif
            for (iba_r = 0 ; iba_r < Ndata ; iba_r = iba_r + 1) 
            begin
                if(QA[iba_r] == dbuf_ra[iba_r])
                begin
                    QA[iba_r] <= #(dly) dbuf_ra[iba_r];
                end
                else 
                begin
                    QA[iba_r] <= #(dly) dbuf_ra[iba_r];
                end // else
            end // for
        end //} valid address
        else 
        begin //{ Invalid address
            if(iba_r <= Naddr) begin 
                if( MES_ALL=="ON" && $realtime != 0)
                    $fwrite (log, "\nWarning! Int_Array instance, %m:",
                         "\n\t Port A read address");
                if( MES_ALL=="ON" && $realtime != 0)
                    $fwrite (log, " x'%0h' out of range", abuf_ra);
                if( MES_ALL=="ON" && $realtime != 0)
                    $fdisplay (log,
                           " at time %t.", $realtime,
                           "\n\t Port A outputs set to unknown.");
            end   
            
            for (iba_r = 0 ; iba_r < Ndata ; iba_r = iba_r + 1)
                QA[iba_r] <= #(dly) 1'bx;
        end //} invalid address
    end //} always @(chgmem or AR)

    // Read memory
    always @(ABR or RDB) 
    begin //{
        for (ibb_r = 0 ; ibb_r < Naddr ; ibb_r = ibb_r + 1) 
        begin
            abuf_rb[ibb_r] = ABR[ibb_r];
            if(abuf_rb[ibb_r] !== 0 && abuf_rb[ibb_r] !== 1)
            begin
                ibb_r = Naddr;
            end
        end
        if(ibb_r == Naddr && abuf_rb < Nword) 
        begin //{ Read valid address
    `ifdef TSMC_INITIALIZE_FAULT
            dbuf_rb = mem[abuf_rb] ^ mem_fault[abuf_rb];
    `else
            dbuf_rb = mem[abuf_rb];
    `endif
            for (ibb_r = 0 ; ibb_r < Ndata ; ibb_r = ibb_r + 1) 
            begin
                if(QB[ibb_r] == dbuf_rb[ibb_r])
                begin
                    QB[ibb_r] <= #(dly) dbuf_rb[ibb_r];
                end
                else 
                begin
                    QB[ibb_r] <= #(dly) dbuf_rb[ibb_r];
                end // else
            end // for
        end //} valid address
        else 
        begin //{ Invalid address
            if(ibb_r <= Naddr) begin 
                if( MES_ALL=="ON" && $realtime != 0)
                    $fwrite (log, "\nWarning! Int_Array instance, %m:",
                         "\n\t Port B read address");
                if( MES_ALL=="ON" && $realtime != 0)
                    $fwrite (log, " x'%0h' out of range", abuf_rb);
                if( MES_ALL=="ON" && $realtime != 0)
                    $fdisplay (log,
                           " at time %t.", $realtime,
                           "\n\t Port B outputs set to unknown.");
            end   
            for (ibb_r = 0 ; ibb_r < Ndata ; ibb_r = ibb_r + 1)
                QB[ibb_r] <= #(dly) 1'bx;
        end //} invalid address
    end //} always @(chgmem or AR)


    // Task for loading contents of a memory
    task preloadData;   
        input [256*8:1] infile;  // Max 256 character File Name
        begin
            $display ("%m: Reading file, %0s, into the register file", infile);
    `ifdef TSMC_INITIALIZE_FORMAT_BINARY
            $readmemb (infile, mem, 0, Nword-1);
    `else
            $readmemh (infile, mem, 0, Nword-1);
    `endif
        end
    endtask

    // Task for displaying contents of a memory
    task printMemoryFromTo;   
        input [Naddr - 1:0] from;   // memory content are printed, start from this address.
        input [Naddr - 1:0] to;     // memory content are printed, end at this address.
        integer i;
    begin //{
        $display ("\n%m: Memory content dump");
        if(from < 0 || from > to || to >= Nword)
        begin
            $display ("Error! Invalid address range (%0d, %0d).", from, to,
                      "\nUsage: %m (from, to);",
                      "\n       where from >= 0 and to <= %0d.", Nword-1);
        end
        else 
        begin
            $display ("\n    Address\tValue");
            for (i = from ; i <= to ; i = i + 1)
                $display ("%d\t%b", i, mem[i]);
        end
    end //}
    endtask //}

    // Task for printing entire memory, including normal array and redundancy array.
    task printMemory;   
        integer i;
        begin
            $display ("Dumping register file...");
            $display("@    Address, content-----");
            for (i = 0; i < Nword; i = i + 1) begin
                $display("@%d, %b", i, mem[i]);
            end 
        end
    endtask

    task xMemoryAll;   
        begin
           for (ib = 0 ; ib < Ndata ; ib = ib + 1)
              dbuf[ib] = 1'bx;
           for (iw = 0 ; iw < Nword ; iw = iw + 1)
              mem[iw] = dbuf; 
        end
    endtask

    task zeroMemoryAll;   
        begin
           for (ib = 0 ; ib < Ndata ; ib = ib + 1)
              dbuf[ib] = 1'b0;
           for (iw = 0 ; iw < Nword ; iw = iw + 1)
              mem[iw] = dbuf; 
        end
    endtask
    endmodule



