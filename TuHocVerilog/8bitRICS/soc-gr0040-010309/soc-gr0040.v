// Copyright (C) 2000, Gray Research LLC.
// All rights reserved.  Use subject to the
// XSOC License Agreement, see LICENSE file,
// http://www.fpgapcu.org/xsoc/LICENSE.html.
// Version 2001.03.09
 
`define W    16  // register width
`define N    15  // register MSB
`define AN   15  // address MSB
`define IN   15  // instruction MSB
 
 module gr0040(
  clk, rst, i_ad_rst,
  insn_ce, i_ad, insn, hit, int_en,
  d_ad, rdy, sw, sb, do, lw, lb, data);
 
  input  clk;         // clock
  input  rst;         // reset (sync)
  input [`AN:0] i_ad_rst; // reset vector
 
  output insn_ce;     // insn clock enable 
  output [`AN:0] i_ad;// next insn address
  input  [`IN:0] insn;// current insn
  input  hit;         // insn is valid
  output int_en;      // OK to intr. now
 
  output [`AN:0] d_ad;// load/store addr
  input  rdy;         // memory ready
  output sw, sb;      // executing sw (sb)
  output [`N:0] do;   // data to store
  output lw, lb;      // executing lw (lb)
  inout  [`N:0] data; // results, load data
 
  // opcode decoding
  `define JAL     (op==0)
  `define ADDI    (op==1)
  `define RR      (op==2)
  `define RI      (op==3)
  `define LW      (op==4)
  `define LB      (op==5)
  `define SW      (op==6)
  `define SB      (op==7)
  `define IMM     (op==8)
  `define Bx      (op==9)
  `define ALU     (`RR|`RI)
 
  // fn decoding
  `define ADD     (fn==0)
  `define SUB     (fn==1)
  `define AND     (fn==2)
  `define XOR     (fn==3)
  `define ADC     (fn==4)
  `define SBC     (fn==5)
  `define CMP     (fn==6)
  `define SRL     (fn==7)
  `define SRA     (fn==8)
  `define SUM     (`ADD|`SUB|`ADC|`SBC)
  `define LOG     (`AND|`XOR)
  `define SR      (`SRL|`SRA)
 
  // instruction decoding
  wire [3:0] op = insn[15:12];
  wire [3:0] rd = insn[11:8];
  wire [3:0] rs = insn[7:4];
  wire [3:0] fn = `RI ? insn[7:4] : insn[3:0];
  wire [3:0] imm = insn[3:0];
  wire [11:0] i12 = insn[11:0];
  wire [3:0] cond = insn[11:8];
  wire [7:0] disp = insn[7:0];
  // register file and program counter
  wire valid_insn_ce = hit & insn_ce;
  wire rf_we = valid_insn_ce & ~rst &
           ((`ALU&~`CMP)|`ADDI|`LB|`LW|`JAL);
  wire [`N:0] dreg, sreg; // d, s registers
  ram16x16d regfile(.clk(clk), .we(rf_we),
    .wr_ad(rd), .ad(`RI ? rd : rs),
    .d(data), .wr_o(dreg), .o(sreg));
  reg [`AN:0] pc;     // program counter
 
  // immediate prefix
  reg imm_pre;          // immediate prefix
  reg [11:0] i12_pre;   // imm prefix value
  always @(posedge clk)
    if (rst)
      imm_pre <= 0;
    else if (valid_insn_ce)
      imm_pre <= `IMM;
  always @(posedge clk)
    if (valid_insn_ce)
      i12_pre <= i12;
 
  // immediate operand
  wire word_off = `LW|`SW|`JAL;
  wire sxi = (`ADDI|`ALU) & imm[3];
  wire [10:0] sxi11 = {11{sxi}};
  wire i_4 = sxi | (word_off&imm[0]);
  wire i_0 = ~word_off&imm[0];
  wire [`N:0] imm16 = imm_pre ? {i12_pre,imm}
                  : {sxi11,i_4,imm[3:1],i_0};
 
  // operand selection
  wire [`N:0] a = `RR ? dreg : imm16;
  wire [`N:0] b = sreg;
  // adder/subtractor
  wire [`N:0] sum;
  wire add = ~(`ALU&(`SUB|`SBC|`CMP));
  reg c; // carry-in if adc/sbc
  wire ci = add ? c : ~c;
  wire c_W, x;
  addsub adder(.add(add), .ci(ci), .a(a),
          .b(b), .sum(sum), .x(x), .co(c_W));
 
  // condition codes
  wire z = sum == 0;             // zero
  wire n = sum[`N];              // negative
  wire co = add ? c_W : ~c_W;    // carry-out
  wire v = c_W^sum[`N]^a[`N]^b[`N];// overflow
  reg ccz, ccn, ccc, ccv; // CC vector
  always @(posedge clk)
    if (rst)
      {ccz,ccn,ccc,ccv} <= 0;
    else if (valid_insn_ce)
      {ccz,ccn,ccc,ccv} <= {z,n,co,v};
  
  // add/subtract-with-carry state
  always @(posedge clk)
    if (rst)
      c <= 0;
    else if (valid_insn_ce)
      c <= co & (`ALU&(`ADC|`SBC));
 
  // logic unit
  wire [`N:0] log = fn[0] ? a^b : a&b;
  // shift right
  wire [`N:0] sr = {(`SRA?b[`N]:0),b[`N:1]};
 
  // result mux
  wire sum_en = (`ALU&`SUM) | `ADDI;
  assign data = sum_en      ? sum : 16'bz;
  assign data = (`ALU&`LOG) ? log : 16'bz;
  assign data = (`ALU&`SR)  ? sr  : 16'bz;
  assign data = `JAL        ? pc  : 16'bz;
 
  // conditional branch decoding
  `define BR      0
  `define BEQ     2
  `define BC      4
  `define BV      6
  `define BLT     8
  `define BLE     'hA
  `define BLTU    'hC
  `define BLEU    'hE
 
  // conditional branches
  reg br, t;
  always @(hit or cond or op or
           ccz or ccn or ccc or ccv) begin
    case (cond&4'b1110)
    `BR:   t = 1;
    `BEQ:  t = ccz;
    `BC:   t = ccc;
    `BV:   t = ccv;
    `BLT:  t = ccn^ccv;
    `BLE:  t = (ccn^ccv)|ccz;
    `BLTU: t = ~ccz&~ccc;
    `BLEU: t = ccz|~ccc;
    endcase
    br = hit & `Bx & (cond[0] ? ~t : t);
  end
 
  // jumps, branches, insn fetch
  wire [6:0] sxd7   = {7{disp[7]}};
  wire [`N:0] sxd16 = {sxd7,disp,1'b0};
  wire [`N:0] pcinc = br ? sxd16 : {hit,1'b0};
  wire [`N:0] pcincd = pc + pcinc;
  assign i_ad  = (hit & `JAL) ? sum : pcincd;
  always @(posedge clk)
    if (rst)
      pc <= i_ad_rst;
    else if (valid_insn_ce)
      pc <= i_ad;
  wire   mem     = hit & (`LB|`LW|`SB|`SW);
  assign insn_ce = rst | ~(mem & ~rdy);
 
  // data loads, stores
  assign d_ad = sum;
  assign do = dreg;
  assign lw = hit & `LW;
  assign lb = hit & `LB;
  assign sw = hit & `SW;
  assign sb = hit & `SB;
 
  // interrupt support
  assign int_en = hit &
               ~(`IMM|`ALU&(`ADC|`SBC|`CMP));
 endmodule
 
 
 module addsub(add, ci, a, b, sum, x, co);
  input  add, ci;
  input  [15:0] a, b;
  output [15:0] sum;
  output x, co;
  assign {co,sum,x}= add ? {a,ci}+{b,1'b1}
                         : {a,ci}-{b,1'b1};
 endmodule
 
 module gr0041(
  clk, rst, i_ad_rst, int_req,
  insn_ce, i_ad, insn, hit, zero_insn,
  d_ad, rdy, sw, sb, do, lw, lb, data);
 
  input  clk;         // clock
  input  rst;         // reset (sync)
  input [`AN:0] i_ad_rst; // reset vector
  input  int_req;     // interrupt request
 
  output insn_ce;     // insn clock enable 
  output [`AN:0] i_ad;// next insn address
  input  [`IN:0] insn;// current insn
  input  hit;         // insn is valid
  output zero_insn;   // force insn to 0000
 
  output [`AN:0] d_ad;// load/store addr
  input  rdy;         // memory ready
  output sw, sb;      // executing sw (sb)
  output [`N:0] do;   // data to store
  output lw, lb;      // executing lw (lb)
  inout  [`N:0] data; // results, load data
 
  wire int_en;        // interrupt enabled
  reg int;            // call intr in progress
 
  // interrupt request rising edge detection
  reg int_req_last, int_pend;
  always @(posedge clk)
    if (rst)
      int_req_last <= 0;
    else
      int_req_last <= int_req;
  always @(posedge clk)
    if (rst)
      int_pend <= 0;
    else if (int)
      int_pend <= 0;
    else if (int_req && ~int_req_last)
      int_pend <= 1;
 
  // insert intr at an auspicious time
  wire int_nxt = int_pend & int_en & ~int;
  always @(posedge clk)
    if (rst)
      int <= 0;
    else if (insn_ce)
      int <= int_nxt;
 
  // on int, fetch 0000 and execute 0002,
  // which is 'jal r0,2(r0)' -- call intr
  assign zero_insn = int_nxt;
  wire [`N:0] insn_int = insn | {int, 1'b0};
 
  gr0040 p(
   .clk(clk), .rst(rst),
   .i_ad_rst(i_ad_rst),
   .insn_ce(insn_ce), .i_ad(i_ad),
   .insn(insn_int), .hit(hit | int),
   .int_en(int_en),
   .d_ad(d_ad), .rdy(rdy),
   .sw(sw), .sb(sb), .do(do),
   .lw(lw), .lb(lb), .data(data));
 endmodule
 
 
// on-chip peripheral bus defines
`define IO       // on-chip periphs enabled
`define CN   31  // ctrl bus MSB
`define CRAN 7   // control reg addr MSB
`define DN   15  // data bus MSB
`define SELN 7   // select bus MSB
 
 
 module soc(clk, rst, par_i, par_o);
  input  clk;         // clock
  input  rst;         // reset (sync)
 
  input  [7:0] par_i; // parallel inputs
  output [7:0] par_o; // parallel outputs
 
  //
  // processor ports and control signals
  //
  wire [`AN:0] i_ad, d_ad;
  wire [`N:0]  insn, do;
  tri  [`N:0]  data;
  wire int_req, zero_insn;
  wire rdy, sw, sb, lw, lb;
 
  gr0041 p(
   .clk(clk), .rst(rst),
   .i_ad_rst(16'h0020), .int_req(int_req),
   .insn_ce(insn_ce), .i_ad(i_ad),
   .insn(insn), .hit(~rst),
   .zero_insn(zero_insn),
   .d_ad(d_ad), .rdy(rdy),
   .sw(sw), .sb(sb), .do(do),
   .lw(lw), .lb(lb), .data(data));
 
  //
  // rdy (wait state) control
  //
  reg loaded; // load data in bram out regs
  always @(posedge clk)
    if (rst)
      loaded <= 0;
    else if (insn_ce)
      loaded <= 0;
    else
      loaded <= (lw|lb);
 
`ifdef IO
  wire io_nxt = d_ad[`AN];
`else
  wire io_nxt = 0;
`endif
 
  reg io; // peripheral I/O access underway
  always @(posedge clk)
    if (rst)
      io <= 0;
    else if (insn_ce)
      io <= 0;
    else
      io <= io_nxt;
  wire io_rdy;
  assign rdy = ~io_nxt&~((lw|lb)&~loaded) |
               io&io_rdy | rst;
 
  //
  // embedded RAM
  //
  wire h_we = ~rst&~io_nxt&(sw|sb&~d_ad[0]);
  wire l_we = ~rst&~io_nxt&(sw|sb&d_ad[0]);
  wire [7:0] do_h = sw ? do[15:8] : do[7:0];
  wire [`N:0] di;
 
  RAMB4_S8_S8 ramh(
   .RSTA(zero_insn), .WEA(1'b0),
   .ENA(insn_ce), .CLKA(clk),
   .ADDRA(i_ad[9:1]), .DIA(8'b0),
   .DOA(insn[15:8]),
   .RSTB(rst), .WEB(h_we),
   .ENB(1'b1), .CLKB(clk),
   .ADDRB(d_ad[9:1]), .DIB(do_h),
   .DOB(di[15:8]));
 
  RAMB4_S8_S8 raml(
   .RSTA(zero_insn), .WEA(1'b0),
   .ENA(insn_ce), .CLKA(clk),
   .ADDRA(i_ad[9:1]), .DIA(8'b0),
   .DOA(insn[7:0]),
   .RSTB(rst), .WEB(l_we),
   .ENB(1'b1), .CLKB(clk),
   .ADDRB(d_ad[9:1]), .DIB(do[7:0]),
   .DOB(di[7:0]));
 
  // load data outputs
  wire w_oe = ~io & lw;
  wire l_oe = ~io & (lb&d_ad[0] | lw);
  wire h_oe = ~io & (lb&~d_ad[0]);
  assign data[15:8] = w_oe ? di[15:8] : 8'bz;
  assign data[7:0]  = l_oe ? di[7:0]  : 8'bz;
  assign data[7:0]  = h_oe ? di[15:8] : 8'bz;
  assign data[15:8] = lb   ? 8'b0     : 8'bz;
 
`ifdef IO
  //
  // on-chip peripheral bus
  //
 
  // peripheral data bus store data outputs
  wire swsb = sw | sb;
  assign data[7:0]  = swsb ? do[7:0]  : 8'bz;
  assign data[15:8] = sb   ? do[7:0]  : 8'bz;
  assign data[15:8] = sw   ? do[15:8] : 8'bz;
 
  // control, sel bus encoding
  reg [`AN:0] io_ad;
  always @(posedge clk) io_ad <= d_ad;
  wire [`CN:0] ctrl;
  wire [`SELN:0] sel;
  ctrl_enc enc(
    .clk(clk), .rst(rst), .io(io),
    .io_ad(io_ad), .lw(lw), .lb(lb), .sw(sw),
    .sb(sb), .ctrl(ctrl), .sel(sel));
  wire [`SELN:0] per_rdy;
  assign io_rdy  = | (sel & per_rdy);
 
  //
  // peripherals
  //
 
  timer timer(
    .ctrl(ctrl), .data(data),
    .sel(sel[0]), .rdy(per_rdy[0]),
    .int_req(int_req), .i(1'b1),
    .cnt_init(16'hFFC0));
 
  pario par(
    .ctrl(ctrl), .data(data),
    .sel(sel[1]), .rdy(per_rdy[1]),
    .i(par_i), .o(par_o));
 
`else
  assign int_req = 0;
`endif
 
 endmodule
 
 
`ifdef IO
 
 module ctrl_enc(
  clk, rst, io, io_ad, lw, lb, sw, sb,
  ctrl, sel);
 
  input  clk;
  input  rst;
  input  io;
  input  [`AN:0] io_ad;
  input  lw, lb, sw, sb;
  output [`CN:0] ctrl;
  output [`SELN:0] sel;
 
  // on-chip bus abstract control bus
  wire [3:0] oe, we;
  assign oe[0] = io & (lw | lb);
  assign oe[1] = io & lw;
  assign oe[2] = 0;
  assign oe[3] = 0;
  assign we[0] = io & (sw | sb);
  assign we[1] = io & sw;
  assign we[2] = 0;
  assign we[3] = 0;
  assign ctrl={oe,we,io_ad[`CRAN:0],rst,clk };
 
  assign sel[0] = io & (io_ad[11:8] == 0);
  assign sel[1] = io & (io_ad[11:8] == 1);
  assign sel[2] = io & (io_ad[11:8] == 2);
  assign sel[3] = io & (io_ad[11:8] == 3);
  assign sel[4] = io & (io_ad[11:8] == 4);
  assign sel[5] = io & (io_ad[11:8] == 5);
  assign sel[6] = io & (io_ad[11:8] == 6);
  assign sel[7] = io & (io_ad[11:8] == 7);
 endmodule
 
 
 module ctrl_dec(
  ctrl, sel, clk, rst, oe, we, ad);
 
  input  [`CN:0] ctrl;// abstract control bus
  input  sel;         // peripheral select
  output clk;         // clock
  output rst;         // reset
  output [3:0] oe;    // byte output enables 
  output [3:0] we;    // byte wire enables
  output [`CRAN:0] ad;// ctrl reg addr
  
  wire [3:0] oe_, we_;
  assign { oe_, we_, ad, rst, clk } = ctrl;
  assign oe[0] = sel & oe_[0];
  assign oe[1] = sel & oe_[1];
  assign oe[2] = sel & oe_[2];
  assign oe[3] = sel & oe_[3];
  assign we[0] = sel & we_[0];
  assign we[1] = sel & we_[1];
  assign we[2] = sel & we_[2];
  assign we[3] = sel & we_[3];
 endmodule
 
 
 // 8-bit parallel I/O peripheral
 module pario(ctrl, data, sel, rdy, i, o);
  input  [`CN:0] ctrl;
  inout  [`DN:0] data;
  input  sel;
  output rdy;
  input  [7:0] i;
  output [7:0] o;
  reg    [7:0] o;
 
  wire clk;
  wire [3:0] oe, we;
  ctrl_dec d(.ctrl(ctrl), .sel(sel),
             .clk(clk), .oe(oe), .we(we));
  assign rdy = sel;
 
  always @(posedge clk)
    if (we[0])
      o <= data[7:0];
  assign data[7:0] = oe[0] ? i[7:0] : 8'bz;
 endmodule
 
 // 16-bit timer/counter peripheral
 module timer(
  ctrl, data, sel, rdy, int_req, i, cnt_init);
 
  input  [`CN:0] ctrl;
  inout  [`DN:0] data;
  input  sel;
  output rdy, int_req;
  input  i;
  input  [15:0] cnt_init;
 
  wire clk, rst;
  wire [3:0] oe, we;
  wire [`CRAN:0] ad;
  ctrl_dec d(.ctrl(ctrl), .sel(sel),
    .clk(clk), .rst(rst), .oe(oe),
    .we(we), .ad(ad));
  assign rdy = sel;
 
  // CR#0: counter control register
  // * resets to non-interrupting timer
  // * readable
  // * bit 0: int_en: interrupt enable
  // * bit 1: timer: 1 if timer, 0 if counter
  reg timer, int_en;
  always @(posedge clk)
    if (rst)
      {timer,int_en} <= {2'b11};
    else if (we[0] & ~ad[1])
      {timer,int_en} <= data[1:0];
 
  // tick counter when:
  // * timer mode:   i (enabled) on clk
  // * counter mode: i rising edge on clk
  reg i_last;
  always @(posedge clk) i_last <= i;
  wire tick = (timer&i | ~timer&i&~i_last);
 
  // counter/timer
  reg [15:0] cnt;
  wire [15:0] cnt_nxt;
  wire v; // overflow (wrap-around to 0)
  assign {v,cnt_nxt} = cnt + 1;
  always @(posedge clk) begin
    if (rst)
      cnt <= cnt_init;
    else if (tick) begin
      if (v)
        cnt <= cnt_init;
      else
        cnt <= cnt_nxt;
    end
  end
 
  // CR#1: interrupt request register
  // * resets to no-request
  // * readable
  // * bit 0: interrupt request
  // * cleared on writes to CR#1
  // * set on counter overflow with int_en
  reg int_req;
  always @(posedge clk)
    if (rst)
      int_req <= 0;
    else if (we[0] && ad[1])
      int_req <= 0;
    else if (tick && v && int_en)
      int_req <= 1;
 
  // read CR#0 or CR#1
  assign data[1:0]
        = oe[0] ? (ad[1]==0 ? {timer,int_en}
                            : int_req)
                : 2'bz;
 
 endmodule
 
`endif
 module ram16x16d(clk, we, wr_ad, ad,d,wr_o,o)
    /* synthesis syn_hier="hard"*/;
  input  clk;          // write clock
  input  we;           // write enable
  input  [3:0] wr_ad;  // write port addr
  input  [3:0] ad;     // read port addr
  input  [15:0] d;     // write data in
  output [15:0] wr_o;  // write port data out
  output [15:0] o;     // read port data out
`ifdef synthesis
  RAM16X1D r0(
    .A0(wr_ad[0]), .A1(wr_ad[1]),
    .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]),
    .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[0]), .SPO(wr_o[0]), .DPO(o[0]),
    .WCLK(clk), .WE(we))
      /* synthesis xc_props="RLOC=R7C0.S0" */;
  RAM16X1D r1(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[1]), .SPO(wr_o[1]), .DPO(o[1]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R7C0.S1" */;
  RAM16X1D r2(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[2]), .SPO(wr_o[2]), .DPO(o[2]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R6C0.S0" */;
  RAM16X1D r3(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[3]), .SPO(wr_o[3]), .DPO(o[3]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R6C0.S1" */;
  RAM16X1D r4(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[4]), .SPO(wr_o[4]), .DPO(o[4]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R5C0.S0" */;
  RAM16X1D r5(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[5]), .SPO(wr_o[5]), .DPO(o[5]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R5C0.S1" */;
  RAM16X1D r6(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[6]), .SPO(wr_o[6]), .DPO(o[6]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R4C0.S0" */;
  RAM16X1D r7(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[7]), .SPO(wr_o[7]), .DPO(o[7]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R4C0.S1" */;
  RAM16X1D r8(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[8]), .SPO(wr_o[8]), .DPO(o[8]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R3C0.S0" */;
  RAM16X1D r9(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[9]), .SPO(wr_o[9]), .DPO(o[9]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R3C0.S1" */;
  RAM16X1D r10(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[10]), .SPO(wr_o[10]), .DPO(o[10]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R2C0.S0" */;
  RAM16X1D r11(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[11]), .SPO(wr_o[11]), .DPO(o[11]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R2C0.S1" */;
  RAM16X1D r12(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[12]), .SPO(wr_o[12]), .DPO(o[12]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R1C0.S0" */;
  RAM16X1D r13(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[13]), .SPO(wr_o[13]), .DPO(o[13]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R1C0.S1" */;
  RAM16X1D r14(
    .A0(wr_ad[0]), .A1(wr_ad[1]), .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]), .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[14]), .SPO(wr_o[14]), .DPO(o[14]), .WCLK(clk), .WE(we))
    /* synthesis xc_props="RLOC=R0C0.S0" */;
  RAM16X1D r15(
    .A0(wr_ad[0]), .A1(wr_ad[1]),
    .A2(wr_ad[2]), .A3(wr_ad[3]),
    .DPRA0(ad[0]), .DPRA1(ad[1]),
    .DPRA2(ad[2]), .DPRA3(ad[3]),
    .D(d[15]), .SPO(wr_o[15]), .DPO(o[15]),
    .WCLK(clk), .WE(we))
      /* synthesis xc_props="RLOC=R0C0.S1" */;
`else /* !synthesis */
  reg [15:0] mem [15:0];
  reg [4:0] i;
  initial begin
        for (i = 0; i < 16; i = i + 1)
      mem[i] = 0;
  end
  always @(posedge clk) begin
    if (we)
      mem[wr_ad] = d;
  end
  assign o    = mem[ad];
  assign wr_o = mem[wr_ad];
`endif
 endmodule
