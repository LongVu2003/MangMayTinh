/* soc-gr-tb.v -- test bench (simulation only)
 *
 * Copyright (C) 1999, 2000, Gray Research LLC.  All rights reserved.
 * The contents of this file are subject to the XSOC License Agreement;
 * you may not use this file except in compliance with this Agreement.
 * See the LICENSE file.
 *
 * (Derived from code originally contributed by Mike Butts)
 */ 

//`timescale 1ns / 100ps

module grt();
	parameter W=16;   // width
	parameter N=W-1;  // index of MSB
	parameter AN=W-1; // address MSB

	reg clk, rst_nxt, rst;
	reg intreq;

    wire [7:0] i = 'h11;
    wire [7:0] o;

	soc s(clk, rst, i, o);

	initial begin
		$readmemh ("raml.mem", s.raml.mem);
		$readmemh ("ramh.mem", s.ramh.mem);
		clk = 1;
		rst = 1;
		rst_nxt = 1;
		intreq = 0;
		$display($time,, "                                                                                    H                                        L");
	end

	always #5 clk = ~clk;	// 100 MHz

	always @(posedge clk) begin
		rst <= rst_nxt;
		rst_nxt <= 0;
	end
	
	always @(posedge clk) begin
		#1
		if ($time % 200 == 101) intreq = 1;
		else if ($time % 200 == 111) intreq = 0;
		#1
		
		if ($time % 200 == 12)
			$display($time,, "ip ok in rs ce   pc i_ad insn hi rf    a    b  sum  d_ad rd sw sb dout  lw lb data  rs we en ada di do  rs we en adb di do   rs we en ada di do  rs we en adb di do");
		$display($time,, " %h  %h  %h  %h  %h %4h %4h %4h  %h  %h %4h %4h %4h  %4h  %h  %h  %h %4h   %h  %h %4h   %h  %h  %h %3h %2h %2h   %h  %h  %h %3h %2h %2h    %h  %h  %h %3h %2h %2h   %h  %h  %h %3h %2h %2h",
			s.p.int_pend, s.p.int_en, s.p.int, s.rst, s.insn_ce, s.p.p.pc, s.i_ad, s.p.p.insn, s.p.p.hit, s.p.p.rf_we, s.p.p.a, s.p.p.b, s.p.p.sum, s.d_ad, s.rdy, s.sw, s.sb, s.do, s.lw, s.lb, s.data,
    s.ramh.RSTA, s.ramh.WEA, s.ramh.ENA,
	s.ramh.ADDRA, s.ramh.DIA, s.ramh.DOA,
	s.ramh.RSTB, s.ramh.WEB, s.ramh.ENB,
	s.ramh.ADDRB, s.ramh.DIB, s.ramh.DOB,

    s.raml.RSTA, s.raml.WEA, s.raml.ENA,
	s.raml.ADDRA, s.raml.DIA, s.raml.DOA,
	s.raml.RSTB, s.raml.WEB, s.raml.ENB,
	s.raml.ADDRB, s.raml.DIB, s.raml.DOB);
		if ($time > 5000 || ($time > 100 && s.i_ad == 'h20)) $finish;
	end
endmodule
