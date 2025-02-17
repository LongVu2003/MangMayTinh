// Copyright (C) 2000, Gray Research LLC.
// All rights reserved.  Use subject to the
// XSOC License Agreement, see LICENSE file,
// http://www.fpgapcu.org/xsoc/LICENSE.html.
// Version 2001.03.09

module RAMB4_S8_S8(
	RSTA, WEA, ENA, CLKA,
	ADDRA, DIA, DOA,
	RSTB, WEB, ENB, CLKB,
	ADDRB, DIB, DOB);
	input WEA, ENA, RSTA, CLKA;
	input [8:0]	ADDRA;
	input [7:0]	DIA;
	output [7:0] DOA;
	input WEB, ENB, RSTB, CLKB;
	input [8:0]	ADDRB;
	input [7:0]	DIB;
	output [7:0] DOB;

	reg [7:0] mem [0:511];
	reg [7:0] DOA, DOB;

	reg [15:0]	pc;
	initial begin
		DOA = 0;
		DOB = 0;
	end

	always @(posedge CLKA) begin
		if (ENA && WEA)
			mem[ADDRA] <= DIA;
	end
	always @(posedge CLKB) begin
		if (ENB && WEB)
			mem[ADDRB] <= DIB;
	end
	always @(posedge CLKA) begin
		if (ENA) begin
			if (RSTA)
				DOA <= 0;
			else if (WEA)
				DOA <= DIA;
			else
				DOA <= mem[ADDRA];
		end
	end
	always @(posedge CLKB) begin
		if (ENB) begin
			if (RSTB)
				DOB <= 0;
			else if (WEB)
				DOB <= DIB;
			else
				DOB <= mem[ADDRB];
		end
	end
endmodule
