	
module du #(parameter dw=8, lat=32)	
	(	
	input 				clk,	
	input 				stall,	
	input [dw-1:0]		din,	
	output [dw-1:0] 	dout );	
	generate	
		if(lat==0) begin : du_0	
			assign dout = din;	
		end	
		else begin : du_pipe	
			reg [dw-1:0] D [0:lat-1];	
			integer i;	
			always @(posedge clk) begin	
				if(!stall) begin	
					D[0] <= din;	
					for(i=1; i<lat; i=i+1) begin	
						D[i] <= D[i-1];	
					end	
				end	
			end	
			assign dout = D[lat-1];	
		end	
	endgenerate	
endmodule	


	
module du_s #(parameter dw=8)	
	(	
	input 				clk,	
	input 				stall,	
	input [dw-1:0]		din,	
	output [dw-1:0] 	dout );	
	reg [dw-1:0] dout_t;	
	wire [dw-1:0] dout_t_w;	
	always @(posedge clk) begin	
		dout_t = din;	
	end	
	assign dout_t_w = dout_t;	
	assign dout = dout_t_w;	
endmodule	


module lut8_BRAM (
  input clk,
  input [7:0]lut8_BRAM_i_0,
  input [7:0]lut8_BRAM_i_1,
  output reg [3:0]lut8_BRAM_o_0,
  output reg [3:0]lut8_BRAM_o_1
);

 (* rom_style = "block" *)
 reg [3:0] lut8w[255:0];

 initial begin
  lut8w[0] = 4'b0000;
  lut8w[1] = 4'b0001;
  lut8w[2] = 4'b0001;
  lut8w[3] = 4'b0010;
  lut8w[4] = 4'b0001;
  lut8w[5] = 4'b0010;
  lut8w[6] = 4'b0010;
  lut8w[7] = 4'b0011;
  lut8w[8] = 4'b0001;
  lut8w[9] = 4'b0010;
  lut8w[10] = 4'b0010;
  lut8w[11] = 4'b0011;
  lut8w[12] = 4'b0010;
  lut8w[13] = 4'b0011;
  lut8w[14] = 4'b0011;
  lut8w[15] = 4'b0100;
  lut8w[16] = 4'b0001;
  lut8w[17] = 4'b0010;
  lut8w[18] = 4'b0010;
  lut8w[19] = 4'b0011;
  lut8w[20] = 4'b0010;
  lut8w[21] = 4'b0011;
  lut8w[22] = 4'b0011;
  lut8w[23] = 4'b0100;
  lut8w[24] = 4'b0010;
  lut8w[25] = 4'b0011;
  lut8w[26] = 4'b0011;
  lut8w[27] = 4'b0100;
  lut8w[28] = 4'b0011;
  lut8w[29] = 4'b0100;
  lut8w[30] = 4'b0100;
  lut8w[31] = 4'b0101;
  lut8w[32] = 4'b0001;
  lut8w[33] = 4'b0010;
  lut8w[34] = 4'b0010;
  lut8w[35] = 4'b0011;
  lut8w[36] = 4'b0010;
  lut8w[37] = 4'b0011;
  lut8w[38] = 4'b0011;
  lut8w[39] = 4'b0100;
  lut8w[40] = 4'b0010;
  lut8w[41] = 4'b0011;
  lut8w[42] = 4'b0011;
  lut8w[43] = 4'b0100;
  lut8w[44] = 4'b0011;
  lut8w[45] = 4'b0100;
  lut8w[46] = 4'b0100;
  lut8w[47] = 4'b0101;
  lut8w[48] = 4'b0010;
  lut8w[49] = 4'b0011;
  lut8w[50] = 4'b0011;
  lut8w[51] = 4'b0100;
  lut8w[52] = 4'b0011;
  lut8w[53] = 4'b0100;
  lut8w[54] = 4'b0100;
  lut8w[55] = 4'b0101;
  lut8w[56] = 4'b0011;
  lut8w[57] = 4'b0100;
  lut8w[58] = 4'b0100;
  lut8w[59] = 4'b0101;
  lut8w[60] = 4'b0100;
  lut8w[61] = 4'b0101;
  lut8w[62] = 4'b0101;
  lut8w[63] = 4'b0110;
  lut8w[64] = 4'b0001;
  lut8w[65] = 4'b0010;
  lut8w[66] = 4'b0010;
  lut8w[67] = 4'b0011;
  lut8w[68] = 4'b0010;
  lut8w[69] = 4'b0011;
  lut8w[70] = 4'b0011;
  lut8w[71] = 4'b0100;
  lut8w[72] = 4'b0010;
  lut8w[73] = 4'b0011;
  lut8w[74] = 4'b0011;
  lut8w[75] = 4'b0100;
  lut8w[76] = 4'b0011;
  lut8w[77] = 4'b0100;
  lut8w[78] = 4'b0100;
  lut8w[79] = 4'b0101;
  lut8w[80] = 4'b0010;
  lut8w[81] = 4'b0011;
  lut8w[82] = 4'b0011;
  lut8w[83] = 4'b0100;
  lut8w[84] = 4'b0011;
  lut8w[85] = 4'b0100;
  lut8w[86] = 4'b0100;
  lut8w[87] = 4'b0101;
  lut8w[88] = 4'b0011;
  lut8w[89] = 4'b0100;
  lut8w[90] = 4'b0100;
  lut8w[91] = 4'b0101;
  lut8w[92] = 4'b0100;
  lut8w[93] = 4'b0101;
  lut8w[94] = 4'b0101;
  lut8w[95] = 4'b0110;
  lut8w[96] = 4'b0010;
  lut8w[97] = 4'b0011;
  lut8w[98] = 4'b0011;
  lut8w[99] = 4'b0100;
  lut8w[100] = 4'b0011;
  lut8w[101] = 4'b0100;
  lut8w[102] = 4'b0100;
  lut8w[103] = 4'b0101;
  lut8w[104] = 4'b0011;
  lut8w[105] = 4'b0100;
  lut8w[106] = 4'b0100;
  lut8w[107] = 4'b0101;
  lut8w[108] = 4'b0100;
  lut8w[109] = 4'b0101;
  lut8w[110] = 4'b0101;
  lut8w[111] = 4'b0110;
  lut8w[112] = 4'b0011;
  lut8w[113] = 4'b0100;
  lut8w[114] = 4'b0100;
  lut8w[115] = 4'b0101;
  lut8w[116] = 4'b0100;
  lut8w[117] = 4'b0101;
  lut8w[118] = 4'b0101;
  lut8w[119] = 4'b0110;
  lut8w[120] = 4'b0100;
  lut8w[121] = 4'b0101;
  lut8w[122] = 4'b0101;
  lut8w[123] = 4'b0110;
  lut8w[124] = 4'b0101;
  lut8w[125] = 4'b0110;
  lut8w[126] = 4'b0110;
  lut8w[127] = 4'b0111;
  lut8w[128] = 4'b0001;
  lut8w[129] = 4'b0010;
  lut8w[130] = 4'b0010;
  lut8w[131] = 4'b0011;
  lut8w[132] = 4'b0010;
  lut8w[133] = 4'b0011;
  lut8w[134] = 4'b0011;
  lut8w[135] = 4'b0100;
  lut8w[136] = 4'b0010;
  lut8w[137] = 4'b0011;
  lut8w[138] = 4'b0011;
  lut8w[139] = 4'b0100;
  lut8w[140] = 4'b0011;
  lut8w[141] = 4'b0100;
  lut8w[142] = 4'b0100;
  lut8w[143] = 4'b0101;
  lut8w[144] = 4'b0010;
  lut8w[145] = 4'b0011;
  lut8w[146] = 4'b0011;
  lut8w[147] = 4'b0100;
  lut8w[148] = 4'b0011;
  lut8w[149] = 4'b0100;
  lut8w[150] = 4'b0100;
  lut8w[151] = 4'b0101;
  lut8w[152] = 4'b0011;
  lut8w[153] = 4'b0100;
  lut8w[154] = 4'b0100;
  lut8w[155] = 4'b0101;
  lut8w[156] = 4'b0100;
  lut8w[157] = 4'b0101;
  lut8w[158] = 4'b0101;
  lut8w[159] = 4'b0110;
  lut8w[160] = 4'b0010;
  lut8w[161] = 4'b0011;
  lut8w[162] = 4'b0011;
  lut8w[163] = 4'b0100;
  lut8w[164] = 4'b0011;
  lut8w[165] = 4'b0100;
  lut8w[166] = 4'b0100;
  lut8w[167] = 4'b0101;
  lut8w[168] = 4'b0011;
  lut8w[169] = 4'b0100;
  lut8w[170] = 4'b0100;
  lut8w[171] = 4'b0101;
  lut8w[172] = 4'b0100;
  lut8w[173] = 4'b0101;
  lut8w[174] = 4'b0101;
  lut8w[175] = 4'b0110;
  lut8w[176] = 4'b0011;
  lut8w[177] = 4'b0100;
  lut8w[178] = 4'b0100;
  lut8w[179] = 4'b0101;
  lut8w[180] = 4'b0100;
  lut8w[181] = 4'b0101;
  lut8w[182] = 4'b0101;
  lut8w[183] = 4'b0110;
  lut8w[184] = 4'b0100;
  lut8w[185] = 4'b0101;
  lut8w[186] = 4'b0101;
  lut8w[187] = 4'b0110;
  lut8w[188] = 4'b0101;
  lut8w[189] = 4'b0110;
  lut8w[190] = 4'b0110;
  lut8w[191] = 4'b0111;
  lut8w[192] = 4'b0010;
  lut8w[193] = 4'b0011;
  lut8w[194] = 4'b0011;
  lut8w[195] = 4'b0100;
  lut8w[196] = 4'b0011;
  lut8w[197] = 4'b0100;
  lut8w[198] = 4'b0100;
  lut8w[199] = 4'b0101;
  lut8w[200] = 4'b0011;
  lut8w[201] = 4'b0100;
  lut8w[202] = 4'b0100;
  lut8w[203] = 4'b0101;
  lut8w[204] = 4'b0100;
  lut8w[205] = 4'b0101;
  lut8w[206] = 4'b0101;
  lut8w[207] = 4'b0110;
  lut8w[208] = 4'b0011;
  lut8w[209] = 4'b0100;
  lut8w[210] = 4'b0100;
  lut8w[211] = 4'b0101;
  lut8w[212] = 4'b0100;
  lut8w[213] = 4'b0101;
  lut8w[214] = 4'b0101;
  lut8w[215] = 4'b0110;
  lut8w[216] = 4'b0100;
  lut8w[217] = 4'b0101;
  lut8w[218] = 4'b0101;
  lut8w[219] = 4'b0110;
  lut8w[220] = 4'b0101;
  lut8w[221] = 4'b0110;
  lut8w[222] = 4'b0110;
  lut8w[223] = 4'b0111;
  lut8w[224] = 4'b0011;
  lut8w[225] = 4'b0100;
  lut8w[226] = 4'b0100;
  lut8w[227] = 4'b0101;
  lut8w[228] = 4'b0100;
  lut8w[229] = 4'b0101;
  lut8w[230] = 4'b0101;
  lut8w[231] = 4'b0110;
  lut8w[232] = 4'b0100;
  lut8w[233] = 4'b0101;
  lut8w[234] = 4'b0101;
  lut8w[235] = 4'b0110;
  lut8w[236] = 4'b0101;
  lut8w[237] = 4'b0110;
  lut8w[238] = 4'b0110;
  lut8w[239] = 4'b0111;
  lut8w[240] = 4'b0100;
  lut8w[241] = 4'b0101;
  lut8w[242] = 4'b0101;
  lut8w[243] = 4'b0110;
  lut8w[244] = 4'b0101;
  lut8w[245] = 4'b0110;
  lut8w[246] = 4'b0110;
  lut8w[247] = 4'b0111;
  lut8w[248] = 4'b0101;
  lut8w[249] = 4'b0110;
  lut8w[250] = 4'b0110;
  lut8w[251] = 4'b0111;
  lut8w[252] = 4'b0110;
  lut8w[253] = 4'b0111;
  lut8w[254] = 4'b0111;
  lut8w[255] = 4'b1000;
 end

 always @(posedge clk) begin
  lut8_BRAM_o_0 = lut8w[lut8_BRAM_i_0];
  lut8_BRAM_o_1 = lut8w[lut8_BRAM_i_1];
 end

endmodule
module lut8_DIST (
  input clk,
  input [7:0]lut8_DIST_i_0,
  input [7:0]lut8_DIST_i_1,
  output reg [3:0]lut8_DIST_o_0,
  output reg [3:0]lut8_DIST_o_1
);

 (* rom_style = "distributed" *)
 reg [3:0] lut8w[255:0];

 initial begin
  lut8w[0] = 4'b0000;
  lut8w[1] = 4'b0001;
  lut8w[2] = 4'b0001;
  lut8w[3] = 4'b0010;
  lut8w[4] = 4'b0001;
  lut8w[5] = 4'b0010;
  lut8w[6] = 4'b0010;
  lut8w[7] = 4'b0011;
  lut8w[8] = 4'b0001;
  lut8w[9] = 4'b0010;
  lut8w[10] = 4'b0010;
  lut8w[11] = 4'b0011;
  lut8w[12] = 4'b0010;
  lut8w[13] = 4'b0011;
  lut8w[14] = 4'b0011;
  lut8w[15] = 4'b0100;
  lut8w[16] = 4'b0001;
  lut8w[17] = 4'b0010;
  lut8w[18] = 4'b0010;
  lut8w[19] = 4'b0011;
  lut8w[20] = 4'b0010;
  lut8w[21] = 4'b0011;
  lut8w[22] = 4'b0011;
  lut8w[23] = 4'b0100;
  lut8w[24] = 4'b0010;
  lut8w[25] = 4'b0011;
  lut8w[26] = 4'b0011;
  lut8w[27] = 4'b0100;
  lut8w[28] = 4'b0011;
  lut8w[29] = 4'b0100;
  lut8w[30] = 4'b0100;
  lut8w[31] = 4'b0101;
  lut8w[32] = 4'b0001;
  lut8w[33] = 4'b0010;
  lut8w[34] = 4'b0010;
  lut8w[35] = 4'b0011;
  lut8w[36] = 4'b0010;
  lut8w[37] = 4'b0011;
  lut8w[38] = 4'b0011;
  lut8w[39] = 4'b0100;
  lut8w[40] = 4'b0010;
  lut8w[41] = 4'b0011;
  lut8w[42] = 4'b0011;
  lut8w[43] = 4'b0100;
  lut8w[44] = 4'b0011;
  lut8w[45] = 4'b0100;
  lut8w[46] = 4'b0100;
  lut8w[47] = 4'b0101;
  lut8w[48] = 4'b0010;
  lut8w[49] = 4'b0011;
  lut8w[50] = 4'b0011;
  lut8w[51] = 4'b0100;
  lut8w[52] = 4'b0011;
  lut8w[53] = 4'b0100;
  lut8w[54] = 4'b0100;
  lut8w[55] = 4'b0101;
  lut8w[56] = 4'b0011;
  lut8w[57] = 4'b0100;
  lut8w[58] = 4'b0100;
  lut8w[59] = 4'b0101;
  lut8w[60] = 4'b0100;
  lut8w[61] = 4'b0101;
  lut8w[62] = 4'b0101;
  lut8w[63] = 4'b0110;
  lut8w[64] = 4'b0001;
  lut8w[65] = 4'b0010;
  lut8w[66] = 4'b0010;
  lut8w[67] = 4'b0011;
  lut8w[68] = 4'b0010;
  lut8w[69] = 4'b0011;
  lut8w[70] = 4'b0011;
  lut8w[71] = 4'b0100;
  lut8w[72] = 4'b0010;
  lut8w[73] = 4'b0011;
  lut8w[74] = 4'b0011;
  lut8w[75] = 4'b0100;
  lut8w[76] = 4'b0011;
  lut8w[77] = 4'b0100;
  lut8w[78] = 4'b0100;
  lut8w[79] = 4'b0101;
  lut8w[80] = 4'b0010;
  lut8w[81] = 4'b0011;
  lut8w[82] = 4'b0011;
  lut8w[83] = 4'b0100;
  lut8w[84] = 4'b0011;
  lut8w[85] = 4'b0100;
  lut8w[86] = 4'b0100;
  lut8w[87] = 4'b0101;
  lut8w[88] = 4'b0011;
  lut8w[89] = 4'b0100;
  lut8w[90] = 4'b0100;
  lut8w[91] = 4'b0101;
  lut8w[92] = 4'b0100;
  lut8w[93] = 4'b0101;
  lut8w[94] = 4'b0101;
  lut8w[95] = 4'b0110;
  lut8w[96] = 4'b0010;
  lut8w[97] = 4'b0011;
  lut8w[98] = 4'b0011;
  lut8w[99] = 4'b0100;
  lut8w[100] = 4'b0011;
  lut8w[101] = 4'b0100;
  lut8w[102] = 4'b0100;
  lut8w[103] = 4'b0101;
  lut8w[104] = 4'b0011;
  lut8w[105] = 4'b0100;
  lut8w[106] = 4'b0100;
  lut8w[107] = 4'b0101;
  lut8w[108] = 4'b0100;
  lut8w[109] = 4'b0101;
  lut8w[110] = 4'b0101;
  lut8w[111] = 4'b0110;
  lut8w[112] = 4'b0011;
  lut8w[113] = 4'b0100;
  lut8w[114] = 4'b0100;
  lut8w[115] = 4'b0101;
  lut8w[116] = 4'b0100;
  lut8w[117] = 4'b0101;
  lut8w[118] = 4'b0101;
  lut8w[119] = 4'b0110;
  lut8w[120] = 4'b0100;
  lut8w[121] = 4'b0101;
  lut8w[122] = 4'b0101;
  lut8w[123] = 4'b0110;
  lut8w[124] = 4'b0101;
  lut8w[125] = 4'b0110;
  lut8w[126] = 4'b0110;
  lut8w[127] = 4'b0111;
  lut8w[128] = 4'b0001;
  lut8w[129] = 4'b0010;
  lut8w[130] = 4'b0010;
  lut8w[131] = 4'b0011;
  lut8w[132] = 4'b0010;
  lut8w[133] = 4'b0011;
  lut8w[134] = 4'b0011;
  lut8w[135] = 4'b0100;
  lut8w[136] = 4'b0010;
  lut8w[137] = 4'b0011;
  lut8w[138] = 4'b0011;
  lut8w[139] = 4'b0100;
  lut8w[140] = 4'b0011;
  lut8w[141] = 4'b0100;
  lut8w[142] = 4'b0100;
  lut8w[143] = 4'b0101;
  lut8w[144] = 4'b0010;
  lut8w[145] = 4'b0011;
  lut8w[146] = 4'b0011;
  lut8w[147] = 4'b0100;
  lut8w[148] = 4'b0011;
  lut8w[149] = 4'b0100;
  lut8w[150] = 4'b0100;
  lut8w[151] = 4'b0101;
  lut8w[152] = 4'b0011;
  lut8w[153] = 4'b0100;
  lut8w[154] = 4'b0100;
  lut8w[155] = 4'b0101;
  lut8w[156] = 4'b0100;
  lut8w[157] = 4'b0101;
  lut8w[158] = 4'b0101;
  lut8w[159] = 4'b0110;
  lut8w[160] = 4'b0010;
  lut8w[161] = 4'b0011;
  lut8w[162] = 4'b0011;
  lut8w[163] = 4'b0100;
  lut8w[164] = 4'b0011;
  lut8w[165] = 4'b0100;
  lut8w[166] = 4'b0100;
  lut8w[167] = 4'b0101;
  lut8w[168] = 4'b0011;
  lut8w[169] = 4'b0100;
  lut8w[170] = 4'b0100;
  lut8w[171] = 4'b0101;
  lut8w[172] = 4'b0100;
  lut8w[173] = 4'b0101;
  lut8w[174] = 4'b0101;
  lut8w[175] = 4'b0110;
  lut8w[176] = 4'b0011;
  lut8w[177] = 4'b0100;
  lut8w[178] = 4'b0100;
  lut8w[179] = 4'b0101;
  lut8w[180] = 4'b0100;
  lut8w[181] = 4'b0101;
  lut8w[182] = 4'b0101;
  lut8w[183] = 4'b0110;
  lut8w[184] = 4'b0100;
  lut8w[185] = 4'b0101;
  lut8w[186] = 4'b0101;
  lut8w[187] = 4'b0110;
  lut8w[188] = 4'b0101;
  lut8w[189] = 4'b0110;
  lut8w[190] = 4'b0110;
  lut8w[191] = 4'b0111;
  lut8w[192] = 4'b0010;
  lut8w[193] = 4'b0011;
  lut8w[194] = 4'b0011;
  lut8w[195] = 4'b0100;
  lut8w[196] = 4'b0011;
  lut8w[197] = 4'b0100;
  lut8w[198] = 4'b0100;
  lut8w[199] = 4'b0101;
  lut8w[200] = 4'b0011;
  lut8w[201] = 4'b0100;
  lut8w[202] = 4'b0100;
  lut8w[203] = 4'b0101;
  lut8w[204] = 4'b0100;
  lut8w[205] = 4'b0101;
  lut8w[206] = 4'b0101;
  lut8w[207] = 4'b0110;
  lut8w[208] = 4'b0011;
  lut8w[209] = 4'b0100;
  lut8w[210] = 4'b0100;
  lut8w[211] = 4'b0101;
  lut8w[212] = 4'b0100;
  lut8w[213] = 4'b0101;
  lut8w[214] = 4'b0101;
  lut8w[215] = 4'b0110;
  lut8w[216] = 4'b0100;
  lut8w[217] = 4'b0101;
  lut8w[218] = 4'b0101;
  lut8w[219] = 4'b0110;
  lut8w[220] = 4'b0101;
  lut8w[221] = 4'b0110;
  lut8w[222] = 4'b0110;
  lut8w[223] = 4'b0111;
  lut8w[224] = 4'b0011;
  lut8w[225] = 4'b0100;
  lut8w[226] = 4'b0100;
  lut8w[227] = 4'b0101;
  lut8w[228] = 4'b0100;
  lut8w[229] = 4'b0101;
  lut8w[230] = 4'b0101;
  lut8w[231] = 4'b0110;
  lut8w[232] = 4'b0100;
  lut8w[233] = 4'b0101;
  lut8w[234] = 4'b0101;
  lut8w[235] = 4'b0110;
  lut8w[236] = 4'b0101;
  lut8w[237] = 4'b0110;
  lut8w[238] = 4'b0110;
  lut8w[239] = 4'b0111;
  lut8w[240] = 4'b0100;
  lut8w[241] = 4'b0101;
  lut8w[242] = 4'b0101;
  lut8w[243] = 4'b0110;
  lut8w[244] = 4'b0101;
  lut8w[245] = 4'b0110;
  lut8w[246] = 4'b0110;
  lut8w[247] = 4'b0111;
  lut8w[248] = 4'b0101;
  lut8w[249] = 4'b0110;
  lut8w[250] = 4'b0110;
  lut8w[251] = 4'b0111;
  lut8w[252] = 4'b0110;
  lut8w[253] = 4'b0111;
  lut8w[254] = 4'b0111;
  lut8w[255] = 4'b1000;
 end

 always @(posedge clk) begin
  lut8_DIST_o_0 = lut8w[lut8_DIST_i_0];
  lut8_DIST_o_1 = lut8w[lut8_DIST_i_1];
 end

endmodule


module popcnt_bram_dsp_128_7 (
input clk,
input	[127:0] input_v,
output	[7:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] input_8;
wire [7:0] input_9;
wire [7:0] input_10;
wire [7:0] input_11;
wire [7:0] input_12;
wire [7:0] input_13;
wire [7:0] input_14;
wire [7:0] input_15;
wire [7:0] lut8_BRAM_i_0;
wire [7:0] lut8_BRAM_i_1;
wire [7:0] lut8_BRAM_i_2;
wire [7:0] lut8_BRAM_i_3;
wire [7:0] lut8_BRAM_i_4;
wire [7:0] lut8_BRAM_i_5;
wire [7:0] lut8_BRAM_i_6;
wire [7:0] lut8_BRAM_i_7;
wire [7:0] lut8_BRAM_i_8;
wire [7:0] lut8_BRAM_i_9;
wire [7:0] lut8_BRAM_i_10;
wire [7:0] lut8_BRAM_i_11;
wire [7:0] lut8_BRAM_i_12;
wire [7:0] lut8_BRAM_i_13;
wire [7:0] lut8_BRAM_i_14;
wire [7:0] lut8_BRAM_i_15;
wire [3:0] lut8_BRAM_o_0;
wire [3:0] lut8_BRAM_o_1;
wire [3:0] lut8_BRAM_o_2;
wire [3:0] lut8_BRAM_o_3;
wire [3:0] lut8_BRAM_o_4;
wire [3:0] lut8_BRAM_o_5;
wire [3:0] lut8_BRAM_o_6;
wire [3:0] lut8_BRAM_o_7;
wire [3:0] lut8_BRAM_o_8;
wire [3:0] lut8_BRAM_o_9;
wire [3:0] lut8_BRAM_o_10;
wire [3:0] lut8_BRAM_o_11;
wire [3:0] lut8_BRAM_o_12;
wire [3:0] lut8_BRAM_o_13;
wire [3:0] lut8_BRAM_o_14;
wire [3:0] lut8_BRAM_o_15;
assign input_0[7:0] = input_v[127:120];
assign input_1[7:0] = input_v[119:112];
assign input_2[7:0] = input_v[111:104];
assign input_3[7:0] = input_v[103:96];
assign input_4[7:0] = input_v[95:88];
assign input_5[7:0] = input_v[87:80];
assign input_6[7:0] = input_v[79:72];
assign input_7[7:0] = input_v[71:64];
assign input_8[7:0] = input_v[63:56];
assign input_9[7:0] = input_v[55:48];
assign input_10[7:0] = input_v[47:40];
assign input_11[7:0] = input_v[39:32];
assign input_12[7:0] = input_v[31:24];
assign input_13[7:0] = input_v[23:16];
assign input_14[7:0] = input_v[15:8];
assign input_15[7:0] = input_v[7:0];
assign lut8_BRAM_i_0[7:0] = input_0[7:0];
assign lut8_BRAM_i_1[7:0] = input_1[7:0];
assign lut8_BRAM_i_2[7:0] = input_2[7:0];
assign lut8_BRAM_i_3[7:0] = input_3[7:0];
assign lut8_BRAM_i_4[7:0] = input_4[7:0];
assign lut8_BRAM_i_5[7:0] = input_5[7:0];
assign lut8_BRAM_i_6[7:0] = input_6[7:0];
assign lut8_BRAM_i_7[7:0] = input_7[7:0];
assign lut8_BRAM_i_8[7:0] = input_8[7:0];
assign lut8_BRAM_i_9[7:0] = input_9[7:0];
assign lut8_BRAM_i_10[7:0] = input_10[7:0];
assign lut8_BRAM_i_11[7:0] = input_11[7:0];
assign lut8_BRAM_i_12[7:0] = input_12[7:0];
assign lut8_BRAM_i_13[7:0] = input_13[7:0];
assign lut8_BRAM_i_14[7:0] = input_14[7:0];
assign lut8_BRAM_i_15[7:0] = input_15[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
lut8_BRAM lut8_BRAM_1 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_2),.lut8_BRAM_i_1(lut8_BRAM_i_3),.lut8_BRAM_o_0(lut8_BRAM_o_2),.lut8_BRAM_o_1(lut8_BRAM_o_3) );
lut8_BRAM lut8_BRAM_2 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_4),.lut8_BRAM_i_1(lut8_BRAM_i_5),.lut8_BRAM_o_0(lut8_BRAM_o_4),.lut8_BRAM_o_1(lut8_BRAM_o_5) );
lut8_BRAM lut8_BRAM_3 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_6),.lut8_BRAM_i_1(lut8_BRAM_i_7),.lut8_BRAM_o_0(lut8_BRAM_o_6),.lut8_BRAM_o_1(lut8_BRAM_o_7) );
lut8_BRAM lut8_BRAM_4 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_8),.lut8_BRAM_i_1(lut8_BRAM_i_9),.lut8_BRAM_o_0(lut8_BRAM_o_8),.lut8_BRAM_o_1(lut8_BRAM_o_9) );
lut8_BRAM lut8_BRAM_5 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_10),.lut8_BRAM_i_1(lut8_BRAM_i_11),.lut8_BRAM_o_0(lut8_BRAM_o_10),.lut8_BRAM_o_1(lut8_BRAM_o_11) );
lut8_BRAM lut8_BRAM_6 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_12),.lut8_BRAM_i_1(lut8_BRAM_i_13),.lut8_BRAM_o_0(lut8_BRAM_o_12),.lut8_BRAM_o_1(lut8_BRAM_o_13) );
lut8_BRAM lut8_BRAM_7 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_14),.lut8_BRAM_i_1(lut8_BRAM_i_15),.lut8_BRAM_o_0(lut8_BRAM_o_14),.lut8_BRAM_o_1(lut8_BRAM_o_15) );
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [3:0] add_4_i_0_0;
wire [3:0] add_4_i_0_1;
wire [3:0] add_4_i_2_0;
wire [3:0] add_4_i_2_1;
wire [4:0] add_4_o_0_0;
wire [4:0] add_2_i_0_0;
wire [4:0] add_2_i_0_1;
wire [4:0] add_2_i_1_0;
wire [4:0] add_2_i_1_1;
wire [5:0] add_2_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [3:0] add_7_i_0_0;
wire [3:0] add_7_i_0_1;
wire [3:0] add_7_i_2_0;
wire [3:0] add_7_i_2_1;
wire [4:0] add_7_o_0_0;
wire [4:0] add_5_i_0_0;
wire [4:0] add_5_i_0_1;
wire [4:0] add_5_i_1_0;
wire [4:0] add_5_i_1_1;
wire [5:0] add_5_o_0_0;
wire [5:0] add_1_i_0_0;
wire [5:0] add_1_i_0_1;
wire [5:0] add_1_i_1_0;
wire [5:0] add_1_i_1_1;
wire [6:0] add_1_o_0_0;
wire [3:0] add_10_i_0_0;
wire [3:0] add_10_i_0_1;
wire [3:0] add_10_i_2_0;
wire [3:0] add_10_i_2_1;
wire [4:0] add_10_o_0_0;
wire [3:0] add_11_i_0_0;
wire [3:0] add_11_i_0_1;
wire [3:0] add_11_i_2_0;
wire [3:0] add_11_i_2_1;
wire [4:0] add_11_o_0_0;
wire [4:0] add_9_i_0_0;
wire [4:0] add_9_i_0_1;
wire [4:0] add_9_i_1_0;
wire [4:0] add_9_i_1_1;
wire [5:0] add_9_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [3:0] add_14_i_0_0;
wire [3:0] add_14_i_0_1;
wire [3:0] add_14_i_2_0;
wire [3:0] add_14_i_2_1;
wire [4:0] add_14_o_0_0;
wire [4:0] add_12_i_0_0;
wire [4:0] add_12_i_0_1;
wire [4:0] add_12_i_1_0;
wire [4:0] add_12_i_1_1;
wire [5:0] add_12_o_0_0;
wire [5:0] add_8_i_0_0;
wire [5:0] add_8_i_0_1;
wire [5:0] add_8_i_1_0;
wire [5:0] add_8_i_1_1;
wire [6:0] add_8_o_0_0;
wire [6:0] add_0_i_0_0;
wire [6:0] add_0_i_0_1;
wire [6:0] add_0_i_1_0;
wire [6:0] add_0_i_1_1;
wire [7:0] add_0_o_0_0;
wire [7:0] add_0_o_1_0;
assign add_3_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_3_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
assign add_4_i_0_0[3:0] = lut8_BRAM_o_2[3:0];
assign add_4_i_0_1[3:0] = lut8_BRAM_o_3[3:0];
assign add_6_i_0_0[3:0] = lut8_BRAM_o_4[3:0];
assign add_6_i_0_1[3:0] = lut8_BRAM_o_5[3:0];
assign add_7_i_0_0[3:0] = lut8_BRAM_o_6[3:0];
assign add_7_i_0_1[3:0] = lut8_BRAM_o_7[3:0];
assign add_10_i_0_0[3:0] = lut8_BRAM_o_8[3:0];
assign add_10_i_0_1[3:0] = lut8_BRAM_o_9[3:0];
assign add_11_i_0_0[3:0] = lut8_BRAM_o_10[3:0];
assign add_11_i_0_1[3:0] = lut8_BRAM_o_11[3:0];
assign add_13_i_0_0[3:0] = lut8_BRAM_o_12[3:0];
assign add_13_i_0_1[3:0] = lut8_BRAM_o_13[3:0];
assign add_14_i_0_0[3:0] = lut8_BRAM_o_14[3:0];
assign add_14_i_0_1[3:0] = lut8_BRAM_o_15[3:0];
assign add_2_i_0_0[4:0] = add_3_o_0_0[4:0];
assign add_2_i_0_1[4:0] = add_4_o_0_0[4:0];
assign add_1_i_0_0[5:0] = add_2_o_0_0[5:0];
assign add_5_i_0_0[4:0] = add_6_o_0_0[4:0];
assign add_5_i_0_1[4:0] = add_7_o_0_0[4:0];
assign add_1_i_0_1[5:0] = add_5_o_0_0[5:0];
assign add_0_i_0_0[6:0] = add_1_o_0_0[6:0];
assign add_9_i_0_0[4:0] = add_10_o_0_0[4:0];
assign add_9_i_0_1[4:0] = add_11_o_0_0[4:0];
assign add_8_i_0_0[5:0] = add_9_o_0_0[5:0];
assign add_12_i_0_0[4:0] = add_13_o_0_0[4:0];
assign add_12_i_0_1[4:0] = add_14_o_0_0[4:0];
assign add_8_i_0_1[5:0] = add_12_o_0_0[5:0];
assign add_0_i_0_1[6:0] = add_8_o_0_0[6:0];
du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(4, 2) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_0),.din(add_4_i_0_0));
du #(4, 2) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_2_0 + add_4_i_2_1;

du #(5, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(5, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(4, 2) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_0),.din(add_7_i_0_0));
du #(4, 2) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_2_0 + add_7_i_2_1;

du #(5, 1) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_0),.din(add_5_i_0_0));
du #(5, 1) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_1_0 + add_5_i_1_1;

du #(6, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(6, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_0),.din(add_10_i_0_0));
du #(4, 2) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_2_0 + add_10_i_2_1;

du #(4, 2) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_0),.din(add_11_i_0_0));
du #(4, 2) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_2_0 + add_11_i_2_1;

du #(5, 1) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_0),.din(add_9_i_0_0));
du #(5, 1) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_1_0 + add_9_i_1_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(4, 2) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_0),.din(add_14_i_0_0));
du #(4, 2) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_2_0 + add_14_i_2_1;

du #(5, 1) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_0),.din(add_12_i_0_0));
du #(5, 1) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_1_0 + add_12_i_1_1;

du #(6, 1) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_0),.din(add_8_i_0_0));
du #(6, 1) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_1_0 + add_8_i_1_1;

du #(7, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(7, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(8, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_bram_nodsp_128_7 (
input clk,
input	[127:0] input_v,
output	[7:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] input_8;
wire [7:0] input_9;
wire [7:0] input_10;
wire [7:0] input_11;
wire [7:0] input_12;
wire [7:0] input_13;
wire [7:0] input_14;
wire [7:0] input_15;
wire [7:0] lut8_BRAM_i_0;
wire [7:0] lut8_BRAM_i_1;
wire [7:0] lut8_BRAM_i_2;
wire [7:0] lut8_BRAM_i_3;
wire [7:0] lut8_BRAM_i_4;
wire [7:0] lut8_BRAM_i_5;
wire [7:0] lut8_BRAM_i_6;
wire [7:0] lut8_BRAM_i_7;
wire [7:0] lut8_BRAM_i_8;
wire [7:0] lut8_BRAM_i_9;
wire [7:0] lut8_BRAM_i_10;
wire [7:0] lut8_BRAM_i_11;
wire [7:0] lut8_BRAM_i_12;
wire [7:0] lut8_BRAM_i_13;
wire [7:0] lut8_BRAM_i_14;
wire [7:0] lut8_BRAM_i_15;
wire [3:0] lut8_BRAM_o_0;
wire [3:0] lut8_BRAM_o_1;
wire [3:0] lut8_BRAM_o_2;
wire [3:0] lut8_BRAM_o_3;
wire [3:0] lut8_BRAM_o_4;
wire [3:0] lut8_BRAM_o_5;
wire [3:0] lut8_BRAM_o_6;
wire [3:0] lut8_BRAM_o_7;
wire [3:0] lut8_BRAM_o_8;
wire [3:0] lut8_BRAM_o_9;
wire [3:0] lut8_BRAM_o_10;
wire [3:0] lut8_BRAM_o_11;
wire [3:0] lut8_BRAM_o_12;
wire [3:0] lut8_BRAM_o_13;
wire [3:0] lut8_BRAM_o_14;
wire [3:0] lut8_BRAM_o_15;
assign input_0[7:0] = input_v[127:120];
assign input_1[7:0] = input_v[119:112];
assign input_2[7:0] = input_v[111:104];
assign input_3[7:0] = input_v[103:96];
assign input_4[7:0] = input_v[95:88];
assign input_5[7:0] = input_v[87:80];
assign input_6[7:0] = input_v[79:72];
assign input_7[7:0] = input_v[71:64];
assign input_8[7:0] = input_v[63:56];
assign input_9[7:0] = input_v[55:48];
assign input_10[7:0] = input_v[47:40];
assign input_11[7:0] = input_v[39:32];
assign input_12[7:0] = input_v[31:24];
assign input_13[7:0] = input_v[23:16];
assign input_14[7:0] = input_v[15:8];
assign input_15[7:0] = input_v[7:0];
assign lut8_BRAM_i_0[7:0] = input_0[7:0];
assign lut8_BRAM_i_1[7:0] = input_1[7:0];
assign lut8_BRAM_i_2[7:0] = input_2[7:0];
assign lut8_BRAM_i_3[7:0] = input_3[7:0];
assign lut8_BRAM_i_4[7:0] = input_4[7:0];
assign lut8_BRAM_i_5[7:0] = input_5[7:0];
assign lut8_BRAM_i_6[7:0] = input_6[7:0];
assign lut8_BRAM_i_7[7:0] = input_7[7:0];
assign lut8_BRAM_i_8[7:0] = input_8[7:0];
assign lut8_BRAM_i_9[7:0] = input_9[7:0];
assign lut8_BRAM_i_10[7:0] = input_10[7:0];
assign lut8_BRAM_i_11[7:0] = input_11[7:0];
assign lut8_BRAM_i_12[7:0] = input_12[7:0];
assign lut8_BRAM_i_13[7:0] = input_13[7:0];
assign lut8_BRAM_i_14[7:0] = input_14[7:0];
assign lut8_BRAM_i_15[7:0] = input_15[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
lut8_BRAM lut8_BRAM_1 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_2),.lut8_BRAM_i_1(lut8_BRAM_i_3),.lut8_BRAM_o_0(lut8_BRAM_o_2),.lut8_BRAM_o_1(lut8_BRAM_o_3) );
lut8_BRAM lut8_BRAM_2 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_4),.lut8_BRAM_i_1(lut8_BRAM_i_5),.lut8_BRAM_o_0(lut8_BRAM_o_4),.lut8_BRAM_o_1(lut8_BRAM_o_5) );
lut8_BRAM lut8_BRAM_3 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_6),.lut8_BRAM_i_1(lut8_BRAM_i_7),.lut8_BRAM_o_0(lut8_BRAM_o_6),.lut8_BRAM_o_1(lut8_BRAM_o_7) );
lut8_BRAM lut8_BRAM_4 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_8),.lut8_BRAM_i_1(lut8_BRAM_i_9),.lut8_BRAM_o_0(lut8_BRAM_o_8),.lut8_BRAM_o_1(lut8_BRAM_o_9) );
lut8_BRAM lut8_BRAM_5 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_10),.lut8_BRAM_i_1(lut8_BRAM_i_11),.lut8_BRAM_o_0(lut8_BRAM_o_10),.lut8_BRAM_o_1(lut8_BRAM_o_11) );
lut8_BRAM lut8_BRAM_6 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_12),.lut8_BRAM_i_1(lut8_BRAM_i_13),.lut8_BRAM_o_0(lut8_BRAM_o_12),.lut8_BRAM_o_1(lut8_BRAM_o_13) );
lut8_BRAM lut8_BRAM_7 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_14),.lut8_BRAM_i_1(lut8_BRAM_i_15),.lut8_BRAM_o_0(lut8_BRAM_o_14),.lut8_BRAM_o_1(lut8_BRAM_o_15) );
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [3:0] add_4_i_0_0;
wire [3:0] add_4_i_0_1;
wire [3:0] add_4_i_2_0;
wire [3:0] add_4_i_2_1;
wire [4:0] add_4_o_0_0;
wire [4:0] add_2_i_0_0;
wire [4:0] add_2_i_0_1;
wire [4:0] add_2_i_1_0;
wire [4:0] add_2_i_1_1;
wire [5:0] add_2_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [3:0] add_7_i_0_0;
wire [3:0] add_7_i_0_1;
wire [3:0] add_7_i_2_0;
wire [3:0] add_7_i_2_1;
wire [4:0] add_7_o_0_0;
wire [4:0] add_5_i_0_0;
wire [4:0] add_5_i_0_1;
wire [4:0] add_5_i_1_0;
wire [4:0] add_5_i_1_1;
wire [5:0] add_5_o_0_0;
wire [5:0] add_1_i_0_0;
wire [5:0] add_1_i_0_1;
wire [5:0] add_1_i_1_0;
wire [5:0] add_1_i_1_1;
wire [6:0] add_1_o_0_0;
wire [3:0] add_10_i_0_0;
wire [3:0] add_10_i_0_1;
wire [3:0] add_10_i_2_0;
wire [3:0] add_10_i_2_1;
wire [4:0] add_10_o_0_0;
wire [3:0] add_11_i_0_0;
wire [3:0] add_11_i_0_1;
wire [3:0] add_11_i_2_0;
wire [3:0] add_11_i_2_1;
wire [4:0] add_11_o_0_0;
wire [4:0] add_9_i_0_0;
wire [4:0] add_9_i_0_1;
wire [4:0] add_9_i_1_0;
wire [4:0] add_9_i_1_1;
wire [5:0] add_9_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [3:0] add_14_i_0_0;
wire [3:0] add_14_i_0_1;
wire [3:0] add_14_i_2_0;
wire [3:0] add_14_i_2_1;
wire [4:0] add_14_o_0_0;
wire [4:0] add_12_i_0_0;
wire [4:0] add_12_i_0_1;
wire [4:0] add_12_i_1_0;
wire [4:0] add_12_i_1_1;
wire [5:0] add_12_o_0_0;
wire [5:0] add_8_i_0_0;
wire [5:0] add_8_i_0_1;
wire [5:0] add_8_i_1_0;
wire [5:0] add_8_i_1_1;
wire [6:0] add_8_o_0_0;
wire [6:0] add_0_i_0_0;
wire [6:0] add_0_i_0_1;
wire [6:0] add_0_i_1_0;
wire [6:0] add_0_i_1_1;
wire [7:0] add_0_o_0_0;
wire [7:0] add_0_o_1_0;
assign add_3_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_3_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
assign add_4_i_0_0[3:0] = lut8_BRAM_o_2[3:0];
assign add_4_i_0_1[3:0] = lut8_BRAM_o_3[3:0];
assign add_6_i_0_0[3:0] = lut8_BRAM_o_4[3:0];
assign add_6_i_0_1[3:0] = lut8_BRAM_o_5[3:0];
assign add_7_i_0_0[3:0] = lut8_BRAM_o_6[3:0];
assign add_7_i_0_1[3:0] = lut8_BRAM_o_7[3:0];
assign add_10_i_0_0[3:0] = lut8_BRAM_o_8[3:0];
assign add_10_i_0_1[3:0] = lut8_BRAM_o_9[3:0];
assign add_11_i_0_0[3:0] = lut8_BRAM_o_10[3:0];
assign add_11_i_0_1[3:0] = lut8_BRAM_o_11[3:0];
assign add_13_i_0_0[3:0] = lut8_BRAM_o_12[3:0];
assign add_13_i_0_1[3:0] = lut8_BRAM_o_13[3:0];
assign add_14_i_0_0[3:0] = lut8_BRAM_o_14[3:0];
assign add_14_i_0_1[3:0] = lut8_BRAM_o_15[3:0];
assign add_2_i_0_0[4:0] = add_3_o_0_0[4:0];
assign add_2_i_0_1[4:0] = add_4_o_0_0[4:0];
assign add_1_i_0_0[5:0] = add_2_o_0_0[5:0];
assign add_5_i_0_0[4:0] = add_6_o_0_0[4:0];
assign add_5_i_0_1[4:0] = add_7_o_0_0[4:0];
assign add_1_i_0_1[5:0] = add_5_o_0_0[5:0];
assign add_0_i_0_0[6:0] = add_1_o_0_0[6:0];
assign add_9_i_0_0[4:0] = add_10_o_0_0[4:0];
assign add_9_i_0_1[4:0] = add_11_o_0_0[4:0];
assign add_8_i_0_0[5:0] = add_9_o_0_0[5:0];
assign add_12_i_0_0[4:0] = add_13_o_0_0[4:0];
assign add_12_i_0_1[4:0] = add_14_o_0_0[4:0];
assign add_8_i_0_1[5:0] = add_12_o_0_0[5:0];
assign add_0_i_0_1[6:0] = add_8_o_0_0[6:0];
du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(4, 2) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_0),.din(add_4_i_0_0));
du #(4, 2) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_2_0 + add_4_i_2_1;

du #(5, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(5, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(4, 2) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_0),.din(add_7_i_0_0));
du #(4, 2) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_2_0 + add_7_i_2_1;

du #(5, 1) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_0),.din(add_5_i_0_0));
du #(5, 1) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_1_0 + add_5_i_1_1;

du #(6, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(6, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_0),.din(add_10_i_0_0));
du #(4, 2) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_2_0 + add_10_i_2_1;

du #(4, 2) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_0),.din(add_11_i_0_0));
du #(4, 2) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_2_0 + add_11_i_2_1;

du #(5, 1) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_0),.din(add_9_i_0_0));
du #(5, 1) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_1_0 + add_9_i_1_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(4, 2) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_0),.din(add_14_i_0_0));
du #(4, 2) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_2_0 + add_14_i_2_1;

du #(5, 1) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_0),.din(add_12_i_0_0));
du #(5, 1) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_1_0 + add_12_i_1_1;

du #(6, 1) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_0),.din(add_8_i_0_0));
du #(6, 1) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_1_0 + add_8_i_1_1;

du #(7, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(7, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(8, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_dist_dsp_128_7 (
input clk,
input	[127:0] input_v,
output	[7:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] input_8;
wire [7:0] input_9;
wire [7:0] input_10;
wire [7:0] input_11;
wire [7:0] input_12;
wire [7:0] input_13;
wire [7:0] input_14;
wire [7:0] input_15;
wire [7:0] lut8_DIST_i_0;
wire [7:0] lut8_DIST_i_1;
wire [7:0] lut8_DIST_i_2;
wire [7:0] lut8_DIST_i_3;
wire [7:0] lut8_DIST_i_4;
wire [7:0] lut8_DIST_i_5;
wire [7:0] lut8_DIST_i_6;
wire [7:0] lut8_DIST_i_7;
wire [7:0] lut8_DIST_i_8;
wire [7:0] lut8_DIST_i_9;
wire [7:0] lut8_DIST_i_10;
wire [7:0] lut8_DIST_i_11;
wire [7:0] lut8_DIST_i_12;
wire [7:0] lut8_DIST_i_13;
wire [7:0] lut8_DIST_i_14;
wire [7:0] lut8_DIST_i_15;
wire [3:0] lut8_DIST_o_0;
wire [3:0] lut8_DIST_o_1;
wire [3:0] lut8_DIST_o_2;
wire [3:0] lut8_DIST_o_3;
wire [3:0] lut8_DIST_o_4;
wire [3:0] lut8_DIST_o_5;
wire [3:0] lut8_DIST_o_6;
wire [3:0] lut8_DIST_o_7;
wire [3:0] lut8_DIST_o_8;
wire [3:0] lut8_DIST_o_9;
wire [3:0] lut8_DIST_o_10;
wire [3:0] lut8_DIST_o_11;
wire [3:0] lut8_DIST_o_12;
wire [3:0] lut8_DIST_o_13;
wire [3:0] lut8_DIST_o_14;
wire [3:0] lut8_DIST_o_15;
assign input_0[7:0] = input_v[127:120];
assign input_1[7:0] = input_v[119:112];
assign input_2[7:0] = input_v[111:104];
assign input_3[7:0] = input_v[103:96];
assign input_4[7:0] = input_v[95:88];
assign input_5[7:0] = input_v[87:80];
assign input_6[7:0] = input_v[79:72];
assign input_7[7:0] = input_v[71:64];
assign input_8[7:0] = input_v[63:56];
assign input_9[7:0] = input_v[55:48];
assign input_10[7:0] = input_v[47:40];
assign input_11[7:0] = input_v[39:32];
assign input_12[7:0] = input_v[31:24];
assign input_13[7:0] = input_v[23:16];
assign input_14[7:0] = input_v[15:8];
assign input_15[7:0] = input_v[7:0];
assign lut8_DIST_i_0[7:0] = input_0[7:0];
assign lut8_DIST_i_1[7:0] = input_1[7:0];
assign lut8_DIST_i_2[7:0] = input_2[7:0];
assign lut8_DIST_i_3[7:0] = input_3[7:0];
assign lut8_DIST_i_4[7:0] = input_4[7:0];
assign lut8_DIST_i_5[7:0] = input_5[7:0];
assign lut8_DIST_i_6[7:0] = input_6[7:0];
assign lut8_DIST_i_7[7:0] = input_7[7:0];
assign lut8_DIST_i_8[7:0] = input_8[7:0];
assign lut8_DIST_i_9[7:0] = input_9[7:0];
assign lut8_DIST_i_10[7:0] = input_10[7:0];
assign lut8_DIST_i_11[7:0] = input_11[7:0];
assign lut8_DIST_i_12[7:0] = input_12[7:0];
assign lut8_DIST_i_13[7:0] = input_13[7:0];
assign lut8_DIST_i_14[7:0] = input_14[7:0];
assign lut8_DIST_i_15[7:0] = input_15[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
lut8_DIST lut8_DIST_1 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_2),.lut8_DIST_i_1(lut8_DIST_i_3),.lut8_DIST_o_0(lut8_DIST_o_2),.lut8_DIST_o_1(lut8_DIST_o_3) );
lut8_DIST lut8_DIST_2 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_4),.lut8_DIST_i_1(lut8_DIST_i_5),.lut8_DIST_o_0(lut8_DIST_o_4),.lut8_DIST_o_1(lut8_DIST_o_5) );
lut8_DIST lut8_DIST_3 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_6),.lut8_DIST_i_1(lut8_DIST_i_7),.lut8_DIST_o_0(lut8_DIST_o_6),.lut8_DIST_o_1(lut8_DIST_o_7) );
lut8_DIST lut8_DIST_4 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_8),.lut8_DIST_i_1(lut8_DIST_i_9),.lut8_DIST_o_0(lut8_DIST_o_8),.lut8_DIST_o_1(lut8_DIST_o_9) );
lut8_DIST lut8_DIST_5 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_10),.lut8_DIST_i_1(lut8_DIST_i_11),.lut8_DIST_o_0(lut8_DIST_o_10),.lut8_DIST_o_1(lut8_DIST_o_11) );
lut8_DIST lut8_DIST_6 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_12),.lut8_DIST_i_1(lut8_DIST_i_13),.lut8_DIST_o_0(lut8_DIST_o_12),.lut8_DIST_o_1(lut8_DIST_o_13) );
lut8_DIST lut8_DIST_7 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_14),.lut8_DIST_i_1(lut8_DIST_i_15),.lut8_DIST_o_0(lut8_DIST_o_14),.lut8_DIST_o_1(lut8_DIST_o_15) );
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [3:0] add_4_i_0_0;
wire [3:0] add_4_i_0_1;
wire [3:0] add_4_i_2_0;
wire [3:0] add_4_i_2_1;
wire [4:0] add_4_o_0_0;
wire [4:0] add_2_i_0_0;
wire [4:0] add_2_i_0_1;
wire [4:0] add_2_i_1_0;
wire [4:0] add_2_i_1_1;
wire [5:0] add_2_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [3:0] add_7_i_0_0;
wire [3:0] add_7_i_0_1;
wire [3:0] add_7_i_2_0;
wire [3:0] add_7_i_2_1;
wire [4:0] add_7_o_0_0;
wire [4:0] add_5_i_0_0;
wire [4:0] add_5_i_0_1;
wire [4:0] add_5_i_1_0;
wire [4:0] add_5_i_1_1;
wire [5:0] add_5_o_0_0;
wire [5:0] add_1_i_0_0;
wire [5:0] add_1_i_0_1;
wire [5:0] add_1_i_1_0;
wire [5:0] add_1_i_1_1;
wire [6:0] add_1_o_0_0;
wire [3:0] add_10_i_0_0;
wire [3:0] add_10_i_0_1;
wire [3:0] add_10_i_2_0;
wire [3:0] add_10_i_2_1;
wire [4:0] add_10_o_0_0;
wire [3:0] add_11_i_0_0;
wire [3:0] add_11_i_0_1;
wire [3:0] add_11_i_2_0;
wire [3:0] add_11_i_2_1;
wire [4:0] add_11_o_0_0;
wire [4:0] add_9_i_0_0;
wire [4:0] add_9_i_0_1;
wire [4:0] add_9_i_1_0;
wire [4:0] add_9_i_1_1;
wire [5:0] add_9_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [3:0] add_14_i_0_0;
wire [3:0] add_14_i_0_1;
wire [3:0] add_14_i_2_0;
wire [3:0] add_14_i_2_1;
wire [4:0] add_14_o_0_0;
wire [4:0] add_12_i_0_0;
wire [4:0] add_12_i_0_1;
wire [4:0] add_12_i_1_0;
wire [4:0] add_12_i_1_1;
wire [5:0] add_12_o_0_0;
wire [5:0] add_8_i_0_0;
wire [5:0] add_8_i_0_1;
wire [5:0] add_8_i_1_0;
wire [5:0] add_8_i_1_1;
wire [6:0] add_8_o_0_0;
wire [6:0] add_0_i_0_0;
wire [6:0] add_0_i_0_1;
wire [6:0] add_0_i_1_0;
wire [6:0] add_0_i_1_1;
wire [7:0] add_0_o_0_0;
wire [7:0] add_0_o_1_0;
assign add_3_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_3_i_0_1[3:0] = lut8_DIST_o_1[3:0];
assign add_4_i_0_0[3:0] = lut8_DIST_o_2[3:0];
assign add_4_i_0_1[3:0] = lut8_DIST_o_3[3:0];
assign add_6_i_0_0[3:0] = lut8_DIST_o_4[3:0];
assign add_6_i_0_1[3:0] = lut8_DIST_o_5[3:0];
assign add_7_i_0_0[3:0] = lut8_DIST_o_6[3:0];
assign add_7_i_0_1[3:0] = lut8_DIST_o_7[3:0];
assign add_10_i_0_0[3:0] = lut8_DIST_o_8[3:0];
assign add_10_i_0_1[3:0] = lut8_DIST_o_9[3:0];
assign add_11_i_0_0[3:0] = lut8_DIST_o_10[3:0];
assign add_11_i_0_1[3:0] = lut8_DIST_o_11[3:0];
assign add_13_i_0_0[3:0] = lut8_DIST_o_12[3:0];
assign add_13_i_0_1[3:0] = lut8_DIST_o_13[3:0];
assign add_14_i_0_0[3:0] = lut8_DIST_o_14[3:0];
assign add_14_i_0_1[3:0] = lut8_DIST_o_15[3:0];
assign add_2_i_0_0[4:0] = add_3_o_0_0[4:0];
assign add_2_i_0_1[4:0] = add_4_o_0_0[4:0];
assign add_1_i_0_0[5:0] = add_2_o_0_0[5:0];
assign add_5_i_0_0[4:0] = add_6_o_0_0[4:0];
assign add_5_i_0_1[4:0] = add_7_o_0_0[4:0];
assign add_1_i_0_1[5:0] = add_5_o_0_0[5:0];
assign add_0_i_0_0[6:0] = add_1_o_0_0[6:0];
assign add_9_i_0_0[4:0] = add_10_o_0_0[4:0];
assign add_9_i_0_1[4:0] = add_11_o_0_0[4:0];
assign add_8_i_0_0[5:0] = add_9_o_0_0[5:0];
assign add_12_i_0_0[4:0] = add_13_o_0_0[4:0];
assign add_12_i_0_1[4:0] = add_14_o_0_0[4:0];
assign add_8_i_0_1[5:0] = add_12_o_0_0[5:0];
assign add_0_i_0_1[6:0] = add_8_o_0_0[6:0];
du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(4, 2) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_0),.din(add_4_i_0_0));
du #(4, 2) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_2_0 + add_4_i_2_1;

du #(5, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(5, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(4, 2) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_0),.din(add_7_i_0_0));
du #(4, 2) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_2_0 + add_7_i_2_1;

du #(5, 1) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_0),.din(add_5_i_0_0));
du #(5, 1) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_1_0 + add_5_i_1_1;

du #(6, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(6, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_0),.din(add_10_i_0_0));
du #(4, 2) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_2_0 + add_10_i_2_1;

du #(4, 2) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_0),.din(add_11_i_0_0));
du #(4, 2) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_2_0 + add_11_i_2_1;

du #(5, 1) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_0),.din(add_9_i_0_0));
du #(5, 1) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_1_0 + add_9_i_1_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(4, 2) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_0),.din(add_14_i_0_0));
du #(4, 2) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_2_0 + add_14_i_2_1;

du #(5, 1) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_0),.din(add_12_i_0_0));
du #(5, 1) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_1_0 + add_12_i_1_1;

du #(6, 1) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_0),.din(add_8_i_0_0));
du #(6, 1) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_1_0 + add_8_i_1_1;

du #(7, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(7, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(8, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_dist_nodsp_128_7 (
input clk,
input	[127:0] input_v,
output	[7:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] input_8;
wire [7:0] input_9;
wire [7:0] input_10;
wire [7:0] input_11;
wire [7:0] input_12;
wire [7:0] input_13;
wire [7:0] input_14;
wire [7:0] input_15;
wire [7:0] lut8_DIST_i_0;
wire [7:0] lut8_DIST_i_1;
wire [7:0] lut8_DIST_i_2;
wire [7:0] lut8_DIST_i_3;
wire [7:0] lut8_DIST_i_4;
wire [7:0] lut8_DIST_i_5;
wire [7:0] lut8_DIST_i_6;
wire [7:0] lut8_DIST_i_7;
wire [7:0] lut8_DIST_i_8;
wire [7:0] lut8_DIST_i_9;
wire [7:0] lut8_DIST_i_10;
wire [7:0] lut8_DIST_i_11;
wire [7:0] lut8_DIST_i_12;
wire [7:0] lut8_DIST_i_13;
wire [7:0] lut8_DIST_i_14;
wire [7:0] lut8_DIST_i_15;
wire [3:0] lut8_DIST_o_0;
wire [3:0] lut8_DIST_o_1;
wire [3:0] lut8_DIST_o_2;
wire [3:0] lut8_DIST_o_3;
wire [3:0] lut8_DIST_o_4;
wire [3:0] lut8_DIST_o_5;
wire [3:0] lut8_DIST_o_6;
wire [3:0] lut8_DIST_o_7;
wire [3:0] lut8_DIST_o_8;
wire [3:0] lut8_DIST_o_9;
wire [3:0] lut8_DIST_o_10;
wire [3:0] lut8_DIST_o_11;
wire [3:0] lut8_DIST_o_12;
wire [3:0] lut8_DIST_o_13;
wire [3:0] lut8_DIST_o_14;
wire [3:0] lut8_DIST_o_15;
assign input_0[7:0] = input_v[127:120];
assign input_1[7:0] = input_v[119:112];
assign input_2[7:0] = input_v[111:104];
assign input_3[7:0] = input_v[103:96];
assign input_4[7:0] = input_v[95:88];
assign input_5[7:0] = input_v[87:80];
assign input_6[7:0] = input_v[79:72];
assign input_7[7:0] = input_v[71:64];
assign input_8[7:0] = input_v[63:56];
assign input_9[7:0] = input_v[55:48];
assign input_10[7:0] = input_v[47:40];
assign input_11[7:0] = input_v[39:32];
assign input_12[7:0] = input_v[31:24];
assign input_13[7:0] = input_v[23:16];
assign input_14[7:0] = input_v[15:8];
assign input_15[7:0] = input_v[7:0];
assign lut8_DIST_i_0[7:0] = input_0[7:0];
assign lut8_DIST_i_1[7:0] = input_1[7:0];
assign lut8_DIST_i_2[7:0] = input_2[7:0];
assign lut8_DIST_i_3[7:0] = input_3[7:0];
assign lut8_DIST_i_4[7:0] = input_4[7:0];
assign lut8_DIST_i_5[7:0] = input_5[7:0];
assign lut8_DIST_i_6[7:0] = input_6[7:0];
assign lut8_DIST_i_7[7:0] = input_7[7:0];
assign lut8_DIST_i_8[7:0] = input_8[7:0];
assign lut8_DIST_i_9[7:0] = input_9[7:0];
assign lut8_DIST_i_10[7:0] = input_10[7:0];
assign lut8_DIST_i_11[7:0] = input_11[7:0];
assign lut8_DIST_i_12[7:0] = input_12[7:0];
assign lut8_DIST_i_13[7:0] = input_13[7:0];
assign lut8_DIST_i_14[7:0] = input_14[7:0];
assign lut8_DIST_i_15[7:0] = input_15[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
lut8_DIST lut8_DIST_1 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_2),.lut8_DIST_i_1(lut8_DIST_i_3),.lut8_DIST_o_0(lut8_DIST_o_2),.lut8_DIST_o_1(lut8_DIST_o_3) );
lut8_DIST lut8_DIST_2 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_4),.lut8_DIST_i_1(lut8_DIST_i_5),.lut8_DIST_o_0(lut8_DIST_o_4),.lut8_DIST_o_1(lut8_DIST_o_5) );
lut8_DIST lut8_DIST_3 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_6),.lut8_DIST_i_1(lut8_DIST_i_7),.lut8_DIST_o_0(lut8_DIST_o_6),.lut8_DIST_o_1(lut8_DIST_o_7) );
lut8_DIST lut8_DIST_4 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_8),.lut8_DIST_i_1(lut8_DIST_i_9),.lut8_DIST_o_0(lut8_DIST_o_8),.lut8_DIST_o_1(lut8_DIST_o_9) );
lut8_DIST lut8_DIST_5 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_10),.lut8_DIST_i_1(lut8_DIST_i_11),.lut8_DIST_o_0(lut8_DIST_o_10),.lut8_DIST_o_1(lut8_DIST_o_11) );
lut8_DIST lut8_DIST_6 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_12),.lut8_DIST_i_1(lut8_DIST_i_13),.lut8_DIST_o_0(lut8_DIST_o_12),.lut8_DIST_o_1(lut8_DIST_o_13) );
lut8_DIST lut8_DIST_7 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_14),.lut8_DIST_i_1(lut8_DIST_i_15),.lut8_DIST_o_0(lut8_DIST_o_14),.lut8_DIST_o_1(lut8_DIST_o_15) );
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [3:0] add_4_i_0_0;
wire [3:0] add_4_i_0_1;
wire [3:0] add_4_i_2_0;
wire [3:0] add_4_i_2_1;
wire [4:0] add_4_o_0_0;
wire [4:0] add_2_i_0_0;
wire [4:0] add_2_i_0_1;
wire [4:0] add_2_i_1_0;
wire [4:0] add_2_i_1_1;
wire [5:0] add_2_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [3:0] add_7_i_0_0;
wire [3:0] add_7_i_0_1;
wire [3:0] add_7_i_2_0;
wire [3:0] add_7_i_2_1;
wire [4:0] add_7_o_0_0;
wire [4:0] add_5_i_0_0;
wire [4:0] add_5_i_0_1;
wire [4:0] add_5_i_1_0;
wire [4:0] add_5_i_1_1;
wire [5:0] add_5_o_0_0;
wire [5:0] add_1_i_0_0;
wire [5:0] add_1_i_0_1;
wire [5:0] add_1_i_1_0;
wire [5:0] add_1_i_1_1;
wire [6:0] add_1_o_0_0;
wire [3:0] add_10_i_0_0;
wire [3:0] add_10_i_0_1;
wire [3:0] add_10_i_2_0;
wire [3:0] add_10_i_2_1;
wire [4:0] add_10_o_0_0;
wire [3:0] add_11_i_0_0;
wire [3:0] add_11_i_0_1;
wire [3:0] add_11_i_2_0;
wire [3:0] add_11_i_2_1;
wire [4:0] add_11_o_0_0;
wire [4:0] add_9_i_0_0;
wire [4:0] add_9_i_0_1;
wire [4:0] add_9_i_1_0;
wire [4:0] add_9_i_1_1;
wire [5:0] add_9_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [3:0] add_14_i_0_0;
wire [3:0] add_14_i_0_1;
wire [3:0] add_14_i_2_0;
wire [3:0] add_14_i_2_1;
wire [4:0] add_14_o_0_0;
wire [4:0] add_12_i_0_0;
wire [4:0] add_12_i_0_1;
wire [4:0] add_12_i_1_0;
wire [4:0] add_12_i_1_1;
wire [5:0] add_12_o_0_0;
wire [5:0] add_8_i_0_0;
wire [5:0] add_8_i_0_1;
wire [5:0] add_8_i_1_0;
wire [5:0] add_8_i_1_1;
wire [6:0] add_8_o_0_0;
wire [6:0] add_0_i_0_0;
wire [6:0] add_0_i_0_1;
wire [6:0] add_0_i_1_0;
wire [6:0] add_0_i_1_1;
wire [7:0] add_0_o_0_0;
wire [7:0] add_0_o_1_0;
assign add_3_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_3_i_0_1[3:0] = lut8_DIST_o_1[3:0];
assign add_4_i_0_0[3:0] = lut8_DIST_o_2[3:0];
assign add_4_i_0_1[3:0] = lut8_DIST_o_3[3:0];
assign add_6_i_0_0[3:0] = lut8_DIST_o_4[3:0];
assign add_6_i_0_1[3:0] = lut8_DIST_o_5[3:0];
assign add_7_i_0_0[3:0] = lut8_DIST_o_6[3:0];
assign add_7_i_0_1[3:0] = lut8_DIST_o_7[3:0];
assign add_10_i_0_0[3:0] = lut8_DIST_o_8[3:0];
assign add_10_i_0_1[3:0] = lut8_DIST_o_9[3:0];
assign add_11_i_0_0[3:0] = lut8_DIST_o_10[3:0];
assign add_11_i_0_1[3:0] = lut8_DIST_o_11[3:0];
assign add_13_i_0_0[3:0] = lut8_DIST_o_12[3:0];
assign add_13_i_0_1[3:0] = lut8_DIST_o_13[3:0];
assign add_14_i_0_0[3:0] = lut8_DIST_o_14[3:0];
assign add_14_i_0_1[3:0] = lut8_DIST_o_15[3:0];
assign add_2_i_0_0[4:0] = add_3_o_0_0[4:0];
assign add_2_i_0_1[4:0] = add_4_o_0_0[4:0];
assign add_1_i_0_0[5:0] = add_2_o_0_0[5:0];
assign add_5_i_0_0[4:0] = add_6_o_0_0[4:0];
assign add_5_i_0_1[4:0] = add_7_o_0_0[4:0];
assign add_1_i_0_1[5:0] = add_5_o_0_0[5:0];
assign add_0_i_0_0[6:0] = add_1_o_0_0[6:0];
assign add_9_i_0_0[4:0] = add_10_o_0_0[4:0];
assign add_9_i_0_1[4:0] = add_11_o_0_0[4:0];
assign add_8_i_0_0[5:0] = add_9_o_0_0[5:0];
assign add_12_i_0_0[4:0] = add_13_o_0_0[4:0];
assign add_12_i_0_1[4:0] = add_14_o_0_0[4:0];
assign add_8_i_0_1[5:0] = add_12_o_0_0[5:0];
assign add_0_i_0_1[6:0] = add_8_o_0_0[6:0];
du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(4, 2) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_0),.din(add_4_i_0_0));
du #(4, 2) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_2_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_2_0 + add_4_i_2_1;

du #(5, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(5, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(4, 2) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_0),.din(add_7_i_0_0));
du #(4, 2) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_2_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_2_0 + add_7_i_2_1;

du #(5, 1) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_0),.din(add_5_i_0_0));
du #(5, 1) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_1_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_1_0 + add_5_i_1_1;

du #(6, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(6, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_0),.din(add_10_i_0_0));
du #(4, 2) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_2_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_2_0 + add_10_i_2_1;

du #(4, 2) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_0),.din(add_11_i_0_0));
du #(4, 2) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_2_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_2_0 + add_11_i_2_1;

du #(5, 1) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_0),.din(add_9_i_0_0));
du #(5, 1) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_1_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_1_0 + add_9_i_1_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(4, 2) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_0),.din(add_14_i_0_0));
du #(4, 2) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_2_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_2_0 + add_14_i_2_1;

du #(5, 1) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_0),.din(add_12_i_0_0));
du #(5, 1) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_1_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_1_0 + add_12_i_1_1;

du #(6, 1) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_0),.din(add_8_i_0_0));
du #(6, 1) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_1_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_1_0 + add_8_i_1_1;

du #(7, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(7, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(8, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
	
module accum #(	
	parameter IPORT_WIDTH=6,	
				 OPORT_WIDTH=32	
)	
(	
	input clk,	
	input rst,	
	input [IPORT_WIDTH-1:0] valin,	
	output reg[OPORT_WIDTH-1:0] valout	
);	
	
	wire [OPORT_WIDTH-1:0] valin_ext;	
	
	assign valin_ext[IPORT_WIDTH-1:0] = valin;	
	assign valin_ext[OPORT_WIDTH-1:IPORT_WIDTH] = {OPORT_WIDTH - IPORT_WIDTH{1'b0}};	
		
	always @(posedge clk)	
	begin	
		if(rst)		
			valout <= valin_ext;//{OPORT_WIDTH {1'b0}};	
		else	
			valout <= valout + valin_ext;	
	end	
		
endmodule	


module applogic_0	#(					
	parameter PORT_WIDTH = 6					
)					
(					
	input vld_in,					
	input [PORT_WIDTH-1:0] val_i,					
	output [PORT_WIDTH-1:0] val_o					
);				   	
	
	wire [PORT_WIDTH-1:0] vld_v; 
						
	assign vld_v = {PORT_WIDTH-1{vld_in}};						
	assign val_o = val_i & vld_v;						
		
endmodule

		module applogic_1		#(					
	parameter PORT_WIDTH = 6					
)					
(					
	input [PORT_WIDTH-1:0] val_i_0,					
	input [PORT_WIDTH-1:0] val_i_1,					
	output [PORT_WIDTH-1:0] val_o					
);				   	
						
	assign val_o = val_i_0 & val_i_1;						
		
endmodule

		module gblock_0			#(					
	parameter IPORT_WIDTH = 128,					
		  OPORT_WIDTH = 32					
)					
(					
	input clk,					
	input rst,					
	input vld_in,					
	input [IPORT_WIDTH-1:0] val_i,					
	output [IPORT_WIDTH-1:0] val_i_t,					
	output [OPORT_WIDTH-1:0] val_o					
);
				  	 
		
	wire [IPORT_WIDTH-1:0] t_l_val_o;   
	wire [7:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_0 #(IPORT_WIDTH) l_inst (.vld_in(vld_in), .val_i(val_i), .val_o(t_l_val_o));					
	assign val_i_t = t_l_val_o;				   	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_128_7 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(8, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1			#(					
	parameter IPORT_WIDTH = 128,					
		  OPORT_WIDTH = 32					
)					
(					
	input clk,					
	input rst,					
	input [IPORT_WIDTH-1:0] val_i_0,					
	input [IPORT_WIDTH-1:0] val_i_1,					
	output [OPORT_WIDTH-1:0] val_o					
);
				   	
		
	wire [IPORT_WIDTH-1:0] t_l_val_o;   
	wire [7:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_128_7 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(8, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1_special			#(					
	parameter IPORT_WIDTH = 128,					
		  OPORT_WIDTH = 32					
)					
(					
	input clk,					
	input rst,					
	input [IPORT_WIDTH-1:0] val_i_0,					
	input [IPORT_WIDTH-1:0] val_i_1,					
	output [OPORT_WIDTH-1:0] val_o					
);
				   	
		
	wire [IPORT_WIDTH-1:0] t_l_val_o;   
	wire [7:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_128_7 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(8, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule



module pw_sim_8_1_128_7 (
input clk,
input rst,
input vld_in,
input	[127:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[127:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[127:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[127:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[127:0] val_i_x_3,
output	[31:0] x_cnt_3,
input	[127:0] val_i_x_4,
output	[31:0] x_cnt_4,
input	[127:0] val_i_x_5,
output	[31:0] x_cnt_5,
input	[127:0] val_i_x_6,
output	[31:0] x_cnt_6,
input	[127:0] val_i_x_7,
output	[31:0] x_cnt_7,
output	[31:0] xy_cnt_0_0,
output	[31:0] xy_cnt_0_1,
output	[31:0] xy_cnt_0_2,
output	[31:0] xy_cnt_0_3,
output	[31:0] xy_cnt_0_4,
output	[31:0] xy_cnt_0_5,
output	[31:0] xy_cnt_0_6,
output	[31:0] xy_cnt_0_7
);

wire [127:0] val_i_y_t_0;

wire [127:0] val_i_x_t_0;
wire [127:0] val_i_x_t_1;
wire [127:0] val_i_x_t_2;
wire [127:0] val_i_x_t_3;
wire [127:0] val_i_x_t_4;
wire [127:0] val_i_x_t_5;
wire [127:0] val_i_x_t_6;
wire [127:0] val_i_x_t_7;

gblock_0 #(128, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(128, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(128, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(128, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));
gblock_0 #(128, 32) gblock_0_x_4 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_4), .val_o(x_cnt_4), .val_i_t(val_i_x_t_4));
gblock_0 #(128, 32) gblock_0_x_5 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_5), .val_o(x_cnt_5), .val_i_t(val_i_x_t_5));
gblock_0 #(128, 32) gblock_0_x_6 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_6), .val_o(x_cnt_6), .val_i_t(val_i_x_t_6));
gblock_0 #(128, 32) gblock_0_x_7 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_7), .val_o(x_cnt_7), .val_i_t(val_i_x_t_7));

gblock_0 #(128, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1 #(128, 32) gblock_1_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1 #(128, 32) gblock_1_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1 #(128, 32) gblock_1_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1 #(128, 32) gblock_1_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));
gblock_1 #(128, 32) gblock_1_0_4 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_4), .val_o(xy_cnt_0_4));
gblock_1 #(128, 32) gblock_1_0_5 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_5), .val_o(xy_cnt_0_5));
gblock_1 #(128, 32) gblock_1_0_6 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_6), .val_o(xy_cnt_0_6));
gblock_1 #(128, 32) gblock_1_0_7 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_7), .val_o(xy_cnt_0_7));


endmodule


module pw_sim_special_8_1_128_7 (
input clk,
input rst,
input vld_in,
input	[127:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[127:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[127:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[127:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[127:0] val_i_x_3,
output	[31:0] x_cnt_3,
input	[127:0] val_i_x_4,
output	[31:0] x_cnt_4,
input	[127:0] val_i_x_5,
output	[31:0] x_cnt_5,
input	[127:0] val_i_x_6,
output	[31:0] x_cnt_6,
input	[127:0] val_i_x_7,
output	[31:0] x_cnt_7,
output	[31:0] xy_cnt_0_0,
output	[31:0] xy_cnt_0_1,
output	[31:0] xy_cnt_0_2,
output	[31:0] xy_cnt_0_3,
output	[31:0] xy_cnt_0_4,
output	[31:0] xy_cnt_0_5,
output	[31:0] xy_cnt_0_6,
output	[31:0] xy_cnt_0_7
);

wire [127:0] val_i_y_t_0;

wire [127:0] val_i_x_t_0;
wire [127:0] val_i_x_t_1;
wire [127:0] val_i_x_t_2;
wire [127:0] val_i_x_t_3;
wire [127:0] val_i_x_t_4;
wire [127:0] val_i_x_t_5;
wire [127:0] val_i_x_t_6;
wire [127:0] val_i_x_t_7;

gblock_0 #(128, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(128, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(128, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(128, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));
gblock_0 #(128, 32) gblock_0_x_4 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_4), .val_o(x_cnt_4), .val_i_t(val_i_x_t_4));
gblock_0 #(128, 32) gblock_0_x_5 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_5), .val_o(x_cnt_5), .val_i_t(val_i_x_t_5));
gblock_0 #(128, 32) gblock_0_x_6 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_6), .val_o(x_cnt_6), .val_i_t(val_i_x_t_6));
gblock_0 #(128, 32) gblock_0_x_7 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_7), .val_o(x_cnt_7), .val_i_t(val_i_x_t_7));

gblock_0 #(128, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1_special #(128, 32) gblock_1_special_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1_special #(128, 32) gblock_1_special_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1_special #(128, 32) gblock_1_special_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1_special #(128, 32) gblock_1_special_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));
gblock_1_special #(128, 32) gblock_1_special_0_4 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_4), .val_o(xy_cnt_0_4));
gblock_1_special #(128, 32) gblock_1_special_0_5 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_5), .val_o(xy_cnt_0_5));
gblock_1_special #(128, 32) gblock_1_special_0_6 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_6), .val_o(xy_cnt_0_6));
gblock_1_special #(128, 32) gblock_1_special_0_7 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_7), .val_o(xy_cnt_0_7));


endmodule


module pw_coefficients_wrapper_8_1_128_7 (
input clk,
input vld_in,
input	[31:0] sample_size,
input	[31:0] y_cnt_0,
input	[31:0] x_cnt_0,
input	[31:0] x_cnt_1,
input	[31:0] x_cnt_2,
input	[31:0] x_cnt_3,
input	[31:0] x_cnt_4,
input	[31:0] x_cnt_5,
input	[31:0] x_cnt_6,
input	[31:0] x_cnt_7,
input	[31:0] xy_cnt_0_0,
input	[31:0] xy_cnt_0_1,
input	[31:0] xy_cnt_0_2,
input	[31:0] xy_cnt_0_3,
input	[31:0] xy_cnt_0_4,
input	[31:0] xy_cnt_0_5,
input	[31:0] xy_cnt_0_6,
input	[31:0] xy_cnt_0_7,
output linkdiseq_wren_0_0,
output linkdiseq_wren_0_1,
output linkdiseq_wren_0_2,
output linkdiseq_wren_0_3,
output linkdiseq_wren_0_4,
output linkdiseq_wren_0_5,
output linkdiseq_wren_0_6,
output linkdiseq_wren_0_7,
output	[31:0] linkdiseq_0_0,
output	[31:0] linkdiseq_0_1,
output	[31:0] linkdiseq_0_2,
output	[31:0] linkdiseq_0_3,
output	[31:0] linkdiseq_0_4,
output	[31:0] linkdiseq_0_5,
output	[31:0] linkdiseq_0_6,
output	[31:0] linkdiseq_0_7
);

similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_0 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_0), .c(xy_cnt_0_0), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_0), .linkdiseq_coefficient(linkdiseq_0_0));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_1 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_1), .c(xy_cnt_0_1), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_1), .linkdiseq_coefficient(linkdiseq_0_1));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_2 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_2), .c(xy_cnt_0_2), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_2), .linkdiseq_coefficient(linkdiseq_0_2));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_3 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_3), .c(xy_cnt_0_3), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_3), .linkdiseq_coefficient(linkdiseq_0_3));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_4 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_4), .c(xy_cnt_0_4), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_4), .linkdiseq_coefficient(linkdiseq_0_4));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_5 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_5), .c(xy_cnt_0_5), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_5), .linkdiseq_coefficient(linkdiseq_0_5));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_6 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_6), .c(xy_cnt_0_6), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_6), .linkdiseq_coefficient(linkdiseq_0_6));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_7 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_7), .c(xy_cnt_0_7), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_7), .linkdiseq_coefficient(linkdiseq_0_7));


endmodule




module pru_array (
input clk,
input rst,
input	[31:0] element_length_t,
input	[31:0] sample_size,
input vld_in,
input	[127:0] val_i_y_0,
input	[127:0] val_i_x_0,
input	[127:0] val_i_x_1,
input	[127:0] val_i_x_2,
input	[127:0] val_i_x_3,
input	[127:0] val_i_x_4,
input	[127:0] val_i_x_5,
input	[127:0] val_i_x_6,
input	[127:0] val_i_x_7,
output linkdiseq_wren_0_0,
output linkdiseq_wren_0_1,
output linkdiseq_wren_0_2,
output linkdiseq_wren_0_3,
output linkdiseq_wren_0_4,
output linkdiseq_wren_0_5,
output linkdiseq_wren_0_6,
output linkdiseq_wren_0_7,
output	[31:0] linkdiseq_0_0,
output	[31:0] linkdiseq_0_1,
output	[31:0] linkdiseq_0_2,
output	[31:0] linkdiseq_0_3,
output	[31:0] linkdiseq_0_4,
output	[31:0] linkdiseq_0_5,
output	[31:0] linkdiseq_0_6,
output	[31:0] linkdiseq_0_7
);

wire vld_in_t;
du #(1, 7) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
reg [31:0] element_cnt;
reg rst_l;
wire rst_l_w;
reg vld_in_tt;
wire vld_in_tt_w;

	
	
	always @(posedge clk) begin	
		if(rst_l || rst) begin	
			element_cnt = 0;	
		end	
		else begin	
			if(vld_in_t) begin	
				element_cnt = element_cnt+1;	
			end	
		end	
	end	
	

	
	
	always @(*) begin	
		if(element_cnt==element_length_t) begin	
			rst_l = 1;	
		end	
		else begin	
			rst_l = 0;	
		end	
	end	
	

	
	assign rst_l_w = rst_l;	
	assign vld_in_tt_w = vld_in_tt;	
	always @(posedge clk) begin	
			vld_in_tt = rst_l_w;	
	end	
	

wire [31:0] t_y_cnt_0;

wire [31:0] t_x_cnt_0;
wire [31:0] t_x_cnt_1;
wire [31:0] t_x_cnt_2;
wire [31:0] t_x_cnt_3;
wire [31:0] t_x_cnt_4;
wire [31:0] t_x_cnt_5;
wire [31:0] t_x_cnt_6;
wire [31:0] t_x_cnt_7;

wire [31:0] t_xy_cnt_0_0;
wire [31:0] t_xy_cnt_0_1;
wire [31:0] t_xy_cnt_0_2;
wire [31:0] t_xy_cnt_0_3;
wire [31:0] t_xy_cnt_0_4;
wire [31:0] t_xy_cnt_0_5;
wire [31:0] t_xy_cnt_0_6;
wire [31:0] t_xy_cnt_0_7;
pw_sim_8_1_128_7 pw_sim_8_1_128_7_0(
 .clk(clk),
 .rst(rst | vld_in_tt_w),
 .vld_in(vld_in),
 .val_i_y_0(val_i_y_0),
 .y_cnt_0(t_y_cnt_0),
 .val_i_x_0(val_i_x_0),
 .x_cnt_0(t_x_cnt_0),
 .val_i_x_1(val_i_x_1),
 .x_cnt_1(t_x_cnt_1),
 .val_i_x_2(val_i_x_2),
 .x_cnt_2(t_x_cnt_2),
 .val_i_x_3(val_i_x_3),
 .x_cnt_3(t_x_cnt_3),
 .val_i_x_4(val_i_x_4),
 .x_cnt_4(t_x_cnt_4),
 .val_i_x_5(val_i_x_5),
 .x_cnt_5(t_x_cnt_5),
 .val_i_x_6(val_i_x_6),
 .x_cnt_6(t_x_cnt_6),
 .val_i_x_7(val_i_x_7),
 .x_cnt_7(t_x_cnt_7),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7)
);
pw_coefficients_wrapper_8_1_128_7 pw_coefficients_wrapper_8_1_128_7_0(
 .clk(clk),
 .vld_in(vld_in_tt),
 .sample_size(sample_size),
 .y_cnt_0(t_y_cnt_0),
 .x_cnt_0(t_x_cnt_0),
 .x_cnt_1(t_x_cnt_1),
 .x_cnt_2(t_x_cnt_2),
 .x_cnt_3(t_x_cnt_3),
 .x_cnt_4(t_x_cnt_4),
 .x_cnt_5(t_x_cnt_5),
 .x_cnt_6(t_x_cnt_6),
 .x_cnt_7(t_x_cnt_7),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7),
 .linkdiseq_0_0(linkdiseq_0_0),
 .linkdiseq_0_1(linkdiseq_0_1),
 .linkdiseq_0_2(linkdiseq_0_2),
 .linkdiseq_0_3(linkdiseq_0_3),
 .linkdiseq_0_4(linkdiseq_0_4),
 .linkdiseq_0_5(linkdiseq_0_5),
 .linkdiseq_0_6(linkdiseq_0_6),
 .linkdiseq_0_7(linkdiseq_0_7)
);
endmodule


module pru_array_special (
input clk,
input rst,
input	[31:0] element_length_t,
input	[31:0] sample_size,
input vld_in,
input	[127:0] val_i_y_0,
input	[127:0] val_i_x_0,
input	[127:0] val_i_x_1,
input	[127:0] val_i_x_2,
input	[127:0] val_i_x_3,
input	[127:0] val_i_x_4,
input	[127:0] val_i_x_5,
input	[127:0] val_i_x_6,
input	[127:0] val_i_x_7,
output linkdiseq_wren_0_0,
output linkdiseq_wren_0_1,
output linkdiseq_wren_0_2,
output linkdiseq_wren_0_3,
output linkdiseq_wren_0_4,
output linkdiseq_wren_0_5,
output linkdiseq_wren_0_6,
output linkdiseq_wren_0_7,
output	[31:0] linkdiseq_0_0,
output	[31:0] linkdiseq_0_1,
output	[31:0] linkdiseq_0_2,
output	[31:0] linkdiseq_0_3,
output	[31:0] linkdiseq_0_4,
output	[31:0] linkdiseq_0_5,
output	[31:0] linkdiseq_0_6,
output	[31:0] linkdiseq_0_7
);

wire vld_in_t;
du #(1, 7) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
reg [31:0] element_cnt;
reg rst_l;
wire rst_l_w;
reg vld_in_tt;
wire vld_in_tt_w;

	
	
	always @(posedge clk) begin	
		if(rst_l || rst) begin	
			element_cnt = 0;	
		end	
		else begin	
			if(vld_in_t) begin	
				element_cnt = element_cnt+1;	
			end	
		end	
	end	
	

	
	
	always @(*) begin	
		if(element_cnt==element_length_t) begin	
			rst_l = 1;	
		end	
		else begin	
			rst_l = 0;	
		end	
	end	
	

	
	assign rst_l_w = rst_l;	
	assign vld_in_tt_w = vld_in_tt;	
	always @(posedge clk) begin	
			vld_in_tt = rst_l_w;	
	end	
	

wire [31:0] t_y_cnt_0;

wire [31:0] t_x_cnt_0;
wire [31:0] t_x_cnt_1;
wire [31:0] t_x_cnt_2;
wire [31:0] t_x_cnt_3;
wire [31:0] t_x_cnt_4;
wire [31:0] t_x_cnt_5;
wire [31:0] t_x_cnt_6;
wire [31:0] t_x_cnt_7;

wire [31:0] t_xy_cnt_0_0;
wire [31:0] t_xy_cnt_0_1;
wire [31:0] t_xy_cnt_0_2;
wire [31:0] t_xy_cnt_0_3;
wire [31:0] t_xy_cnt_0_4;
wire [31:0] t_xy_cnt_0_5;
wire [31:0] t_xy_cnt_0_6;
wire [31:0] t_xy_cnt_0_7;
pw_sim_special_8_1_128_7 pw_sim_special_8_1_128_7_0(
 .clk(clk),
 .rst(rst | vld_in_tt_w),
 .vld_in(vld_in),
 .val_i_y_0(val_i_y_0),
 .y_cnt_0(t_y_cnt_0),
 .val_i_x_0(val_i_x_0),
 .x_cnt_0(t_x_cnt_0),
 .val_i_x_1(val_i_x_1),
 .x_cnt_1(t_x_cnt_1),
 .val_i_x_2(val_i_x_2),
 .x_cnt_2(t_x_cnt_2),
 .val_i_x_3(val_i_x_3),
 .x_cnt_3(t_x_cnt_3),
 .val_i_x_4(val_i_x_4),
 .x_cnt_4(t_x_cnt_4),
 .val_i_x_5(val_i_x_5),
 .x_cnt_5(t_x_cnt_5),
 .val_i_x_6(val_i_x_6),
 .x_cnt_6(t_x_cnt_6),
 .val_i_x_7(val_i_x_7),
 .x_cnt_7(t_x_cnt_7),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7)
);
pw_coefficients_wrapper_8_1_128_7 pw_coefficients_wrapper_8_1_128_7_0(
 .clk(clk),
 .vld_in(vld_in_tt),
 .sample_size(sample_size),
 .y_cnt_0(t_y_cnt_0),
 .x_cnt_0(t_x_cnt_0),
 .x_cnt_1(t_x_cnt_1),
 .x_cnt_2(t_x_cnt_2),
 .x_cnt_3(t_x_cnt_3),
 .x_cnt_4(t_x_cnt_4),
 .x_cnt_5(t_x_cnt_5),
 .x_cnt_6(t_x_cnt_6),
 .x_cnt_7(t_x_cnt_7),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7),
 .linkdiseq_0_0(linkdiseq_0_0),
 .linkdiseq_0_1(linkdiseq_0_1),
 .linkdiseq_0_2(linkdiseq_0_2),
 .linkdiseq_0_3(linkdiseq_0_3),
 .linkdiseq_0_4(linkdiseq_0_4),
 .linkdiseq_0_5(linkdiseq_0_5),
 .linkdiseq_0_6(linkdiseq_0_6),
 .linkdiseq_0_7(linkdiseq_0_7)
);
endmodule
	
module bram_tdp #(	
    parameter DATA = 18,	
    parameter ADDR = 9	
)	
 (	
    // Port A	
    input   wire                a_clk,	
    input   wire                a_wr,	
    input   wire    [ADDR-1:0]  a_addr,	
    input   wire    [DATA-1:0]  a_din,	
    output  reg     [DATA-1:0]  a_dout,	
     	
    // Port B	
    input   wire                b_clk,	
    input   wire                b_wr,	
    input   wire    [ADDR-1:0]  b_addr,	
    input   wire    [DATA-1:0]  b_din,	
    output  reg     [DATA-1:0]  b_dout	
);	
 	
// Shared memory	
(* ram_style = "block" *)	
reg [DATA-1:0] mem [(2**ADDR)-1:0];	
 	
// Port A	
always @(posedge a_clk) begin	
    a_dout      <= mem[a_addr];	
    if(a_wr) begin	
        a_dout      <= a_din;	
        mem[a_addr] <= a_din;	
    end	
end	
 	
// Port B	
always @(posedge b_clk) begin	
    b_dout      <= mem[b_addr];	
    if(b_wr) begin	
        b_dout      <= b_din;	
        mem[b_addr] <= b_din;	
    end	
end	
 	
endmodule	
	


	
module dist_tdp #(	
    parameter DATA = 18,	
    parameter ADDR = 9	
)	
 (	
    // Port A	
    input   wire                a_clk,	
    input   wire                a_wr,	
    input   wire    [ADDR-1:0]  a_addr,	
    input   wire    [DATA-1:0]  a_din,	
    output  reg     [DATA-1:0]  a_dout,	
     	
    // Port B	
    input   wire                b_clk,	
    input   wire                b_wr,	
    input   wire    [ADDR-1:0]  b_addr,	
    input   wire    [DATA-1:0]  b_din,	
    output  reg     [DATA-1:0]  b_dout	
);	
 	
// Shared memory	
(* ram_style = "distributed" *)	
reg [DATA-1:0] mem [(2**ADDR)-1:0];	
 	
// Port A	
always @(posedge a_clk) begin	
    a_dout      <= mem[a_addr];	
    if(a_wr) begin	
        a_dout      <= a_din;	
        mem[a_addr] <= a_din;	
    end	
end	
 	
// Port B	
always @(posedge b_clk) begin	
    b_dout      <= mem[b_addr];	
    if(b_wr) begin	
        b_dout      <= b_din;	
        mem[b_addr] <= b_din;	
    end	
end	
 	
endmodule	
	


module imem_bank (
  input clk,
  input [2:0] bank_id,
  input wren_q,
  input [7:0]wraddr_q,
  input [2:0]wrid_q,
  input [127:0]wrdin_q,
  input [7:0]rdaddr_a,
  output [127:0]rddout_a,
  input [7:0]rdaddr_b,
  output [127:0]rddout_b
);

wire wren_a;
assign wren_a = (wren_q==1) & (wrid_q==bank_id);
wire [7:0] addr_a;
assign addr_a = (wren_a==1)?wraddr_q:rdaddr_a;
bram_tdp #(128,8) umem ( .a_clk(clk), .a_wr(wren_a), .a_addr(addr_a), .a_din(wrdin_q), .a_dout(rddout_a), 
  .b_clk(clk), .b_wr(1'b0), .b_addr(rdaddr_b), .b_din(128'b0), .b_dout(rddout_b)); 
endmodule
module omem_bank (
  input clk,
  input [2:0] bank_id,
  input wren,
  input [10:0]wraddr,
  input [31:0]wrdin,
  input rden_q,
  input [10:0]rdaddr_q,
  input [2:0]rdid_q,
  output [31:0]rddout_q
);

reg rden_q_1c_r;
wire rden_q_1c_w;
reg [2:0]rdid_q_1c_r;
wire [2:0]rdid_q_1c_w;
wire [31:0]rddout_q_tmp;
wire rden_sel;
	
	always @(posedge clk) begin	
		rden_q_1c_r = rden_q;	
		rdid_q_1c_r = rdid_q;	
	end	
	
	assign rden_q_1c_w = rden_q_1c_r;	
	assign rdid_q_1c_w = rdid_q_1c_r;	
	assign rden_sel = (rden_q_1c_w==1) & (rdid_q_1c_w==bank_id);
assign rddout_q = (rden_sel==1)?rddout_q_tmp:0;
bram_tdp #(32,11) umem ( .a_clk(clk), .a_wr(wren), .a_addr(wraddr), .a_din(wrdin), .a_dout(), 
  .b_clk(clk), .b_wr(1'b0), .b_addr(rdaddr_q), .b_din(32'b0), .b_dout(rddout_q_tmp)); 
endmodule
module multi_b_imem (
input clk,
input wren_q,
input	[7:0] wraddr_q,
input	[2:0] wrid_q,
input	[127:0] wrdin_q,
input	[7:0] rdaddr_a,
input	[7:0] rdaddr_b,
output	[127:0] rddout_a_0_0,
output	[127:0] rddout_b_0_0,
output	[127:0] rddout_a_0_1,
output	[127:0] rddout_b_0_1,
output	[127:0] rddout_a_0_2,
output	[127:0] rddout_b_0_2,
output	[127:0] rddout_a_0_3,
output	[127:0] rddout_b_0_3,
output	[127:0] rddout_a_0_4,
output	[127:0] rddout_b_0_4,
output	[127:0] rddout_a_0_5,
output	[127:0] rddout_b_0_5,
output	[127:0] rddout_a_0_6,
output	[127:0] rddout_b_0_6,
output	[127:0] rddout_a_0_7,
output	[127:0] rddout_b_0_7
);

wire [2:0] bank_id_0_0;
assign bank_id_0_0 = 0;
imem_bank mem_bank_0_0 (
  .clk(clk),
  .bank_id(bank_id_0_0),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_0),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_0)
);

wire [2:0] bank_id_0_1;
assign bank_id_0_1 = 1;
imem_bank mem_bank_0_1 (
  .clk(clk),
  .bank_id(bank_id_0_1),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_1),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_1)
);

wire [2:0] bank_id_0_2;
assign bank_id_0_2 = 2;
imem_bank mem_bank_0_2 (
  .clk(clk),
  .bank_id(bank_id_0_2),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_2),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_2)
);

wire [2:0] bank_id_0_3;
assign bank_id_0_3 = 3;
imem_bank mem_bank_0_3 (
  .clk(clk),
  .bank_id(bank_id_0_3),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_3),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_3)
);

wire [2:0] bank_id_0_4;
assign bank_id_0_4 = 4;
imem_bank mem_bank_0_4 (
  .clk(clk),
  .bank_id(bank_id_0_4),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_4),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_4)
);

wire [2:0] bank_id_0_5;
assign bank_id_0_5 = 5;
imem_bank mem_bank_0_5 (
  .clk(clk),
  .bank_id(bank_id_0_5),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_5),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_5)
);

wire [2:0] bank_id_0_6;
assign bank_id_0_6 = 6;
imem_bank mem_bank_0_6 (
  .clk(clk),
  .bank_id(bank_id_0_6),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_6),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_6)
);

wire [2:0] bank_id_0_7;
assign bank_id_0_7 = 7;
imem_bank mem_bank_0_7 (
  .clk(clk),
  .bank_id(bank_id_0_7),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_7),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_7)
);

endmodule
module multi_b_omem (
input clk,
input	[10:0] wraddr,
input wren_a_0_0,
input wren_b_0_0,
input wren_a_0_1,
input wren_b_0_1,
input wren_a_0_2,
input wren_b_0_2,
input wren_a_0_3,
input wren_b_0_3,
input wren_a_0_4,
input wren_b_0_4,
input wren_a_0_5,
input wren_b_0_5,
input wren_a_0_6,
input wren_b_0_6,
input wren_a_0_7,
input wren_b_0_7,
input	[31:0] wrdin_a_0_0,
input	[31:0] wrdin_b_0_0,
input	[31:0] wrdin_a_0_1,
input	[31:0] wrdin_b_0_1,
input	[31:0] wrdin_a_0_2,
input	[31:0] wrdin_b_0_2,
input	[31:0] wrdin_a_0_3,
input	[31:0] wrdin_b_0_3,
input	[31:0] wrdin_a_0_4,
input	[31:0] wrdin_b_0_4,
input	[31:0] wrdin_a_0_5,
input	[31:0] wrdin_b_0_5,
input	[31:0] wrdin_a_0_6,
input	[31:0] wrdin_b_0_6,
input	[31:0] wrdin_a_0_7,
input	[31:0] wrdin_b_0_7,
input rden_q,
input	[10:0] rdaddr_q,
input	[2:0] rdid_q,
output	[31:0] rddout_q_a_0,
output	[31:0] rddout_q_b_0
);

wire [31:0] rddout_q_a_0_0;
wire [31:0] rddout_q_a_0_1;
wire [31:0] rddout_q_a_0_2;
wire [31:0] rddout_q_a_0_3;
wire [31:0] rddout_q_a_0_4;
wire [31:0] rddout_q_a_0_5;
wire [31:0] rddout_q_a_0_6;
wire [31:0] rddout_q_a_0_7;
wire [31:0] rddout_q_a_0_0_0;
wire [31:0] rddout_q_a_0_1_1;
wire [31:0] rddout_q_a_0_2_2;
wire [31:0] rddout_q_a_0_3_3;
wire [31:0] rddout_q_a_0_4_4;
wire [31:0] rddout_q_a_0_5_5;
wire [31:0] rddout_q_a_0_6_6;
wire [31:0] rddout_q_a_0_7_7;
assign rddout_q_a_0_0_0[31:0] = rddout_q_a_0_0;
assign rddout_q_a_0_1_1[31:0] = rddout_q_a_0_1;
assign rddout_q_a_0_2_2[31:0] = rddout_q_a_0_2;
assign rddout_q_a_0_3_3[31:0] = rddout_q_a_0_3;
assign rddout_q_a_0_4_4[31:0] = rddout_q_a_0_4;
assign rddout_q_a_0_5_5[31:0] = rddout_q_a_0_5;
assign rddout_q_a_0_6_6[31:0] = rddout_q_a_0_6;
assign rddout_q_a_0_7_7[31:0] = rddout_q_a_0_7;
wire [31:0] rddout_q_a_0_tmp_0_0;
wire [31:0] rddout_q_a_0_tmp_0_1;
wire [31:0] rddout_q_a_0_tmp_0_2;
wire [31:0] rddout_q_a_0_tmp_0_3;
wire [31:0] rddout_q_a_0_tmp_d_0_0;
wire [31:0] rddout_q_a_0_tmp_d_0_1;
wire [31:0] rddout_q_a_0_tmp_d_0_2;
wire [31:0] rddout_q_a_0_tmp_d_0_3;
wire [31:0] rddout_q_a_0_tmp_1_0;
wire [31:0] rddout_q_a_0_tmp_1_1;
wire [31:0] rddout_q_a_0_tmp_d_1_0;
wire [31:0] rddout_q_a_0_tmp_d_1_1;
wire [31:0] rddout_q_a_0_tmp_2_0;
wire [31:0] rddout_q_a_0_tmp_d_2_0;

 assign rddout_q_a_0_tmp_0_0 = rddout_q_a_0_0_0 | rddout_q_a_0_1_1 ;
 
 assign rddout_q_a_0_tmp_0_1 = rddout_q_a_0_2_2 | rddout_q_a_0_3_3 ;
 
 assign rddout_q_a_0_tmp_0_2 = rddout_q_a_0_4_4 | rddout_q_a_0_5_5 ;
 
 assign rddout_q_a_0_tmp_0_3 = rddout_q_a_0_6_6 | rddout_q_a_0_7_7 ;
 
 assign rddout_q_a_0_tmp_1_0 = rddout_q_a_0_tmp_d_0_0 | rddout_q_a_0_tmp_d_0_1 ;
 
 assign rddout_q_a_0_tmp_1_1 = rddout_q_a_0_tmp_d_0_2 | rddout_q_a_0_tmp_d_0_3 ;
 
 assign rddout_q_a_0_tmp_2_0 = rddout_q_a_0_tmp_d_1_0 | rddout_q_a_0_tmp_d_1_1 ;
 du_s #(32) du_0_0_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_0),.din(rddout_q_a_0_tmp_0_0));
du_s #(32) du_0_0_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_1),.din(rddout_q_a_0_tmp_0_1));
du_s #(32) du_0_0_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_2),.din(rddout_q_a_0_tmp_0_2));
du_s #(32) du_0_0_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_3),.din(rddout_q_a_0_tmp_0_3));
du_s #(32) du_0_1_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_0),.din(rddout_q_a_0_tmp_1_0));
du_s #(32) du_0_1_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_1),.din(rddout_q_a_0_tmp_1_1));
du_s #(32) du_0_2_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_2_0),.din(rddout_q_a_0_tmp_2_0));
assign rddout_q_a_0 = rddout_q_a_0_tmp_d_2_0;
wire [31:0] rddout_q_b_0_0;
wire [31:0] rddout_q_b_0_1;
wire [31:0] rddout_q_b_0_2;
wire [31:0] rddout_q_b_0_3;
wire [31:0] rddout_q_b_0_4;
wire [31:0] rddout_q_b_0_5;
wire [31:0] rddout_q_b_0_6;
wire [31:0] rddout_q_b_0_7;
wire [31:0] rddout_q_b_0_0_0;
wire [31:0] rddout_q_b_0_1_1;
wire [31:0] rddout_q_b_0_2_2;
wire [31:0] rddout_q_b_0_3_3;
wire [31:0] rddout_q_b_0_4_4;
wire [31:0] rddout_q_b_0_5_5;
wire [31:0] rddout_q_b_0_6_6;
wire [31:0] rddout_q_b_0_7_7;
assign rddout_q_b_0_0_0[31:0] = rddout_q_b_0_0;
assign rddout_q_b_0_1_1[31:0] = rddout_q_b_0_1;
assign rddout_q_b_0_2_2[31:0] = rddout_q_b_0_2;
assign rddout_q_b_0_3_3[31:0] = rddout_q_b_0_3;
assign rddout_q_b_0_4_4[31:0] = rddout_q_b_0_4;
assign rddout_q_b_0_5_5[31:0] = rddout_q_b_0_5;
assign rddout_q_b_0_6_6[31:0] = rddout_q_b_0_6;
assign rddout_q_b_0_7_7[31:0] = rddout_q_b_0_7;
wire [31:0] rddout_q_b_0_tmp_0_0;
wire [31:0] rddout_q_b_0_tmp_0_1;
wire [31:0] rddout_q_b_0_tmp_0_2;
wire [31:0] rddout_q_b_0_tmp_0_3;
wire [31:0] rddout_q_b_0_tmp_d_0_0;
wire [31:0] rddout_q_b_0_tmp_d_0_1;
wire [31:0] rddout_q_b_0_tmp_d_0_2;
wire [31:0] rddout_q_b_0_tmp_d_0_3;
wire [31:0] rddout_q_b_0_tmp_1_0;
wire [31:0] rddout_q_b_0_tmp_1_1;
wire [31:0] rddout_q_b_0_tmp_d_1_0;
wire [31:0] rddout_q_b_0_tmp_d_1_1;
wire [31:0] rddout_q_b_0_tmp_2_0;
wire [31:0] rddout_q_b_0_tmp_d_2_0;

 assign rddout_q_b_0_tmp_0_0 = rddout_q_b_0_0_0 | rddout_q_b_0_1_1 ;
 
 assign rddout_q_b_0_tmp_0_1 = rddout_q_b_0_2_2 | rddout_q_b_0_3_3 ;
 
 assign rddout_q_b_0_tmp_0_2 = rddout_q_b_0_4_4 | rddout_q_b_0_5_5 ;
 
 assign rddout_q_b_0_tmp_0_3 = rddout_q_b_0_6_6 | rddout_q_b_0_7_7 ;
 
 assign rddout_q_b_0_tmp_1_0 = rddout_q_b_0_tmp_d_0_0 | rddout_q_b_0_tmp_d_0_1 ;
 
 assign rddout_q_b_0_tmp_1_1 = rddout_q_b_0_tmp_d_0_2 | rddout_q_b_0_tmp_d_0_3 ;
 
 assign rddout_q_b_0_tmp_2_0 = rddout_q_b_0_tmp_d_1_0 | rddout_q_b_0_tmp_d_1_1 ;
 du_s #(32) du_1_0_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_0),.din(rddout_q_b_0_tmp_0_0));
du_s #(32) du_1_0_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_1),.din(rddout_q_b_0_tmp_0_1));
du_s #(32) du_1_0_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_2),.din(rddout_q_b_0_tmp_0_2));
du_s #(32) du_1_0_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_3),.din(rddout_q_b_0_tmp_0_3));
du_s #(32) du_1_1_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_0),.din(rddout_q_b_0_tmp_1_0));
du_s #(32) du_1_1_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_1),.din(rddout_q_b_0_tmp_1_1));
du_s #(32) du_1_2_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_2_0),.din(rddout_q_b_0_tmp_2_0));
assign rddout_q_b_0 = rddout_q_b_0_tmp_d_2_0;
omem_bank mem_bank_0_0_a (
  .clk(clk),
  .bank_id(3'b000),
  .wren(wren_a_0_0),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_0),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_0)
);

omem_bank mem_bank_0_0_b (
  .clk(clk),
  .bank_id(3'b000),
  .wren(wren_b_0_0),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_0),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_0)
);

omem_bank mem_bank_0_1_a (
  .clk(clk),
  .bank_id(3'b001),
  .wren(wren_a_0_1),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_1),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_1)
);

omem_bank mem_bank_0_1_b (
  .clk(clk),
  .bank_id(3'b001),
  .wren(wren_b_0_1),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_1),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_1)
);

omem_bank mem_bank_0_2_a (
  .clk(clk),
  .bank_id(3'b010),
  .wren(wren_a_0_2),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_2),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_2)
);

omem_bank mem_bank_0_2_b (
  .clk(clk),
  .bank_id(3'b010),
  .wren(wren_b_0_2),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_2),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_2)
);

omem_bank mem_bank_0_3_a (
  .clk(clk),
  .bank_id(3'b011),
  .wren(wren_a_0_3),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_3),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_3)
);

omem_bank mem_bank_0_3_b (
  .clk(clk),
  .bank_id(3'b011),
  .wren(wren_b_0_3),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_3),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_3)
);

omem_bank mem_bank_0_4_a (
  .clk(clk),
  .bank_id(3'b100),
  .wren(wren_a_0_4),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_4),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_4)
);

omem_bank mem_bank_0_4_b (
  .clk(clk),
  .bank_id(3'b100),
  .wren(wren_b_0_4),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_4),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_4)
);

omem_bank mem_bank_0_5_a (
  .clk(clk),
  .bank_id(3'b101),
  .wren(wren_a_0_5),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_5),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_5)
);

omem_bank mem_bank_0_5_b (
  .clk(clk),
  .bank_id(3'b101),
  .wren(wren_b_0_5),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_5),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_5)
);

omem_bank mem_bank_0_6_a (
  .clk(clk),
  .bank_id(3'b110),
  .wren(wren_a_0_6),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_6),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_6)
);

omem_bank mem_bank_0_6_b (
  .clk(clk),
  .bank_id(3'b110),
  .wren(wren_b_0_6),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_6),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_6)
);

omem_bank mem_bank_0_7_a (
  .clk(clk),
  .bank_id(3'b111),
  .wren(wren_a_0_7),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_7),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_7)
);

omem_bank mem_bank_0_7_b (
  .clk(clk),
  .bank_id(3'b111),
  .wren(wren_b_0_7),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_7),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_7)
);

endmodule
module top (
input clk,
input rst,
input	[31:0] element_length,
input	[31:0] sample_size,
input wren_q,
input	[7:0] wraddr_q,
input	[2:0] wrid_q,
input	[127:0] wrdin_q,
input vld_in,
input	[10:0] rdaddr_a,
input	[10:0] rdaddr_b,
input	[127:0] val_i_y_0,
input rden_q,
input	[7:0] rdaddr_q,
input	[2:0] rdid_q,
output	[31:0] rddout_q_a_0,
output	[31:0] rddout_q_b_0
);

reg [31:0] element_length_t;
reg [31:0] cur_element_length;
reg vld_in_t, rst_l;
reg [7:0] wraddr;
wire [127:0] d1_val_i_y_0;
du_s #(128) du_99_0 (.clk(clk), .stall(1'b0), .dout(d1_val_i_y_0),.din(val_i_y_0));

	
	
	always @(posedge clk) begin	
		if(rst)	
			element_length_t <= element_length-1;	
	end	
	

	
	
	always @(posedge clk) begin	
		vld_in_t <= vld_in;	
	end	
	

	
	
	always @(posedge clk) begin	
		if(rst_l || rst) begin	
			cur_element_length = 0;	
		end	
		else begin	
			if(vld_in_t) begin	
				cur_element_length = cur_element_length+1;	
			end	
		end	
	end	
	

	
	
	always @(posedge clk) begin	
		if(cur_element_length==element_length_t) begin	
			rst_l = 1;	
		end	
		else begin	
			rst_l = 0;	
		end	
	end	
	


wire [127:0] rddout_a_0_0;
wire [127:0] rddout_a_0_1;
wire [127:0] rddout_a_0_2;
wire [127:0] rddout_a_0_3;
wire [127:0] rddout_a_0_4;
wire [127:0] rddout_a_0_5;
wire [127:0] rddout_a_0_6;
wire [127:0] rddout_a_0_7;


wire [127:0] rddout_b_0_0;
wire [127:0] rddout_b_0_1;
wire [127:0] rddout_b_0_2;
wire [127:0] rddout_b_0_3;
wire [127:0] rddout_b_0_4;
wire [127:0] rddout_b_0_5;
wire [127:0] rddout_b_0_6;
wire [127:0] rddout_b_0_7;




wire [127:0] val_i_x_0_a;
wire [127:0] val_i_x_1_a;
wire [127:0] val_i_x_2_a;
wire [127:0] val_i_x_3_a;
wire [127:0] val_i_x_4_a;
wire [127:0] val_i_x_5_a;
wire [127:0] val_i_x_6_a;
wire [127:0] val_i_x_7_a;
wire [31:0] linkdiseq_0_0_a;
wire [31:0] linkdiseq_0_1_a;
wire [31:0] linkdiseq_0_2_a;
wire [31:0] linkdiseq_0_3_a;
wire [31:0] linkdiseq_0_4_a;
wire [31:0] linkdiseq_0_5_a;
wire [31:0] linkdiseq_0_6_a;
wire [31:0] linkdiseq_0_7_a;
wire [0:0] linkdiseq_wren_0_0_a;
wire [0:0] linkdiseq_wren_0_1_a;
wire [0:0] linkdiseq_wren_0_2_a;
wire [0:0] linkdiseq_wren_0_3_a;
wire [0:0] linkdiseq_wren_0_4_a;
wire [0:0] linkdiseq_wren_0_5_a;
wire [0:0] linkdiseq_wren_0_6_a;
wire [0:0] linkdiseq_wren_0_7_a;


wire [127:0] val_i_x_0_b;
wire [127:0] val_i_x_1_b;
wire [127:0] val_i_x_2_b;
wire [127:0] val_i_x_3_b;
wire [127:0] val_i_x_4_b;
wire [127:0] val_i_x_5_b;
wire [127:0] val_i_x_6_b;
wire [127:0] val_i_x_7_b;
wire [31:0] linkdiseq_0_0_b;
wire [31:0] linkdiseq_0_1_b;
wire [31:0] linkdiseq_0_2_b;
wire [31:0] linkdiseq_0_3_b;
wire [31:0] linkdiseq_0_4_b;
wire [31:0] linkdiseq_0_5_b;
wire [31:0] linkdiseq_0_6_b;
wire [31:0] linkdiseq_0_7_b;
wire [0:0] linkdiseq_wren_0_0_b;
wire [0:0] linkdiseq_wren_0_1_b;
wire [0:0] linkdiseq_wren_0_2_b;
wire [0:0] linkdiseq_wren_0_3_b;
wire [0:0] linkdiseq_wren_0_4_b;
wire [0:0] linkdiseq_wren_0_5_b;
wire [0:0] linkdiseq_wren_0_6_b;
wire [0:0] linkdiseq_wren_0_7_b;


wire [0:0] wren_a_0_0;
wire [0:0] wren_a_0_1;
wire [0:0] wren_a_0_2;
wire [0:0] wren_a_0_3;
wire [0:0] wren_a_0_4;
wire [0:0] wren_a_0_5;
wire [0:0] wren_a_0_6;
wire [0:0] wren_a_0_7;
wire [31:0] wrdin_a_0_0;
wire [31:0] wrdin_a_0_1;
wire [31:0] wrdin_a_0_2;
wire [31:0] wrdin_a_0_3;
wire [31:0] wrdin_a_0_4;
wire [31:0] wrdin_a_0_5;
wire [31:0] wrdin_a_0_6;
wire [31:0] wrdin_a_0_7;


wire [0:0] wren_b_0_0;
wire [0:0] wren_b_0_1;
wire [0:0] wren_b_0_2;
wire [0:0] wren_b_0_3;
wire [0:0] wren_b_0_4;
wire [0:0] wren_b_0_5;
wire [0:0] wren_b_0_6;
wire [0:0] wren_b_0_7;
wire [31:0] wrdin_b_0_0;
wire [31:0] wrdin_b_0_1;
wire [31:0] wrdin_b_0_2;
wire [31:0] wrdin_b_0_3;
wire [31:0] wrdin_b_0_4;
wire [31:0] wrdin_b_0_5;
wire [31:0] wrdin_b_0_6;
wire [31:0] wrdin_b_0_7;



	
	
	always @(posedge clk) begin	
		if(rst) begin	
			wraddr = 0;	
		end	
		else begin	
			if(wren_a_0_0) begin	
				wraddr = wraddr+{8'b1};	
			end	
		end	
	end	
	


assign val_i_x_0_a[127:0] = rddout_a_0_0[127:0];
assign val_i_x_1_a[127:0] = rddout_a_0_1[127:0];
assign val_i_x_2_a[127:0] = rddout_a_0_2[127:0];
assign val_i_x_3_a[127:0] = rddout_a_0_3[127:0];
assign val_i_x_4_a[127:0] = rddout_a_0_4[127:0];
assign val_i_x_5_a[127:0] = rddout_a_0_5[127:0];
assign val_i_x_6_a[127:0] = rddout_a_0_6[127:0];
assign val_i_x_7_a[127:0] = rddout_a_0_7[127:0];


assign val_i_x_0_b[127:0] = rddout_b_0_0[127:0];
assign val_i_x_1_b[127:0] = rddout_b_0_1[127:0];
assign val_i_x_2_b[127:0] = rddout_b_0_2[127:0];
assign val_i_x_3_b[127:0] = rddout_b_0_3[127:0];
assign val_i_x_4_b[127:0] = rddout_b_0_4[127:0];
assign val_i_x_5_b[127:0] = rddout_b_0_5[127:0];
assign val_i_x_6_b[127:0] = rddout_b_0_6[127:0];
assign val_i_x_7_b[127:0] = rddout_b_0_7[127:0];


du_s #(1) du_2_0 (.clk(clk), .stall(1'b0), .dout(wren_a_0_0),.din(linkdiseq_wren_0_0_a));
du_s #(1) du_2_1 (.clk(clk), .stall(1'b0), .dout(wren_a_0_1),.din(linkdiseq_wren_0_1_a));
du_s #(1) du_2_2 (.clk(clk), .stall(1'b0), .dout(wren_a_0_2),.din(linkdiseq_wren_0_2_a));
du_s #(1) du_2_3 (.clk(clk), .stall(1'b0), .dout(wren_a_0_3),.din(linkdiseq_wren_0_3_a));
du_s #(1) du_2_4 (.clk(clk), .stall(1'b0), .dout(wren_a_0_4),.din(linkdiseq_wren_0_4_a));
du_s #(1) du_2_5 (.clk(clk), .stall(1'b0), .dout(wren_a_0_5),.din(linkdiseq_wren_0_5_a));
du_s #(1) du_2_6 (.clk(clk), .stall(1'b0), .dout(wren_a_0_6),.din(linkdiseq_wren_0_6_a));
du_s #(1) du_2_7 (.clk(clk), .stall(1'b0), .dout(wren_a_0_7),.din(linkdiseq_wren_0_7_a));
du_s #(1) du_3_0 (.clk(clk), .stall(1'b0), .dout(wren_b_0_0),.din(linkdiseq_wren_0_0_b));
du_s #(1) du_3_1 (.clk(clk), .stall(1'b0), .dout(wren_b_0_1),.din(linkdiseq_wren_0_1_b));
du_s #(1) du_3_2 (.clk(clk), .stall(1'b0), .dout(wren_b_0_2),.din(linkdiseq_wren_0_2_b));
du_s #(1) du_3_3 (.clk(clk), .stall(1'b0), .dout(wren_b_0_3),.din(linkdiseq_wren_0_3_b));
du_s #(1) du_3_4 (.clk(clk), .stall(1'b0), .dout(wren_b_0_4),.din(linkdiseq_wren_0_4_b));
du_s #(1) du_3_5 (.clk(clk), .stall(1'b0), .dout(wren_b_0_5),.din(linkdiseq_wren_0_5_b));
du_s #(1) du_3_6 (.clk(clk), .stall(1'b0), .dout(wren_b_0_6),.din(linkdiseq_wren_0_6_b));
du_s #(1) du_3_7 (.clk(clk), .stall(1'b0), .dout(wren_b_0_7),.din(linkdiseq_wren_0_7_b));
du_s #(32) du_4_0 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_0),.din(linkdiseq_0_0_a));
du_s #(32) du_4_1 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_1),.din(linkdiseq_0_1_a));
du_s #(32) du_4_2 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_2),.din(linkdiseq_0_2_a));
du_s #(32) du_4_3 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_3),.din(linkdiseq_0_3_a));
du_s #(32) du_4_4 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_4),.din(linkdiseq_0_4_a));
du_s #(32) du_4_5 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_5),.din(linkdiseq_0_5_a));
du_s #(32) du_4_6 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_6),.din(linkdiseq_0_6_a));
du_s #(32) du_4_7 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_7),.din(linkdiseq_0_7_a));
du_s #(32) du_5_0 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_0),.din(linkdiseq_0_0_b));
du_s #(32) du_5_1 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_1),.din(linkdiseq_0_1_b));
du_s #(32) du_5_2 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_2),.din(linkdiseq_0_2_b));
du_s #(32) du_5_3 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_3),.din(linkdiseq_0_3_b));
du_s #(32) du_5_4 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_4),.din(linkdiseq_0_4_b));
du_s #(32) du_5_5 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_5),.din(linkdiseq_0_5_b));
du_s #(32) du_5_6 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_6),.din(linkdiseq_0_6_b));
du_s #(32) du_5_7 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_7),.din(linkdiseq_0_7_b));


multi_b_imem multi_b_imem_0(
 .clk(clk),
 .wren_q(wren_q),
 .wraddr_q(wraddr_q),
 .wrid_q(wrid_q),
 .wrdin_q(wrdin_q),
 .rdaddr_a(rdaddr_a),
 .rdaddr_b(rdaddr_b),
 .rddout_a_0_0(rddout_a_0_0),
 .rddout_b_0_0(rddout_b_0_0),
 .rddout_a_0_1(rddout_a_0_1),
 .rddout_b_0_1(rddout_b_0_1),
 .rddout_a_0_2(rddout_a_0_2),
 .rddout_b_0_2(rddout_b_0_2),
 .rddout_a_0_3(rddout_a_0_3),
 .rddout_b_0_3(rddout_b_0_3),
 .rddout_a_0_4(rddout_a_0_4),
 .rddout_b_0_4(rddout_b_0_4),
 .rddout_a_0_5(rddout_a_0_5),
 .rddout_b_0_5(rddout_b_0_5),
 .rddout_a_0_6(rddout_a_0_6),
 .rddout_b_0_6(rddout_b_0_6),
 .rddout_a_0_7(rddout_a_0_7),
 .rddout_b_0_7(rddout_b_0_7)
);
pru_array pru_array_a(
 .clk(clk),
 .rst(rst),
 .element_length_t(element_length_t),
 .sample_size(sample_size),
 .vld_in(vld_in_t),
 .val_i_y_0(d1_val_i_y_0),
 .val_i_x_0(val_i_x_0_a),
 .val_i_x_1(val_i_x_1_a),
 .val_i_x_2(val_i_x_2_a),
 .val_i_x_3(val_i_x_3_a),
 .val_i_x_4(val_i_x_4_a),
 .val_i_x_5(val_i_x_5_a),
 .val_i_x_6(val_i_x_6_a),
 .val_i_x_7(val_i_x_7_a),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0_a),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1_a),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2_a),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3_a),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4_a),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5_a),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6_a),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7_a),
 .linkdiseq_0_0(linkdiseq_0_0_a),
 .linkdiseq_0_1(linkdiseq_0_1_a),
 .linkdiseq_0_2(linkdiseq_0_2_a),
 .linkdiseq_0_3(linkdiseq_0_3_a),
 .linkdiseq_0_4(linkdiseq_0_4_a),
 .linkdiseq_0_5(linkdiseq_0_5_a),
 .linkdiseq_0_6(linkdiseq_0_6_a),
 .linkdiseq_0_7(linkdiseq_0_7_a)
);
pru_array_special pru_array_special_b(
 .clk(clk),
 .rst(rst),
 .element_length_t(element_length_t),
 .sample_size(sample_size),
 .vld_in(vld_in_t),
 .val_i_y_0(d1_val_i_y_0),
 .val_i_x_0(val_i_x_0_b),
 .val_i_x_1(val_i_x_1_b),
 .val_i_x_2(val_i_x_2_b),
 .val_i_x_3(val_i_x_3_b),
 .val_i_x_4(val_i_x_4_b),
 .val_i_x_5(val_i_x_5_b),
 .val_i_x_6(val_i_x_6_b),
 .val_i_x_7(val_i_x_7_b),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0_b),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1_b),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2_b),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3_b),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4_b),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5_b),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6_b),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7_b),
 .linkdiseq_0_0(linkdiseq_0_0_b),
 .linkdiseq_0_1(linkdiseq_0_1_b),
 .linkdiseq_0_2(linkdiseq_0_2_b),
 .linkdiseq_0_3(linkdiseq_0_3_b),
 .linkdiseq_0_4(linkdiseq_0_4_b),
 .linkdiseq_0_5(linkdiseq_0_5_b),
 .linkdiseq_0_6(linkdiseq_0_6_b),
 .linkdiseq_0_7(linkdiseq_0_7_b)
);


multi_b_omem multi_b_omem_0(
 .clk(clk),
 .wraddr(wraddr),
 .wren_a_0_0(wren_a_0_0),
 .wren_b_0_0(wren_b_0_0),
 .wren_a_0_1(wren_a_0_1),
 .wren_b_0_1(wren_b_0_1),
 .wren_a_0_2(wren_a_0_2),
 .wren_b_0_2(wren_b_0_2),
 .wren_a_0_3(wren_a_0_3),
 .wren_b_0_3(wren_b_0_3),
 .wren_a_0_4(wren_a_0_4),
 .wren_b_0_4(wren_b_0_4),
 .wren_a_0_5(wren_a_0_5),
 .wren_b_0_5(wren_b_0_5),
 .wren_a_0_6(wren_a_0_6),
 .wren_b_0_6(wren_b_0_6),
 .wren_a_0_7(wren_a_0_7),
 .wren_b_0_7(wren_b_0_7),
 .wrdin_a_0_0(wrdin_a_0_0),
 .wrdin_b_0_0(wrdin_b_0_0),
 .wrdin_a_0_1(wrdin_a_0_1),
 .wrdin_b_0_1(wrdin_b_0_1),
 .wrdin_a_0_2(wrdin_a_0_2),
 .wrdin_b_0_2(wrdin_b_0_2),
 .wrdin_a_0_3(wrdin_a_0_3),
 .wrdin_b_0_3(wrdin_b_0_3),
 .wrdin_a_0_4(wrdin_a_0_4),
 .wrdin_b_0_4(wrdin_b_0_4),
 .wrdin_a_0_5(wrdin_a_0_5),
 .wrdin_b_0_5(wrdin_b_0_5),
 .wrdin_a_0_6(wrdin_a_0_6),
 .wrdin_b_0_6(wrdin_b_0_6),
 .wrdin_a_0_7(wrdin_a_0_7),
 .wrdin_b_0_7(wrdin_b_0_7),
 .rden_q(rden_q),
 .rdaddr_q(rdaddr_q),
 .rdid_q(rdid_q),
 .rddout_q_a_0(rddout_q_a_0),
 .rddout_q_b_0(rddout_q_b_0)
);


endmodule
