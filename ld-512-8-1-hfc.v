	
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


module popcnt_bram_dsp_512_9 (
input clk,
input	[511:0] input_v,
output	[9:0] output_v
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
wire [7:0] input_16;
wire [7:0] input_17;
wire [7:0] input_18;
wire [7:0] input_19;
wire [7:0] input_20;
wire [7:0] input_21;
wire [7:0] input_22;
wire [7:0] input_23;
wire [7:0] input_24;
wire [7:0] input_25;
wire [7:0] input_26;
wire [7:0] input_27;
wire [7:0] input_28;
wire [7:0] input_29;
wire [7:0] input_30;
wire [7:0] input_31;
wire [7:0] input_32;
wire [7:0] input_33;
wire [7:0] input_34;
wire [7:0] input_35;
wire [7:0] input_36;
wire [7:0] input_37;
wire [7:0] input_38;
wire [7:0] input_39;
wire [7:0] input_40;
wire [7:0] input_41;
wire [7:0] input_42;
wire [7:0] input_43;
wire [7:0] input_44;
wire [7:0] input_45;
wire [7:0] input_46;
wire [7:0] input_47;
wire [7:0] input_48;
wire [7:0] input_49;
wire [7:0] input_50;
wire [7:0] input_51;
wire [7:0] input_52;
wire [7:0] input_53;
wire [7:0] input_54;
wire [7:0] input_55;
wire [7:0] input_56;
wire [7:0] input_57;
wire [7:0] input_58;
wire [7:0] input_59;
wire [7:0] input_60;
wire [7:0] input_61;
wire [7:0] input_62;
wire [7:0] input_63;
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
wire [7:0] lut8_BRAM_i_16;
wire [7:0] lut8_BRAM_i_17;
wire [7:0] lut8_BRAM_i_18;
wire [7:0] lut8_BRAM_i_19;
wire [7:0] lut8_BRAM_i_20;
wire [7:0] lut8_BRAM_i_21;
wire [7:0] lut8_BRAM_i_22;
wire [7:0] lut8_BRAM_i_23;
wire [7:0] lut8_BRAM_i_24;
wire [7:0] lut8_BRAM_i_25;
wire [7:0] lut8_BRAM_i_26;
wire [7:0] lut8_BRAM_i_27;
wire [7:0] lut8_BRAM_i_28;
wire [7:0] lut8_BRAM_i_29;
wire [7:0] lut8_BRAM_i_30;
wire [7:0] lut8_BRAM_i_31;
wire [7:0] lut8_BRAM_i_32;
wire [7:0] lut8_BRAM_i_33;
wire [7:0] lut8_BRAM_i_34;
wire [7:0] lut8_BRAM_i_35;
wire [7:0] lut8_BRAM_i_36;
wire [7:0] lut8_BRAM_i_37;
wire [7:0] lut8_BRAM_i_38;
wire [7:0] lut8_BRAM_i_39;
wire [7:0] lut8_BRAM_i_40;
wire [7:0] lut8_BRAM_i_41;
wire [7:0] lut8_BRAM_i_42;
wire [7:0] lut8_BRAM_i_43;
wire [7:0] lut8_BRAM_i_44;
wire [7:0] lut8_BRAM_i_45;
wire [7:0] lut8_BRAM_i_46;
wire [7:0] lut8_BRAM_i_47;
wire [7:0] lut8_BRAM_i_48;
wire [7:0] lut8_BRAM_i_49;
wire [7:0] lut8_BRAM_i_50;
wire [7:0] lut8_BRAM_i_51;
wire [7:0] lut8_BRAM_i_52;
wire [7:0] lut8_BRAM_i_53;
wire [7:0] lut8_BRAM_i_54;
wire [7:0] lut8_BRAM_i_55;
wire [7:0] lut8_BRAM_i_56;
wire [7:0] lut8_BRAM_i_57;
wire [7:0] lut8_BRAM_i_58;
wire [7:0] lut8_BRAM_i_59;
wire [7:0] lut8_BRAM_i_60;
wire [7:0] lut8_BRAM_i_61;
wire [7:0] lut8_BRAM_i_62;
wire [7:0] lut8_BRAM_i_63;
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
wire [3:0] lut8_BRAM_o_16;
wire [3:0] lut8_BRAM_o_17;
wire [3:0] lut8_BRAM_o_18;
wire [3:0] lut8_BRAM_o_19;
wire [3:0] lut8_BRAM_o_20;
wire [3:0] lut8_BRAM_o_21;
wire [3:0] lut8_BRAM_o_22;
wire [3:0] lut8_BRAM_o_23;
wire [3:0] lut8_BRAM_o_24;
wire [3:0] lut8_BRAM_o_25;
wire [3:0] lut8_BRAM_o_26;
wire [3:0] lut8_BRAM_o_27;
wire [3:0] lut8_BRAM_o_28;
wire [3:0] lut8_BRAM_o_29;
wire [3:0] lut8_BRAM_o_30;
wire [3:0] lut8_BRAM_o_31;
wire [3:0] lut8_BRAM_o_32;
wire [3:0] lut8_BRAM_o_33;
wire [3:0] lut8_BRAM_o_34;
wire [3:0] lut8_BRAM_o_35;
wire [3:0] lut8_BRAM_o_36;
wire [3:0] lut8_BRAM_o_37;
wire [3:0] lut8_BRAM_o_38;
wire [3:0] lut8_BRAM_o_39;
wire [3:0] lut8_BRAM_o_40;
wire [3:0] lut8_BRAM_o_41;
wire [3:0] lut8_BRAM_o_42;
wire [3:0] lut8_BRAM_o_43;
wire [3:0] lut8_BRAM_o_44;
wire [3:0] lut8_BRAM_o_45;
wire [3:0] lut8_BRAM_o_46;
wire [3:0] lut8_BRAM_o_47;
wire [3:0] lut8_BRAM_o_48;
wire [3:0] lut8_BRAM_o_49;
wire [3:0] lut8_BRAM_o_50;
wire [3:0] lut8_BRAM_o_51;
wire [3:0] lut8_BRAM_o_52;
wire [3:0] lut8_BRAM_o_53;
wire [3:0] lut8_BRAM_o_54;
wire [3:0] lut8_BRAM_o_55;
wire [3:0] lut8_BRAM_o_56;
wire [3:0] lut8_BRAM_o_57;
wire [3:0] lut8_BRAM_o_58;
wire [3:0] lut8_BRAM_o_59;
wire [3:0] lut8_BRAM_o_60;
wire [3:0] lut8_BRAM_o_61;
wire [3:0] lut8_BRAM_o_62;
wire [3:0] lut8_BRAM_o_63;
assign input_0[7:0] = input_v[511:504];
assign input_1[7:0] = input_v[503:496];
assign input_2[7:0] = input_v[495:488];
assign input_3[7:0] = input_v[487:480];
assign input_4[7:0] = input_v[479:472];
assign input_5[7:0] = input_v[471:464];
assign input_6[7:0] = input_v[463:456];
assign input_7[7:0] = input_v[455:448];
assign input_8[7:0] = input_v[447:440];
assign input_9[7:0] = input_v[439:432];
assign input_10[7:0] = input_v[431:424];
assign input_11[7:0] = input_v[423:416];
assign input_12[7:0] = input_v[415:408];
assign input_13[7:0] = input_v[407:400];
assign input_14[7:0] = input_v[399:392];
assign input_15[7:0] = input_v[391:384];
assign input_16[7:0] = input_v[383:376];
assign input_17[7:0] = input_v[375:368];
assign input_18[7:0] = input_v[367:360];
assign input_19[7:0] = input_v[359:352];
assign input_20[7:0] = input_v[351:344];
assign input_21[7:0] = input_v[343:336];
assign input_22[7:0] = input_v[335:328];
assign input_23[7:0] = input_v[327:320];
assign input_24[7:0] = input_v[319:312];
assign input_25[7:0] = input_v[311:304];
assign input_26[7:0] = input_v[303:296];
assign input_27[7:0] = input_v[295:288];
assign input_28[7:0] = input_v[287:280];
assign input_29[7:0] = input_v[279:272];
assign input_30[7:0] = input_v[271:264];
assign input_31[7:0] = input_v[263:256];
assign input_32[7:0] = input_v[255:248];
assign input_33[7:0] = input_v[247:240];
assign input_34[7:0] = input_v[239:232];
assign input_35[7:0] = input_v[231:224];
assign input_36[7:0] = input_v[223:216];
assign input_37[7:0] = input_v[215:208];
assign input_38[7:0] = input_v[207:200];
assign input_39[7:0] = input_v[199:192];
assign input_40[7:0] = input_v[191:184];
assign input_41[7:0] = input_v[183:176];
assign input_42[7:0] = input_v[175:168];
assign input_43[7:0] = input_v[167:160];
assign input_44[7:0] = input_v[159:152];
assign input_45[7:0] = input_v[151:144];
assign input_46[7:0] = input_v[143:136];
assign input_47[7:0] = input_v[135:128];
assign input_48[7:0] = input_v[127:120];
assign input_49[7:0] = input_v[119:112];
assign input_50[7:0] = input_v[111:104];
assign input_51[7:0] = input_v[103:96];
assign input_52[7:0] = input_v[95:88];
assign input_53[7:0] = input_v[87:80];
assign input_54[7:0] = input_v[79:72];
assign input_55[7:0] = input_v[71:64];
assign input_56[7:0] = input_v[63:56];
assign input_57[7:0] = input_v[55:48];
assign input_58[7:0] = input_v[47:40];
assign input_59[7:0] = input_v[39:32];
assign input_60[7:0] = input_v[31:24];
assign input_61[7:0] = input_v[23:16];
assign input_62[7:0] = input_v[15:8];
assign input_63[7:0] = input_v[7:0];
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
assign lut8_BRAM_i_16[7:0] = input_16[7:0];
assign lut8_BRAM_i_17[7:0] = input_17[7:0];
assign lut8_BRAM_i_18[7:0] = input_18[7:0];
assign lut8_BRAM_i_19[7:0] = input_19[7:0];
assign lut8_BRAM_i_20[7:0] = input_20[7:0];
assign lut8_BRAM_i_21[7:0] = input_21[7:0];
assign lut8_BRAM_i_22[7:0] = input_22[7:0];
assign lut8_BRAM_i_23[7:0] = input_23[7:0];
assign lut8_BRAM_i_24[7:0] = input_24[7:0];
assign lut8_BRAM_i_25[7:0] = input_25[7:0];
assign lut8_BRAM_i_26[7:0] = input_26[7:0];
assign lut8_BRAM_i_27[7:0] = input_27[7:0];
assign lut8_BRAM_i_28[7:0] = input_28[7:0];
assign lut8_BRAM_i_29[7:0] = input_29[7:0];
assign lut8_BRAM_i_30[7:0] = input_30[7:0];
assign lut8_BRAM_i_31[7:0] = input_31[7:0];
assign lut8_BRAM_i_32[7:0] = input_32[7:0];
assign lut8_BRAM_i_33[7:0] = input_33[7:0];
assign lut8_BRAM_i_34[7:0] = input_34[7:0];
assign lut8_BRAM_i_35[7:0] = input_35[7:0];
assign lut8_BRAM_i_36[7:0] = input_36[7:0];
assign lut8_BRAM_i_37[7:0] = input_37[7:0];
assign lut8_BRAM_i_38[7:0] = input_38[7:0];
assign lut8_BRAM_i_39[7:0] = input_39[7:0];
assign lut8_BRAM_i_40[7:0] = input_40[7:0];
assign lut8_BRAM_i_41[7:0] = input_41[7:0];
assign lut8_BRAM_i_42[7:0] = input_42[7:0];
assign lut8_BRAM_i_43[7:0] = input_43[7:0];
assign lut8_BRAM_i_44[7:0] = input_44[7:0];
assign lut8_BRAM_i_45[7:0] = input_45[7:0];
assign lut8_BRAM_i_46[7:0] = input_46[7:0];
assign lut8_BRAM_i_47[7:0] = input_47[7:0];
assign lut8_BRAM_i_48[7:0] = input_48[7:0];
assign lut8_BRAM_i_49[7:0] = input_49[7:0];
assign lut8_BRAM_i_50[7:0] = input_50[7:0];
assign lut8_BRAM_i_51[7:0] = input_51[7:0];
assign lut8_BRAM_i_52[7:0] = input_52[7:0];
assign lut8_BRAM_i_53[7:0] = input_53[7:0];
assign lut8_BRAM_i_54[7:0] = input_54[7:0];
assign lut8_BRAM_i_55[7:0] = input_55[7:0];
assign lut8_BRAM_i_56[7:0] = input_56[7:0];
assign lut8_BRAM_i_57[7:0] = input_57[7:0];
assign lut8_BRAM_i_58[7:0] = input_58[7:0];
assign lut8_BRAM_i_59[7:0] = input_59[7:0];
assign lut8_BRAM_i_60[7:0] = input_60[7:0];
assign lut8_BRAM_i_61[7:0] = input_61[7:0];
assign lut8_BRAM_i_62[7:0] = input_62[7:0];
assign lut8_BRAM_i_63[7:0] = input_63[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
lut8_BRAM lut8_BRAM_1 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_2),.lut8_BRAM_i_1(lut8_BRAM_i_3),.lut8_BRAM_o_0(lut8_BRAM_o_2),.lut8_BRAM_o_1(lut8_BRAM_o_3) );
lut8_BRAM lut8_BRAM_2 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_4),.lut8_BRAM_i_1(lut8_BRAM_i_5),.lut8_BRAM_o_0(lut8_BRAM_o_4),.lut8_BRAM_o_1(lut8_BRAM_o_5) );
lut8_BRAM lut8_BRAM_3 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_6),.lut8_BRAM_i_1(lut8_BRAM_i_7),.lut8_BRAM_o_0(lut8_BRAM_o_6),.lut8_BRAM_o_1(lut8_BRAM_o_7) );
lut8_BRAM lut8_BRAM_4 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_8),.lut8_BRAM_i_1(lut8_BRAM_i_9),.lut8_BRAM_o_0(lut8_BRAM_o_8),.lut8_BRAM_o_1(lut8_BRAM_o_9) );
lut8_BRAM lut8_BRAM_5 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_10),.lut8_BRAM_i_1(lut8_BRAM_i_11),.lut8_BRAM_o_0(lut8_BRAM_o_10),.lut8_BRAM_o_1(lut8_BRAM_o_11) );
lut8_BRAM lut8_BRAM_6 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_12),.lut8_BRAM_i_1(lut8_BRAM_i_13),.lut8_BRAM_o_0(lut8_BRAM_o_12),.lut8_BRAM_o_1(lut8_BRAM_o_13) );
lut8_BRAM lut8_BRAM_7 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_14),.lut8_BRAM_i_1(lut8_BRAM_i_15),.lut8_BRAM_o_0(lut8_BRAM_o_14),.lut8_BRAM_o_1(lut8_BRAM_o_15) );
lut8_BRAM lut8_BRAM_8 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_16),.lut8_BRAM_i_1(lut8_BRAM_i_17),.lut8_BRAM_o_0(lut8_BRAM_o_16),.lut8_BRAM_o_1(lut8_BRAM_o_17) );
lut8_BRAM lut8_BRAM_9 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_18),.lut8_BRAM_i_1(lut8_BRAM_i_19),.lut8_BRAM_o_0(lut8_BRAM_o_18),.lut8_BRAM_o_1(lut8_BRAM_o_19) );
lut8_BRAM lut8_BRAM_10 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_20),.lut8_BRAM_i_1(lut8_BRAM_i_21),.lut8_BRAM_o_0(lut8_BRAM_o_20),.lut8_BRAM_o_1(lut8_BRAM_o_21) );
lut8_BRAM lut8_BRAM_11 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_22),.lut8_BRAM_i_1(lut8_BRAM_i_23),.lut8_BRAM_o_0(lut8_BRAM_o_22),.lut8_BRAM_o_1(lut8_BRAM_o_23) );
lut8_BRAM lut8_BRAM_12 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_24),.lut8_BRAM_i_1(lut8_BRAM_i_25),.lut8_BRAM_o_0(lut8_BRAM_o_24),.lut8_BRAM_o_1(lut8_BRAM_o_25) );
lut8_BRAM lut8_BRAM_13 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_26),.lut8_BRAM_i_1(lut8_BRAM_i_27),.lut8_BRAM_o_0(lut8_BRAM_o_26),.lut8_BRAM_o_1(lut8_BRAM_o_27) );
lut8_BRAM lut8_BRAM_14 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_28),.lut8_BRAM_i_1(lut8_BRAM_i_29),.lut8_BRAM_o_0(lut8_BRAM_o_28),.lut8_BRAM_o_1(lut8_BRAM_o_29) );
lut8_BRAM lut8_BRAM_15 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_30),.lut8_BRAM_i_1(lut8_BRAM_i_31),.lut8_BRAM_o_0(lut8_BRAM_o_30),.lut8_BRAM_o_1(lut8_BRAM_o_31) );
lut8_BRAM lut8_BRAM_16 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_32),.lut8_BRAM_i_1(lut8_BRAM_i_33),.lut8_BRAM_o_0(lut8_BRAM_o_32),.lut8_BRAM_o_1(lut8_BRAM_o_33) );
lut8_BRAM lut8_BRAM_17 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_34),.lut8_BRAM_i_1(lut8_BRAM_i_35),.lut8_BRAM_o_0(lut8_BRAM_o_34),.lut8_BRAM_o_1(lut8_BRAM_o_35) );
lut8_BRAM lut8_BRAM_18 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_36),.lut8_BRAM_i_1(lut8_BRAM_i_37),.lut8_BRAM_o_0(lut8_BRAM_o_36),.lut8_BRAM_o_1(lut8_BRAM_o_37) );
lut8_BRAM lut8_BRAM_19 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_38),.lut8_BRAM_i_1(lut8_BRAM_i_39),.lut8_BRAM_o_0(lut8_BRAM_o_38),.lut8_BRAM_o_1(lut8_BRAM_o_39) );
lut8_BRAM lut8_BRAM_20 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_40),.lut8_BRAM_i_1(lut8_BRAM_i_41),.lut8_BRAM_o_0(lut8_BRAM_o_40),.lut8_BRAM_o_1(lut8_BRAM_o_41) );
lut8_BRAM lut8_BRAM_21 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_42),.lut8_BRAM_i_1(lut8_BRAM_i_43),.lut8_BRAM_o_0(lut8_BRAM_o_42),.lut8_BRAM_o_1(lut8_BRAM_o_43) );
lut8_BRAM lut8_BRAM_22 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_44),.lut8_BRAM_i_1(lut8_BRAM_i_45),.lut8_BRAM_o_0(lut8_BRAM_o_44),.lut8_BRAM_o_1(lut8_BRAM_o_45) );
lut8_BRAM lut8_BRAM_23 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_46),.lut8_BRAM_i_1(lut8_BRAM_i_47),.lut8_BRAM_o_0(lut8_BRAM_o_46),.lut8_BRAM_o_1(lut8_BRAM_o_47) );
lut8_BRAM lut8_BRAM_24 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_48),.lut8_BRAM_i_1(lut8_BRAM_i_49),.lut8_BRAM_o_0(lut8_BRAM_o_48),.lut8_BRAM_o_1(lut8_BRAM_o_49) );
lut8_BRAM lut8_BRAM_25 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_50),.lut8_BRAM_i_1(lut8_BRAM_i_51),.lut8_BRAM_o_0(lut8_BRAM_o_50),.lut8_BRAM_o_1(lut8_BRAM_o_51) );
lut8_BRAM lut8_BRAM_26 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_52),.lut8_BRAM_i_1(lut8_BRAM_i_53),.lut8_BRAM_o_0(lut8_BRAM_o_52),.lut8_BRAM_o_1(lut8_BRAM_o_53) );
lut8_BRAM lut8_BRAM_27 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_54),.lut8_BRAM_i_1(lut8_BRAM_i_55),.lut8_BRAM_o_0(lut8_BRAM_o_54),.lut8_BRAM_o_1(lut8_BRAM_o_55) );
lut8_BRAM lut8_BRAM_28 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_56),.lut8_BRAM_i_1(lut8_BRAM_i_57),.lut8_BRAM_o_0(lut8_BRAM_o_56),.lut8_BRAM_o_1(lut8_BRAM_o_57) );
lut8_BRAM lut8_BRAM_29 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_58),.lut8_BRAM_i_1(lut8_BRAM_i_59),.lut8_BRAM_o_0(lut8_BRAM_o_58),.lut8_BRAM_o_1(lut8_BRAM_o_59) );
lut8_BRAM lut8_BRAM_30 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_60),.lut8_BRAM_i_1(lut8_BRAM_i_61),.lut8_BRAM_o_0(lut8_BRAM_o_60),.lut8_BRAM_o_1(lut8_BRAM_o_61) );
lut8_BRAM lut8_BRAM_31 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_62),.lut8_BRAM_i_1(lut8_BRAM_i_63),.lut8_BRAM_o_0(lut8_BRAM_o_62),.lut8_BRAM_o_1(lut8_BRAM_o_63) );
wire [3:0] add_5_i_0_0;
wire [3:0] add_5_i_0_1;
wire [3:0] add_5_i_2_0;
wire [3:0] add_5_i_2_1;
wire [4:0] add_5_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [4:0] add_4_i_0_0;
wire [4:0] add_4_i_0_1;
wire [4:0] add_4_i_1_0;
wire [4:0] add_4_i_1_1;
wire [5:0] add_4_o_0_0;
wire [3:0] add_8_i_0_0;
wire [3:0] add_8_i_0_1;
wire [3:0] add_8_i_2_0;
wire [3:0] add_8_i_2_1;
wire [4:0] add_8_o_0_0;
wire [3:0] add_9_i_0_0;
wire [3:0] add_9_i_0_1;
wire [3:0] add_9_i_2_0;
wire [3:0] add_9_i_2_1;
wire [4:0] add_9_o_0_0;
wire [4:0] add_7_i_0_0;
wire [4:0] add_7_i_0_1;
wire [4:0] add_7_i_1_0;
wire [4:0] add_7_i_1_1;
wire [5:0] add_7_o_0_0;
wire [5:0] add_3_i_0_0;
wire [5:0] add_3_i_0_1;
wire [5:0] add_3_i_1_0;
wire [5:0] add_3_i_1_1;
wire [6:0] add_3_o_0_0;
wire [3:0] add_12_i_0_0;
wire [3:0] add_12_i_0_1;
wire [3:0] add_12_i_2_0;
wire [3:0] add_12_i_2_1;
wire [4:0] add_12_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [4:0] add_11_i_0_0;
wire [4:0] add_11_i_0_1;
wire [4:0] add_11_i_1_0;
wire [4:0] add_11_i_1_1;
wire [5:0] add_11_o_0_0;
wire [3:0] add_15_i_0_0;
wire [3:0] add_15_i_0_1;
wire [3:0] add_15_i_2_0;
wire [3:0] add_15_i_2_1;
wire [4:0] add_15_o_0_0;
wire [3:0] add_16_i_0_0;
wire [3:0] add_16_i_0_1;
wire [3:0] add_16_i_2_0;
wire [3:0] add_16_i_2_1;
wire [4:0] add_16_o_0_0;
wire [4:0] add_14_i_0_0;
wire [4:0] add_14_i_0_1;
wire [4:0] add_14_i_1_0;
wire [4:0] add_14_i_1_1;
wire [5:0] add_14_o_0_0;
wire [5:0] add_10_i_0_0;
wire [5:0] add_10_i_0_1;
wire [5:0] add_10_i_1_0;
wire [5:0] add_10_i_1_1;
wire [6:0] add_10_o_0_0;
wire [6:0] add_2_i_0_0;
wire [6:0] add_2_i_0_1;
wire [6:0] add_2_i_1_0;
wire [6:0] add_2_i_1_1;
wire [7:0] add_2_o_0_0;
wire [3:0] add_20_i_0_0;
wire [3:0] add_20_i_0_1;
wire [3:0] add_20_i_2_0;
wire [3:0] add_20_i_2_1;
wire [4:0] add_20_o_0_0;
wire [3:0] add_21_i_0_0;
wire [3:0] add_21_i_0_1;
wire [3:0] add_21_i_2_0;
wire [3:0] add_21_i_2_1;
wire [4:0] add_21_o_0_0;
wire [4:0] add_19_i_0_0;
wire [4:0] add_19_i_0_1;
wire [4:0] add_19_i_1_0;
wire [4:0] add_19_i_1_1;
wire [5:0] add_19_o_0_0;
wire [3:0] add_23_i_0_0;
wire [3:0] add_23_i_0_1;
wire [3:0] add_23_i_2_0;
wire [3:0] add_23_i_2_1;
wire [4:0] add_23_o_0_0;
wire [3:0] add_24_i_0_0;
wire [3:0] add_24_i_0_1;
wire [3:0] add_24_i_2_0;
wire [3:0] add_24_i_2_1;
wire [4:0] add_24_o_0_0;
wire [4:0] add_22_i_0_0;
wire [4:0] add_22_i_0_1;
wire [4:0] add_22_i_1_0;
wire [4:0] add_22_i_1_1;
wire [5:0] add_22_o_0_0;
wire [5:0] add_18_i_0_0;
wire [5:0] add_18_i_0_1;
wire [5:0] add_18_i_1_0;
wire [5:0] add_18_i_1_1;
wire [6:0] add_18_o_0_0;
wire [3:0] add_27_i_0_0;
wire [3:0] add_27_i_0_1;
wire [3:0] add_27_i_2_0;
wire [3:0] add_27_i_2_1;
wire [4:0] add_27_o_0_0;
wire [3:0] add_28_i_0_0;
wire [3:0] add_28_i_0_1;
wire [3:0] add_28_i_2_0;
wire [3:0] add_28_i_2_1;
wire [4:0] add_28_o_0_0;
wire [4:0] add_26_i_0_0;
wire [4:0] add_26_i_0_1;
wire [4:0] add_26_i_1_0;
wire [4:0] add_26_i_1_1;
wire [5:0] add_26_o_0_0;
wire [3:0] add_30_i_0_0;
wire [3:0] add_30_i_0_1;
wire [3:0] add_30_i_2_0;
wire [3:0] add_30_i_2_1;
wire [4:0] add_30_o_0_0;
wire [3:0] add_31_i_0_0;
wire [3:0] add_31_i_0_1;
wire [3:0] add_31_i_2_0;
wire [3:0] add_31_i_2_1;
wire [4:0] add_31_o_0_0;
wire [4:0] add_29_i_0_0;
wire [4:0] add_29_i_0_1;
wire [4:0] add_29_i_1_0;
wire [4:0] add_29_i_1_1;
wire [5:0] add_29_o_0_0;
wire [5:0] add_25_i_0_0;
wire [5:0] add_25_i_0_1;
wire [5:0] add_25_i_1_0;
wire [5:0] add_25_i_1_1;
wire [6:0] add_25_o_0_0;
wire [6:0] add_17_i_0_0;
wire [6:0] add_17_i_0_1;
wire [6:0] add_17_i_1_0;
wire [6:0] add_17_i_1_1;
wire [7:0] add_17_o_0_0;
wire [7:0] add_1_i_0_0;
wire [7:0] add_1_i_0_1;
wire [7:0] add_1_i_1_0;
wire [7:0] add_1_i_1_1;
wire [8:0] add_1_o_0_0;
wire [3:0] add_36_i_0_0;
wire [3:0] add_36_i_0_1;
wire [3:0] add_36_i_2_0;
wire [3:0] add_36_i_2_1;
wire [4:0] add_36_o_0_0;
wire [3:0] add_37_i_0_0;
wire [3:0] add_37_i_0_1;
wire [3:0] add_37_i_2_0;
wire [3:0] add_37_i_2_1;
wire [4:0] add_37_o_0_0;
wire [4:0] add_35_i_0_0;
wire [4:0] add_35_i_0_1;
wire [4:0] add_35_i_1_0;
wire [4:0] add_35_i_1_1;
wire [5:0] add_35_o_0_0;
wire [3:0] add_39_i_0_0;
wire [3:0] add_39_i_0_1;
wire [3:0] add_39_i_2_0;
wire [3:0] add_39_i_2_1;
wire [4:0] add_39_o_0_0;
wire [3:0] add_40_i_0_0;
wire [3:0] add_40_i_0_1;
wire [3:0] add_40_i_2_0;
wire [3:0] add_40_i_2_1;
wire [4:0] add_40_o_0_0;
wire [4:0] add_38_i_0_0;
wire [4:0] add_38_i_0_1;
wire [4:0] add_38_i_1_0;
wire [4:0] add_38_i_1_1;
wire [5:0] add_38_o_0_0;
wire [5:0] add_34_i_0_0;
wire [5:0] add_34_i_0_1;
wire [5:0] add_34_i_1_0;
wire [5:0] add_34_i_1_1;
wire [6:0] add_34_o_0_0;
wire [3:0] add_43_i_0_0;
wire [3:0] add_43_i_0_1;
wire [3:0] add_43_i_2_0;
wire [3:0] add_43_i_2_1;
wire [4:0] add_43_o_0_0;
wire [3:0] add_44_i_0_0;
wire [3:0] add_44_i_0_1;
wire [3:0] add_44_i_2_0;
wire [3:0] add_44_i_2_1;
wire [4:0] add_44_o_0_0;
wire [4:0] add_42_i_0_0;
wire [4:0] add_42_i_0_1;
wire [4:0] add_42_i_1_0;
wire [4:0] add_42_i_1_1;
wire [5:0] add_42_o_0_0;
wire [3:0] add_46_i_0_0;
wire [3:0] add_46_i_0_1;
wire [3:0] add_46_i_2_0;
wire [3:0] add_46_i_2_1;
wire [4:0] add_46_o_0_0;
wire [3:0] add_47_i_0_0;
wire [3:0] add_47_i_0_1;
wire [3:0] add_47_i_2_0;
wire [3:0] add_47_i_2_1;
wire [4:0] add_47_o_0_0;
wire [4:0] add_45_i_0_0;
wire [4:0] add_45_i_0_1;
wire [4:0] add_45_i_1_0;
wire [4:0] add_45_i_1_1;
wire [5:0] add_45_o_0_0;
wire [5:0] add_41_i_0_0;
wire [5:0] add_41_i_0_1;
wire [5:0] add_41_i_1_0;
wire [5:0] add_41_i_1_1;
wire [6:0] add_41_o_0_0;
wire [6:0] add_33_i_0_0;
wire [6:0] add_33_i_0_1;
wire [6:0] add_33_i_1_0;
wire [6:0] add_33_i_1_1;
wire [7:0] add_33_o_0_0;
wire [3:0] add_51_i_0_0;
wire [3:0] add_51_i_0_1;
wire [3:0] add_51_i_2_0;
wire [3:0] add_51_i_2_1;
wire [4:0] add_51_o_0_0;
wire [3:0] add_52_i_0_0;
wire [3:0] add_52_i_0_1;
wire [3:0] add_52_i_2_0;
wire [3:0] add_52_i_2_1;
wire [4:0] add_52_o_0_0;
wire [4:0] add_50_i_0_0;
wire [4:0] add_50_i_0_1;
wire [4:0] add_50_i_1_0;
wire [4:0] add_50_i_1_1;
wire [5:0] add_50_o_0_0;
wire [3:0] add_54_i_0_0;
wire [3:0] add_54_i_0_1;
wire [3:0] add_54_i_2_0;
wire [3:0] add_54_i_2_1;
wire [4:0] add_54_o_0_0;
wire [3:0] add_55_i_0_0;
wire [3:0] add_55_i_0_1;
wire [3:0] add_55_i_2_0;
wire [3:0] add_55_i_2_1;
wire [4:0] add_55_o_0_0;
wire [4:0] add_53_i_0_0;
wire [4:0] add_53_i_0_1;
wire [4:0] add_53_i_1_0;
wire [4:0] add_53_i_1_1;
wire [5:0] add_53_o_0_0;
wire [5:0] add_49_i_0_0;
wire [5:0] add_49_i_0_1;
wire [5:0] add_49_i_1_0;
wire [5:0] add_49_i_1_1;
wire [6:0] add_49_o_0_0;
wire [3:0] add_58_i_0_0;
wire [3:0] add_58_i_0_1;
wire [3:0] add_58_i_2_0;
wire [3:0] add_58_i_2_1;
wire [4:0] add_58_o_0_0;
wire [3:0] add_59_i_0_0;
wire [3:0] add_59_i_0_1;
wire [3:0] add_59_i_2_0;
wire [3:0] add_59_i_2_1;
wire [4:0] add_59_o_0_0;
wire [4:0] add_57_i_0_0;
wire [4:0] add_57_i_0_1;
wire [4:0] add_57_i_1_0;
wire [4:0] add_57_i_1_1;
wire [5:0] add_57_o_0_0;
wire [3:0] add_61_i_0_0;
wire [3:0] add_61_i_0_1;
wire [3:0] add_61_i_2_0;
wire [3:0] add_61_i_2_1;
wire [4:0] add_61_o_0_0;
wire [3:0] add_62_i_0_0;
wire [3:0] add_62_i_0_1;
wire [3:0] add_62_i_2_0;
wire [3:0] add_62_i_2_1;
wire [4:0] add_62_o_0_0;
wire [4:0] add_60_i_0_0;
wire [4:0] add_60_i_0_1;
wire [4:0] add_60_i_1_0;
wire [4:0] add_60_i_1_1;
wire [5:0] add_60_o_0_0;
wire [5:0] add_56_i_0_0;
wire [5:0] add_56_i_0_1;
wire [5:0] add_56_i_1_0;
wire [5:0] add_56_i_1_1;
wire [6:0] add_56_o_0_0;
wire [6:0] add_48_i_0_0;
wire [6:0] add_48_i_0_1;
wire [6:0] add_48_i_1_0;
wire [6:0] add_48_i_1_1;
wire [7:0] add_48_o_0_0;
wire [7:0] add_32_i_0_0;
wire [7:0] add_32_i_0_1;
wire [7:0] add_32_i_1_0;
wire [7:0] add_32_i_1_1;
wire [8:0] add_32_o_0_0;
wire [8:0] add_0_i_0_0;
wire [8:0] add_0_i_0_1;
wire [8:0] add_0_i_1_0;
wire [8:0] add_0_i_1_1;
wire [9:0] add_0_o_0_0;
wire [9:0] add_0_o_1_0;
assign add_5_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_5_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
assign add_6_i_0_0[3:0] = lut8_BRAM_o_2[3:0];
assign add_6_i_0_1[3:0] = lut8_BRAM_o_3[3:0];
assign add_8_i_0_0[3:0] = lut8_BRAM_o_4[3:0];
assign add_8_i_0_1[3:0] = lut8_BRAM_o_5[3:0];
assign add_9_i_0_0[3:0] = lut8_BRAM_o_6[3:0];
assign add_9_i_0_1[3:0] = lut8_BRAM_o_7[3:0];
assign add_12_i_0_0[3:0] = lut8_BRAM_o_8[3:0];
assign add_12_i_0_1[3:0] = lut8_BRAM_o_9[3:0];
assign add_13_i_0_0[3:0] = lut8_BRAM_o_10[3:0];
assign add_13_i_0_1[3:0] = lut8_BRAM_o_11[3:0];
assign add_15_i_0_0[3:0] = lut8_BRAM_o_12[3:0];
assign add_15_i_0_1[3:0] = lut8_BRAM_o_13[3:0];
assign add_16_i_0_0[3:0] = lut8_BRAM_o_14[3:0];
assign add_16_i_0_1[3:0] = lut8_BRAM_o_15[3:0];
assign add_20_i_0_0[3:0] = lut8_BRAM_o_16[3:0];
assign add_20_i_0_1[3:0] = lut8_BRAM_o_17[3:0];
assign add_21_i_0_0[3:0] = lut8_BRAM_o_18[3:0];
assign add_21_i_0_1[3:0] = lut8_BRAM_o_19[3:0];
assign add_23_i_0_0[3:0] = lut8_BRAM_o_20[3:0];
assign add_23_i_0_1[3:0] = lut8_BRAM_o_21[3:0];
assign add_24_i_0_0[3:0] = lut8_BRAM_o_22[3:0];
assign add_24_i_0_1[3:0] = lut8_BRAM_o_23[3:0];
assign add_27_i_0_0[3:0] = lut8_BRAM_o_24[3:0];
assign add_27_i_0_1[3:0] = lut8_BRAM_o_25[3:0];
assign add_28_i_0_0[3:0] = lut8_BRAM_o_26[3:0];
assign add_28_i_0_1[3:0] = lut8_BRAM_o_27[3:0];
assign add_30_i_0_0[3:0] = lut8_BRAM_o_28[3:0];
assign add_30_i_0_1[3:0] = lut8_BRAM_o_29[3:0];
assign add_31_i_0_0[3:0] = lut8_BRAM_o_30[3:0];
assign add_31_i_0_1[3:0] = lut8_BRAM_o_31[3:0];
assign add_36_i_0_0[3:0] = lut8_BRAM_o_32[3:0];
assign add_36_i_0_1[3:0] = lut8_BRAM_o_33[3:0];
assign add_37_i_0_0[3:0] = lut8_BRAM_o_34[3:0];
assign add_37_i_0_1[3:0] = lut8_BRAM_o_35[3:0];
assign add_39_i_0_0[3:0] = lut8_BRAM_o_36[3:0];
assign add_39_i_0_1[3:0] = lut8_BRAM_o_37[3:0];
assign add_40_i_0_0[3:0] = lut8_BRAM_o_38[3:0];
assign add_40_i_0_1[3:0] = lut8_BRAM_o_39[3:0];
assign add_43_i_0_0[3:0] = lut8_BRAM_o_40[3:0];
assign add_43_i_0_1[3:0] = lut8_BRAM_o_41[3:0];
assign add_44_i_0_0[3:0] = lut8_BRAM_o_42[3:0];
assign add_44_i_0_1[3:0] = lut8_BRAM_o_43[3:0];
assign add_46_i_0_0[3:0] = lut8_BRAM_o_44[3:0];
assign add_46_i_0_1[3:0] = lut8_BRAM_o_45[3:0];
assign add_47_i_0_0[3:0] = lut8_BRAM_o_46[3:0];
assign add_47_i_0_1[3:0] = lut8_BRAM_o_47[3:0];
assign add_51_i_0_0[3:0] = lut8_BRAM_o_48[3:0];
assign add_51_i_0_1[3:0] = lut8_BRAM_o_49[3:0];
assign add_52_i_0_0[3:0] = lut8_BRAM_o_50[3:0];
assign add_52_i_0_1[3:0] = lut8_BRAM_o_51[3:0];
assign add_54_i_0_0[3:0] = lut8_BRAM_o_52[3:0];
assign add_54_i_0_1[3:0] = lut8_BRAM_o_53[3:0];
assign add_55_i_0_0[3:0] = lut8_BRAM_o_54[3:0];
assign add_55_i_0_1[3:0] = lut8_BRAM_o_55[3:0];
assign add_58_i_0_0[3:0] = lut8_BRAM_o_56[3:0];
assign add_58_i_0_1[3:0] = lut8_BRAM_o_57[3:0];
assign add_59_i_0_0[3:0] = lut8_BRAM_o_58[3:0];
assign add_59_i_0_1[3:0] = lut8_BRAM_o_59[3:0];
assign add_61_i_0_0[3:0] = lut8_BRAM_o_60[3:0];
assign add_61_i_0_1[3:0] = lut8_BRAM_o_61[3:0];
assign add_62_i_0_0[3:0] = lut8_BRAM_o_62[3:0];
assign add_62_i_0_1[3:0] = lut8_BRAM_o_63[3:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_3_i_0_0[5:0] = add_4_o_0_0[5:0];
assign add_7_i_0_0[4:0] = add_8_o_0_0[4:0];
assign add_7_i_0_1[4:0] = add_9_o_0_0[4:0];
assign add_3_i_0_1[5:0] = add_7_o_0_0[5:0];
assign add_2_i_0_0[6:0] = add_3_o_0_0[6:0];
assign add_11_i_0_0[4:0] = add_12_o_0_0[4:0];
assign add_11_i_0_1[4:0] = add_13_o_0_0[4:0];
assign add_10_i_0_0[5:0] = add_11_o_0_0[5:0];
assign add_14_i_0_0[4:0] = add_15_o_0_0[4:0];
assign add_14_i_0_1[4:0] = add_16_o_0_0[4:0];
assign add_10_i_0_1[5:0] = add_14_o_0_0[5:0];
assign add_2_i_0_1[6:0] = add_10_o_0_0[6:0];
assign add_1_i_0_0[7:0] = add_2_o_0_0[7:0];
assign add_19_i_0_0[4:0] = add_20_o_0_0[4:0];
assign add_19_i_0_1[4:0] = add_21_o_0_0[4:0];
assign add_18_i_0_0[5:0] = add_19_o_0_0[5:0];
assign add_22_i_0_0[4:0] = add_23_o_0_0[4:0];
assign add_22_i_0_1[4:0] = add_24_o_0_0[4:0];
assign add_18_i_0_1[5:0] = add_22_o_0_0[5:0];
assign add_17_i_0_0[6:0] = add_18_o_0_0[6:0];
assign add_26_i_0_0[4:0] = add_27_o_0_0[4:0];
assign add_26_i_0_1[4:0] = add_28_o_0_0[4:0];
assign add_25_i_0_0[5:0] = add_26_o_0_0[5:0];
assign add_29_i_0_0[4:0] = add_30_o_0_0[4:0];
assign add_29_i_0_1[4:0] = add_31_o_0_0[4:0];
assign add_25_i_0_1[5:0] = add_29_o_0_0[5:0];
assign add_17_i_0_1[6:0] = add_25_o_0_0[6:0];
assign add_1_i_0_1[7:0] = add_17_o_0_0[7:0];
assign add_0_i_0_0[8:0] = add_1_o_0_0[8:0];
assign add_35_i_0_0[4:0] = add_36_o_0_0[4:0];
assign add_35_i_0_1[4:0] = add_37_o_0_0[4:0];
assign add_34_i_0_0[5:0] = add_35_o_0_0[5:0];
assign add_38_i_0_0[4:0] = add_39_o_0_0[4:0];
assign add_38_i_0_1[4:0] = add_40_o_0_0[4:0];
assign add_34_i_0_1[5:0] = add_38_o_0_0[5:0];
assign add_33_i_0_0[6:0] = add_34_o_0_0[6:0];
assign add_42_i_0_0[4:0] = add_43_o_0_0[4:0];
assign add_42_i_0_1[4:0] = add_44_o_0_0[4:0];
assign add_41_i_0_0[5:0] = add_42_o_0_0[5:0];
assign add_45_i_0_0[4:0] = add_46_o_0_0[4:0];
assign add_45_i_0_1[4:0] = add_47_o_0_0[4:0];
assign add_41_i_0_1[5:0] = add_45_o_0_0[5:0];
assign add_33_i_0_1[6:0] = add_41_o_0_0[6:0];
assign add_32_i_0_0[7:0] = add_33_o_0_0[7:0];
assign add_50_i_0_0[4:0] = add_51_o_0_0[4:0];
assign add_50_i_0_1[4:0] = add_52_o_0_0[4:0];
assign add_49_i_0_0[5:0] = add_50_o_0_0[5:0];
assign add_53_i_0_0[4:0] = add_54_o_0_0[4:0];
assign add_53_i_0_1[4:0] = add_55_o_0_0[4:0];
assign add_49_i_0_1[5:0] = add_53_o_0_0[5:0];
assign add_48_i_0_0[6:0] = add_49_o_0_0[6:0];
assign add_57_i_0_0[4:0] = add_58_o_0_0[4:0];
assign add_57_i_0_1[4:0] = add_59_o_0_0[4:0];
assign add_56_i_0_0[5:0] = add_57_o_0_0[5:0];
assign add_60_i_0_0[4:0] = add_61_o_0_0[4:0];
assign add_60_i_0_1[4:0] = add_62_o_0_0[4:0];
assign add_56_i_0_1[5:0] = add_60_o_0_0[5:0];
assign add_48_i_0_1[6:0] = add_56_o_0_0[6:0];
assign add_32_i_0_1[7:0] = add_48_o_0_0[7:0];
assign add_0_i_0_1[8:0] = add_32_o_0_0[8:0];
du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(4, 2) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_0),.din(add_8_i_0_0));
du #(4, 2) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_2_0 + add_8_i_2_1;

du #(4, 2) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_0),.din(add_9_i_0_0));
du #(4, 2) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_2_0 + add_9_i_2_1;

du #(5, 1) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_0),.din(add_7_i_0_0));
du #(5, 1) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_1_0 + add_7_i_1_1;

du #(6, 1) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_0),.din(add_3_i_0_0));
du #(6, 1) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_1_0 + add_3_i_1_1;

du #(4, 2) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_0),.din(add_12_i_0_0));
du #(4, 2) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_2_0 + add_12_i_2_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(5, 1) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_0),.din(add_11_i_0_0));
du #(5, 1) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_1_0 + add_11_i_1_1;

du #(4, 2) du_15_0 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_0),.din(add_15_i_0_0));
du #(4, 2) du_15_1 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_1),.din(add_15_i_0_1));

assign add_15_o_0_0 = add_15_i_2_0 + add_15_i_2_1;

du #(4, 2) du_16_0 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_0),.din(add_16_i_0_0));
du #(4, 2) du_16_1 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_1),.din(add_16_i_0_1));

assign add_16_o_0_0 = add_16_i_2_0 + add_16_i_2_1;

du #(5, 1) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_0),.din(add_14_i_0_0));
du #(5, 1) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_1_0 + add_14_i_1_1;

du #(6, 1) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_0),.din(add_10_i_0_0));
du #(6, 1) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_1_0 + add_10_i_1_1;

du #(7, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(7, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_20_0 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_0),.din(add_20_i_0_0));
du #(4, 2) du_20_1 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_1),.din(add_20_i_0_1));

assign add_20_o_0_0 = add_20_i_2_0 + add_20_i_2_1;

du #(4, 2) du_21_0 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_0),.din(add_21_i_0_0));
du #(4, 2) du_21_1 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_1),.din(add_21_i_0_1));

assign add_21_o_0_0 = add_21_i_2_0 + add_21_i_2_1;

du #(5, 1) du_19_0 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_0),.din(add_19_i_0_0));
du #(5, 1) du_19_1 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_1),.din(add_19_i_0_1));

assign add_19_o_0_0 = add_19_i_1_0 + add_19_i_1_1;

du #(4, 2) du_23_0 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_0),.din(add_23_i_0_0));
du #(4, 2) du_23_1 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_1),.din(add_23_i_0_1));

assign add_23_o_0_0 = add_23_i_2_0 + add_23_i_2_1;

du #(4, 2) du_24_0 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_0),.din(add_24_i_0_0));
du #(4, 2) du_24_1 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_1),.din(add_24_i_0_1));

assign add_24_o_0_0 = add_24_i_2_0 + add_24_i_2_1;

du #(5, 1) du_22_0 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_0),.din(add_22_i_0_0));
du #(5, 1) du_22_1 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_1),.din(add_22_i_0_1));

assign add_22_o_0_0 = add_22_i_1_0 + add_22_i_1_1;

du #(6, 1) du_18_0 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_0),.din(add_18_i_0_0));
du #(6, 1) du_18_1 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_1),.din(add_18_i_0_1));

assign add_18_o_0_0 = add_18_i_1_0 + add_18_i_1_1;

du #(4, 2) du_27_0 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_0),.din(add_27_i_0_0));
du #(4, 2) du_27_1 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_1),.din(add_27_i_0_1));

assign add_27_o_0_0 = add_27_i_2_0 + add_27_i_2_1;

du #(4, 2) du_28_0 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_0),.din(add_28_i_0_0));
du #(4, 2) du_28_1 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_1),.din(add_28_i_0_1));

assign add_28_o_0_0 = add_28_i_2_0 + add_28_i_2_1;

du #(5, 1) du_26_0 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_0),.din(add_26_i_0_0));
du #(5, 1) du_26_1 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_1),.din(add_26_i_0_1));

assign add_26_o_0_0 = add_26_i_1_0 + add_26_i_1_1;

du #(4, 2) du_30_0 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_0),.din(add_30_i_0_0));
du #(4, 2) du_30_1 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_1),.din(add_30_i_0_1));

assign add_30_o_0_0 = add_30_i_2_0 + add_30_i_2_1;

du #(4, 2) du_31_0 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_0),.din(add_31_i_0_0));
du #(4, 2) du_31_1 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_1),.din(add_31_i_0_1));

assign add_31_o_0_0 = add_31_i_2_0 + add_31_i_2_1;

du #(5, 1) du_29_0 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_0),.din(add_29_i_0_0));
du #(5, 1) du_29_1 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_1),.din(add_29_i_0_1));

assign add_29_o_0_0 = add_29_i_1_0 + add_29_i_1_1;

du #(6, 1) du_25_0 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_0),.din(add_25_i_0_0));
du #(6, 1) du_25_1 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_1),.din(add_25_i_0_1));

assign add_25_o_0_0 = add_25_i_1_0 + add_25_i_1_1;

du #(7, 1) du_17_0 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_0),.din(add_17_i_0_0));
du #(7, 1) du_17_1 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_1),.din(add_17_i_0_1));

assign add_17_o_0_0 = add_17_i_1_0 + add_17_i_1_1;

du #(8, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(8, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_36_0 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_0),.din(add_36_i_0_0));
du #(4, 2) du_36_1 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_1),.din(add_36_i_0_1));

assign add_36_o_0_0 = add_36_i_2_0 + add_36_i_2_1;

du #(4, 2) du_37_0 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_0),.din(add_37_i_0_0));
du #(4, 2) du_37_1 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_1),.din(add_37_i_0_1));

assign add_37_o_0_0 = add_37_i_2_0 + add_37_i_2_1;

du #(5, 1) du_35_0 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_0),.din(add_35_i_0_0));
du #(5, 1) du_35_1 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_1),.din(add_35_i_0_1));

assign add_35_o_0_0 = add_35_i_1_0 + add_35_i_1_1;

du #(4, 2) du_39_0 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_0),.din(add_39_i_0_0));
du #(4, 2) du_39_1 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_1),.din(add_39_i_0_1));

assign add_39_o_0_0 = add_39_i_2_0 + add_39_i_2_1;

du #(4, 2) du_40_0 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_0),.din(add_40_i_0_0));
du #(4, 2) du_40_1 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_1),.din(add_40_i_0_1));

assign add_40_o_0_0 = add_40_i_2_0 + add_40_i_2_1;

du #(5, 1) du_38_0 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_0),.din(add_38_i_0_0));
du #(5, 1) du_38_1 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_1),.din(add_38_i_0_1));

assign add_38_o_0_0 = add_38_i_1_0 + add_38_i_1_1;

du #(6, 1) du_34_0 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_0),.din(add_34_i_0_0));
du #(6, 1) du_34_1 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_1),.din(add_34_i_0_1));

assign add_34_o_0_0 = add_34_i_1_0 + add_34_i_1_1;

du #(4, 2) du_43_0 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_0),.din(add_43_i_0_0));
du #(4, 2) du_43_1 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_1),.din(add_43_i_0_1));

assign add_43_o_0_0 = add_43_i_2_0 + add_43_i_2_1;

du #(4, 2) du_44_0 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_0),.din(add_44_i_0_0));
du #(4, 2) du_44_1 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_1),.din(add_44_i_0_1));

assign add_44_o_0_0 = add_44_i_2_0 + add_44_i_2_1;

du #(5, 1) du_42_0 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_0),.din(add_42_i_0_0));
du #(5, 1) du_42_1 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_1),.din(add_42_i_0_1));

assign add_42_o_0_0 = add_42_i_1_0 + add_42_i_1_1;

du #(4, 2) du_46_0 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_0),.din(add_46_i_0_0));
du #(4, 2) du_46_1 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_1),.din(add_46_i_0_1));

assign add_46_o_0_0 = add_46_i_2_0 + add_46_i_2_1;

du #(4, 2) du_47_0 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_0),.din(add_47_i_0_0));
du #(4, 2) du_47_1 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_1),.din(add_47_i_0_1));

assign add_47_o_0_0 = add_47_i_2_0 + add_47_i_2_1;

du #(5, 1) du_45_0 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_0),.din(add_45_i_0_0));
du #(5, 1) du_45_1 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_1),.din(add_45_i_0_1));

assign add_45_o_0_0 = add_45_i_1_0 + add_45_i_1_1;

du #(6, 1) du_41_0 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_0),.din(add_41_i_0_0));
du #(6, 1) du_41_1 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_1),.din(add_41_i_0_1));

assign add_41_o_0_0 = add_41_i_1_0 + add_41_i_1_1;

du #(7, 1) du_33_0 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_0),.din(add_33_i_0_0));
du #(7, 1) du_33_1 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_1),.din(add_33_i_0_1));

assign add_33_o_0_0 = add_33_i_1_0 + add_33_i_1_1;

du #(4, 2) du_51_0 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_0),.din(add_51_i_0_0));
du #(4, 2) du_51_1 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_1),.din(add_51_i_0_1));

assign add_51_o_0_0 = add_51_i_2_0 + add_51_i_2_1;

du #(4, 2) du_52_0 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_0),.din(add_52_i_0_0));
du #(4, 2) du_52_1 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_1),.din(add_52_i_0_1));

assign add_52_o_0_0 = add_52_i_2_0 + add_52_i_2_1;

du #(5, 1) du_50_0 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_0),.din(add_50_i_0_0));
du #(5, 1) du_50_1 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_1),.din(add_50_i_0_1));

assign add_50_o_0_0 = add_50_i_1_0 + add_50_i_1_1;

du #(4, 2) du_54_0 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_0),.din(add_54_i_0_0));
du #(4, 2) du_54_1 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_1),.din(add_54_i_0_1));

assign add_54_o_0_0 = add_54_i_2_0 + add_54_i_2_1;

du #(4, 2) du_55_0 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_0),.din(add_55_i_0_0));
du #(4, 2) du_55_1 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_1),.din(add_55_i_0_1));

assign add_55_o_0_0 = add_55_i_2_0 + add_55_i_2_1;

du #(5, 1) du_53_0 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_0),.din(add_53_i_0_0));
du #(5, 1) du_53_1 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_1),.din(add_53_i_0_1));

assign add_53_o_0_0 = add_53_i_1_0 + add_53_i_1_1;

du #(6, 1) du_49_0 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_0),.din(add_49_i_0_0));
du #(6, 1) du_49_1 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_1),.din(add_49_i_0_1));

assign add_49_o_0_0 = add_49_i_1_0 + add_49_i_1_1;

du #(4, 2) du_58_0 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_0),.din(add_58_i_0_0));
du #(4, 2) du_58_1 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_1),.din(add_58_i_0_1));

assign add_58_o_0_0 = add_58_i_2_0 + add_58_i_2_1;

du #(4, 2) du_59_0 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_0),.din(add_59_i_0_0));
du #(4, 2) du_59_1 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_1),.din(add_59_i_0_1));

assign add_59_o_0_0 = add_59_i_2_0 + add_59_i_2_1;

du #(5, 1) du_57_0 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_0),.din(add_57_i_0_0));
du #(5, 1) du_57_1 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_1),.din(add_57_i_0_1));

assign add_57_o_0_0 = add_57_i_1_0 + add_57_i_1_1;

du #(4, 2) du_61_0 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_0),.din(add_61_i_0_0));
du #(4, 2) du_61_1 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_1),.din(add_61_i_0_1));

assign add_61_o_0_0 = add_61_i_2_0 + add_61_i_2_1;

du #(4, 2) du_62_0 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_0),.din(add_62_i_0_0));
du #(4, 2) du_62_1 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_1),.din(add_62_i_0_1));

assign add_62_o_0_0 = add_62_i_2_0 + add_62_i_2_1;

du #(5, 1) du_60_0 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_0),.din(add_60_i_0_0));
du #(5, 1) du_60_1 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_1),.din(add_60_i_0_1));

assign add_60_o_0_0 = add_60_i_1_0 + add_60_i_1_1;

du #(6, 1) du_56_0 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_0),.din(add_56_i_0_0));
du #(6, 1) du_56_1 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_1),.din(add_56_i_0_1));

assign add_56_o_0_0 = add_56_i_1_0 + add_56_i_1_1;

du #(7, 1) du_48_0 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_0),.din(add_48_i_0_0));
du #(7, 1) du_48_1 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_1),.din(add_48_i_0_1));

assign add_48_o_0_0 = add_48_i_1_0 + add_48_i_1_1;

du #(8, 1) du_32_0 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_0),.din(add_32_i_0_0));
du #(8, 1) du_32_1 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_1),.din(add_32_i_0_1));

assign add_32_o_0_0 = add_32_i_1_0 + add_32_i_1_1;

du #(9, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(9, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(10, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_bram_nodsp_512_9 (
input clk,
input	[511:0] input_v,
output	[9:0] output_v
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
wire [7:0] input_16;
wire [7:0] input_17;
wire [7:0] input_18;
wire [7:0] input_19;
wire [7:0] input_20;
wire [7:0] input_21;
wire [7:0] input_22;
wire [7:0] input_23;
wire [7:0] input_24;
wire [7:0] input_25;
wire [7:0] input_26;
wire [7:0] input_27;
wire [7:0] input_28;
wire [7:0] input_29;
wire [7:0] input_30;
wire [7:0] input_31;
wire [7:0] input_32;
wire [7:0] input_33;
wire [7:0] input_34;
wire [7:0] input_35;
wire [7:0] input_36;
wire [7:0] input_37;
wire [7:0] input_38;
wire [7:0] input_39;
wire [7:0] input_40;
wire [7:0] input_41;
wire [7:0] input_42;
wire [7:0] input_43;
wire [7:0] input_44;
wire [7:0] input_45;
wire [7:0] input_46;
wire [7:0] input_47;
wire [7:0] input_48;
wire [7:0] input_49;
wire [7:0] input_50;
wire [7:0] input_51;
wire [7:0] input_52;
wire [7:0] input_53;
wire [7:0] input_54;
wire [7:0] input_55;
wire [7:0] input_56;
wire [7:0] input_57;
wire [7:0] input_58;
wire [7:0] input_59;
wire [7:0] input_60;
wire [7:0] input_61;
wire [7:0] input_62;
wire [7:0] input_63;
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
wire [7:0] lut8_BRAM_i_16;
wire [7:0] lut8_BRAM_i_17;
wire [7:0] lut8_BRAM_i_18;
wire [7:0] lut8_BRAM_i_19;
wire [7:0] lut8_BRAM_i_20;
wire [7:0] lut8_BRAM_i_21;
wire [7:0] lut8_BRAM_i_22;
wire [7:0] lut8_BRAM_i_23;
wire [7:0] lut8_BRAM_i_24;
wire [7:0] lut8_BRAM_i_25;
wire [7:0] lut8_BRAM_i_26;
wire [7:0] lut8_BRAM_i_27;
wire [7:0] lut8_BRAM_i_28;
wire [7:0] lut8_BRAM_i_29;
wire [7:0] lut8_BRAM_i_30;
wire [7:0] lut8_BRAM_i_31;
wire [7:0] lut8_BRAM_i_32;
wire [7:0] lut8_BRAM_i_33;
wire [7:0] lut8_BRAM_i_34;
wire [7:0] lut8_BRAM_i_35;
wire [7:0] lut8_BRAM_i_36;
wire [7:0] lut8_BRAM_i_37;
wire [7:0] lut8_BRAM_i_38;
wire [7:0] lut8_BRAM_i_39;
wire [7:0] lut8_BRAM_i_40;
wire [7:0] lut8_BRAM_i_41;
wire [7:0] lut8_BRAM_i_42;
wire [7:0] lut8_BRAM_i_43;
wire [7:0] lut8_BRAM_i_44;
wire [7:0] lut8_BRAM_i_45;
wire [7:0] lut8_BRAM_i_46;
wire [7:0] lut8_BRAM_i_47;
wire [7:0] lut8_BRAM_i_48;
wire [7:0] lut8_BRAM_i_49;
wire [7:0] lut8_BRAM_i_50;
wire [7:0] lut8_BRAM_i_51;
wire [7:0] lut8_BRAM_i_52;
wire [7:0] lut8_BRAM_i_53;
wire [7:0] lut8_BRAM_i_54;
wire [7:0] lut8_BRAM_i_55;
wire [7:0] lut8_BRAM_i_56;
wire [7:0] lut8_BRAM_i_57;
wire [7:0] lut8_BRAM_i_58;
wire [7:0] lut8_BRAM_i_59;
wire [7:0] lut8_BRAM_i_60;
wire [7:0] lut8_BRAM_i_61;
wire [7:0] lut8_BRAM_i_62;
wire [7:0] lut8_BRAM_i_63;
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
wire [3:0] lut8_BRAM_o_16;
wire [3:0] lut8_BRAM_o_17;
wire [3:0] lut8_BRAM_o_18;
wire [3:0] lut8_BRAM_o_19;
wire [3:0] lut8_BRAM_o_20;
wire [3:0] lut8_BRAM_o_21;
wire [3:0] lut8_BRAM_o_22;
wire [3:0] lut8_BRAM_o_23;
wire [3:0] lut8_BRAM_o_24;
wire [3:0] lut8_BRAM_o_25;
wire [3:0] lut8_BRAM_o_26;
wire [3:0] lut8_BRAM_o_27;
wire [3:0] lut8_BRAM_o_28;
wire [3:0] lut8_BRAM_o_29;
wire [3:0] lut8_BRAM_o_30;
wire [3:0] lut8_BRAM_o_31;
wire [3:0] lut8_BRAM_o_32;
wire [3:0] lut8_BRAM_o_33;
wire [3:0] lut8_BRAM_o_34;
wire [3:0] lut8_BRAM_o_35;
wire [3:0] lut8_BRAM_o_36;
wire [3:0] lut8_BRAM_o_37;
wire [3:0] lut8_BRAM_o_38;
wire [3:0] lut8_BRAM_o_39;
wire [3:0] lut8_BRAM_o_40;
wire [3:0] lut8_BRAM_o_41;
wire [3:0] lut8_BRAM_o_42;
wire [3:0] lut8_BRAM_o_43;
wire [3:0] lut8_BRAM_o_44;
wire [3:0] lut8_BRAM_o_45;
wire [3:0] lut8_BRAM_o_46;
wire [3:0] lut8_BRAM_o_47;
wire [3:0] lut8_BRAM_o_48;
wire [3:0] lut8_BRAM_o_49;
wire [3:0] lut8_BRAM_o_50;
wire [3:0] lut8_BRAM_o_51;
wire [3:0] lut8_BRAM_o_52;
wire [3:0] lut8_BRAM_o_53;
wire [3:0] lut8_BRAM_o_54;
wire [3:0] lut8_BRAM_o_55;
wire [3:0] lut8_BRAM_o_56;
wire [3:0] lut8_BRAM_o_57;
wire [3:0] lut8_BRAM_o_58;
wire [3:0] lut8_BRAM_o_59;
wire [3:0] lut8_BRAM_o_60;
wire [3:0] lut8_BRAM_o_61;
wire [3:0] lut8_BRAM_o_62;
wire [3:0] lut8_BRAM_o_63;
assign input_0[7:0] = input_v[511:504];
assign input_1[7:0] = input_v[503:496];
assign input_2[7:0] = input_v[495:488];
assign input_3[7:0] = input_v[487:480];
assign input_4[7:0] = input_v[479:472];
assign input_5[7:0] = input_v[471:464];
assign input_6[7:0] = input_v[463:456];
assign input_7[7:0] = input_v[455:448];
assign input_8[7:0] = input_v[447:440];
assign input_9[7:0] = input_v[439:432];
assign input_10[7:0] = input_v[431:424];
assign input_11[7:0] = input_v[423:416];
assign input_12[7:0] = input_v[415:408];
assign input_13[7:0] = input_v[407:400];
assign input_14[7:0] = input_v[399:392];
assign input_15[7:0] = input_v[391:384];
assign input_16[7:0] = input_v[383:376];
assign input_17[7:0] = input_v[375:368];
assign input_18[7:0] = input_v[367:360];
assign input_19[7:0] = input_v[359:352];
assign input_20[7:0] = input_v[351:344];
assign input_21[7:0] = input_v[343:336];
assign input_22[7:0] = input_v[335:328];
assign input_23[7:0] = input_v[327:320];
assign input_24[7:0] = input_v[319:312];
assign input_25[7:0] = input_v[311:304];
assign input_26[7:0] = input_v[303:296];
assign input_27[7:0] = input_v[295:288];
assign input_28[7:0] = input_v[287:280];
assign input_29[7:0] = input_v[279:272];
assign input_30[7:0] = input_v[271:264];
assign input_31[7:0] = input_v[263:256];
assign input_32[7:0] = input_v[255:248];
assign input_33[7:0] = input_v[247:240];
assign input_34[7:0] = input_v[239:232];
assign input_35[7:0] = input_v[231:224];
assign input_36[7:0] = input_v[223:216];
assign input_37[7:0] = input_v[215:208];
assign input_38[7:0] = input_v[207:200];
assign input_39[7:0] = input_v[199:192];
assign input_40[7:0] = input_v[191:184];
assign input_41[7:0] = input_v[183:176];
assign input_42[7:0] = input_v[175:168];
assign input_43[7:0] = input_v[167:160];
assign input_44[7:0] = input_v[159:152];
assign input_45[7:0] = input_v[151:144];
assign input_46[7:0] = input_v[143:136];
assign input_47[7:0] = input_v[135:128];
assign input_48[7:0] = input_v[127:120];
assign input_49[7:0] = input_v[119:112];
assign input_50[7:0] = input_v[111:104];
assign input_51[7:0] = input_v[103:96];
assign input_52[7:0] = input_v[95:88];
assign input_53[7:0] = input_v[87:80];
assign input_54[7:0] = input_v[79:72];
assign input_55[7:0] = input_v[71:64];
assign input_56[7:0] = input_v[63:56];
assign input_57[7:0] = input_v[55:48];
assign input_58[7:0] = input_v[47:40];
assign input_59[7:0] = input_v[39:32];
assign input_60[7:0] = input_v[31:24];
assign input_61[7:0] = input_v[23:16];
assign input_62[7:0] = input_v[15:8];
assign input_63[7:0] = input_v[7:0];
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
assign lut8_BRAM_i_16[7:0] = input_16[7:0];
assign lut8_BRAM_i_17[7:0] = input_17[7:0];
assign lut8_BRAM_i_18[7:0] = input_18[7:0];
assign lut8_BRAM_i_19[7:0] = input_19[7:0];
assign lut8_BRAM_i_20[7:0] = input_20[7:0];
assign lut8_BRAM_i_21[7:0] = input_21[7:0];
assign lut8_BRAM_i_22[7:0] = input_22[7:0];
assign lut8_BRAM_i_23[7:0] = input_23[7:0];
assign lut8_BRAM_i_24[7:0] = input_24[7:0];
assign lut8_BRAM_i_25[7:0] = input_25[7:0];
assign lut8_BRAM_i_26[7:0] = input_26[7:0];
assign lut8_BRAM_i_27[7:0] = input_27[7:0];
assign lut8_BRAM_i_28[7:0] = input_28[7:0];
assign lut8_BRAM_i_29[7:0] = input_29[7:0];
assign lut8_BRAM_i_30[7:0] = input_30[7:0];
assign lut8_BRAM_i_31[7:0] = input_31[7:0];
assign lut8_BRAM_i_32[7:0] = input_32[7:0];
assign lut8_BRAM_i_33[7:0] = input_33[7:0];
assign lut8_BRAM_i_34[7:0] = input_34[7:0];
assign lut8_BRAM_i_35[7:0] = input_35[7:0];
assign lut8_BRAM_i_36[7:0] = input_36[7:0];
assign lut8_BRAM_i_37[7:0] = input_37[7:0];
assign lut8_BRAM_i_38[7:0] = input_38[7:0];
assign lut8_BRAM_i_39[7:0] = input_39[7:0];
assign lut8_BRAM_i_40[7:0] = input_40[7:0];
assign lut8_BRAM_i_41[7:0] = input_41[7:0];
assign lut8_BRAM_i_42[7:0] = input_42[7:0];
assign lut8_BRAM_i_43[7:0] = input_43[7:0];
assign lut8_BRAM_i_44[7:0] = input_44[7:0];
assign lut8_BRAM_i_45[7:0] = input_45[7:0];
assign lut8_BRAM_i_46[7:0] = input_46[7:0];
assign lut8_BRAM_i_47[7:0] = input_47[7:0];
assign lut8_BRAM_i_48[7:0] = input_48[7:0];
assign lut8_BRAM_i_49[7:0] = input_49[7:0];
assign lut8_BRAM_i_50[7:0] = input_50[7:0];
assign lut8_BRAM_i_51[7:0] = input_51[7:0];
assign lut8_BRAM_i_52[7:0] = input_52[7:0];
assign lut8_BRAM_i_53[7:0] = input_53[7:0];
assign lut8_BRAM_i_54[7:0] = input_54[7:0];
assign lut8_BRAM_i_55[7:0] = input_55[7:0];
assign lut8_BRAM_i_56[7:0] = input_56[7:0];
assign lut8_BRAM_i_57[7:0] = input_57[7:0];
assign lut8_BRAM_i_58[7:0] = input_58[7:0];
assign lut8_BRAM_i_59[7:0] = input_59[7:0];
assign lut8_BRAM_i_60[7:0] = input_60[7:0];
assign lut8_BRAM_i_61[7:0] = input_61[7:0];
assign lut8_BRAM_i_62[7:0] = input_62[7:0];
assign lut8_BRAM_i_63[7:0] = input_63[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
lut8_BRAM lut8_BRAM_1 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_2),.lut8_BRAM_i_1(lut8_BRAM_i_3),.lut8_BRAM_o_0(lut8_BRAM_o_2),.lut8_BRAM_o_1(lut8_BRAM_o_3) );
lut8_BRAM lut8_BRAM_2 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_4),.lut8_BRAM_i_1(lut8_BRAM_i_5),.lut8_BRAM_o_0(lut8_BRAM_o_4),.lut8_BRAM_o_1(lut8_BRAM_o_5) );
lut8_BRAM lut8_BRAM_3 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_6),.lut8_BRAM_i_1(lut8_BRAM_i_7),.lut8_BRAM_o_0(lut8_BRAM_o_6),.lut8_BRAM_o_1(lut8_BRAM_o_7) );
lut8_BRAM lut8_BRAM_4 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_8),.lut8_BRAM_i_1(lut8_BRAM_i_9),.lut8_BRAM_o_0(lut8_BRAM_o_8),.lut8_BRAM_o_1(lut8_BRAM_o_9) );
lut8_BRAM lut8_BRAM_5 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_10),.lut8_BRAM_i_1(lut8_BRAM_i_11),.lut8_BRAM_o_0(lut8_BRAM_o_10),.lut8_BRAM_o_1(lut8_BRAM_o_11) );
lut8_BRAM lut8_BRAM_6 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_12),.lut8_BRAM_i_1(lut8_BRAM_i_13),.lut8_BRAM_o_0(lut8_BRAM_o_12),.lut8_BRAM_o_1(lut8_BRAM_o_13) );
lut8_BRAM lut8_BRAM_7 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_14),.lut8_BRAM_i_1(lut8_BRAM_i_15),.lut8_BRAM_o_0(lut8_BRAM_o_14),.lut8_BRAM_o_1(lut8_BRAM_o_15) );
lut8_BRAM lut8_BRAM_8 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_16),.lut8_BRAM_i_1(lut8_BRAM_i_17),.lut8_BRAM_o_0(lut8_BRAM_o_16),.lut8_BRAM_o_1(lut8_BRAM_o_17) );
lut8_BRAM lut8_BRAM_9 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_18),.lut8_BRAM_i_1(lut8_BRAM_i_19),.lut8_BRAM_o_0(lut8_BRAM_o_18),.lut8_BRAM_o_1(lut8_BRAM_o_19) );
lut8_BRAM lut8_BRAM_10 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_20),.lut8_BRAM_i_1(lut8_BRAM_i_21),.lut8_BRAM_o_0(lut8_BRAM_o_20),.lut8_BRAM_o_1(lut8_BRAM_o_21) );
lut8_BRAM lut8_BRAM_11 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_22),.lut8_BRAM_i_1(lut8_BRAM_i_23),.lut8_BRAM_o_0(lut8_BRAM_o_22),.lut8_BRAM_o_1(lut8_BRAM_o_23) );
lut8_BRAM lut8_BRAM_12 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_24),.lut8_BRAM_i_1(lut8_BRAM_i_25),.lut8_BRAM_o_0(lut8_BRAM_o_24),.lut8_BRAM_o_1(lut8_BRAM_o_25) );
lut8_BRAM lut8_BRAM_13 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_26),.lut8_BRAM_i_1(lut8_BRAM_i_27),.lut8_BRAM_o_0(lut8_BRAM_o_26),.lut8_BRAM_o_1(lut8_BRAM_o_27) );
lut8_BRAM lut8_BRAM_14 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_28),.lut8_BRAM_i_1(lut8_BRAM_i_29),.lut8_BRAM_o_0(lut8_BRAM_o_28),.lut8_BRAM_o_1(lut8_BRAM_o_29) );
lut8_BRAM lut8_BRAM_15 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_30),.lut8_BRAM_i_1(lut8_BRAM_i_31),.lut8_BRAM_o_0(lut8_BRAM_o_30),.lut8_BRAM_o_1(lut8_BRAM_o_31) );
lut8_BRAM lut8_BRAM_16 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_32),.lut8_BRAM_i_1(lut8_BRAM_i_33),.lut8_BRAM_o_0(lut8_BRAM_o_32),.lut8_BRAM_o_1(lut8_BRAM_o_33) );
lut8_BRAM lut8_BRAM_17 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_34),.lut8_BRAM_i_1(lut8_BRAM_i_35),.lut8_BRAM_o_0(lut8_BRAM_o_34),.lut8_BRAM_o_1(lut8_BRAM_o_35) );
lut8_BRAM lut8_BRAM_18 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_36),.lut8_BRAM_i_1(lut8_BRAM_i_37),.lut8_BRAM_o_0(lut8_BRAM_o_36),.lut8_BRAM_o_1(lut8_BRAM_o_37) );
lut8_BRAM lut8_BRAM_19 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_38),.lut8_BRAM_i_1(lut8_BRAM_i_39),.lut8_BRAM_o_0(lut8_BRAM_o_38),.lut8_BRAM_o_1(lut8_BRAM_o_39) );
lut8_BRAM lut8_BRAM_20 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_40),.lut8_BRAM_i_1(lut8_BRAM_i_41),.lut8_BRAM_o_0(lut8_BRAM_o_40),.lut8_BRAM_o_1(lut8_BRAM_o_41) );
lut8_BRAM lut8_BRAM_21 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_42),.lut8_BRAM_i_1(lut8_BRAM_i_43),.lut8_BRAM_o_0(lut8_BRAM_o_42),.lut8_BRAM_o_1(lut8_BRAM_o_43) );
lut8_BRAM lut8_BRAM_22 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_44),.lut8_BRAM_i_1(lut8_BRAM_i_45),.lut8_BRAM_o_0(lut8_BRAM_o_44),.lut8_BRAM_o_1(lut8_BRAM_o_45) );
lut8_BRAM lut8_BRAM_23 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_46),.lut8_BRAM_i_1(lut8_BRAM_i_47),.lut8_BRAM_o_0(lut8_BRAM_o_46),.lut8_BRAM_o_1(lut8_BRAM_o_47) );
lut8_BRAM lut8_BRAM_24 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_48),.lut8_BRAM_i_1(lut8_BRAM_i_49),.lut8_BRAM_o_0(lut8_BRAM_o_48),.lut8_BRAM_o_1(lut8_BRAM_o_49) );
lut8_BRAM lut8_BRAM_25 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_50),.lut8_BRAM_i_1(lut8_BRAM_i_51),.lut8_BRAM_o_0(lut8_BRAM_o_50),.lut8_BRAM_o_1(lut8_BRAM_o_51) );
lut8_BRAM lut8_BRAM_26 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_52),.lut8_BRAM_i_1(lut8_BRAM_i_53),.lut8_BRAM_o_0(lut8_BRAM_o_52),.lut8_BRAM_o_1(lut8_BRAM_o_53) );
lut8_BRAM lut8_BRAM_27 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_54),.lut8_BRAM_i_1(lut8_BRAM_i_55),.lut8_BRAM_o_0(lut8_BRAM_o_54),.lut8_BRAM_o_1(lut8_BRAM_o_55) );
lut8_BRAM lut8_BRAM_28 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_56),.lut8_BRAM_i_1(lut8_BRAM_i_57),.lut8_BRAM_o_0(lut8_BRAM_o_56),.lut8_BRAM_o_1(lut8_BRAM_o_57) );
lut8_BRAM lut8_BRAM_29 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_58),.lut8_BRAM_i_1(lut8_BRAM_i_59),.lut8_BRAM_o_0(lut8_BRAM_o_58),.lut8_BRAM_o_1(lut8_BRAM_o_59) );
lut8_BRAM lut8_BRAM_30 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_60),.lut8_BRAM_i_1(lut8_BRAM_i_61),.lut8_BRAM_o_0(lut8_BRAM_o_60),.lut8_BRAM_o_1(lut8_BRAM_o_61) );
lut8_BRAM lut8_BRAM_31 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_62),.lut8_BRAM_i_1(lut8_BRAM_i_63),.lut8_BRAM_o_0(lut8_BRAM_o_62),.lut8_BRAM_o_1(lut8_BRAM_o_63) );
wire [3:0] add_5_i_0_0;
wire [3:0] add_5_i_0_1;
wire [3:0] add_5_i_2_0;
wire [3:0] add_5_i_2_1;
wire [4:0] add_5_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [4:0] add_4_i_0_0;
wire [4:0] add_4_i_0_1;
wire [4:0] add_4_i_1_0;
wire [4:0] add_4_i_1_1;
wire [5:0] add_4_o_0_0;
wire [3:0] add_8_i_0_0;
wire [3:0] add_8_i_0_1;
wire [3:0] add_8_i_2_0;
wire [3:0] add_8_i_2_1;
wire [4:0] add_8_o_0_0;
wire [3:0] add_9_i_0_0;
wire [3:0] add_9_i_0_1;
wire [3:0] add_9_i_2_0;
wire [3:0] add_9_i_2_1;
wire [4:0] add_9_o_0_0;
wire [4:0] add_7_i_0_0;
wire [4:0] add_7_i_0_1;
wire [4:0] add_7_i_1_0;
wire [4:0] add_7_i_1_1;
wire [5:0] add_7_o_0_0;
wire [5:0] add_3_i_0_0;
wire [5:0] add_3_i_0_1;
wire [5:0] add_3_i_1_0;
wire [5:0] add_3_i_1_1;
wire [6:0] add_3_o_0_0;
wire [3:0] add_12_i_0_0;
wire [3:0] add_12_i_0_1;
wire [3:0] add_12_i_2_0;
wire [3:0] add_12_i_2_1;
wire [4:0] add_12_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [4:0] add_11_i_0_0;
wire [4:0] add_11_i_0_1;
wire [4:0] add_11_i_1_0;
wire [4:0] add_11_i_1_1;
wire [5:0] add_11_o_0_0;
wire [3:0] add_15_i_0_0;
wire [3:0] add_15_i_0_1;
wire [3:0] add_15_i_2_0;
wire [3:0] add_15_i_2_1;
wire [4:0] add_15_o_0_0;
wire [3:0] add_16_i_0_0;
wire [3:0] add_16_i_0_1;
wire [3:0] add_16_i_2_0;
wire [3:0] add_16_i_2_1;
wire [4:0] add_16_o_0_0;
wire [4:0] add_14_i_0_0;
wire [4:0] add_14_i_0_1;
wire [4:0] add_14_i_1_0;
wire [4:0] add_14_i_1_1;
wire [5:0] add_14_o_0_0;
wire [5:0] add_10_i_0_0;
wire [5:0] add_10_i_0_1;
wire [5:0] add_10_i_1_0;
wire [5:0] add_10_i_1_1;
wire [6:0] add_10_o_0_0;
wire [6:0] add_2_i_0_0;
wire [6:0] add_2_i_0_1;
wire [6:0] add_2_i_1_0;
wire [6:0] add_2_i_1_1;
wire [7:0] add_2_o_0_0;
wire [3:0] add_20_i_0_0;
wire [3:0] add_20_i_0_1;
wire [3:0] add_20_i_2_0;
wire [3:0] add_20_i_2_1;
wire [4:0] add_20_o_0_0;
wire [3:0] add_21_i_0_0;
wire [3:0] add_21_i_0_1;
wire [3:0] add_21_i_2_0;
wire [3:0] add_21_i_2_1;
wire [4:0] add_21_o_0_0;
wire [4:0] add_19_i_0_0;
wire [4:0] add_19_i_0_1;
wire [4:0] add_19_i_1_0;
wire [4:0] add_19_i_1_1;
wire [5:0] add_19_o_0_0;
wire [3:0] add_23_i_0_0;
wire [3:0] add_23_i_0_1;
wire [3:0] add_23_i_2_0;
wire [3:0] add_23_i_2_1;
wire [4:0] add_23_o_0_0;
wire [3:0] add_24_i_0_0;
wire [3:0] add_24_i_0_1;
wire [3:0] add_24_i_2_0;
wire [3:0] add_24_i_2_1;
wire [4:0] add_24_o_0_0;
wire [4:0] add_22_i_0_0;
wire [4:0] add_22_i_0_1;
wire [4:0] add_22_i_1_0;
wire [4:0] add_22_i_1_1;
wire [5:0] add_22_o_0_0;
wire [5:0] add_18_i_0_0;
wire [5:0] add_18_i_0_1;
wire [5:0] add_18_i_1_0;
wire [5:0] add_18_i_1_1;
wire [6:0] add_18_o_0_0;
wire [3:0] add_27_i_0_0;
wire [3:0] add_27_i_0_1;
wire [3:0] add_27_i_2_0;
wire [3:0] add_27_i_2_1;
wire [4:0] add_27_o_0_0;
wire [3:0] add_28_i_0_0;
wire [3:0] add_28_i_0_1;
wire [3:0] add_28_i_2_0;
wire [3:0] add_28_i_2_1;
wire [4:0] add_28_o_0_0;
wire [4:0] add_26_i_0_0;
wire [4:0] add_26_i_0_1;
wire [4:0] add_26_i_1_0;
wire [4:0] add_26_i_1_1;
wire [5:0] add_26_o_0_0;
wire [3:0] add_30_i_0_0;
wire [3:0] add_30_i_0_1;
wire [3:0] add_30_i_2_0;
wire [3:0] add_30_i_2_1;
wire [4:0] add_30_o_0_0;
wire [3:0] add_31_i_0_0;
wire [3:0] add_31_i_0_1;
wire [3:0] add_31_i_2_0;
wire [3:0] add_31_i_2_1;
wire [4:0] add_31_o_0_0;
wire [4:0] add_29_i_0_0;
wire [4:0] add_29_i_0_1;
wire [4:0] add_29_i_1_0;
wire [4:0] add_29_i_1_1;
wire [5:0] add_29_o_0_0;
wire [5:0] add_25_i_0_0;
wire [5:0] add_25_i_0_1;
wire [5:0] add_25_i_1_0;
wire [5:0] add_25_i_1_1;
wire [6:0] add_25_o_0_0;
wire [6:0] add_17_i_0_0;
wire [6:0] add_17_i_0_1;
wire [6:0] add_17_i_1_0;
wire [6:0] add_17_i_1_1;
wire [7:0] add_17_o_0_0;
wire [7:0] add_1_i_0_0;
wire [7:0] add_1_i_0_1;
wire [7:0] add_1_i_1_0;
wire [7:0] add_1_i_1_1;
wire [8:0] add_1_o_0_0;
wire [3:0] add_36_i_0_0;
wire [3:0] add_36_i_0_1;
wire [3:0] add_36_i_2_0;
wire [3:0] add_36_i_2_1;
wire [4:0] add_36_o_0_0;
wire [3:0] add_37_i_0_0;
wire [3:0] add_37_i_0_1;
wire [3:0] add_37_i_2_0;
wire [3:0] add_37_i_2_1;
wire [4:0] add_37_o_0_0;
wire [4:0] add_35_i_0_0;
wire [4:0] add_35_i_0_1;
wire [4:0] add_35_i_1_0;
wire [4:0] add_35_i_1_1;
wire [5:0] add_35_o_0_0;
wire [3:0] add_39_i_0_0;
wire [3:0] add_39_i_0_1;
wire [3:0] add_39_i_2_0;
wire [3:0] add_39_i_2_1;
wire [4:0] add_39_o_0_0;
wire [3:0] add_40_i_0_0;
wire [3:0] add_40_i_0_1;
wire [3:0] add_40_i_2_0;
wire [3:0] add_40_i_2_1;
wire [4:0] add_40_o_0_0;
wire [4:0] add_38_i_0_0;
wire [4:0] add_38_i_0_1;
wire [4:0] add_38_i_1_0;
wire [4:0] add_38_i_1_1;
wire [5:0] add_38_o_0_0;
wire [5:0] add_34_i_0_0;
wire [5:0] add_34_i_0_1;
wire [5:0] add_34_i_1_0;
wire [5:0] add_34_i_1_1;
wire [6:0] add_34_o_0_0;
wire [3:0] add_43_i_0_0;
wire [3:0] add_43_i_0_1;
wire [3:0] add_43_i_2_0;
wire [3:0] add_43_i_2_1;
wire [4:0] add_43_o_0_0;
wire [3:0] add_44_i_0_0;
wire [3:0] add_44_i_0_1;
wire [3:0] add_44_i_2_0;
wire [3:0] add_44_i_2_1;
wire [4:0] add_44_o_0_0;
wire [4:0] add_42_i_0_0;
wire [4:0] add_42_i_0_1;
wire [4:0] add_42_i_1_0;
wire [4:0] add_42_i_1_1;
wire [5:0] add_42_o_0_0;
wire [3:0] add_46_i_0_0;
wire [3:0] add_46_i_0_1;
wire [3:0] add_46_i_2_0;
wire [3:0] add_46_i_2_1;
wire [4:0] add_46_o_0_0;
wire [3:0] add_47_i_0_0;
wire [3:0] add_47_i_0_1;
wire [3:0] add_47_i_2_0;
wire [3:0] add_47_i_2_1;
wire [4:0] add_47_o_0_0;
wire [4:0] add_45_i_0_0;
wire [4:0] add_45_i_0_1;
wire [4:0] add_45_i_1_0;
wire [4:0] add_45_i_1_1;
wire [5:0] add_45_o_0_0;
wire [5:0] add_41_i_0_0;
wire [5:0] add_41_i_0_1;
wire [5:0] add_41_i_1_0;
wire [5:0] add_41_i_1_1;
wire [6:0] add_41_o_0_0;
wire [6:0] add_33_i_0_0;
wire [6:0] add_33_i_0_1;
wire [6:0] add_33_i_1_0;
wire [6:0] add_33_i_1_1;
wire [7:0] add_33_o_0_0;
wire [3:0] add_51_i_0_0;
wire [3:0] add_51_i_0_1;
wire [3:0] add_51_i_2_0;
wire [3:0] add_51_i_2_1;
wire [4:0] add_51_o_0_0;
wire [3:0] add_52_i_0_0;
wire [3:0] add_52_i_0_1;
wire [3:0] add_52_i_2_0;
wire [3:0] add_52_i_2_1;
wire [4:0] add_52_o_0_0;
wire [4:0] add_50_i_0_0;
wire [4:0] add_50_i_0_1;
wire [4:0] add_50_i_1_0;
wire [4:0] add_50_i_1_1;
wire [5:0] add_50_o_0_0;
wire [3:0] add_54_i_0_0;
wire [3:0] add_54_i_0_1;
wire [3:0] add_54_i_2_0;
wire [3:0] add_54_i_2_1;
wire [4:0] add_54_o_0_0;
wire [3:0] add_55_i_0_0;
wire [3:0] add_55_i_0_1;
wire [3:0] add_55_i_2_0;
wire [3:0] add_55_i_2_1;
wire [4:0] add_55_o_0_0;
wire [4:0] add_53_i_0_0;
wire [4:0] add_53_i_0_1;
wire [4:0] add_53_i_1_0;
wire [4:0] add_53_i_1_1;
wire [5:0] add_53_o_0_0;
wire [5:0] add_49_i_0_0;
wire [5:0] add_49_i_0_1;
wire [5:0] add_49_i_1_0;
wire [5:0] add_49_i_1_1;
wire [6:0] add_49_o_0_0;
wire [3:0] add_58_i_0_0;
wire [3:0] add_58_i_0_1;
wire [3:0] add_58_i_2_0;
wire [3:0] add_58_i_2_1;
wire [4:0] add_58_o_0_0;
wire [3:0] add_59_i_0_0;
wire [3:0] add_59_i_0_1;
wire [3:0] add_59_i_2_0;
wire [3:0] add_59_i_2_1;
wire [4:0] add_59_o_0_0;
wire [4:0] add_57_i_0_0;
wire [4:0] add_57_i_0_1;
wire [4:0] add_57_i_1_0;
wire [4:0] add_57_i_1_1;
wire [5:0] add_57_o_0_0;
wire [3:0] add_61_i_0_0;
wire [3:0] add_61_i_0_1;
wire [3:0] add_61_i_2_0;
wire [3:0] add_61_i_2_1;
wire [4:0] add_61_o_0_0;
wire [3:0] add_62_i_0_0;
wire [3:0] add_62_i_0_1;
wire [3:0] add_62_i_2_0;
wire [3:0] add_62_i_2_1;
wire [4:0] add_62_o_0_0;
wire [4:0] add_60_i_0_0;
wire [4:0] add_60_i_0_1;
wire [4:0] add_60_i_1_0;
wire [4:0] add_60_i_1_1;
wire [5:0] add_60_o_0_0;
wire [5:0] add_56_i_0_0;
wire [5:0] add_56_i_0_1;
wire [5:0] add_56_i_1_0;
wire [5:0] add_56_i_1_1;
wire [6:0] add_56_o_0_0;
wire [6:0] add_48_i_0_0;
wire [6:0] add_48_i_0_1;
wire [6:0] add_48_i_1_0;
wire [6:0] add_48_i_1_1;
wire [7:0] add_48_o_0_0;
wire [7:0] add_32_i_0_0;
wire [7:0] add_32_i_0_1;
wire [7:0] add_32_i_1_0;
wire [7:0] add_32_i_1_1;
wire [8:0] add_32_o_0_0;
wire [8:0] add_0_i_0_0;
wire [8:0] add_0_i_0_1;
wire [8:0] add_0_i_1_0;
wire [8:0] add_0_i_1_1;
wire [9:0] add_0_o_0_0;
wire [9:0] add_0_o_1_0;
assign add_5_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_5_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
assign add_6_i_0_0[3:0] = lut8_BRAM_o_2[3:0];
assign add_6_i_0_1[3:0] = lut8_BRAM_o_3[3:0];
assign add_8_i_0_0[3:0] = lut8_BRAM_o_4[3:0];
assign add_8_i_0_1[3:0] = lut8_BRAM_o_5[3:0];
assign add_9_i_0_0[3:0] = lut8_BRAM_o_6[3:0];
assign add_9_i_0_1[3:0] = lut8_BRAM_o_7[3:0];
assign add_12_i_0_0[3:0] = lut8_BRAM_o_8[3:0];
assign add_12_i_0_1[3:0] = lut8_BRAM_o_9[3:0];
assign add_13_i_0_0[3:0] = lut8_BRAM_o_10[3:0];
assign add_13_i_0_1[3:0] = lut8_BRAM_o_11[3:0];
assign add_15_i_0_0[3:0] = lut8_BRAM_o_12[3:0];
assign add_15_i_0_1[3:0] = lut8_BRAM_o_13[3:0];
assign add_16_i_0_0[3:0] = lut8_BRAM_o_14[3:0];
assign add_16_i_0_1[3:0] = lut8_BRAM_o_15[3:0];
assign add_20_i_0_0[3:0] = lut8_BRAM_o_16[3:0];
assign add_20_i_0_1[3:0] = lut8_BRAM_o_17[3:0];
assign add_21_i_0_0[3:0] = lut8_BRAM_o_18[3:0];
assign add_21_i_0_1[3:0] = lut8_BRAM_o_19[3:0];
assign add_23_i_0_0[3:0] = lut8_BRAM_o_20[3:0];
assign add_23_i_0_1[3:0] = lut8_BRAM_o_21[3:0];
assign add_24_i_0_0[3:0] = lut8_BRAM_o_22[3:0];
assign add_24_i_0_1[3:0] = lut8_BRAM_o_23[3:0];
assign add_27_i_0_0[3:0] = lut8_BRAM_o_24[3:0];
assign add_27_i_0_1[3:0] = lut8_BRAM_o_25[3:0];
assign add_28_i_0_0[3:0] = lut8_BRAM_o_26[3:0];
assign add_28_i_0_1[3:0] = lut8_BRAM_o_27[3:0];
assign add_30_i_0_0[3:0] = lut8_BRAM_o_28[3:0];
assign add_30_i_0_1[3:0] = lut8_BRAM_o_29[3:0];
assign add_31_i_0_0[3:0] = lut8_BRAM_o_30[3:0];
assign add_31_i_0_1[3:0] = lut8_BRAM_o_31[3:0];
assign add_36_i_0_0[3:0] = lut8_BRAM_o_32[3:0];
assign add_36_i_0_1[3:0] = lut8_BRAM_o_33[3:0];
assign add_37_i_0_0[3:0] = lut8_BRAM_o_34[3:0];
assign add_37_i_0_1[3:0] = lut8_BRAM_o_35[3:0];
assign add_39_i_0_0[3:0] = lut8_BRAM_o_36[3:0];
assign add_39_i_0_1[3:0] = lut8_BRAM_o_37[3:0];
assign add_40_i_0_0[3:0] = lut8_BRAM_o_38[3:0];
assign add_40_i_0_1[3:0] = lut8_BRAM_o_39[3:0];
assign add_43_i_0_0[3:0] = lut8_BRAM_o_40[3:0];
assign add_43_i_0_1[3:0] = lut8_BRAM_o_41[3:0];
assign add_44_i_0_0[3:0] = lut8_BRAM_o_42[3:0];
assign add_44_i_0_1[3:0] = lut8_BRAM_o_43[3:0];
assign add_46_i_0_0[3:0] = lut8_BRAM_o_44[3:0];
assign add_46_i_0_1[3:0] = lut8_BRAM_o_45[3:0];
assign add_47_i_0_0[3:0] = lut8_BRAM_o_46[3:0];
assign add_47_i_0_1[3:0] = lut8_BRAM_o_47[3:0];
assign add_51_i_0_0[3:0] = lut8_BRAM_o_48[3:0];
assign add_51_i_0_1[3:0] = lut8_BRAM_o_49[3:0];
assign add_52_i_0_0[3:0] = lut8_BRAM_o_50[3:0];
assign add_52_i_0_1[3:0] = lut8_BRAM_o_51[3:0];
assign add_54_i_0_0[3:0] = lut8_BRAM_o_52[3:0];
assign add_54_i_0_1[3:0] = lut8_BRAM_o_53[3:0];
assign add_55_i_0_0[3:0] = lut8_BRAM_o_54[3:0];
assign add_55_i_0_1[3:0] = lut8_BRAM_o_55[3:0];
assign add_58_i_0_0[3:0] = lut8_BRAM_o_56[3:0];
assign add_58_i_0_1[3:0] = lut8_BRAM_o_57[3:0];
assign add_59_i_0_0[3:0] = lut8_BRAM_o_58[3:0];
assign add_59_i_0_1[3:0] = lut8_BRAM_o_59[3:0];
assign add_61_i_0_0[3:0] = lut8_BRAM_o_60[3:0];
assign add_61_i_0_1[3:0] = lut8_BRAM_o_61[3:0];
assign add_62_i_0_0[3:0] = lut8_BRAM_o_62[3:0];
assign add_62_i_0_1[3:0] = lut8_BRAM_o_63[3:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_3_i_0_0[5:0] = add_4_o_0_0[5:0];
assign add_7_i_0_0[4:0] = add_8_o_0_0[4:0];
assign add_7_i_0_1[4:0] = add_9_o_0_0[4:0];
assign add_3_i_0_1[5:0] = add_7_o_0_0[5:0];
assign add_2_i_0_0[6:0] = add_3_o_0_0[6:0];
assign add_11_i_0_0[4:0] = add_12_o_0_0[4:0];
assign add_11_i_0_1[4:0] = add_13_o_0_0[4:0];
assign add_10_i_0_0[5:0] = add_11_o_0_0[5:0];
assign add_14_i_0_0[4:0] = add_15_o_0_0[4:0];
assign add_14_i_0_1[4:0] = add_16_o_0_0[4:0];
assign add_10_i_0_1[5:0] = add_14_o_0_0[5:0];
assign add_2_i_0_1[6:0] = add_10_o_0_0[6:0];
assign add_1_i_0_0[7:0] = add_2_o_0_0[7:0];
assign add_19_i_0_0[4:0] = add_20_o_0_0[4:0];
assign add_19_i_0_1[4:0] = add_21_o_0_0[4:0];
assign add_18_i_0_0[5:0] = add_19_o_0_0[5:0];
assign add_22_i_0_0[4:0] = add_23_o_0_0[4:0];
assign add_22_i_0_1[4:0] = add_24_o_0_0[4:0];
assign add_18_i_0_1[5:0] = add_22_o_0_0[5:0];
assign add_17_i_0_0[6:0] = add_18_o_0_0[6:0];
assign add_26_i_0_0[4:0] = add_27_o_0_0[4:0];
assign add_26_i_0_1[4:0] = add_28_o_0_0[4:0];
assign add_25_i_0_0[5:0] = add_26_o_0_0[5:0];
assign add_29_i_0_0[4:0] = add_30_o_0_0[4:0];
assign add_29_i_0_1[4:0] = add_31_o_0_0[4:0];
assign add_25_i_0_1[5:0] = add_29_o_0_0[5:0];
assign add_17_i_0_1[6:0] = add_25_o_0_0[6:0];
assign add_1_i_0_1[7:0] = add_17_o_0_0[7:0];
assign add_0_i_0_0[8:0] = add_1_o_0_0[8:0];
assign add_35_i_0_0[4:0] = add_36_o_0_0[4:0];
assign add_35_i_0_1[4:0] = add_37_o_0_0[4:0];
assign add_34_i_0_0[5:0] = add_35_o_0_0[5:0];
assign add_38_i_0_0[4:0] = add_39_o_0_0[4:0];
assign add_38_i_0_1[4:0] = add_40_o_0_0[4:0];
assign add_34_i_0_1[5:0] = add_38_o_0_0[5:0];
assign add_33_i_0_0[6:0] = add_34_o_0_0[6:0];
assign add_42_i_0_0[4:0] = add_43_o_0_0[4:0];
assign add_42_i_0_1[4:0] = add_44_o_0_0[4:0];
assign add_41_i_0_0[5:0] = add_42_o_0_0[5:0];
assign add_45_i_0_0[4:0] = add_46_o_0_0[4:0];
assign add_45_i_0_1[4:0] = add_47_o_0_0[4:0];
assign add_41_i_0_1[5:0] = add_45_o_0_0[5:0];
assign add_33_i_0_1[6:0] = add_41_o_0_0[6:0];
assign add_32_i_0_0[7:0] = add_33_o_0_0[7:0];
assign add_50_i_0_0[4:0] = add_51_o_0_0[4:0];
assign add_50_i_0_1[4:0] = add_52_o_0_0[4:0];
assign add_49_i_0_0[5:0] = add_50_o_0_0[5:0];
assign add_53_i_0_0[4:0] = add_54_o_0_0[4:0];
assign add_53_i_0_1[4:0] = add_55_o_0_0[4:0];
assign add_49_i_0_1[5:0] = add_53_o_0_0[5:0];
assign add_48_i_0_0[6:0] = add_49_o_0_0[6:0];
assign add_57_i_0_0[4:0] = add_58_o_0_0[4:0];
assign add_57_i_0_1[4:0] = add_59_o_0_0[4:0];
assign add_56_i_0_0[5:0] = add_57_o_0_0[5:0];
assign add_60_i_0_0[4:0] = add_61_o_0_0[4:0];
assign add_60_i_0_1[4:0] = add_62_o_0_0[4:0];
assign add_56_i_0_1[5:0] = add_60_o_0_0[5:0];
assign add_48_i_0_1[6:0] = add_56_o_0_0[6:0];
assign add_32_i_0_1[7:0] = add_48_o_0_0[7:0];
assign add_0_i_0_1[8:0] = add_32_o_0_0[8:0];
du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(4, 2) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_0),.din(add_8_i_0_0));
du #(4, 2) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_2_0 + add_8_i_2_1;

du #(4, 2) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_0),.din(add_9_i_0_0));
du #(4, 2) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_2_0 + add_9_i_2_1;

du #(5, 1) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_0),.din(add_7_i_0_0));
du #(5, 1) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_1_0 + add_7_i_1_1;

du #(6, 1) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_0),.din(add_3_i_0_0));
du #(6, 1) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_1_0 + add_3_i_1_1;

du #(4, 2) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_0),.din(add_12_i_0_0));
du #(4, 2) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_2_0 + add_12_i_2_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(5, 1) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_0),.din(add_11_i_0_0));
du #(5, 1) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_1_0 + add_11_i_1_1;

du #(4, 2) du_15_0 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_0),.din(add_15_i_0_0));
du #(4, 2) du_15_1 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_1),.din(add_15_i_0_1));

assign add_15_o_0_0 = add_15_i_2_0 + add_15_i_2_1;

du #(4, 2) du_16_0 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_0),.din(add_16_i_0_0));
du #(4, 2) du_16_1 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_1),.din(add_16_i_0_1));

assign add_16_o_0_0 = add_16_i_2_0 + add_16_i_2_1;

du #(5, 1) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_0),.din(add_14_i_0_0));
du #(5, 1) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_1_0 + add_14_i_1_1;

du #(6, 1) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_0),.din(add_10_i_0_0));
du #(6, 1) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_1_0 + add_10_i_1_1;

du #(7, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(7, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_20_0 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_0),.din(add_20_i_0_0));
du #(4, 2) du_20_1 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_1),.din(add_20_i_0_1));

assign add_20_o_0_0 = add_20_i_2_0 + add_20_i_2_1;

du #(4, 2) du_21_0 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_0),.din(add_21_i_0_0));
du #(4, 2) du_21_1 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_1),.din(add_21_i_0_1));

assign add_21_o_0_0 = add_21_i_2_0 + add_21_i_2_1;

du #(5, 1) du_19_0 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_0),.din(add_19_i_0_0));
du #(5, 1) du_19_1 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_1),.din(add_19_i_0_1));

assign add_19_o_0_0 = add_19_i_1_0 + add_19_i_1_1;

du #(4, 2) du_23_0 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_0),.din(add_23_i_0_0));
du #(4, 2) du_23_1 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_1),.din(add_23_i_0_1));

assign add_23_o_0_0 = add_23_i_2_0 + add_23_i_2_1;

du #(4, 2) du_24_0 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_0),.din(add_24_i_0_0));
du #(4, 2) du_24_1 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_1),.din(add_24_i_0_1));

assign add_24_o_0_0 = add_24_i_2_0 + add_24_i_2_1;

du #(5, 1) du_22_0 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_0),.din(add_22_i_0_0));
du #(5, 1) du_22_1 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_1),.din(add_22_i_0_1));

assign add_22_o_0_0 = add_22_i_1_0 + add_22_i_1_1;

du #(6, 1) du_18_0 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_0),.din(add_18_i_0_0));
du #(6, 1) du_18_1 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_1),.din(add_18_i_0_1));

assign add_18_o_0_0 = add_18_i_1_0 + add_18_i_1_1;

du #(4, 2) du_27_0 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_0),.din(add_27_i_0_0));
du #(4, 2) du_27_1 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_1),.din(add_27_i_0_1));

assign add_27_o_0_0 = add_27_i_2_0 + add_27_i_2_1;

du #(4, 2) du_28_0 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_0),.din(add_28_i_0_0));
du #(4, 2) du_28_1 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_1),.din(add_28_i_0_1));

assign add_28_o_0_0 = add_28_i_2_0 + add_28_i_2_1;

du #(5, 1) du_26_0 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_0),.din(add_26_i_0_0));
du #(5, 1) du_26_1 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_1),.din(add_26_i_0_1));

assign add_26_o_0_0 = add_26_i_1_0 + add_26_i_1_1;

du #(4, 2) du_30_0 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_0),.din(add_30_i_0_0));
du #(4, 2) du_30_1 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_1),.din(add_30_i_0_1));

assign add_30_o_0_0 = add_30_i_2_0 + add_30_i_2_1;

du #(4, 2) du_31_0 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_0),.din(add_31_i_0_0));
du #(4, 2) du_31_1 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_1),.din(add_31_i_0_1));

assign add_31_o_0_0 = add_31_i_2_0 + add_31_i_2_1;

du #(5, 1) du_29_0 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_0),.din(add_29_i_0_0));
du #(5, 1) du_29_1 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_1),.din(add_29_i_0_1));

assign add_29_o_0_0 = add_29_i_1_0 + add_29_i_1_1;

du #(6, 1) du_25_0 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_0),.din(add_25_i_0_0));
du #(6, 1) du_25_1 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_1),.din(add_25_i_0_1));

assign add_25_o_0_0 = add_25_i_1_0 + add_25_i_1_1;

du #(7, 1) du_17_0 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_0),.din(add_17_i_0_0));
du #(7, 1) du_17_1 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_1),.din(add_17_i_0_1));

assign add_17_o_0_0 = add_17_i_1_0 + add_17_i_1_1;

du #(8, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(8, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_36_0 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_0),.din(add_36_i_0_0));
du #(4, 2) du_36_1 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_1),.din(add_36_i_0_1));

assign add_36_o_0_0 = add_36_i_2_0 + add_36_i_2_1;

du #(4, 2) du_37_0 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_0),.din(add_37_i_0_0));
du #(4, 2) du_37_1 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_1),.din(add_37_i_0_1));

assign add_37_o_0_0 = add_37_i_2_0 + add_37_i_2_1;

du #(5, 1) du_35_0 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_0),.din(add_35_i_0_0));
du #(5, 1) du_35_1 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_1),.din(add_35_i_0_1));

assign add_35_o_0_0 = add_35_i_1_0 + add_35_i_1_1;

du #(4, 2) du_39_0 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_0),.din(add_39_i_0_0));
du #(4, 2) du_39_1 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_1),.din(add_39_i_0_1));

assign add_39_o_0_0 = add_39_i_2_0 + add_39_i_2_1;

du #(4, 2) du_40_0 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_0),.din(add_40_i_0_0));
du #(4, 2) du_40_1 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_1),.din(add_40_i_0_1));

assign add_40_o_0_0 = add_40_i_2_0 + add_40_i_2_1;

du #(5, 1) du_38_0 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_0),.din(add_38_i_0_0));
du #(5, 1) du_38_1 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_1),.din(add_38_i_0_1));

assign add_38_o_0_0 = add_38_i_1_0 + add_38_i_1_1;

du #(6, 1) du_34_0 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_0),.din(add_34_i_0_0));
du #(6, 1) du_34_1 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_1),.din(add_34_i_0_1));

assign add_34_o_0_0 = add_34_i_1_0 + add_34_i_1_1;

du #(4, 2) du_43_0 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_0),.din(add_43_i_0_0));
du #(4, 2) du_43_1 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_1),.din(add_43_i_0_1));

assign add_43_o_0_0 = add_43_i_2_0 + add_43_i_2_1;

du #(4, 2) du_44_0 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_0),.din(add_44_i_0_0));
du #(4, 2) du_44_1 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_1),.din(add_44_i_0_1));

assign add_44_o_0_0 = add_44_i_2_0 + add_44_i_2_1;

du #(5, 1) du_42_0 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_0),.din(add_42_i_0_0));
du #(5, 1) du_42_1 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_1),.din(add_42_i_0_1));

assign add_42_o_0_0 = add_42_i_1_0 + add_42_i_1_1;

du #(4, 2) du_46_0 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_0),.din(add_46_i_0_0));
du #(4, 2) du_46_1 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_1),.din(add_46_i_0_1));

assign add_46_o_0_0 = add_46_i_2_0 + add_46_i_2_1;

du #(4, 2) du_47_0 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_0),.din(add_47_i_0_0));
du #(4, 2) du_47_1 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_1),.din(add_47_i_0_1));

assign add_47_o_0_0 = add_47_i_2_0 + add_47_i_2_1;

du #(5, 1) du_45_0 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_0),.din(add_45_i_0_0));
du #(5, 1) du_45_1 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_1),.din(add_45_i_0_1));

assign add_45_o_0_0 = add_45_i_1_0 + add_45_i_1_1;

du #(6, 1) du_41_0 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_0),.din(add_41_i_0_0));
du #(6, 1) du_41_1 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_1),.din(add_41_i_0_1));

assign add_41_o_0_0 = add_41_i_1_0 + add_41_i_1_1;

du #(7, 1) du_33_0 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_0),.din(add_33_i_0_0));
du #(7, 1) du_33_1 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_1),.din(add_33_i_0_1));

assign add_33_o_0_0 = add_33_i_1_0 + add_33_i_1_1;

du #(4, 2) du_51_0 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_0),.din(add_51_i_0_0));
du #(4, 2) du_51_1 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_1),.din(add_51_i_0_1));

assign add_51_o_0_0 = add_51_i_2_0 + add_51_i_2_1;

du #(4, 2) du_52_0 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_0),.din(add_52_i_0_0));
du #(4, 2) du_52_1 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_1),.din(add_52_i_0_1));

assign add_52_o_0_0 = add_52_i_2_0 + add_52_i_2_1;

du #(5, 1) du_50_0 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_0),.din(add_50_i_0_0));
du #(5, 1) du_50_1 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_1),.din(add_50_i_0_1));

assign add_50_o_0_0 = add_50_i_1_0 + add_50_i_1_1;

du #(4, 2) du_54_0 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_0),.din(add_54_i_0_0));
du #(4, 2) du_54_1 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_1),.din(add_54_i_0_1));

assign add_54_o_0_0 = add_54_i_2_0 + add_54_i_2_1;

du #(4, 2) du_55_0 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_0),.din(add_55_i_0_0));
du #(4, 2) du_55_1 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_1),.din(add_55_i_0_1));

assign add_55_o_0_0 = add_55_i_2_0 + add_55_i_2_1;

du #(5, 1) du_53_0 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_0),.din(add_53_i_0_0));
du #(5, 1) du_53_1 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_1),.din(add_53_i_0_1));

assign add_53_o_0_0 = add_53_i_1_0 + add_53_i_1_1;

du #(6, 1) du_49_0 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_0),.din(add_49_i_0_0));
du #(6, 1) du_49_1 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_1),.din(add_49_i_0_1));

assign add_49_o_0_0 = add_49_i_1_0 + add_49_i_1_1;

du #(4, 2) du_58_0 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_0),.din(add_58_i_0_0));
du #(4, 2) du_58_1 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_1),.din(add_58_i_0_1));

assign add_58_o_0_0 = add_58_i_2_0 + add_58_i_2_1;

du #(4, 2) du_59_0 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_0),.din(add_59_i_0_0));
du #(4, 2) du_59_1 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_1),.din(add_59_i_0_1));

assign add_59_o_0_0 = add_59_i_2_0 + add_59_i_2_1;

du #(5, 1) du_57_0 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_0),.din(add_57_i_0_0));
du #(5, 1) du_57_1 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_1),.din(add_57_i_0_1));

assign add_57_o_0_0 = add_57_i_1_0 + add_57_i_1_1;

du #(4, 2) du_61_0 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_0),.din(add_61_i_0_0));
du #(4, 2) du_61_1 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_1),.din(add_61_i_0_1));

assign add_61_o_0_0 = add_61_i_2_0 + add_61_i_2_1;

du #(4, 2) du_62_0 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_0),.din(add_62_i_0_0));
du #(4, 2) du_62_1 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_1),.din(add_62_i_0_1));

assign add_62_o_0_0 = add_62_i_2_0 + add_62_i_2_1;

du #(5, 1) du_60_0 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_0),.din(add_60_i_0_0));
du #(5, 1) du_60_1 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_1),.din(add_60_i_0_1));

assign add_60_o_0_0 = add_60_i_1_0 + add_60_i_1_1;

du #(6, 1) du_56_0 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_0),.din(add_56_i_0_0));
du #(6, 1) du_56_1 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_1),.din(add_56_i_0_1));

assign add_56_o_0_0 = add_56_i_1_0 + add_56_i_1_1;

du #(7, 1) du_48_0 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_0),.din(add_48_i_0_0));
du #(7, 1) du_48_1 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_1),.din(add_48_i_0_1));

assign add_48_o_0_0 = add_48_i_1_0 + add_48_i_1_1;

du #(8, 1) du_32_0 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_0),.din(add_32_i_0_0));
du #(8, 1) du_32_1 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_1),.din(add_32_i_0_1));

assign add_32_o_0_0 = add_32_i_1_0 + add_32_i_1_1;

du #(9, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(9, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(10, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_dist_dsp_512_9 (
input clk,
input	[511:0] input_v,
output	[9:0] output_v
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
wire [7:0] input_16;
wire [7:0] input_17;
wire [7:0] input_18;
wire [7:0] input_19;
wire [7:0] input_20;
wire [7:0] input_21;
wire [7:0] input_22;
wire [7:0] input_23;
wire [7:0] input_24;
wire [7:0] input_25;
wire [7:0] input_26;
wire [7:0] input_27;
wire [7:0] input_28;
wire [7:0] input_29;
wire [7:0] input_30;
wire [7:0] input_31;
wire [7:0] input_32;
wire [7:0] input_33;
wire [7:0] input_34;
wire [7:0] input_35;
wire [7:0] input_36;
wire [7:0] input_37;
wire [7:0] input_38;
wire [7:0] input_39;
wire [7:0] input_40;
wire [7:0] input_41;
wire [7:0] input_42;
wire [7:0] input_43;
wire [7:0] input_44;
wire [7:0] input_45;
wire [7:0] input_46;
wire [7:0] input_47;
wire [7:0] input_48;
wire [7:0] input_49;
wire [7:0] input_50;
wire [7:0] input_51;
wire [7:0] input_52;
wire [7:0] input_53;
wire [7:0] input_54;
wire [7:0] input_55;
wire [7:0] input_56;
wire [7:0] input_57;
wire [7:0] input_58;
wire [7:0] input_59;
wire [7:0] input_60;
wire [7:0] input_61;
wire [7:0] input_62;
wire [7:0] input_63;
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
wire [7:0] lut8_DIST_i_16;
wire [7:0] lut8_DIST_i_17;
wire [7:0] lut8_DIST_i_18;
wire [7:0] lut8_DIST_i_19;
wire [7:0] lut8_DIST_i_20;
wire [7:0] lut8_DIST_i_21;
wire [7:0] lut8_DIST_i_22;
wire [7:0] lut8_DIST_i_23;
wire [7:0] lut8_DIST_i_24;
wire [7:0] lut8_DIST_i_25;
wire [7:0] lut8_DIST_i_26;
wire [7:0] lut8_DIST_i_27;
wire [7:0] lut8_DIST_i_28;
wire [7:0] lut8_DIST_i_29;
wire [7:0] lut8_DIST_i_30;
wire [7:0] lut8_DIST_i_31;
wire [7:0] lut8_DIST_i_32;
wire [7:0] lut8_DIST_i_33;
wire [7:0] lut8_DIST_i_34;
wire [7:0] lut8_DIST_i_35;
wire [7:0] lut8_DIST_i_36;
wire [7:0] lut8_DIST_i_37;
wire [7:0] lut8_DIST_i_38;
wire [7:0] lut8_DIST_i_39;
wire [7:0] lut8_DIST_i_40;
wire [7:0] lut8_DIST_i_41;
wire [7:0] lut8_DIST_i_42;
wire [7:0] lut8_DIST_i_43;
wire [7:0] lut8_DIST_i_44;
wire [7:0] lut8_DIST_i_45;
wire [7:0] lut8_DIST_i_46;
wire [7:0] lut8_DIST_i_47;
wire [7:0] lut8_DIST_i_48;
wire [7:0] lut8_DIST_i_49;
wire [7:0] lut8_DIST_i_50;
wire [7:0] lut8_DIST_i_51;
wire [7:0] lut8_DIST_i_52;
wire [7:0] lut8_DIST_i_53;
wire [7:0] lut8_DIST_i_54;
wire [7:0] lut8_DIST_i_55;
wire [7:0] lut8_DIST_i_56;
wire [7:0] lut8_DIST_i_57;
wire [7:0] lut8_DIST_i_58;
wire [7:0] lut8_DIST_i_59;
wire [7:0] lut8_DIST_i_60;
wire [7:0] lut8_DIST_i_61;
wire [7:0] lut8_DIST_i_62;
wire [7:0] lut8_DIST_i_63;
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
wire [3:0] lut8_DIST_o_16;
wire [3:0] lut8_DIST_o_17;
wire [3:0] lut8_DIST_o_18;
wire [3:0] lut8_DIST_o_19;
wire [3:0] lut8_DIST_o_20;
wire [3:0] lut8_DIST_o_21;
wire [3:0] lut8_DIST_o_22;
wire [3:0] lut8_DIST_o_23;
wire [3:0] lut8_DIST_o_24;
wire [3:0] lut8_DIST_o_25;
wire [3:0] lut8_DIST_o_26;
wire [3:0] lut8_DIST_o_27;
wire [3:0] lut8_DIST_o_28;
wire [3:0] lut8_DIST_o_29;
wire [3:0] lut8_DIST_o_30;
wire [3:0] lut8_DIST_o_31;
wire [3:0] lut8_DIST_o_32;
wire [3:0] lut8_DIST_o_33;
wire [3:0] lut8_DIST_o_34;
wire [3:0] lut8_DIST_o_35;
wire [3:0] lut8_DIST_o_36;
wire [3:0] lut8_DIST_o_37;
wire [3:0] lut8_DIST_o_38;
wire [3:0] lut8_DIST_o_39;
wire [3:0] lut8_DIST_o_40;
wire [3:0] lut8_DIST_o_41;
wire [3:0] lut8_DIST_o_42;
wire [3:0] lut8_DIST_o_43;
wire [3:0] lut8_DIST_o_44;
wire [3:0] lut8_DIST_o_45;
wire [3:0] lut8_DIST_o_46;
wire [3:0] lut8_DIST_o_47;
wire [3:0] lut8_DIST_o_48;
wire [3:0] lut8_DIST_o_49;
wire [3:0] lut8_DIST_o_50;
wire [3:0] lut8_DIST_o_51;
wire [3:0] lut8_DIST_o_52;
wire [3:0] lut8_DIST_o_53;
wire [3:0] lut8_DIST_o_54;
wire [3:0] lut8_DIST_o_55;
wire [3:0] lut8_DIST_o_56;
wire [3:0] lut8_DIST_o_57;
wire [3:0] lut8_DIST_o_58;
wire [3:0] lut8_DIST_o_59;
wire [3:0] lut8_DIST_o_60;
wire [3:0] lut8_DIST_o_61;
wire [3:0] lut8_DIST_o_62;
wire [3:0] lut8_DIST_o_63;
assign input_0[7:0] = input_v[511:504];
assign input_1[7:0] = input_v[503:496];
assign input_2[7:0] = input_v[495:488];
assign input_3[7:0] = input_v[487:480];
assign input_4[7:0] = input_v[479:472];
assign input_5[7:0] = input_v[471:464];
assign input_6[7:0] = input_v[463:456];
assign input_7[7:0] = input_v[455:448];
assign input_8[7:0] = input_v[447:440];
assign input_9[7:0] = input_v[439:432];
assign input_10[7:0] = input_v[431:424];
assign input_11[7:0] = input_v[423:416];
assign input_12[7:0] = input_v[415:408];
assign input_13[7:0] = input_v[407:400];
assign input_14[7:0] = input_v[399:392];
assign input_15[7:0] = input_v[391:384];
assign input_16[7:0] = input_v[383:376];
assign input_17[7:0] = input_v[375:368];
assign input_18[7:0] = input_v[367:360];
assign input_19[7:0] = input_v[359:352];
assign input_20[7:0] = input_v[351:344];
assign input_21[7:0] = input_v[343:336];
assign input_22[7:0] = input_v[335:328];
assign input_23[7:0] = input_v[327:320];
assign input_24[7:0] = input_v[319:312];
assign input_25[7:0] = input_v[311:304];
assign input_26[7:0] = input_v[303:296];
assign input_27[7:0] = input_v[295:288];
assign input_28[7:0] = input_v[287:280];
assign input_29[7:0] = input_v[279:272];
assign input_30[7:0] = input_v[271:264];
assign input_31[7:0] = input_v[263:256];
assign input_32[7:0] = input_v[255:248];
assign input_33[7:0] = input_v[247:240];
assign input_34[7:0] = input_v[239:232];
assign input_35[7:0] = input_v[231:224];
assign input_36[7:0] = input_v[223:216];
assign input_37[7:0] = input_v[215:208];
assign input_38[7:0] = input_v[207:200];
assign input_39[7:0] = input_v[199:192];
assign input_40[7:0] = input_v[191:184];
assign input_41[7:0] = input_v[183:176];
assign input_42[7:0] = input_v[175:168];
assign input_43[7:0] = input_v[167:160];
assign input_44[7:0] = input_v[159:152];
assign input_45[7:0] = input_v[151:144];
assign input_46[7:0] = input_v[143:136];
assign input_47[7:0] = input_v[135:128];
assign input_48[7:0] = input_v[127:120];
assign input_49[7:0] = input_v[119:112];
assign input_50[7:0] = input_v[111:104];
assign input_51[7:0] = input_v[103:96];
assign input_52[7:0] = input_v[95:88];
assign input_53[7:0] = input_v[87:80];
assign input_54[7:0] = input_v[79:72];
assign input_55[7:0] = input_v[71:64];
assign input_56[7:0] = input_v[63:56];
assign input_57[7:0] = input_v[55:48];
assign input_58[7:0] = input_v[47:40];
assign input_59[7:0] = input_v[39:32];
assign input_60[7:0] = input_v[31:24];
assign input_61[7:0] = input_v[23:16];
assign input_62[7:0] = input_v[15:8];
assign input_63[7:0] = input_v[7:0];
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
assign lut8_DIST_i_16[7:0] = input_16[7:0];
assign lut8_DIST_i_17[7:0] = input_17[7:0];
assign lut8_DIST_i_18[7:0] = input_18[7:0];
assign lut8_DIST_i_19[7:0] = input_19[7:0];
assign lut8_DIST_i_20[7:0] = input_20[7:0];
assign lut8_DIST_i_21[7:0] = input_21[7:0];
assign lut8_DIST_i_22[7:0] = input_22[7:0];
assign lut8_DIST_i_23[7:0] = input_23[7:0];
assign lut8_DIST_i_24[7:0] = input_24[7:0];
assign lut8_DIST_i_25[7:0] = input_25[7:0];
assign lut8_DIST_i_26[7:0] = input_26[7:0];
assign lut8_DIST_i_27[7:0] = input_27[7:0];
assign lut8_DIST_i_28[7:0] = input_28[7:0];
assign lut8_DIST_i_29[7:0] = input_29[7:0];
assign lut8_DIST_i_30[7:0] = input_30[7:0];
assign lut8_DIST_i_31[7:0] = input_31[7:0];
assign lut8_DIST_i_32[7:0] = input_32[7:0];
assign lut8_DIST_i_33[7:0] = input_33[7:0];
assign lut8_DIST_i_34[7:0] = input_34[7:0];
assign lut8_DIST_i_35[7:0] = input_35[7:0];
assign lut8_DIST_i_36[7:0] = input_36[7:0];
assign lut8_DIST_i_37[7:0] = input_37[7:0];
assign lut8_DIST_i_38[7:0] = input_38[7:0];
assign lut8_DIST_i_39[7:0] = input_39[7:0];
assign lut8_DIST_i_40[7:0] = input_40[7:0];
assign lut8_DIST_i_41[7:0] = input_41[7:0];
assign lut8_DIST_i_42[7:0] = input_42[7:0];
assign lut8_DIST_i_43[7:0] = input_43[7:0];
assign lut8_DIST_i_44[7:0] = input_44[7:0];
assign lut8_DIST_i_45[7:0] = input_45[7:0];
assign lut8_DIST_i_46[7:0] = input_46[7:0];
assign lut8_DIST_i_47[7:0] = input_47[7:0];
assign lut8_DIST_i_48[7:0] = input_48[7:0];
assign lut8_DIST_i_49[7:0] = input_49[7:0];
assign lut8_DIST_i_50[7:0] = input_50[7:0];
assign lut8_DIST_i_51[7:0] = input_51[7:0];
assign lut8_DIST_i_52[7:0] = input_52[7:0];
assign lut8_DIST_i_53[7:0] = input_53[7:0];
assign lut8_DIST_i_54[7:0] = input_54[7:0];
assign lut8_DIST_i_55[7:0] = input_55[7:0];
assign lut8_DIST_i_56[7:0] = input_56[7:0];
assign lut8_DIST_i_57[7:0] = input_57[7:0];
assign lut8_DIST_i_58[7:0] = input_58[7:0];
assign lut8_DIST_i_59[7:0] = input_59[7:0];
assign lut8_DIST_i_60[7:0] = input_60[7:0];
assign lut8_DIST_i_61[7:0] = input_61[7:0];
assign lut8_DIST_i_62[7:0] = input_62[7:0];
assign lut8_DIST_i_63[7:0] = input_63[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
lut8_DIST lut8_DIST_1 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_2),.lut8_DIST_i_1(lut8_DIST_i_3),.lut8_DIST_o_0(lut8_DIST_o_2),.lut8_DIST_o_1(lut8_DIST_o_3) );
lut8_DIST lut8_DIST_2 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_4),.lut8_DIST_i_1(lut8_DIST_i_5),.lut8_DIST_o_0(lut8_DIST_o_4),.lut8_DIST_o_1(lut8_DIST_o_5) );
lut8_DIST lut8_DIST_3 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_6),.lut8_DIST_i_1(lut8_DIST_i_7),.lut8_DIST_o_0(lut8_DIST_o_6),.lut8_DIST_o_1(lut8_DIST_o_7) );
lut8_DIST lut8_DIST_4 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_8),.lut8_DIST_i_1(lut8_DIST_i_9),.lut8_DIST_o_0(lut8_DIST_o_8),.lut8_DIST_o_1(lut8_DIST_o_9) );
lut8_DIST lut8_DIST_5 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_10),.lut8_DIST_i_1(lut8_DIST_i_11),.lut8_DIST_o_0(lut8_DIST_o_10),.lut8_DIST_o_1(lut8_DIST_o_11) );
lut8_DIST lut8_DIST_6 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_12),.lut8_DIST_i_1(lut8_DIST_i_13),.lut8_DIST_o_0(lut8_DIST_o_12),.lut8_DIST_o_1(lut8_DIST_o_13) );
lut8_DIST lut8_DIST_7 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_14),.lut8_DIST_i_1(lut8_DIST_i_15),.lut8_DIST_o_0(lut8_DIST_o_14),.lut8_DIST_o_1(lut8_DIST_o_15) );
lut8_DIST lut8_DIST_8 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_16),.lut8_DIST_i_1(lut8_DIST_i_17),.lut8_DIST_o_0(lut8_DIST_o_16),.lut8_DIST_o_1(lut8_DIST_o_17) );
lut8_DIST lut8_DIST_9 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_18),.lut8_DIST_i_1(lut8_DIST_i_19),.lut8_DIST_o_0(lut8_DIST_o_18),.lut8_DIST_o_1(lut8_DIST_o_19) );
lut8_DIST lut8_DIST_10 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_20),.lut8_DIST_i_1(lut8_DIST_i_21),.lut8_DIST_o_0(lut8_DIST_o_20),.lut8_DIST_o_1(lut8_DIST_o_21) );
lut8_DIST lut8_DIST_11 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_22),.lut8_DIST_i_1(lut8_DIST_i_23),.lut8_DIST_o_0(lut8_DIST_o_22),.lut8_DIST_o_1(lut8_DIST_o_23) );
lut8_DIST lut8_DIST_12 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_24),.lut8_DIST_i_1(lut8_DIST_i_25),.lut8_DIST_o_0(lut8_DIST_o_24),.lut8_DIST_o_1(lut8_DIST_o_25) );
lut8_DIST lut8_DIST_13 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_26),.lut8_DIST_i_1(lut8_DIST_i_27),.lut8_DIST_o_0(lut8_DIST_o_26),.lut8_DIST_o_1(lut8_DIST_o_27) );
lut8_DIST lut8_DIST_14 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_28),.lut8_DIST_i_1(lut8_DIST_i_29),.lut8_DIST_o_0(lut8_DIST_o_28),.lut8_DIST_o_1(lut8_DIST_o_29) );
lut8_DIST lut8_DIST_15 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_30),.lut8_DIST_i_1(lut8_DIST_i_31),.lut8_DIST_o_0(lut8_DIST_o_30),.lut8_DIST_o_1(lut8_DIST_o_31) );
lut8_DIST lut8_DIST_16 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_32),.lut8_DIST_i_1(lut8_DIST_i_33),.lut8_DIST_o_0(lut8_DIST_o_32),.lut8_DIST_o_1(lut8_DIST_o_33) );
lut8_DIST lut8_DIST_17 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_34),.lut8_DIST_i_1(lut8_DIST_i_35),.lut8_DIST_o_0(lut8_DIST_o_34),.lut8_DIST_o_1(lut8_DIST_o_35) );
lut8_DIST lut8_DIST_18 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_36),.lut8_DIST_i_1(lut8_DIST_i_37),.lut8_DIST_o_0(lut8_DIST_o_36),.lut8_DIST_o_1(lut8_DIST_o_37) );
lut8_DIST lut8_DIST_19 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_38),.lut8_DIST_i_1(lut8_DIST_i_39),.lut8_DIST_o_0(lut8_DIST_o_38),.lut8_DIST_o_1(lut8_DIST_o_39) );
lut8_DIST lut8_DIST_20 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_40),.lut8_DIST_i_1(lut8_DIST_i_41),.lut8_DIST_o_0(lut8_DIST_o_40),.lut8_DIST_o_1(lut8_DIST_o_41) );
lut8_DIST lut8_DIST_21 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_42),.lut8_DIST_i_1(lut8_DIST_i_43),.lut8_DIST_o_0(lut8_DIST_o_42),.lut8_DIST_o_1(lut8_DIST_o_43) );
lut8_DIST lut8_DIST_22 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_44),.lut8_DIST_i_1(lut8_DIST_i_45),.lut8_DIST_o_0(lut8_DIST_o_44),.lut8_DIST_o_1(lut8_DIST_o_45) );
lut8_DIST lut8_DIST_23 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_46),.lut8_DIST_i_1(lut8_DIST_i_47),.lut8_DIST_o_0(lut8_DIST_o_46),.lut8_DIST_o_1(lut8_DIST_o_47) );
lut8_DIST lut8_DIST_24 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_48),.lut8_DIST_i_1(lut8_DIST_i_49),.lut8_DIST_o_0(lut8_DIST_o_48),.lut8_DIST_o_1(lut8_DIST_o_49) );
lut8_DIST lut8_DIST_25 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_50),.lut8_DIST_i_1(lut8_DIST_i_51),.lut8_DIST_o_0(lut8_DIST_o_50),.lut8_DIST_o_1(lut8_DIST_o_51) );
lut8_DIST lut8_DIST_26 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_52),.lut8_DIST_i_1(lut8_DIST_i_53),.lut8_DIST_o_0(lut8_DIST_o_52),.lut8_DIST_o_1(lut8_DIST_o_53) );
lut8_DIST lut8_DIST_27 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_54),.lut8_DIST_i_1(lut8_DIST_i_55),.lut8_DIST_o_0(lut8_DIST_o_54),.lut8_DIST_o_1(lut8_DIST_o_55) );
lut8_DIST lut8_DIST_28 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_56),.lut8_DIST_i_1(lut8_DIST_i_57),.lut8_DIST_o_0(lut8_DIST_o_56),.lut8_DIST_o_1(lut8_DIST_o_57) );
lut8_DIST lut8_DIST_29 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_58),.lut8_DIST_i_1(lut8_DIST_i_59),.lut8_DIST_o_0(lut8_DIST_o_58),.lut8_DIST_o_1(lut8_DIST_o_59) );
lut8_DIST lut8_DIST_30 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_60),.lut8_DIST_i_1(lut8_DIST_i_61),.lut8_DIST_o_0(lut8_DIST_o_60),.lut8_DIST_o_1(lut8_DIST_o_61) );
lut8_DIST lut8_DIST_31 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_62),.lut8_DIST_i_1(lut8_DIST_i_63),.lut8_DIST_o_0(lut8_DIST_o_62),.lut8_DIST_o_1(lut8_DIST_o_63) );
wire [3:0] add_5_i_0_0;
wire [3:0] add_5_i_0_1;
wire [3:0] add_5_i_2_0;
wire [3:0] add_5_i_2_1;
wire [4:0] add_5_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [4:0] add_4_i_0_0;
wire [4:0] add_4_i_0_1;
wire [4:0] add_4_i_1_0;
wire [4:0] add_4_i_1_1;
wire [5:0] add_4_o_0_0;
wire [3:0] add_8_i_0_0;
wire [3:0] add_8_i_0_1;
wire [3:0] add_8_i_2_0;
wire [3:0] add_8_i_2_1;
wire [4:0] add_8_o_0_0;
wire [3:0] add_9_i_0_0;
wire [3:0] add_9_i_0_1;
wire [3:0] add_9_i_2_0;
wire [3:0] add_9_i_2_1;
wire [4:0] add_9_o_0_0;
wire [4:0] add_7_i_0_0;
wire [4:0] add_7_i_0_1;
wire [4:0] add_7_i_1_0;
wire [4:0] add_7_i_1_1;
wire [5:0] add_7_o_0_0;
wire [5:0] add_3_i_0_0;
wire [5:0] add_3_i_0_1;
wire [5:0] add_3_i_1_0;
wire [5:0] add_3_i_1_1;
wire [6:0] add_3_o_0_0;
wire [3:0] add_12_i_0_0;
wire [3:0] add_12_i_0_1;
wire [3:0] add_12_i_2_0;
wire [3:0] add_12_i_2_1;
wire [4:0] add_12_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [4:0] add_11_i_0_0;
wire [4:0] add_11_i_0_1;
wire [4:0] add_11_i_1_0;
wire [4:0] add_11_i_1_1;
wire [5:0] add_11_o_0_0;
wire [3:0] add_15_i_0_0;
wire [3:0] add_15_i_0_1;
wire [3:0] add_15_i_2_0;
wire [3:0] add_15_i_2_1;
wire [4:0] add_15_o_0_0;
wire [3:0] add_16_i_0_0;
wire [3:0] add_16_i_0_1;
wire [3:0] add_16_i_2_0;
wire [3:0] add_16_i_2_1;
wire [4:0] add_16_o_0_0;
wire [4:0] add_14_i_0_0;
wire [4:0] add_14_i_0_1;
wire [4:0] add_14_i_1_0;
wire [4:0] add_14_i_1_1;
wire [5:0] add_14_o_0_0;
wire [5:0] add_10_i_0_0;
wire [5:0] add_10_i_0_1;
wire [5:0] add_10_i_1_0;
wire [5:0] add_10_i_1_1;
wire [6:0] add_10_o_0_0;
wire [6:0] add_2_i_0_0;
wire [6:0] add_2_i_0_1;
wire [6:0] add_2_i_1_0;
wire [6:0] add_2_i_1_1;
wire [7:0] add_2_o_0_0;
wire [3:0] add_20_i_0_0;
wire [3:0] add_20_i_0_1;
wire [3:0] add_20_i_2_0;
wire [3:0] add_20_i_2_1;
wire [4:0] add_20_o_0_0;
wire [3:0] add_21_i_0_0;
wire [3:0] add_21_i_0_1;
wire [3:0] add_21_i_2_0;
wire [3:0] add_21_i_2_1;
wire [4:0] add_21_o_0_0;
wire [4:0] add_19_i_0_0;
wire [4:0] add_19_i_0_1;
wire [4:0] add_19_i_1_0;
wire [4:0] add_19_i_1_1;
wire [5:0] add_19_o_0_0;
wire [3:0] add_23_i_0_0;
wire [3:0] add_23_i_0_1;
wire [3:0] add_23_i_2_0;
wire [3:0] add_23_i_2_1;
wire [4:0] add_23_o_0_0;
wire [3:0] add_24_i_0_0;
wire [3:0] add_24_i_0_1;
wire [3:0] add_24_i_2_0;
wire [3:0] add_24_i_2_1;
wire [4:0] add_24_o_0_0;
wire [4:0] add_22_i_0_0;
wire [4:0] add_22_i_0_1;
wire [4:0] add_22_i_1_0;
wire [4:0] add_22_i_1_1;
wire [5:0] add_22_o_0_0;
wire [5:0] add_18_i_0_0;
wire [5:0] add_18_i_0_1;
wire [5:0] add_18_i_1_0;
wire [5:0] add_18_i_1_1;
wire [6:0] add_18_o_0_0;
wire [3:0] add_27_i_0_0;
wire [3:0] add_27_i_0_1;
wire [3:0] add_27_i_2_0;
wire [3:0] add_27_i_2_1;
wire [4:0] add_27_o_0_0;
wire [3:0] add_28_i_0_0;
wire [3:0] add_28_i_0_1;
wire [3:0] add_28_i_2_0;
wire [3:0] add_28_i_2_1;
wire [4:0] add_28_o_0_0;
wire [4:0] add_26_i_0_0;
wire [4:0] add_26_i_0_1;
wire [4:0] add_26_i_1_0;
wire [4:0] add_26_i_1_1;
wire [5:0] add_26_o_0_0;
wire [3:0] add_30_i_0_0;
wire [3:0] add_30_i_0_1;
wire [3:0] add_30_i_2_0;
wire [3:0] add_30_i_2_1;
wire [4:0] add_30_o_0_0;
wire [3:0] add_31_i_0_0;
wire [3:0] add_31_i_0_1;
wire [3:0] add_31_i_2_0;
wire [3:0] add_31_i_2_1;
wire [4:0] add_31_o_0_0;
wire [4:0] add_29_i_0_0;
wire [4:0] add_29_i_0_1;
wire [4:0] add_29_i_1_0;
wire [4:0] add_29_i_1_1;
wire [5:0] add_29_o_0_0;
wire [5:0] add_25_i_0_0;
wire [5:0] add_25_i_0_1;
wire [5:0] add_25_i_1_0;
wire [5:0] add_25_i_1_1;
wire [6:0] add_25_o_0_0;
wire [6:0] add_17_i_0_0;
wire [6:0] add_17_i_0_1;
wire [6:0] add_17_i_1_0;
wire [6:0] add_17_i_1_1;
wire [7:0] add_17_o_0_0;
wire [7:0] add_1_i_0_0;
wire [7:0] add_1_i_0_1;
wire [7:0] add_1_i_1_0;
wire [7:0] add_1_i_1_1;
wire [8:0] add_1_o_0_0;
wire [3:0] add_36_i_0_0;
wire [3:0] add_36_i_0_1;
wire [3:0] add_36_i_2_0;
wire [3:0] add_36_i_2_1;
wire [4:0] add_36_o_0_0;
wire [3:0] add_37_i_0_0;
wire [3:0] add_37_i_0_1;
wire [3:0] add_37_i_2_0;
wire [3:0] add_37_i_2_1;
wire [4:0] add_37_o_0_0;
wire [4:0] add_35_i_0_0;
wire [4:0] add_35_i_0_1;
wire [4:0] add_35_i_1_0;
wire [4:0] add_35_i_1_1;
wire [5:0] add_35_o_0_0;
wire [3:0] add_39_i_0_0;
wire [3:0] add_39_i_0_1;
wire [3:0] add_39_i_2_0;
wire [3:0] add_39_i_2_1;
wire [4:0] add_39_o_0_0;
wire [3:0] add_40_i_0_0;
wire [3:0] add_40_i_0_1;
wire [3:0] add_40_i_2_0;
wire [3:0] add_40_i_2_1;
wire [4:0] add_40_o_0_0;
wire [4:0] add_38_i_0_0;
wire [4:0] add_38_i_0_1;
wire [4:0] add_38_i_1_0;
wire [4:0] add_38_i_1_1;
wire [5:0] add_38_o_0_0;
wire [5:0] add_34_i_0_0;
wire [5:0] add_34_i_0_1;
wire [5:0] add_34_i_1_0;
wire [5:0] add_34_i_1_1;
wire [6:0] add_34_o_0_0;
wire [3:0] add_43_i_0_0;
wire [3:0] add_43_i_0_1;
wire [3:0] add_43_i_2_0;
wire [3:0] add_43_i_2_1;
wire [4:0] add_43_o_0_0;
wire [3:0] add_44_i_0_0;
wire [3:0] add_44_i_0_1;
wire [3:0] add_44_i_2_0;
wire [3:0] add_44_i_2_1;
wire [4:0] add_44_o_0_0;
wire [4:0] add_42_i_0_0;
wire [4:0] add_42_i_0_1;
wire [4:0] add_42_i_1_0;
wire [4:0] add_42_i_1_1;
wire [5:0] add_42_o_0_0;
wire [3:0] add_46_i_0_0;
wire [3:0] add_46_i_0_1;
wire [3:0] add_46_i_2_0;
wire [3:0] add_46_i_2_1;
wire [4:0] add_46_o_0_0;
wire [3:0] add_47_i_0_0;
wire [3:0] add_47_i_0_1;
wire [3:0] add_47_i_2_0;
wire [3:0] add_47_i_2_1;
wire [4:0] add_47_o_0_0;
wire [4:0] add_45_i_0_0;
wire [4:0] add_45_i_0_1;
wire [4:0] add_45_i_1_0;
wire [4:0] add_45_i_1_1;
wire [5:0] add_45_o_0_0;
wire [5:0] add_41_i_0_0;
wire [5:0] add_41_i_0_1;
wire [5:0] add_41_i_1_0;
wire [5:0] add_41_i_1_1;
wire [6:0] add_41_o_0_0;
wire [6:0] add_33_i_0_0;
wire [6:0] add_33_i_0_1;
wire [6:0] add_33_i_1_0;
wire [6:0] add_33_i_1_1;
wire [7:0] add_33_o_0_0;
wire [3:0] add_51_i_0_0;
wire [3:0] add_51_i_0_1;
wire [3:0] add_51_i_2_0;
wire [3:0] add_51_i_2_1;
wire [4:0] add_51_o_0_0;
wire [3:0] add_52_i_0_0;
wire [3:0] add_52_i_0_1;
wire [3:0] add_52_i_2_0;
wire [3:0] add_52_i_2_1;
wire [4:0] add_52_o_0_0;
wire [4:0] add_50_i_0_0;
wire [4:0] add_50_i_0_1;
wire [4:0] add_50_i_1_0;
wire [4:0] add_50_i_1_1;
wire [5:0] add_50_o_0_0;
wire [3:0] add_54_i_0_0;
wire [3:0] add_54_i_0_1;
wire [3:0] add_54_i_2_0;
wire [3:0] add_54_i_2_1;
wire [4:0] add_54_o_0_0;
wire [3:0] add_55_i_0_0;
wire [3:0] add_55_i_0_1;
wire [3:0] add_55_i_2_0;
wire [3:0] add_55_i_2_1;
wire [4:0] add_55_o_0_0;
wire [4:0] add_53_i_0_0;
wire [4:0] add_53_i_0_1;
wire [4:0] add_53_i_1_0;
wire [4:0] add_53_i_1_1;
wire [5:0] add_53_o_0_0;
wire [5:0] add_49_i_0_0;
wire [5:0] add_49_i_0_1;
wire [5:0] add_49_i_1_0;
wire [5:0] add_49_i_1_1;
wire [6:0] add_49_o_0_0;
wire [3:0] add_58_i_0_0;
wire [3:0] add_58_i_0_1;
wire [3:0] add_58_i_2_0;
wire [3:0] add_58_i_2_1;
wire [4:0] add_58_o_0_0;
wire [3:0] add_59_i_0_0;
wire [3:0] add_59_i_0_1;
wire [3:0] add_59_i_2_0;
wire [3:0] add_59_i_2_1;
wire [4:0] add_59_o_0_0;
wire [4:0] add_57_i_0_0;
wire [4:0] add_57_i_0_1;
wire [4:0] add_57_i_1_0;
wire [4:0] add_57_i_1_1;
wire [5:0] add_57_o_0_0;
wire [3:0] add_61_i_0_0;
wire [3:0] add_61_i_0_1;
wire [3:0] add_61_i_2_0;
wire [3:0] add_61_i_2_1;
wire [4:0] add_61_o_0_0;
wire [3:0] add_62_i_0_0;
wire [3:0] add_62_i_0_1;
wire [3:0] add_62_i_2_0;
wire [3:0] add_62_i_2_1;
wire [4:0] add_62_o_0_0;
wire [4:0] add_60_i_0_0;
wire [4:0] add_60_i_0_1;
wire [4:0] add_60_i_1_0;
wire [4:0] add_60_i_1_1;
wire [5:0] add_60_o_0_0;
wire [5:0] add_56_i_0_0;
wire [5:0] add_56_i_0_1;
wire [5:0] add_56_i_1_0;
wire [5:0] add_56_i_1_1;
wire [6:0] add_56_o_0_0;
wire [6:0] add_48_i_0_0;
wire [6:0] add_48_i_0_1;
wire [6:0] add_48_i_1_0;
wire [6:0] add_48_i_1_1;
wire [7:0] add_48_o_0_0;
wire [7:0] add_32_i_0_0;
wire [7:0] add_32_i_0_1;
wire [7:0] add_32_i_1_0;
wire [7:0] add_32_i_1_1;
wire [8:0] add_32_o_0_0;
wire [8:0] add_0_i_0_0;
wire [8:0] add_0_i_0_1;
wire [8:0] add_0_i_1_0;
wire [8:0] add_0_i_1_1;
wire [9:0] add_0_o_0_0;
wire [9:0] add_0_o_1_0;
assign add_5_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_5_i_0_1[3:0] = lut8_DIST_o_1[3:0];
assign add_6_i_0_0[3:0] = lut8_DIST_o_2[3:0];
assign add_6_i_0_1[3:0] = lut8_DIST_o_3[3:0];
assign add_8_i_0_0[3:0] = lut8_DIST_o_4[3:0];
assign add_8_i_0_1[3:0] = lut8_DIST_o_5[3:0];
assign add_9_i_0_0[3:0] = lut8_DIST_o_6[3:0];
assign add_9_i_0_1[3:0] = lut8_DIST_o_7[3:0];
assign add_12_i_0_0[3:0] = lut8_DIST_o_8[3:0];
assign add_12_i_0_1[3:0] = lut8_DIST_o_9[3:0];
assign add_13_i_0_0[3:0] = lut8_DIST_o_10[3:0];
assign add_13_i_0_1[3:0] = lut8_DIST_o_11[3:0];
assign add_15_i_0_0[3:0] = lut8_DIST_o_12[3:0];
assign add_15_i_0_1[3:0] = lut8_DIST_o_13[3:0];
assign add_16_i_0_0[3:0] = lut8_DIST_o_14[3:0];
assign add_16_i_0_1[3:0] = lut8_DIST_o_15[3:0];
assign add_20_i_0_0[3:0] = lut8_DIST_o_16[3:0];
assign add_20_i_0_1[3:0] = lut8_DIST_o_17[3:0];
assign add_21_i_0_0[3:0] = lut8_DIST_o_18[3:0];
assign add_21_i_0_1[3:0] = lut8_DIST_o_19[3:0];
assign add_23_i_0_0[3:0] = lut8_DIST_o_20[3:0];
assign add_23_i_0_1[3:0] = lut8_DIST_o_21[3:0];
assign add_24_i_0_0[3:0] = lut8_DIST_o_22[3:0];
assign add_24_i_0_1[3:0] = lut8_DIST_o_23[3:0];
assign add_27_i_0_0[3:0] = lut8_DIST_o_24[3:0];
assign add_27_i_0_1[3:0] = lut8_DIST_o_25[3:0];
assign add_28_i_0_0[3:0] = lut8_DIST_o_26[3:0];
assign add_28_i_0_1[3:0] = lut8_DIST_o_27[3:0];
assign add_30_i_0_0[3:0] = lut8_DIST_o_28[3:0];
assign add_30_i_0_1[3:0] = lut8_DIST_o_29[3:0];
assign add_31_i_0_0[3:0] = lut8_DIST_o_30[3:0];
assign add_31_i_0_1[3:0] = lut8_DIST_o_31[3:0];
assign add_36_i_0_0[3:0] = lut8_DIST_o_32[3:0];
assign add_36_i_0_1[3:0] = lut8_DIST_o_33[3:0];
assign add_37_i_0_0[3:0] = lut8_DIST_o_34[3:0];
assign add_37_i_0_1[3:0] = lut8_DIST_o_35[3:0];
assign add_39_i_0_0[3:0] = lut8_DIST_o_36[3:0];
assign add_39_i_0_1[3:0] = lut8_DIST_o_37[3:0];
assign add_40_i_0_0[3:0] = lut8_DIST_o_38[3:0];
assign add_40_i_0_1[3:0] = lut8_DIST_o_39[3:0];
assign add_43_i_0_0[3:0] = lut8_DIST_o_40[3:0];
assign add_43_i_0_1[3:0] = lut8_DIST_o_41[3:0];
assign add_44_i_0_0[3:0] = lut8_DIST_o_42[3:0];
assign add_44_i_0_1[3:0] = lut8_DIST_o_43[3:0];
assign add_46_i_0_0[3:0] = lut8_DIST_o_44[3:0];
assign add_46_i_0_1[3:0] = lut8_DIST_o_45[3:0];
assign add_47_i_0_0[3:0] = lut8_DIST_o_46[3:0];
assign add_47_i_0_1[3:0] = lut8_DIST_o_47[3:0];
assign add_51_i_0_0[3:0] = lut8_DIST_o_48[3:0];
assign add_51_i_0_1[3:0] = lut8_DIST_o_49[3:0];
assign add_52_i_0_0[3:0] = lut8_DIST_o_50[3:0];
assign add_52_i_0_1[3:0] = lut8_DIST_o_51[3:0];
assign add_54_i_0_0[3:0] = lut8_DIST_o_52[3:0];
assign add_54_i_0_1[3:0] = lut8_DIST_o_53[3:0];
assign add_55_i_0_0[3:0] = lut8_DIST_o_54[3:0];
assign add_55_i_0_1[3:0] = lut8_DIST_o_55[3:0];
assign add_58_i_0_0[3:0] = lut8_DIST_o_56[3:0];
assign add_58_i_0_1[3:0] = lut8_DIST_o_57[3:0];
assign add_59_i_0_0[3:0] = lut8_DIST_o_58[3:0];
assign add_59_i_0_1[3:0] = lut8_DIST_o_59[3:0];
assign add_61_i_0_0[3:0] = lut8_DIST_o_60[3:0];
assign add_61_i_0_1[3:0] = lut8_DIST_o_61[3:0];
assign add_62_i_0_0[3:0] = lut8_DIST_o_62[3:0];
assign add_62_i_0_1[3:0] = lut8_DIST_o_63[3:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_3_i_0_0[5:0] = add_4_o_0_0[5:0];
assign add_7_i_0_0[4:0] = add_8_o_0_0[4:0];
assign add_7_i_0_1[4:0] = add_9_o_0_0[4:0];
assign add_3_i_0_1[5:0] = add_7_o_0_0[5:0];
assign add_2_i_0_0[6:0] = add_3_o_0_0[6:0];
assign add_11_i_0_0[4:0] = add_12_o_0_0[4:0];
assign add_11_i_0_1[4:0] = add_13_o_0_0[4:0];
assign add_10_i_0_0[5:0] = add_11_o_0_0[5:0];
assign add_14_i_0_0[4:0] = add_15_o_0_0[4:0];
assign add_14_i_0_1[4:0] = add_16_o_0_0[4:0];
assign add_10_i_0_1[5:0] = add_14_o_0_0[5:0];
assign add_2_i_0_1[6:0] = add_10_o_0_0[6:0];
assign add_1_i_0_0[7:0] = add_2_o_0_0[7:0];
assign add_19_i_0_0[4:0] = add_20_o_0_0[4:0];
assign add_19_i_0_1[4:0] = add_21_o_0_0[4:0];
assign add_18_i_0_0[5:0] = add_19_o_0_0[5:0];
assign add_22_i_0_0[4:0] = add_23_o_0_0[4:0];
assign add_22_i_0_1[4:0] = add_24_o_0_0[4:0];
assign add_18_i_0_1[5:0] = add_22_o_0_0[5:0];
assign add_17_i_0_0[6:0] = add_18_o_0_0[6:0];
assign add_26_i_0_0[4:0] = add_27_o_0_0[4:0];
assign add_26_i_0_1[4:0] = add_28_o_0_0[4:0];
assign add_25_i_0_0[5:0] = add_26_o_0_0[5:0];
assign add_29_i_0_0[4:0] = add_30_o_0_0[4:0];
assign add_29_i_0_1[4:0] = add_31_o_0_0[4:0];
assign add_25_i_0_1[5:0] = add_29_o_0_0[5:0];
assign add_17_i_0_1[6:0] = add_25_o_0_0[6:0];
assign add_1_i_0_1[7:0] = add_17_o_0_0[7:0];
assign add_0_i_0_0[8:0] = add_1_o_0_0[8:0];
assign add_35_i_0_0[4:0] = add_36_o_0_0[4:0];
assign add_35_i_0_1[4:0] = add_37_o_0_0[4:0];
assign add_34_i_0_0[5:0] = add_35_o_0_0[5:0];
assign add_38_i_0_0[4:0] = add_39_o_0_0[4:0];
assign add_38_i_0_1[4:0] = add_40_o_0_0[4:0];
assign add_34_i_0_1[5:0] = add_38_o_0_0[5:0];
assign add_33_i_0_0[6:0] = add_34_o_0_0[6:0];
assign add_42_i_0_0[4:0] = add_43_o_0_0[4:0];
assign add_42_i_0_1[4:0] = add_44_o_0_0[4:0];
assign add_41_i_0_0[5:0] = add_42_o_0_0[5:0];
assign add_45_i_0_0[4:0] = add_46_o_0_0[4:0];
assign add_45_i_0_1[4:0] = add_47_o_0_0[4:0];
assign add_41_i_0_1[5:0] = add_45_o_0_0[5:0];
assign add_33_i_0_1[6:0] = add_41_o_0_0[6:0];
assign add_32_i_0_0[7:0] = add_33_o_0_0[7:0];
assign add_50_i_0_0[4:0] = add_51_o_0_0[4:0];
assign add_50_i_0_1[4:0] = add_52_o_0_0[4:0];
assign add_49_i_0_0[5:0] = add_50_o_0_0[5:0];
assign add_53_i_0_0[4:0] = add_54_o_0_0[4:0];
assign add_53_i_0_1[4:0] = add_55_o_0_0[4:0];
assign add_49_i_0_1[5:0] = add_53_o_0_0[5:0];
assign add_48_i_0_0[6:0] = add_49_o_0_0[6:0];
assign add_57_i_0_0[4:0] = add_58_o_0_0[4:0];
assign add_57_i_0_1[4:0] = add_59_o_0_0[4:0];
assign add_56_i_0_0[5:0] = add_57_o_0_0[5:0];
assign add_60_i_0_0[4:0] = add_61_o_0_0[4:0];
assign add_60_i_0_1[4:0] = add_62_o_0_0[4:0];
assign add_56_i_0_1[5:0] = add_60_o_0_0[5:0];
assign add_48_i_0_1[6:0] = add_56_o_0_0[6:0];
assign add_32_i_0_1[7:0] = add_48_o_0_0[7:0];
assign add_0_i_0_1[8:0] = add_32_o_0_0[8:0];
du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(4, 2) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_0),.din(add_8_i_0_0));
du #(4, 2) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_2_0 + add_8_i_2_1;

du #(4, 2) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_0),.din(add_9_i_0_0));
du #(4, 2) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_2_0 + add_9_i_2_1;

du #(5, 1) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_0),.din(add_7_i_0_0));
du #(5, 1) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_1_0 + add_7_i_1_1;

du #(6, 1) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_0),.din(add_3_i_0_0));
du #(6, 1) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_1_0 + add_3_i_1_1;

du #(4, 2) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_0),.din(add_12_i_0_0));
du #(4, 2) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_2_0 + add_12_i_2_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(5, 1) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_0),.din(add_11_i_0_0));
du #(5, 1) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_1_0 + add_11_i_1_1;

du #(4, 2) du_15_0 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_0),.din(add_15_i_0_0));
du #(4, 2) du_15_1 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_1),.din(add_15_i_0_1));

assign add_15_o_0_0 = add_15_i_2_0 + add_15_i_2_1;

du #(4, 2) du_16_0 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_0),.din(add_16_i_0_0));
du #(4, 2) du_16_1 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_1),.din(add_16_i_0_1));

assign add_16_o_0_0 = add_16_i_2_0 + add_16_i_2_1;

du #(5, 1) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_0),.din(add_14_i_0_0));
du #(5, 1) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_1_0 + add_14_i_1_1;

du #(6, 1) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_0),.din(add_10_i_0_0));
du #(6, 1) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_1_0 + add_10_i_1_1;

du #(7, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(7, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_20_0 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_0),.din(add_20_i_0_0));
du #(4, 2) du_20_1 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_1),.din(add_20_i_0_1));

assign add_20_o_0_0 = add_20_i_2_0 + add_20_i_2_1;

du #(4, 2) du_21_0 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_0),.din(add_21_i_0_0));
du #(4, 2) du_21_1 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_1),.din(add_21_i_0_1));

assign add_21_o_0_0 = add_21_i_2_0 + add_21_i_2_1;

du #(5, 1) du_19_0 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_0),.din(add_19_i_0_0));
du #(5, 1) du_19_1 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_1),.din(add_19_i_0_1));

assign add_19_o_0_0 = add_19_i_1_0 + add_19_i_1_1;

du #(4, 2) du_23_0 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_0),.din(add_23_i_0_0));
du #(4, 2) du_23_1 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_1),.din(add_23_i_0_1));

assign add_23_o_0_0 = add_23_i_2_0 + add_23_i_2_1;

du #(4, 2) du_24_0 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_0),.din(add_24_i_0_0));
du #(4, 2) du_24_1 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_1),.din(add_24_i_0_1));

assign add_24_o_0_0 = add_24_i_2_0 + add_24_i_2_1;

du #(5, 1) du_22_0 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_0),.din(add_22_i_0_0));
du #(5, 1) du_22_1 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_1),.din(add_22_i_0_1));

assign add_22_o_0_0 = add_22_i_1_0 + add_22_i_1_1;

du #(6, 1) du_18_0 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_0),.din(add_18_i_0_0));
du #(6, 1) du_18_1 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_1),.din(add_18_i_0_1));

assign add_18_o_0_0 = add_18_i_1_0 + add_18_i_1_1;

du #(4, 2) du_27_0 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_0),.din(add_27_i_0_0));
du #(4, 2) du_27_1 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_1),.din(add_27_i_0_1));

assign add_27_o_0_0 = add_27_i_2_0 + add_27_i_2_1;

du #(4, 2) du_28_0 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_0),.din(add_28_i_0_0));
du #(4, 2) du_28_1 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_1),.din(add_28_i_0_1));

assign add_28_o_0_0 = add_28_i_2_0 + add_28_i_2_1;

du #(5, 1) du_26_0 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_0),.din(add_26_i_0_0));
du #(5, 1) du_26_1 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_1),.din(add_26_i_0_1));

assign add_26_o_0_0 = add_26_i_1_0 + add_26_i_1_1;

du #(4, 2) du_30_0 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_0),.din(add_30_i_0_0));
du #(4, 2) du_30_1 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_1),.din(add_30_i_0_1));

assign add_30_o_0_0 = add_30_i_2_0 + add_30_i_2_1;

du #(4, 2) du_31_0 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_0),.din(add_31_i_0_0));
du #(4, 2) du_31_1 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_1),.din(add_31_i_0_1));

assign add_31_o_0_0 = add_31_i_2_0 + add_31_i_2_1;

du #(5, 1) du_29_0 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_0),.din(add_29_i_0_0));
du #(5, 1) du_29_1 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_1),.din(add_29_i_0_1));

assign add_29_o_0_0 = add_29_i_1_0 + add_29_i_1_1;

du #(6, 1) du_25_0 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_0),.din(add_25_i_0_0));
du #(6, 1) du_25_1 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_1),.din(add_25_i_0_1));

assign add_25_o_0_0 = add_25_i_1_0 + add_25_i_1_1;

du #(7, 1) du_17_0 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_0),.din(add_17_i_0_0));
du #(7, 1) du_17_1 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_1),.din(add_17_i_0_1));

assign add_17_o_0_0 = add_17_i_1_0 + add_17_i_1_1;

du #(8, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(8, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_36_0 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_0),.din(add_36_i_0_0));
du #(4, 2) du_36_1 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_1),.din(add_36_i_0_1));

assign add_36_o_0_0 = add_36_i_2_0 + add_36_i_2_1;

du #(4, 2) du_37_0 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_0),.din(add_37_i_0_0));
du #(4, 2) du_37_1 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_1),.din(add_37_i_0_1));

assign add_37_o_0_0 = add_37_i_2_0 + add_37_i_2_1;

du #(5, 1) du_35_0 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_0),.din(add_35_i_0_0));
du #(5, 1) du_35_1 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_1),.din(add_35_i_0_1));

assign add_35_o_0_0 = add_35_i_1_0 + add_35_i_1_1;

du #(4, 2) du_39_0 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_0),.din(add_39_i_0_0));
du #(4, 2) du_39_1 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_1),.din(add_39_i_0_1));

assign add_39_o_0_0 = add_39_i_2_0 + add_39_i_2_1;

du #(4, 2) du_40_0 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_0),.din(add_40_i_0_0));
du #(4, 2) du_40_1 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_1),.din(add_40_i_0_1));

assign add_40_o_0_0 = add_40_i_2_0 + add_40_i_2_1;

du #(5, 1) du_38_0 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_0),.din(add_38_i_0_0));
du #(5, 1) du_38_1 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_1),.din(add_38_i_0_1));

assign add_38_o_0_0 = add_38_i_1_0 + add_38_i_1_1;

du #(6, 1) du_34_0 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_0),.din(add_34_i_0_0));
du #(6, 1) du_34_1 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_1),.din(add_34_i_0_1));

assign add_34_o_0_0 = add_34_i_1_0 + add_34_i_1_1;

du #(4, 2) du_43_0 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_0),.din(add_43_i_0_0));
du #(4, 2) du_43_1 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_1),.din(add_43_i_0_1));

assign add_43_o_0_0 = add_43_i_2_0 + add_43_i_2_1;

du #(4, 2) du_44_0 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_0),.din(add_44_i_0_0));
du #(4, 2) du_44_1 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_1),.din(add_44_i_0_1));

assign add_44_o_0_0 = add_44_i_2_0 + add_44_i_2_1;

du #(5, 1) du_42_0 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_0),.din(add_42_i_0_0));
du #(5, 1) du_42_1 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_1),.din(add_42_i_0_1));

assign add_42_o_0_0 = add_42_i_1_0 + add_42_i_1_1;

du #(4, 2) du_46_0 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_0),.din(add_46_i_0_0));
du #(4, 2) du_46_1 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_1),.din(add_46_i_0_1));

assign add_46_o_0_0 = add_46_i_2_0 + add_46_i_2_1;

du #(4, 2) du_47_0 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_0),.din(add_47_i_0_0));
du #(4, 2) du_47_1 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_1),.din(add_47_i_0_1));

assign add_47_o_0_0 = add_47_i_2_0 + add_47_i_2_1;

du #(5, 1) du_45_0 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_0),.din(add_45_i_0_0));
du #(5, 1) du_45_1 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_1),.din(add_45_i_0_1));

assign add_45_o_0_0 = add_45_i_1_0 + add_45_i_1_1;

du #(6, 1) du_41_0 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_0),.din(add_41_i_0_0));
du #(6, 1) du_41_1 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_1),.din(add_41_i_0_1));

assign add_41_o_0_0 = add_41_i_1_0 + add_41_i_1_1;

du #(7, 1) du_33_0 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_0),.din(add_33_i_0_0));
du #(7, 1) du_33_1 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_1),.din(add_33_i_0_1));

assign add_33_o_0_0 = add_33_i_1_0 + add_33_i_1_1;

du #(4, 2) du_51_0 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_0),.din(add_51_i_0_0));
du #(4, 2) du_51_1 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_1),.din(add_51_i_0_1));

assign add_51_o_0_0 = add_51_i_2_0 + add_51_i_2_1;

du #(4, 2) du_52_0 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_0),.din(add_52_i_0_0));
du #(4, 2) du_52_1 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_1),.din(add_52_i_0_1));

assign add_52_o_0_0 = add_52_i_2_0 + add_52_i_2_1;

du #(5, 1) du_50_0 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_0),.din(add_50_i_0_0));
du #(5, 1) du_50_1 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_1),.din(add_50_i_0_1));

assign add_50_o_0_0 = add_50_i_1_0 + add_50_i_1_1;

du #(4, 2) du_54_0 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_0),.din(add_54_i_0_0));
du #(4, 2) du_54_1 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_1),.din(add_54_i_0_1));

assign add_54_o_0_0 = add_54_i_2_0 + add_54_i_2_1;

du #(4, 2) du_55_0 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_0),.din(add_55_i_0_0));
du #(4, 2) du_55_1 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_1),.din(add_55_i_0_1));

assign add_55_o_0_0 = add_55_i_2_0 + add_55_i_2_1;

du #(5, 1) du_53_0 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_0),.din(add_53_i_0_0));
du #(5, 1) du_53_1 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_1),.din(add_53_i_0_1));

assign add_53_o_0_0 = add_53_i_1_0 + add_53_i_1_1;

du #(6, 1) du_49_0 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_0),.din(add_49_i_0_0));
du #(6, 1) du_49_1 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_1),.din(add_49_i_0_1));

assign add_49_o_0_0 = add_49_i_1_0 + add_49_i_1_1;

du #(4, 2) du_58_0 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_0),.din(add_58_i_0_0));
du #(4, 2) du_58_1 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_1),.din(add_58_i_0_1));

assign add_58_o_0_0 = add_58_i_2_0 + add_58_i_2_1;

du #(4, 2) du_59_0 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_0),.din(add_59_i_0_0));
du #(4, 2) du_59_1 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_1),.din(add_59_i_0_1));

assign add_59_o_0_0 = add_59_i_2_0 + add_59_i_2_1;

du #(5, 1) du_57_0 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_0),.din(add_57_i_0_0));
du #(5, 1) du_57_1 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_1),.din(add_57_i_0_1));

assign add_57_o_0_0 = add_57_i_1_0 + add_57_i_1_1;

du #(4, 2) du_61_0 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_0),.din(add_61_i_0_0));
du #(4, 2) du_61_1 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_1),.din(add_61_i_0_1));

assign add_61_o_0_0 = add_61_i_2_0 + add_61_i_2_1;

du #(4, 2) du_62_0 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_0),.din(add_62_i_0_0));
du #(4, 2) du_62_1 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_1),.din(add_62_i_0_1));

assign add_62_o_0_0 = add_62_i_2_0 + add_62_i_2_1;

du #(5, 1) du_60_0 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_0),.din(add_60_i_0_0));
du #(5, 1) du_60_1 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_1),.din(add_60_i_0_1));

assign add_60_o_0_0 = add_60_i_1_0 + add_60_i_1_1;

du #(6, 1) du_56_0 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_0),.din(add_56_i_0_0));
du #(6, 1) du_56_1 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_1),.din(add_56_i_0_1));

assign add_56_o_0_0 = add_56_i_1_0 + add_56_i_1_1;

du #(7, 1) du_48_0 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_0),.din(add_48_i_0_0));
du #(7, 1) du_48_1 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_1),.din(add_48_i_0_1));

assign add_48_o_0_0 = add_48_i_1_0 + add_48_i_1_1;

du #(8, 1) du_32_0 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_0),.din(add_32_i_0_0));
du #(8, 1) du_32_1 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_1),.din(add_32_i_0_1));

assign add_32_o_0_0 = add_32_i_1_0 + add_32_i_1_1;

du #(9, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(9, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(10, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_dist_nodsp_512_9 (
input clk,
input	[511:0] input_v,
output	[9:0] output_v
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
wire [7:0] input_16;
wire [7:0] input_17;
wire [7:0] input_18;
wire [7:0] input_19;
wire [7:0] input_20;
wire [7:0] input_21;
wire [7:0] input_22;
wire [7:0] input_23;
wire [7:0] input_24;
wire [7:0] input_25;
wire [7:0] input_26;
wire [7:0] input_27;
wire [7:0] input_28;
wire [7:0] input_29;
wire [7:0] input_30;
wire [7:0] input_31;
wire [7:0] input_32;
wire [7:0] input_33;
wire [7:0] input_34;
wire [7:0] input_35;
wire [7:0] input_36;
wire [7:0] input_37;
wire [7:0] input_38;
wire [7:0] input_39;
wire [7:0] input_40;
wire [7:0] input_41;
wire [7:0] input_42;
wire [7:0] input_43;
wire [7:0] input_44;
wire [7:0] input_45;
wire [7:0] input_46;
wire [7:0] input_47;
wire [7:0] input_48;
wire [7:0] input_49;
wire [7:0] input_50;
wire [7:0] input_51;
wire [7:0] input_52;
wire [7:0] input_53;
wire [7:0] input_54;
wire [7:0] input_55;
wire [7:0] input_56;
wire [7:0] input_57;
wire [7:0] input_58;
wire [7:0] input_59;
wire [7:0] input_60;
wire [7:0] input_61;
wire [7:0] input_62;
wire [7:0] input_63;
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
wire [7:0] lut8_DIST_i_16;
wire [7:0] lut8_DIST_i_17;
wire [7:0] lut8_DIST_i_18;
wire [7:0] lut8_DIST_i_19;
wire [7:0] lut8_DIST_i_20;
wire [7:0] lut8_DIST_i_21;
wire [7:0] lut8_DIST_i_22;
wire [7:0] lut8_DIST_i_23;
wire [7:0] lut8_DIST_i_24;
wire [7:0] lut8_DIST_i_25;
wire [7:0] lut8_DIST_i_26;
wire [7:0] lut8_DIST_i_27;
wire [7:0] lut8_DIST_i_28;
wire [7:0] lut8_DIST_i_29;
wire [7:0] lut8_DIST_i_30;
wire [7:0] lut8_DIST_i_31;
wire [7:0] lut8_DIST_i_32;
wire [7:0] lut8_DIST_i_33;
wire [7:0] lut8_DIST_i_34;
wire [7:0] lut8_DIST_i_35;
wire [7:0] lut8_DIST_i_36;
wire [7:0] lut8_DIST_i_37;
wire [7:0] lut8_DIST_i_38;
wire [7:0] lut8_DIST_i_39;
wire [7:0] lut8_DIST_i_40;
wire [7:0] lut8_DIST_i_41;
wire [7:0] lut8_DIST_i_42;
wire [7:0] lut8_DIST_i_43;
wire [7:0] lut8_DIST_i_44;
wire [7:0] lut8_DIST_i_45;
wire [7:0] lut8_DIST_i_46;
wire [7:0] lut8_DIST_i_47;
wire [7:0] lut8_DIST_i_48;
wire [7:0] lut8_DIST_i_49;
wire [7:0] lut8_DIST_i_50;
wire [7:0] lut8_DIST_i_51;
wire [7:0] lut8_DIST_i_52;
wire [7:0] lut8_DIST_i_53;
wire [7:0] lut8_DIST_i_54;
wire [7:0] lut8_DIST_i_55;
wire [7:0] lut8_DIST_i_56;
wire [7:0] lut8_DIST_i_57;
wire [7:0] lut8_DIST_i_58;
wire [7:0] lut8_DIST_i_59;
wire [7:0] lut8_DIST_i_60;
wire [7:0] lut8_DIST_i_61;
wire [7:0] lut8_DIST_i_62;
wire [7:0] lut8_DIST_i_63;
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
wire [3:0] lut8_DIST_o_16;
wire [3:0] lut8_DIST_o_17;
wire [3:0] lut8_DIST_o_18;
wire [3:0] lut8_DIST_o_19;
wire [3:0] lut8_DIST_o_20;
wire [3:0] lut8_DIST_o_21;
wire [3:0] lut8_DIST_o_22;
wire [3:0] lut8_DIST_o_23;
wire [3:0] lut8_DIST_o_24;
wire [3:0] lut8_DIST_o_25;
wire [3:0] lut8_DIST_o_26;
wire [3:0] lut8_DIST_o_27;
wire [3:0] lut8_DIST_o_28;
wire [3:0] lut8_DIST_o_29;
wire [3:0] lut8_DIST_o_30;
wire [3:0] lut8_DIST_o_31;
wire [3:0] lut8_DIST_o_32;
wire [3:0] lut8_DIST_o_33;
wire [3:0] lut8_DIST_o_34;
wire [3:0] lut8_DIST_o_35;
wire [3:0] lut8_DIST_o_36;
wire [3:0] lut8_DIST_o_37;
wire [3:0] lut8_DIST_o_38;
wire [3:0] lut8_DIST_o_39;
wire [3:0] lut8_DIST_o_40;
wire [3:0] lut8_DIST_o_41;
wire [3:0] lut8_DIST_o_42;
wire [3:0] lut8_DIST_o_43;
wire [3:0] lut8_DIST_o_44;
wire [3:0] lut8_DIST_o_45;
wire [3:0] lut8_DIST_o_46;
wire [3:0] lut8_DIST_o_47;
wire [3:0] lut8_DIST_o_48;
wire [3:0] lut8_DIST_o_49;
wire [3:0] lut8_DIST_o_50;
wire [3:0] lut8_DIST_o_51;
wire [3:0] lut8_DIST_o_52;
wire [3:0] lut8_DIST_o_53;
wire [3:0] lut8_DIST_o_54;
wire [3:0] lut8_DIST_o_55;
wire [3:0] lut8_DIST_o_56;
wire [3:0] lut8_DIST_o_57;
wire [3:0] lut8_DIST_o_58;
wire [3:0] lut8_DIST_o_59;
wire [3:0] lut8_DIST_o_60;
wire [3:0] lut8_DIST_o_61;
wire [3:0] lut8_DIST_o_62;
wire [3:0] lut8_DIST_o_63;
assign input_0[7:0] = input_v[511:504];
assign input_1[7:0] = input_v[503:496];
assign input_2[7:0] = input_v[495:488];
assign input_3[7:0] = input_v[487:480];
assign input_4[7:0] = input_v[479:472];
assign input_5[7:0] = input_v[471:464];
assign input_6[7:0] = input_v[463:456];
assign input_7[7:0] = input_v[455:448];
assign input_8[7:0] = input_v[447:440];
assign input_9[7:0] = input_v[439:432];
assign input_10[7:0] = input_v[431:424];
assign input_11[7:0] = input_v[423:416];
assign input_12[7:0] = input_v[415:408];
assign input_13[7:0] = input_v[407:400];
assign input_14[7:0] = input_v[399:392];
assign input_15[7:0] = input_v[391:384];
assign input_16[7:0] = input_v[383:376];
assign input_17[7:0] = input_v[375:368];
assign input_18[7:0] = input_v[367:360];
assign input_19[7:0] = input_v[359:352];
assign input_20[7:0] = input_v[351:344];
assign input_21[7:0] = input_v[343:336];
assign input_22[7:0] = input_v[335:328];
assign input_23[7:0] = input_v[327:320];
assign input_24[7:0] = input_v[319:312];
assign input_25[7:0] = input_v[311:304];
assign input_26[7:0] = input_v[303:296];
assign input_27[7:0] = input_v[295:288];
assign input_28[7:0] = input_v[287:280];
assign input_29[7:0] = input_v[279:272];
assign input_30[7:0] = input_v[271:264];
assign input_31[7:0] = input_v[263:256];
assign input_32[7:0] = input_v[255:248];
assign input_33[7:0] = input_v[247:240];
assign input_34[7:0] = input_v[239:232];
assign input_35[7:0] = input_v[231:224];
assign input_36[7:0] = input_v[223:216];
assign input_37[7:0] = input_v[215:208];
assign input_38[7:0] = input_v[207:200];
assign input_39[7:0] = input_v[199:192];
assign input_40[7:0] = input_v[191:184];
assign input_41[7:0] = input_v[183:176];
assign input_42[7:0] = input_v[175:168];
assign input_43[7:0] = input_v[167:160];
assign input_44[7:0] = input_v[159:152];
assign input_45[7:0] = input_v[151:144];
assign input_46[7:0] = input_v[143:136];
assign input_47[7:0] = input_v[135:128];
assign input_48[7:0] = input_v[127:120];
assign input_49[7:0] = input_v[119:112];
assign input_50[7:0] = input_v[111:104];
assign input_51[7:0] = input_v[103:96];
assign input_52[7:0] = input_v[95:88];
assign input_53[7:0] = input_v[87:80];
assign input_54[7:0] = input_v[79:72];
assign input_55[7:0] = input_v[71:64];
assign input_56[7:0] = input_v[63:56];
assign input_57[7:0] = input_v[55:48];
assign input_58[7:0] = input_v[47:40];
assign input_59[7:0] = input_v[39:32];
assign input_60[7:0] = input_v[31:24];
assign input_61[7:0] = input_v[23:16];
assign input_62[7:0] = input_v[15:8];
assign input_63[7:0] = input_v[7:0];
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
assign lut8_DIST_i_16[7:0] = input_16[7:0];
assign lut8_DIST_i_17[7:0] = input_17[7:0];
assign lut8_DIST_i_18[7:0] = input_18[7:0];
assign lut8_DIST_i_19[7:0] = input_19[7:0];
assign lut8_DIST_i_20[7:0] = input_20[7:0];
assign lut8_DIST_i_21[7:0] = input_21[7:0];
assign lut8_DIST_i_22[7:0] = input_22[7:0];
assign lut8_DIST_i_23[7:0] = input_23[7:0];
assign lut8_DIST_i_24[7:0] = input_24[7:0];
assign lut8_DIST_i_25[7:0] = input_25[7:0];
assign lut8_DIST_i_26[7:0] = input_26[7:0];
assign lut8_DIST_i_27[7:0] = input_27[7:0];
assign lut8_DIST_i_28[7:0] = input_28[7:0];
assign lut8_DIST_i_29[7:0] = input_29[7:0];
assign lut8_DIST_i_30[7:0] = input_30[7:0];
assign lut8_DIST_i_31[7:0] = input_31[7:0];
assign lut8_DIST_i_32[7:0] = input_32[7:0];
assign lut8_DIST_i_33[7:0] = input_33[7:0];
assign lut8_DIST_i_34[7:0] = input_34[7:0];
assign lut8_DIST_i_35[7:0] = input_35[7:0];
assign lut8_DIST_i_36[7:0] = input_36[7:0];
assign lut8_DIST_i_37[7:0] = input_37[7:0];
assign lut8_DIST_i_38[7:0] = input_38[7:0];
assign lut8_DIST_i_39[7:0] = input_39[7:0];
assign lut8_DIST_i_40[7:0] = input_40[7:0];
assign lut8_DIST_i_41[7:0] = input_41[7:0];
assign lut8_DIST_i_42[7:0] = input_42[7:0];
assign lut8_DIST_i_43[7:0] = input_43[7:0];
assign lut8_DIST_i_44[7:0] = input_44[7:0];
assign lut8_DIST_i_45[7:0] = input_45[7:0];
assign lut8_DIST_i_46[7:0] = input_46[7:0];
assign lut8_DIST_i_47[7:0] = input_47[7:0];
assign lut8_DIST_i_48[7:0] = input_48[7:0];
assign lut8_DIST_i_49[7:0] = input_49[7:0];
assign lut8_DIST_i_50[7:0] = input_50[7:0];
assign lut8_DIST_i_51[7:0] = input_51[7:0];
assign lut8_DIST_i_52[7:0] = input_52[7:0];
assign lut8_DIST_i_53[7:0] = input_53[7:0];
assign lut8_DIST_i_54[7:0] = input_54[7:0];
assign lut8_DIST_i_55[7:0] = input_55[7:0];
assign lut8_DIST_i_56[7:0] = input_56[7:0];
assign lut8_DIST_i_57[7:0] = input_57[7:0];
assign lut8_DIST_i_58[7:0] = input_58[7:0];
assign lut8_DIST_i_59[7:0] = input_59[7:0];
assign lut8_DIST_i_60[7:0] = input_60[7:0];
assign lut8_DIST_i_61[7:0] = input_61[7:0];
assign lut8_DIST_i_62[7:0] = input_62[7:0];
assign lut8_DIST_i_63[7:0] = input_63[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
lut8_DIST lut8_DIST_1 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_2),.lut8_DIST_i_1(lut8_DIST_i_3),.lut8_DIST_o_0(lut8_DIST_o_2),.lut8_DIST_o_1(lut8_DIST_o_3) );
lut8_DIST lut8_DIST_2 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_4),.lut8_DIST_i_1(lut8_DIST_i_5),.lut8_DIST_o_0(lut8_DIST_o_4),.lut8_DIST_o_1(lut8_DIST_o_5) );
lut8_DIST lut8_DIST_3 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_6),.lut8_DIST_i_1(lut8_DIST_i_7),.lut8_DIST_o_0(lut8_DIST_o_6),.lut8_DIST_o_1(lut8_DIST_o_7) );
lut8_DIST lut8_DIST_4 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_8),.lut8_DIST_i_1(lut8_DIST_i_9),.lut8_DIST_o_0(lut8_DIST_o_8),.lut8_DIST_o_1(lut8_DIST_o_9) );
lut8_DIST lut8_DIST_5 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_10),.lut8_DIST_i_1(lut8_DIST_i_11),.lut8_DIST_o_0(lut8_DIST_o_10),.lut8_DIST_o_1(lut8_DIST_o_11) );
lut8_DIST lut8_DIST_6 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_12),.lut8_DIST_i_1(lut8_DIST_i_13),.lut8_DIST_o_0(lut8_DIST_o_12),.lut8_DIST_o_1(lut8_DIST_o_13) );
lut8_DIST lut8_DIST_7 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_14),.lut8_DIST_i_1(lut8_DIST_i_15),.lut8_DIST_o_0(lut8_DIST_o_14),.lut8_DIST_o_1(lut8_DIST_o_15) );
lut8_DIST lut8_DIST_8 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_16),.lut8_DIST_i_1(lut8_DIST_i_17),.lut8_DIST_o_0(lut8_DIST_o_16),.lut8_DIST_o_1(lut8_DIST_o_17) );
lut8_DIST lut8_DIST_9 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_18),.lut8_DIST_i_1(lut8_DIST_i_19),.lut8_DIST_o_0(lut8_DIST_o_18),.lut8_DIST_o_1(lut8_DIST_o_19) );
lut8_DIST lut8_DIST_10 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_20),.lut8_DIST_i_1(lut8_DIST_i_21),.lut8_DIST_o_0(lut8_DIST_o_20),.lut8_DIST_o_1(lut8_DIST_o_21) );
lut8_DIST lut8_DIST_11 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_22),.lut8_DIST_i_1(lut8_DIST_i_23),.lut8_DIST_o_0(lut8_DIST_o_22),.lut8_DIST_o_1(lut8_DIST_o_23) );
lut8_DIST lut8_DIST_12 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_24),.lut8_DIST_i_1(lut8_DIST_i_25),.lut8_DIST_o_0(lut8_DIST_o_24),.lut8_DIST_o_1(lut8_DIST_o_25) );
lut8_DIST lut8_DIST_13 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_26),.lut8_DIST_i_1(lut8_DIST_i_27),.lut8_DIST_o_0(lut8_DIST_o_26),.lut8_DIST_o_1(lut8_DIST_o_27) );
lut8_DIST lut8_DIST_14 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_28),.lut8_DIST_i_1(lut8_DIST_i_29),.lut8_DIST_o_0(lut8_DIST_o_28),.lut8_DIST_o_1(lut8_DIST_o_29) );
lut8_DIST lut8_DIST_15 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_30),.lut8_DIST_i_1(lut8_DIST_i_31),.lut8_DIST_o_0(lut8_DIST_o_30),.lut8_DIST_o_1(lut8_DIST_o_31) );
lut8_DIST lut8_DIST_16 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_32),.lut8_DIST_i_1(lut8_DIST_i_33),.lut8_DIST_o_0(lut8_DIST_o_32),.lut8_DIST_o_1(lut8_DIST_o_33) );
lut8_DIST lut8_DIST_17 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_34),.lut8_DIST_i_1(lut8_DIST_i_35),.lut8_DIST_o_0(lut8_DIST_o_34),.lut8_DIST_o_1(lut8_DIST_o_35) );
lut8_DIST lut8_DIST_18 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_36),.lut8_DIST_i_1(lut8_DIST_i_37),.lut8_DIST_o_0(lut8_DIST_o_36),.lut8_DIST_o_1(lut8_DIST_o_37) );
lut8_DIST lut8_DIST_19 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_38),.lut8_DIST_i_1(lut8_DIST_i_39),.lut8_DIST_o_0(lut8_DIST_o_38),.lut8_DIST_o_1(lut8_DIST_o_39) );
lut8_DIST lut8_DIST_20 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_40),.lut8_DIST_i_1(lut8_DIST_i_41),.lut8_DIST_o_0(lut8_DIST_o_40),.lut8_DIST_o_1(lut8_DIST_o_41) );
lut8_DIST lut8_DIST_21 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_42),.lut8_DIST_i_1(lut8_DIST_i_43),.lut8_DIST_o_0(lut8_DIST_o_42),.lut8_DIST_o_1(lut8_DIST_o_43) );
lut8_DIST lut8_DIST_22 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_44),.lut8_DIST_i_1(lut8_DIST_i_45),.lut8_DIST_o_0(lut8_DIST_o_44),.lut8_DIST_o_1(lut8_DIST_o_45) );
lut8_DIST lut8_DIST_23 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_46),.lut8_DIST_i_1(lut8_DIST_i_47),.lut8_DIST_o_0(lut8_DIST_o_46),.lut8_DIST_o_1(lut8_DIST_o_47) );
lut8_DIST lut8_DIST_24 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_48),.lut8_DIST_i_1(lut8_DIST_i_49),.lut8_DIST_o_0(lut8_DIST_o_48),.lut8_DIST_o_1(lut8_DIST_o_49) );
lut8_DIST lut8_DIST_25 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_50),.lut8_DIST_i_1(lut8_DIST_i_51),.lut8_DIST_o_0(lut8_DIST_o_50),.lut8_DIST_o_1(lut8_DIST_o_51) );
lut8_DIST lut8_DIST_26 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_52),.lut8_DIST_i_1(lut8_DIST_i_53),.lut8_DIST_o_0(lut8_DIST_o_52),.lut8_DIST_o_1(lut8_DIST_o_53) );
lut8_DIST lut8_DIST_27 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_54),.lut8_DIST_i_1(lut8_DIST_i_55),.lut8_DIST_o_0(lut8_DIST_o_54),.lut8_DIST_o_1(lut8_DIST_o_55) );
lut8_DIST lut8_DIST_28 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_56),.lut8_DIST_i_1(lut8_DIST_i_57),.lut8_DIST_o_0(lut8_DIST_o_56),.lut8_DIST_o_1(lut8_DIST_o_57) );
lut8_DIST lut8_DIST_29 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_58),.lut8_DIST_i_1(lut8_DIST_i_59),.lut8_DIST_o_0(lut8_DIST_o_58),.lut8_DIST_o_1(lut8_DIST_o_59) );
lut8_DIST lut8_DIST_30 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_60),.lut8_DIST_i_1(lut8_DIST_i_61),.lut8_DIST_o_0(lut8_DIST_o_60),.lut8_DIST_o_1(lut8_DIST_o_61) );
lut8_DIST lut8_DIST_31 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_62),.lut8_DIST_i_1(lut8_DIST_i_63),.lut8_DIST_o_0(lut8_DIST_o_62),.lut8_DIST_o_1(lut8_DIST_o_63) );
wire [3:0] add_5_i_0_0;
wire [3:0] add_5_i_0_1;
wire [3:0] add_5_i_2_0;
wire [3:0] add_5_i_2_1;
wire [4:0] add_5_o_0_0;
wire [3:0] add_6_i_0_0;
wire [3:0] add_6_i_0_1;
wire [3:0] add_6_i_2_0;
wire [3:0] add_6_i_2_1;
wire [4:0] add_6_o_0_0;
wire [4:0] add_4_i_0_0;
wire [4:0] add_4_i_0_1;
wire [4:0] add_4_i_1_0;
wire [4:0] add_4_i_1_1;
wire [5:0] add_4_o_0_0;
wire [3:0] add_8_i_0_0;
wire [3:0] add_8_i_0_1;
wire [3:0] add_8_i_2_0;
wire [3:0] add_8_i_2_1;
wire [4:0] add_8_o_0_0;
wire [3:0] add_9_i_0_0;
wire [3:0] add_9_i_0_1;
wire [3:0] add_9_i_2_0;
wire [3:0] add_9_i_2_1;
wire [4:0] add_9_o_0_0;
wire [4:0] add_7_i_0_0;
wire [4:0] add_7_i_0_1;
wire [4:0] add_7_i_1_0;
wire [4:0] add_7_i_1_1;
wire [5:0] add_7_o_0_0;
wire [5:0] add_3_i_0_0;
wire [5:0] add_3_i_0_1;
wire [5:0] add_3_i_1_0;
wire [5:0] add_3_i_1_1;
wire [6:0] add_3_o_0_0;
wire [3:0] add_12_i_0_0;
wire [3:0] add_12_i_0_1;
wire [3:0] add_12_i_2_0;
wire [3:0] add_12_i_2_1;
wire [4:0] add_12_o_0_0;
wire [3:0] add_13_i_0_0;
wire [3:0] add_13_i_0_1;
wire [3:0] add_13_i_2_0;
wire [3:0] add_13_i_2_1;
wire [4:0] add_13_o_0_0;
wire [4:0] add_11_i_0_0;
wire [4:0] add_11_i_0_1;
wire [4:0] add_11_i_1_0;
wire [4:0] add_11_i_1_1;
wire [5:0] add_11_o_0_0;
wire [3:0] add_15_i_0_0;
wire [3:0] add_15_i_0_1;
wire [3:0] add_15_i_2_0;
wire [3:0] add_15_i_2_1;
wire [4:0] add_15_o_0_0;
wire [3:0] add_16_i_0_0;
wire [3:0] add_16_i_0_1;
wire [3:0] add_16_i_2_0;
wire [3:0] add_16_i_2_1;
wire [4:0] add_16_o_0_0;
wire [4:0] add_14_i_0_0;
wire [4:0] add_14_i_0_1;
wire [4:0] add_14_i_1_0;
wire [4:0] add_14_i_1_1;
wire [5:0] add_14_o_0_0;
wire [5:0] add_10_i_0_0;
wire [5:0] add_10_i_0_1;
wire [5:0] add_10_i_1_0;
wire [5:0] add_10_i_1_1;
wire [6:0] add_10_o_0_0;
wire [6:0] add_2_i_0_0;
wire [6:0] add_2_i_0_1;
wire [6:0] add_2_i_1_0;
wire [6:0] add_2_i_1_1;
wire [7:0] add_2_o_0_0;
wire [3:0] add_20_i_0_0;
wire [3:0] add_20_i_0_1;
wire [3:0] add_20_i_2_0;
wire [3:0] add_20_i_2_1;
wire [4:0] add_20_o_0_0;
wire [3:0] add_21_i_0_0;
wire [3:0] add_21_i_0_1;
wire [3:0] add_21_i_2_0;
wire [3:0] add_21_i_2_1;
wire [4:0] add_21_o_0_0;
wire [4:0] add_19_i_0_0;
wire [4:0] add_19_i_0_1;
wire [4:0] add_19_i_1_0;
wire [4:0] add_19_i_1_1;
wire [5:0] add_19_o_0_0;
wire [3:0] add_23_i_0_0;
wire [3:0] add_23_i_0_1;
wire [3:0] add_23_i_2_0;
wire [3:0] add_23_i_2_1;
wire [4:0] add_23_o_0_0;
wire [3:0] add_24_i_0_0;
wire [3:0] add_24_i_0_1;
wire [3:0] add_24_i_2_0;
wire [3:0] add_24_i_2_1;
wire [4:0] add_24_o_0_0;
wire [4:0] add_22_i_0_0;
wire [4:0] add_22_i_0_1;
wire [4:0] add_22_i_1_0;
wire [4:0] add_22_i_1_1;
wire [5:0] add_22_o_0_0;
wire [5:0] add_18_i_0_0;
wire [5:0] add_18_i_0_1;
wire [5:0] add_18_i_1_0;
wire [5:0] add_18_i_1_1;
wire [6:0] add_18_o_0_0;
wire [3:0] add_27_i_0_0;
wire [3:0] add_27_i_0_1;
wire [3:0] add_27_i_2_0;
wire [3:0] add_27_i_2_1;
wire [4:0] add_27_o_0_0;
wire [3:0] add_28_i_0_0;
wire [3:0] add_28_i_0_1;
wire [3:0] add_28_i_2_0;
wire [3:0] add_28_i_2_1;
wire [4:0] add_28_o_0_0;
wire [4:0] add_26_i_0_0;
wire [4:0] add_26_i_0_1;
wire [4:0] add_26_i_1_0;
wire [4:0] add_26_i_1_1;
wire [5:0] add_26_o_0_0;
wire [3:0] add_30_i_0_0;
wire [3:0] add_30_i_0_1;
wire [3:0] add_30_i_2_0;
wire [3:0] add_30_i_2_1;
wire [4:0] add_30_o_0_0;
wire [3:0] add_31_i_0_0;
wire [3:0] add_31_i_0_1;
wire [3:0] add_31_i_2_0;
wire [3:0] add_31_i_2_1;
wire [4:0] add_31_o_0_0;
wire [4:0] add_29_i_0_0;
wire [4:0] add_29_i_0_1;
wire [4:0] add_29_i_1_0;
wire [4:0] add_29_i_1_1;
wire [5:0] add_29_o_0_0;
wire [5:0] add_25_i_0_0;
wire [5:0] add_25_i_0_1;
wire [5:0] add_25_i_1_0;
wire [5:0] add_25_i_1_1;
wire [6:0] add_25_o_0_0;
wire [6:0] add_17_i_0_0;
wire [6:0] add_17_i_0_1;
wire [6:0] add_17_i_1_0;
wire [6:0] add_17_i_1_1;
wire [7:0] add_17_o_0_0;
wire [7:0] add_1_i_0_0;
wire [7:0] add_1_i_0_1;
wire [7:0] add_1_i_1_0;
wire [7:0] add_1_i_1_1;
wire [8:0] add_1_o_0_0;
wire [3:0] add_36_i_0_0;
wire [3:0] add_36_i_0_1;
wire [3:0] add_36_i_2_0;
wire [3:0] add_36_i_2_1;
wire [4:0] add_36_o_0_0;
wire [3:0] add_37_i_0_0;
wire [3:0] add_37_i_0_1;
wire [3:0] add_37_i_2_0;
wire [3:0] add_37_i_2_1;
wire [4:0] add_37_o_0_0;
wire [4:0] add_35_i_0_0;
wire [4:0] add_35_i_0_1;
wire [4:0] add_35_i_1_0;
wire [4:0] add_35_i_1_1;
wire [5:0] add_35_o_0_0;
wire [3:0] add_39_i_0_0;
wire [3:0] add_39_i_0_1;
wire [3:0] add_39_i_2_0;
wire [3:0] add_39_i_2_1;
wire [4:0] add_39_o_0_0;
wire [3:0] add_40_i_0_0;
wire [3:0] add_40_i_0_1;
wire [3:0] add_40_i_2_0;
wire [3:0] add_40_i_2_1;
wire [4:0] add_40_o_0_0;
wire [4:0] add_38_i_0_0;
wire [4:0] add_38_i_0_1;
wire [4:0] add_38_i_1_0;
wire [4:0] add_38_i_1_1;
wire [5:0] add_38_o_0_0;
wire [5:0] add_34_i_0_0;
wire [5:0] add_34_i_0_1;
wire [5:0] add_34_i_1_0;
wire [5:0] add_34_i_1_1;
wire [6:0] add_34_o_0_0;
wire [3:0] add_43_i_0_0;
wire [3:0] add_43_i_0_1;
wire [3:0] add_43_i_2_0;
wire [3:0] add_43_i_2_1;
wire [4:0] add_43_o_0_0;
wire [3:0] add_44_i_0_0;
wire [3:0] add_44_i_0_1;
wire [3:0] add_44_i_2_0;
wire [3:0] add_44_i_2_1;
wire [4:0] add_44_o_0_0;
wire [4:0] add_42_i_0_0;
wire [4:0] add_42_i_0_1;
wire [4:0] add_42_i_1_0;
wire [4:0] add_42_i_1_1;
wire [5:0] add_42_o_0_0;
wire [3:0] add_46_i_0_0;
wire [3:0] add_46_i_0_1;
wire [3:0] add_46_i_2_0;
wire [3:0] add_46_i_2_1;
wire [4:0] add_46_o_0_0;
wire [3:0] add_47_i_0_0;
wire [3:0] add_47_i_0_1;
wire [3:0] add_47_i_2_0;
wire [3:0] add_47_i_2_1;
wire [4:0] add_47_o_0_0;
wire [4:0] add_45_i_0_0;
wire [4:0] add_45_i_0_1;
wire [4:0] add_45_i_1_0;
wire [4:0] add_45_i_1_1;
wire [5:0] add_45_o_0_0;
wire [5:0] add_41_i_0_0;
wire [5:0] add_41_i_0_1;
wire [5:0] add_41_i_1_0;
wire [5:0] add_41_i_1_1;
wire [6:0] add_41_o_0_0;
wire [6:0] add_33_i_0_0;
wire [6:0] add_33_i_0_1;
wire [6:0] add_33_i_1_0;
wire [6:0] add_33_i_1_1;
wire [7:0] add_33_o_0_0;
wire [3:0] add_51_i_0_0;
wire [3:0] add_51_i_0_1;
wire [3:0] add_51_i_2_0;
wire [3:0] add_51_i_2_1;
wire [4:0] add_51_o_0_0;
wire [3:0] add_52_i_0_0;
wire [3:0] add_52_i_0_1;
wire [3:0] add_52_i_2_0;
wire [3:0] add_52_i_2_1;
wire [4:0] add_52_o_0_0;
wire [4:0] add_50_i_0_0;
wire [4:0] add_50_i_0_1;
wire [4:0] add_50_i_1_0;
wire [4:0] add_50_i_1_1;
wire [5:0] add_50_o_0_0;
wire [3:0] add_54_i_0_0;
wire [3:0] add_54_i_0_1;
wire [3:0] add_54_i_2_0;
wire [3:0] add_54_i_2_1;
wire [4:0] add_54_o_0_0;
wire [3:0] add_55_i_0_0;
wire [3:0] add_55_i_0_1;
wire [3:0] add_55_i_2_0;
wire [3:0] add_55_i_2_1;
wire [4:0] add_55_o_0_0;
wire [4:0] add_53_i_0_0;
wire [4:0] add_53_i_0_1;
wire [4:0] add_53_i_1_0;
wire [4:0] add_53_i_1_1;
wire [5:0] add_53_o_0_0;
wire [5:0] add_49_i_0_0;
wire [5:0] add_49_i_0_1;
wire [5:0] add_49_i_1_0;
wire [5:0] add_49_i_1_1;
wire [6:0] add_49_o_0_0;
wire [3:0] add_58_i_0_0;
wire [3:0] add_58_i_0_1;
wire [3:0] add_58_i_2_0;
wire [3:0] add_58_i_2_1;
wire [4:0] add_58_o_0_0;
wire [3:0] add_59_i_0_0;
wire [3:0] add_59_i_0_1;
wire [3:0] add_59_i_2_0;
wire [3:0] add_59_i_2_1;
wire [4:0] add_59_o_0_0;
wire [4:0] add_57_i_0_0;
wire [4:0] add_57_i_0_1;
wire [4:0] add_57_i_1_0;
wire [4:0] add_57_i_1_1;
wire [5:0] add_57_o_0_0;
wire [3:0] add_61_i_0_0;
wire [3:0] add_61_i_0_1;
wire [3:0] add_61_i_2_0;
wire [3:0] add_61_i_2_1;
wire [4:0] add_61_o_0_0;
wire [3:0] add_62_i_0_0;
wire [3:0] add_62_i_0_1;
wire [3:0] add_62_i_2_0;
wire [3:0] add_62_i_2_1;
wire [4:0] add_62_o_0_0;
wire [4:0] add_60_i_0_0;
wire [4:0] add_60_i_0_1;
wire [4:0] add_60_i_1_0;
wire [4:0] add_60_i_1_1;
wire [5:0] add_60_o_0_0;
wire [5:0] add_56_i_0_0;
wire [5:0] add_56_i_0_1;
wire [5:0] add_56_i_1_0;
wire [5:0] add_56_i_1_1;
wire [6:0] add_56_o_0_0;
wire [6:0] add_48_i_0_0;
wire [6:0] add_48_i_0_1;
wire [6:0] add_48_i_1_0;
wire [6:0] add_48_i_1_1;
wire [7:0] add_48_o_0_0;
wire [7:0] add_32_i_0_0;
wire [7:0] add_32_i_0_1;
wire [7:0] add_32_i_1_0;
wire [7:0] add_32_i_1_1;
wire [8:0] add_32_o_0_0;
wire [8:0] add_0_i_0_0;
wire [8:0] add_0_i_0_1;
wire [8:0] add_0_i_1_0;
wire [8:0] add_0_i_1_1;
wire [9:0] add_0_o_0_0;
wire [9:0] add_0_o_1_0;
assign add_5_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_5_i_0_1[3:0] = lut8_DIST_o_1[3:0];
assign add_6_i_0_0[3:0] = lut8_DIST_o_2[3:0];
assign add_6_i_0_1[3:0] = lut8_DIST_o_3[3:0];
assign add_8_i_0_0[3:0] = lut8_DIST_o_4[3:0];
assign add_8_i_0_1[3:0] = lut8_DIST_o_5[3:0];
assign add_9_i_0_0[3:0] = lut8_DIST_o_6[3:0];
assign add_9_i_0_1[3:0] = lut8_DIST_o_7[3:0];
assign add_12_i_0_0[3:0] = lut8_DIST_o_8[3:0];
assign add_12_i_0_1[3:0] = lut8_DIST_o_9[3:0];
assign add_13_i_0_0[3:0] = lut8_DIST_o_10[3:0];
assign add_13_i_0_1[3:0] = lut8_DIST_o_11[3:0];
assign add_15_i_0_0[3:0] = lut8_DIST_o_12[3:0];
assign add_15_i_0_1[3:0] = lut8_DIST_o_13[3:0];
assign add_16_i_0_0[3:0] = lut8_DIST_o_14[3:0];
assign add_16_i_0_1[3:0] = lut8_DIST_o_15[3:0];
assign add_20_i_0_0[3:0] = lut8_DIST_o_16[3:0];
assign add_20_i_0_1[3:0] = lut8_DIST_o_17[3:0];
assign add_21_i_0_0[3:0] = lut8_DIST_o_18[3:0];
assign add_21_i_0_1[3:0] = lut8_DIST_o_19[3:0];
assign add_23_i_0_0[3:0] = lut8_DIST_o_20[3:0];
assign add_23_i_0_1[3:0] = lut8_DIST_o_21[3:0];
assign add_24_i_0_0[3:0] = lut8_DIST_o_22[3:0];
assign add_24_i_0_1[3:0] = lut8_DIST_o_23[3:0];
assign add_27_i_0_0[3:0] = lut8_DIST_o_24[3:0];
assign add_27_i_0_1[3:0] = lut8_DIST_o_25[3:0];
assign add_28_i_0_0[3:0] = lut8_DIST_o_26[3:0];
assign add_28_i_0_1[3:0] = lut8_DIST_o_27[3:0];
assign add_30_i_0_0[3:0] = lut8_DIST_o_28[3:0];
assign add_30_i_0_1[3:0] = lut8_DIST_o_29[3:0];
assign add_31_i_0_0[3:0] = lut8_DIST_o_30[3:0];
assign add_31_i_0_1[3:0] = lut8_DIST_o_31[3:0];
assign add_36_i_0_0[3:0] = lut8_DIST_o_32[3:0];
assign add_36_i_0_1[3:0] = lut8_DIST_o_33[3:0];
assign add_37_i_0_0[3:0] = lut8_DIST_o_34[3:0];
assign add_37_i_0_1[3:0] = lut8_DIST_o_35[3:0];
assign add_39_i_0_0[3:0] = lut8_DIST_o_36[3:0];
assign add_39_i_0_1[3:0] = lut8_DIST_o_37[3:0];
assign add_40_i_0_0[3:0] = lut8_DIST_o_38[3:0];
assign add_40_i_0_1[3:0] = lut8_DIST_o_39[3:0];
assign add_43_i_0_0[3:0] = lut8_DIST_o_40[3:0];
assign add_43_i_0_1[3:0] = lut8_DIST_o_41[3:0];
assign add_44_i_0_0[3:0] = lut8_DIST_o_42[3:0];
assign add_44_i_0_1[3:0] = lut8_DIST_o_43[3:0];
assign add_46_i_0_0[3:0] = lut8_DIST_o_44[3:0];
assign add_46_i_0_1[3:0] = lut8_DIST_o_45[3:0];
assign add_47_i_0_0[3:0] = lut8_DIST_o_46[3:0];
assign add_47_i_0_1[3:0] = lut8_DIST_o_47[3:0];
assign add_51_i_0_0[3:0] = lut8_DIST_o_48[3:0];
assign add_51_i_0_1[3:0] = lut8_DIST_o_49[3:0];
assign add_52_i_0_0[3:0] = lut8_DIST_o_50[3:0];
assign add_52_i_0_1[3:0] = lut8_DIST_o_51[3:0];
assign add_54_i_0_0[3:0] = lut8_DIST_o_52[3:0];
assign add_54_i_0_1[3:0] = lut8_DIST_o_53[3:0];
assign add_55_i_0_0[3:0] = lut8_DIST_o_54[3:0];
assign add_55_i_0_1[3:0] = lut8_DIST_o_55[3:0];
assign add_58_i_0_0[3:0] = lut8_DIST_o_56[3:0];
assign add_58_i_0_1[3:0] = lut8_DIST_o_57[3:0];
assign add_59_i_0_0[3:0] = lut8_DIST_o_58[3:0];
assign add_59_i_0_1[3:0] = lut8_DIST_o_59[3:0];
assign add_61_i_0_0[3:0] = lut8_DIST_o_60[3:0];
assign add_61_i_0_1[3:0] = lut8_DIST_o_61[3:0];
assign add_62_i_0_0[3:0] = lut8_DIST_o_62[3:0];
assign add_62_i_0_1[3:0] = lut8_DIST_o_63[3:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_3_i_0_0[5:0] = add_4_o_0_0[5:0];
assign add_7_i_0_0[4:0] = add_8_o_0_0[4:0];
assign add_7_i_0_1[4:0] = add_9_o_0_0[4:0];
assign add_3_i_0_1[5:0] = add_7_o_0_0[5:0];
assign add_2_i_0_0[6:0] = add_3_o_0_0[6:0];
assign add_11_i_0_0[4:0] = add_12_o_0_0[4:0];
assign add_11_i_0_1[4:0] = add_13_o_0_0[4:0];
assign add_10_i_0_0[5:0] = add_11_o_0_0[5:0];
assign add_14_i_0_0[4:0] = add_15_o_0_0[4:0];
assign add_14_i_0_1[4:0] = add_16_o_0_0[4:0];
assign add_10_i_0_1[5:0] = add_14_o_0_0[5:0];
assign add_2_i_0_1[6:0] = add_10_o_0_0[6:0];
assign add_1_i_0_0[7:0] = add_2_o_0_0[7:0];
assign add_19_i_0_0[4:0] = add_20_o_0_0[4:0];
assign add_19_i_0_1[4:0] = add_21_o_0_0[4:0];
assign add_18_i_0_0[5:0] = add_19_o_0_0[5:0];
assign add_22_i_0_0[4:0] = add_23_o_0_0[4:0];
assign add_22_i_0_1[4:0] = add_24_o_0_0[4:0];
assign add_18_i_0_1[5:0] = add_22_o_0_0[5:0];
assign add_17_i_0_0[6:0] = add_18_o_0_0[6:0];
assign add_26_i_0_0[4:0] = add_27_o_0_0[4:0];
assign add_26_i_0_1[4:0] = add_28_o_0_0[4:0];
assign add_25_i_0_0[5:0] = add_26_o_0_0[5:0];
assign add_29_i_0_0[4:0] = add_30_o_0_0[4:0];
assign add_29_i_0_1[4:0] = add_31_o_0_0[4:0];
assign add_25_i_0_1[5:0] = add_29_o_0_0[5:0];
assign add_17_i_0_1[6:0] = add_25_o_0_0[6:0];
assign add_1_i_0_1[7:0] = add_17_o_0_0[7:0];
assign add_0_i_0_0[8:0] = add_1_o_0_0[8:0];
assign add_35_i_0_0[4:0] = add_36_o_0_0[4:0];
assign add_35_i_0_1[4:0] = add_37_o_0_0[4:0];
assign add_34_i_0_0[5:0] = add_35_o_0_0[5:0];
assign add_38_i_0_0[4:0] = add_39_o_0_0[4:0];
assign add_38_i_0_1[4:0] = add_40_o_0_0[4:0];
assign add_34_i_0_1[5:0] = add_38_o_0_0[5:0];
assign add_33_i_0_0[6:0] = add_34_o_0_0[6:0];
assign add_42_i_0_0[4:0] = add_43_o_0_0[4:0];
assign add_42_i_0_1[4:0] = add_44_o_0_0[4:0];
assign add_41_i_0_0[5:0] = add_42_o_0_0[5:0];
assign add_45_i_0_0[4:0] = add_46_o_0_0[4:0];
assign add_45_i_0_1[4:0] = add_47_o_0_0[4:0];
assign add_41_i_0_1[5:0] = add_45_o_0_0[5:0];
assign add_33_i_0_1[6:0] = add_41_o_0_0[6:0];
assign add_32_i_0_0[7:0] = add_33_o_0_0[7:0];
assign add_50_i_0_0[4:0] = add_51_o_0_0[4:0];
assign add_50_i_0_1[4:0] = add_52_o_0_0[4:0];
assign add_49_i_0_0[5:0] = add_50_o_0_0[5:0];
assign add_53_i_0_0[4:0] = add_54_o_0_0[4:0];
assign add_53_i_0_1[4:0] = add_55_o_0_0[4:0];
assign add_49_i_0_1[5:0] = add_53_o_0_0[5:0];
assign add_48_i_0_0[6:0] = add_49_o_0_0[6:0];
assign add_57_i_0_0[4:0] = add_58_o_0_0[4:0];
assign add_57_i_0_1[4:0] = add_59_o_0_0[4:0];
assign add_56_i_0_0[5:0] = add_57_o_0_0[5:0];
assign add_60_i_0_0[4:0] = add_61_o_0_0[4:0];
assign add_60_i_0_1[4:0] = add_62_o_0_0[4:0];
assign add_56_i_0_1[5:0] = add_60_o_0_0[5:0];
assign add_48_i_0_1[6:0] = add_56_o_0_0[6:0];
assign add_32_i_0_1[7:0] = add_48_o_0_0[7:0];
assign add_0_i_0_1[8:0] = add_32_o_0_0[8:0];
du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(4, 2) du_8_0 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_0),.din(add_8_i_0_0));
du #(4, 2) du_8_1 (.clk(clk), .stall(1'b0), .dout(add_8_i_2_1),.din(add_8_i_0_1));

assign add_8_o_0_0 = add_8_i_2_0 + add_8_i_2_1;

du #(4, 2) du_9_0 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_0),.din(add_9_i_0_0));
du #(4, 2) du_9_1 (.clk(clk), .stall(1'b0), .dout(add_9_i_2_1),.din(add_9_i_0_1));

assign add_9_o_0_0 = add_9_i_2_0 + add_9_i_2_1;

du #(5, 1) du_7_0 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_0),.din(add_7_i_0_0));
du #(5, 1) du_7_1 (.clk(clk), .stall(1'b0), .dout(add_7_i_1_1),.din(add_7_i_0_1));

assign add_7_o_0_0 = add_7_i_1_0 + add_7_i_1_1;

du #(6, 1) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_0),.din(add_3_i_0_0));
du #(6, 1) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_1_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_1_0 + add_3_i_1_1;

du #(4, 2) du_12_0 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_0),.din(add_12_i_0_0));
du #(4, 2) du_12_1 (.clk(clk), .stall(1'b0), .dout(add_12_i_2_1),.din(add_12_i_0_1));

assign add_12_o_0_0 = add_12_i_2_0 + add_12_i_2_1;

du #(4, 2) du_13_0 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_0),.din(add_13_i_0_0));
du #(4, 2) du_13_1 (.clk(clk), .stall(1'b0), .dout(add_13_i_2_1),.din(add_13_i_0_1));

assign add_13_o_0_0 = add_13_i_2_0 + add_13_i_2_1;

du #(5, 1) du_11_0 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_0),.din(add_11_i_0_0));
du #(5, 1) du_11_1 (.clk(clk), .stall(1'b0), .dout(add_11_i_1_1),.din(add_11_i_0_1));

assign add_11_o_0_0 = add_11_i_1_0 + add_11_i_1_1;

du #(4, 2) du_15_0 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_0),.din(add_15_i_0_0));
du #(4, 2) du_15_1 (.clk(clk), .stall(1'b0), .dout(add_15_i_2_1),.din(add_15_i_0_1));

assign add_15_o_0_0 = add_15_i_2_0 + add_15_i_2_1;

du #(4, 2) du_16_0 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_0),.din(add_16_i_0_0));
du #(4, 2) du_16_1 (.clk(clk), .stall(1'b0), .dout(add_16_i_2_1),.din(add_16_i_0_1));

assign add_16_o_0_0 = add_16_i_2_0 + add_16_i_2_1;

du #(5, 1) du_14_0 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_0),.din(add_14_i_0_0));
du #(5, 1) du_14_1 (.clk(clk), .stall(1'b0), .dout(add_14_i_1_1),.din(add_14_i_0_1));

assign add_14_o_0_0 = add_14_i_1_0 + add_14_i_1_1;

du #(6, 1) du_10_0 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_0),.din(add_10_i_0_0));
du #(6, 1) du_10_1 (.clk(clk), .stall(1'b0), .dout(add_10_i_1_1),.din(add_10_i_0_1));

assign add_10_o_0_0 = add_10_i_1_0 + add_10_i_1_1;

du #(7, 1) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_0),.din(add_2_i_0_0));
du #(7, 1) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_1_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_1_0 + add_2_i_1_1;

du #(4, 2) du_20_0 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_0),.din(add_20_i_0_0));
du #(4, 2) du_20_1 (.clk(clk), .stall(1'b0), .dout(add_20_i_2_1),.din(add_20_i_0_1));

assign add_20_o_0_0 = add_20_i_2_0 + add_20_i_2_1;

du #(4, 2) du_21_0 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_0),.din(add_21_i_0_0));
du #(4, 2) du_21_1 (.clk(clk), .stall(1'b0), .dout(add_21_i_2_1),.din(add_21_i_0_1));

assign add_21_o_0_0 = add_21_i_2_0 + add_21_i_2_1;

du #(5, 1) du_19_0 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_0),.din(add_19_i_0_0));
du #(5, 1) du_19_1 (.clk(clk), .stall(1'b0), .dout(add_19_i_1_1),.din(add_19_i_0_1));

assign add_19_o_0_0 = add_19_i_1_0 + add_19_i_1_1;

du #(4, 2) du_23_0 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_0),.din(add_23_i_0_0));
du #(4, 2) du_23_1 (.clk(clk), .stall(1'b0), .dout(add_23_i_2_1),.din(add_23_i_0_1));

assign add_23_o_0_0 = add_23_i_2_0 + add_23_i_2_1;

du #(4, 2) du_24_0 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_0),.din(add_24_i_0_0));
du #(4, 2) du_24_1 (.clk(clk), .stall(1'b0), .dout(add_24_i_2_1),.din(add_24_i_0_1));

assign add_24_o_0_0 = add_24_i_2_0 + add_24_i_2_1;

du #(5, 1) du_22_0 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_0),.din(add_22_i_0_0));
du #(5, 1) du_22_1 (.clk(clk), .stall(1'b0), .dout(add_22_i_1_1),.din(add_22_i_0_1));

assign add_22_o_0_0 = add_22_i_1_0 + add_22_i_1_1;

du #(6, 1) du_18_0 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_0),.din(add_18_i_0_0));
du #(6, 1) du_18_1 (.clk(clk), .stall(1'b0), .dout(add_18_i_1_1),.din(add_18_i_0_1));

assign add_18_o_0_0 = add_18_i_1_0 + add_18_i_1_1;

du #(4, 2) du_27_0 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_0),.din(add_27_i_0_0));
du #(4, 2) du_27_1 (.clk(clk), .stall(1'b0), .dout(add_27_i_2_1),.din(add_27_i_0_1));

assign add_27_o_0_0 = add_27_i_2_0 + add_27_i_2_1;

du #(4, 2) du_28_0 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_0),.din(add_28_i_0_0));
du #(4, 2) du_28_1 (.clk(clk), .stall(1'b0), .dout(add_28_i_2_1),.din(add_28_i_0_1));

assign add_28_o_0_0 = add_28_i_2_0 + add_28_i_2_1;

du #(5, 1) du_26_0 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_0),.din(add_26_i_0_0));
du #(5, 1) du_26_1 (.clk(clk), .stall(1'b0), .dout(add_26_i_1_1),.din(add_26_i_0_1));

assign add_26_o_0_0 = add_26_i_1_0 + add_26_i_1_1;

du #(4, 2) du_30_0 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_0),.din(add_30_i_0_0));
du #(4, 2) du_30_1 (.clk(clk), .stall(1'b0), .dout(add_30_i_2_1),.din(add_30_i_0_1));

assign add_30_o_0_0 = add_30_i_2_0 + add_30_i_2_1;

du #(4, 2) du_31_0 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_0),.din(add_31_i_0_0));
du #(4, 2) du_31_1 (.clk(clk), .stall(1'b0), .dout(add_31_i_2_1),.din(add_31_i_0_1));

assign add_31_o_0_0 = add_31_i_2_0 + add_31_i_2_1;

du #(5, 1) du_29_0 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_0),.din(add_29_i_0_0));
du #(5, 1) du_29_1 (.clk(clk), .stall(1'b0), .dout(add_29_i_1_1),.din(add_29_i_0_1));

assign add_29_o_0_0 = add_29_i_1_0 + add_29_i_1_1;

du #(6, 1) du_25_0 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_0),.din(add_25_i_0_0));
du #(6, 1) du_25_1 (.clk(clk), .stall(1'b0), .dout(add_25_i_1_1),.din(add_25_i_0_1));

assign add_25_o_0_0 = add_25_i_1_0 + add_25_i_1_1;

du #(7, 1) du_17_0 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_0),.din(add_17_i_0_0));
du #(7, 1) du_17_1 (.clk(clk), .stall(1'b0), .dout(add_17_i_1_1),.din(add_17_i_0_1));

assign add_17_o_0_0 = add_17_i_1_0 + add_17_i_1_1;

du #(8, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(8, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_36_0 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_0),.din(add_36_i_0_0));
du #(4, 2) du_36_1 (.clk(clk), .stall(1'b0), .dout(add_36_i_2_1),.din(add_36_i_0_1));

assign add_36_o_0_0 = add_36_i_2_0 + add_36_i_2_1;

du #(4, 2) du_37_0 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_0),.din(add_37_i_0_0));
du #(4, 2) du_37_1 (.clk(clk), .stall(1'b0), .dout(add_37_i_2_1),.din(add_37_i_0_1));

assign add_37_o_0_0 = add_37_i_2_0 + add_37_i_2_1;

du #(5, 1) du_35_0 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_0),.din(add_35_i_0_0));
du #(5, 1) du_35_1 (.clk(clk), .stall(1'b0), .dout(add_35_i_1_1),.din(add_35_i_0_1));

assign add_35_o_0_0 = add_35_i_1_0 + add_35_i_1_1;

du #(4, 2) du_39_0 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_0),.din(add_39_i_0_0));
du #(4, 2) du_39_1 (.clk(clk), .stall(1'b0), .dout(add_39_i_2_1),.din(add_39_i_0_1));

assign add_39_o_0_0 = add_39_i_2_0 + add_39_i_2_1;

du #(4, 2) du_40_0 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_0),.din(add_40_i_0_0));
du #(4, 2) du_40_1 (.clk(clk), .stall(1'b0), .dout(add_40_i_2_1),.din(add_40_i_0_1));

assign add_40_o_0_0 = add_40_i_2_0 + add_40_i_2_1;

du #(5, 1) du_38_0 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_0),.din(add_38_i_0_0));
du #(5, 1) du_38_1 (.clk(clk), .stall(1'b0), .dout(add_38_i_1_1),.din(add_38_i_0_1));

assign add_38_o_0_0 = add_38_i_1_0 + add_38_i_1_1;

du #(6, 1) du_34_0 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_0),.din(add_34_i_0_0));
du #(6, 1) du_34_1 (.clk(clk), .stall(1'b0), .dout(add_34_i_1_1),.din(add_34_i_0_1));

assign add_34_o_0_0 = add_34_i_1_0 + add_34_i_1_1;

du #(4, 2) du_43_0 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_0),.din(add_43_i_0_0));
du #(4, 2) du_43_1 (.clk(clk), .stall(1'b0), .dout(add_43_i_2_1),.din(add_43_i_0_1));

assign add_43_o_0_0 = add_43_i_2_0 + add_43_i_2_1;

du #(4, 2) du_44_0 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_0),.din(add_44_i_0_0));
du #(4, 2) du_44_1 (.clk(clk), .stall(1'b0), .dout(add_44_i_2_1),.din(add_44_i_0_1));

assign add_44_o_0_0 = add_44_i_2_0 + add_44_i_2_1;

du #(5, 1) du_42_0 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_0),.din(add_42_i_0_0));
du #(5, 1) du_42_1 (.clk(clk), .stall(1'b0), .dout(add_42_i_1_1),.din(add_42_i_0_1));

assign add_42_o_0_0 = add_42_i_1_0 + add_42_i_1_1;

du #(4, 2) du_46_0 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_0),.din(add_46_i_0_0));
du #(4, 2) du_46_1 (.clk(clk), .stall(1'b0), .dout(add_46_i_2_1),.din(add_46_i_0_1));

assign add_46_o_0_0 = add_46_i_2_0 + add_46_i_2_1;

du #(4, 2) du_47_0 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_0),.din(add_47_i_0_0));
du #(4, 2) du_47_1 (.clk(clk), .stall(1'b0), .dout(add_47_i_2_1),.din(add_47_i_0_1));

assign add_47_o_0_0 = add_47_i_2_0 + add_47_i_2_1;

du #(5, 1) du_45_0 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_0),.din(add_45_i_0_0));
du #(5, 1) du_45_1 (.clk(clk), .stall(1'b0), .dout(add_45_i_1_1),.din(add_45_i_0_1));

assign add_45_o_0_0 = add_45_i_1_0 + add_45_i_1_1;

du #(6, 1) du_41_0 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_0),.din(add_41_i_0_0));
du #(6, 1) du_41_1 (.clk(clk), .stall(1'b0), .dout(add_41_i_1_1),.din(add_41_i_0_1));

assign add_41_o_0_0 = add_41_i_1_0 + add_41_i_1_1;

du #(7, 1) du_33_0 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_0),.din(add_33_i_0_0));
du #(7, 1) du_33_1 (.clk(clk), .stall(1'b0), .dout(add_33_i_1_1),.din(add_33_i_0_1));

assign add_33_o_0_0 = add_33_i_1_0 + add_33_i_1_1;

du #(4, 2) du_51_0 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_0),.din(add_51_i_0_0));
du #(4, 2) du_51_1 (.clk(clk), .stall(1'b0), .dout(add_51_i_2_1),.din(add_51_i_0_1));

assign add_51_o_0_0 = add_51_i_2_0 + add_51_i_2_1;

du #(4, 2) du_52_0 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_0),.din(add_52_i_0_0));
du #(4, 2) du_52_1 (.clk(clk), .stall(1'b0), .dout(add_52_i_2_1),.din(add_52_i_0_1));

assign add_52_o_0_0 = add_52_i_2_0 + add_52_i_2_1;

du #(5, 1) du_50_0 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_0),.din(add_50_i_0_0));
du #(5, 1) du_50_1 (.clk(clk), .stall(1'b0), .dout(add_50_i_1_1),.din(add_50_i_0_1));

assign add_50_o_0_0 = add_50_i_1_0 + add_50_i_1_1;

du #(4, 2) du_54_0 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_0),.din(add_54_i_0_0));
du #(4, 2) du_54_1 (.clk(clk), .stall(1'b0), .dout(add_54_i_2_1),.din(add_54_i_0_1));

assign add_54_o_0_0 = add_54_i_2_0 + add_54_i_2_1;

du #(4, 2) du_55_0 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_0),.din(add_55_i_0_0));
du #(4, 2) du_55_1 (.clk(clk), .stall(1'b0), .dout(add_55_i_2_1),.din(add_55_i_0_1));

assign add_55_o_0_0 = add_55_i_2_0 + add_55_i_2_1;

du #(5, 1) du_53_0 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_0),.din(add_53_i_0_0));
du #(5, 1) du_53_1 (.clk(clk), .stall(1'b0), .dout(add_53_i_1_1),.din(add_53_i_0_1));

assign add_53_o_0_0 = add_53_i_1_0 + add_53_i_1_1;

du #(6, 1) du_49_0 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_0),.din(add_49_i_0_0));
du #(6, 1) du_49_1 (.clk(clk), .stall(1'b0), .dout(add_49_i_1_1),.din(add_49_i_0_1));

assign add_49_o_0_0 = add_49_i_1_0 + add_49_i_1_1;

du #(4, 2) du_58_0 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_0),.din(add_58_i_0_0));
du #(4, 2) du_58_1 (.clk(clk), .stall(1'b0), .dout(add_58_i_2_1),.din(add_58_i_0_1));

assign add_58_o_0_0 = add_58_i_2_0 + add_58_i_2_1;

du #(4, 2) du_59_0 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_0),.din(add_59_i_0_0));
du #(4, 2) du_59_1 (.clk(clk), .stall(1'b0), .dout(add_59_i_2_1),.din(add_59_i_0_1));

assign add_59_o_0_0 = add_59_i_2_0 + add_59_i_2_1;

du #(5, 1) du_57_0 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_0),.din(add_57_i_0_0));
du #(5, 1) du_57_1 (.clk(clk), .stall(1'b0), .dout(add_57_i_1_1),.din(add_57_i_0_1));

assign add_57_o_0_0 = add_57_i_1_0 + add_57_i_1_1;

du #(4, 2) du_61_0 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_0),.din(add_61_i_0_0));
du #(4, 2) du_61_1 (.clk(clk), .stall(1'b0), .dout(add_61_i_2_1),.din(add_61_i_0_1));

assign add_61_o_0_0 = add_61_i_2_0 + add_61_i_2_1;

du #(4, 2) du_62_0 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_0),.din(add_62_i_0_0));
du #(4, 2) du_62_1 (.clk(clk), .stall(1'b0), .dout(add_62_i_2_1),.din(add_62_i_0_1));

assign add_62_o_0_0 = add_62_i_2_0 + add_62_i_2_1;

du #(5, 1) du_60_0 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_0),.din(add_60_i_0_0));
du #(5, 1) du_60_1 (.clk(clk), .stall(1'b0), .dout(add_60_i_1_1),.din(add_60_i_0_1));

assign add_60_o_0_0 = add_60_i_1_0 + add_60_i_1_1;

du #(6, 1) du_56_0 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_0),.din(add_56_i_0_0));
du #(6, 1) du_56_1 (.clk(clk), .stall(1'b0), .dout(add_56_i_1_1),.din(add_56_i_0_1));

assign add_56_o_0_0 = add_56_i_1_0 + add_56_i_1_1;

du #(7, 1) du_48_0 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_0),.din(add_48_i_0_0));
du #(7, 1) du_48_1 (.clk(clk), .stall(1'b0), .dout(add_48_i_1_1),.din(add_48_i_0_1));

assign add_48_o_0_0 = add_48_i_1_0 + add_48_i_1_1;

du #(8, 1) du_32_0 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_0),.din(add_32_i_0_0));
du #(8, 1) du_32_1 (.clk(clk), .stall(1'b0), .dout(add_32_i_1_1),.din(add_32_i_0_1));

assign add_32_o_0_0 = add_32_i_1_0 + add_32_i_1_1;

du #(9, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(9, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(10, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

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
	parameter IPORT_WIDTH = 512,					
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
	wire [9:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_0 #(IPORT_WIDTH) l_inst (.vld_in(vld_in), .val_i(val_i), .val_o(t_l_val_o));					
	assign val_i_t = t_l_val_o;				   	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_512_9 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(10, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1			#(					
	parameter IPORT_WIDTH = 512,					
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
	wire [9:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "no" *) popcnt_dist_nodsp_512_9 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(10, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1_special			#(					
	parameter IPORT_WIDTH = 512,					
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
	wire [9:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_512_9 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(10, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule



module pw_sim_8_1_512_9 (
input clk,
input rst,
input vld_in,
input	[511:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[511:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[511:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[511:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[511:0] val_i_x_3,
output	[31:0] x_cnt_3,
input	[511:0] val_i_x_4,
output	[31:0] x_cnt_4,
input	[511:0] val_i_x_5,
output	[31:0] x_cnt_5,
input	[511:0] val_i_x_6,
output	[31:0] x_cnt_6,
input	[511:0] val_i_x_7,
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

wire [511:0] val_i_y_t_0;

wire [511:0] val_i_x_t_0;
wire [511:0] val_i_x_t_1;
wire [511:0] val_i_x_t_2;
wire [511:0] val_i_x_t_3;
wire [511:0] val_i_x_t_4;
wire [511:0] val_i_x_t_5;
wire [511:0] val_i_x_t_6;
wire [511:0] val_i_x_t_7;

gblock_0 #(512, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(512, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(512, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(512, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));
gblock_0 #(512, 32) gblock_0_x_4 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_4), .val_o(x_cnt_4), .val_i_t(val_i_x_t_4));
gblock_0 #(512, 32) gblock_0_x_5 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_5), .val_o(x_cnt_5), .val_i_t(val_i_x_t_5));
gblock_0 #(512, 32) gblock_0_x_6 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_6), .val_o(x_cnt_6), .val_i_t(val_i_x_t_6));
gblock_0 #(512, 32) gblock_0_x_7 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_7), .val_o(x_cnt_7), .val_i_t(val_i_x_t_7));

gblock_0 #(512, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1 #(512, 32) gblock_1_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1 #(512, 32) gblock_1_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1 #(512, 32) gblock_1_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1 #(512, 32) gblock_1_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));
gblock_1 #(512, 32) gblock_1_0_4 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_4), .val_o(xy_cnt_0_4));
gblock_1 #(512, 32) gblock_1_0_5 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_5), .val_o(xy_cnt_0_5));
gblock_1 #(512, 32) gblock_1_0_6 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_6), .val_o(xy_cnt_0_6));
gblock_1 #(512, 32) gblock_1_0_7 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_7), .val_o(xy_cnt_0_7));


endmodule


module pw_sim_special_8_1_512_9 (
input clk,
input rst,
input vld_in,
input	[511:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[511:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[511:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[511:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[511:0] val_i_x_3,
output	[31:0] x_cnt_3,
input	[511:0] val_i_x_4,
output	[31:0] x_cnt_4,
input	[511:0] val_i_x_5,
output	[31:0] x_cnt_5,
input	[511:0] val_i_x_6,
output	[31:0] x_cnt_6,
input	[511:0] val_i_x_7,
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

wire [511:0] val_i_y_t_0;

wire [511:0] val_i_x_t_0;
wire [511:0] val_i_x_t_1;
wire [511:0] val_i_x_t_2;
wire [511:0] val_i_x_t_3;
wire [511:0] val_i_x_t_4;
wire [511:0] val_i_x_t_5;
wire [511:0] val_i_x_t_6;
wire [511:0] val_i_x_t_7;

gblock_0 #(512, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(512, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(512, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(512, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));
gblock_0 #(512, 32) gblock_0_x_4 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_4), .val_o(x_cnt_4), .val_i_t(val_i_x_t_4));
gblock_0 #(512, 32) gblock_0_x_5 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_5), .val_o(x_cnt_5), .val_i_t(val_i_x_t_5));
gblock_0 #(512, 32) gblock_0_x_6 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_6), .val_o(x_cnt_6), .val_i_t(val_i_x_t_6));
gblock_0 #(512, 32) gblock_0_x_7 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_7), .val_o(x_cnt_7), .val_i_t(val_i_x_t_7));

gblock_0 #(512, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1_special #(512, 32) gblock_1_special_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1_special #(512, 32) gblock_1_special_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1_special #(512, 32) gblock_1_special_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1_special #(512, 32) gblock_1_special_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));
gblock_1_special #(512, 32) gblock_1_special_0_4 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_4), .val_o(xy_cnt_0_4));
gblock_1_special #(512, 32) gblock_1_special_0_5 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_5), .val_o(xy_cnt_0_5));
gblock_1_special #(512, 32) gblock_1_special_0_6 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_6), .val_o(xy_cnt_0_6));
gblock_1_special #(512, 32) gblock_1_special_0_7 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_7), .val_o(xy_cnt_0_7));


endmodule


module pw_coefficients_wrapper_8_1_512_9 (
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
input	[511:0] val_i_y_0,
input	[511:0] val_i_x_0,
input	[511:0] val_i_x_1,
input	[511:0] val_i_x_2,
input	[511:0] val_i_x_3,
input	[511:0] val_i_x_4,
input	[511:0] val_i_x_5,
input	[511:0] val_i_x_6,
input	[511:0] val_i_x_7,
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
du #(1, 9) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
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
pw_sim_8_1_512_9 pw_sim_8_1_512_9_0(
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
pw_coefficients_wrapper_8_1_512_9 pw_coefficients_wrapper_8_1_512_9_0(
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
input	[511:0] val_i_y_0,
input	[511:0] val_i_x_0,
input	[511:0] val_i_x_1,
input	[511:0] val_i_x_2,
input	[511:0] val_i_x_3,
input	[511:0] val_i_x_4,
input	[511:0] val_i_x_5,
input	[511:0] val_i_x_6,
input	[511:0] val_i_x_7,
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
du #(1, 9) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
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
pw_sim_special_8_1_512_9 pw_sim_special_8_1_512_9_0(
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
pw_coefficients_wrapper_8_1_512_9 pw_coefficients_wrapper_8_1_512_9_0(
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
  input [511:0]wrdin_q,
  input [7:0]rdaddr_a,
  output [511:0]rddout_a,
  input [7:0]rdaddr_b,
  output [511:0]rddout_b
);

wire wren_a;
assign wren_a = (wren_q==1) & (wrid_q==bank_id);
wire [7:0] addr_a;
assign addr_a = (wren_a==1)?wraddr_q:rdaddr_a;
bram_tdp #(512,8) umem ( .a_clk(clk), .a_wr(wren_a), .a_addr(addr_a), .a_din(wrdin_q), .a_dout(rddout_a), 
  .b_clk(clk), .b_wr(1'b0), .b_addr(rdaddr_b), .b_din(512'b0), .b_dout(rddout_b)); 
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
input	[511:0] wrdin_q,
input	[7:0] rdaddr_a,
input	[7:0] rdaddr_b,
output	[511:0] rddout_a_0_0,
output	[511:0] rddout_b_0_0,
output	[511:0] rddout_a_0_1,
output	[511:0] rddout_b_0_1,
output	[511:0] rddout_a_0_2,
output	[511:0] rddout_b_0_2,
output	[511:0] rddout_a_0_3,
output	[511:0] rddout_b_0_3,
output	[511:0] rddout_a_0_4,
output	[511:0] rddout_b_0_4,
output	[511:0] rddout_a_0_5,
output	[511:0] rddout_b_0_5,
output	[511:0] rddout_a_0_6,
output	[511:0] rddout_b_0_6,
output	[511:0] rddout_a_0_7,
output	[511:0] rddout_b_0_7
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
input	[511:0] wrdin_q,
input vld_in,
input	[10:0] rdaddr_a,
input	[10:0] rdaddr_b,
input	[511:0] val_i_y_0,
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
wire [511:0] d1_val_i_y_0;
du_s #(512) du_99_0 (.clk(clk), .stall(1'b0), .dout(d1_val_i_y_0),.din(val_i_y_0));

	
	
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
	


wire [511:0] rddout_a_0_0;
wire [511:0] rddout_a_0_1;
wire [511:0] rddout_a_0_2;
wire [511:0] rddout_a_0_3;
wire [511:0] rddout_a_0_4;
wire [511:0] rddout_a_0_5;
wire [511:0] rddout_a_0_6;
wire [511:0] rddout_a_0_7;


wire [511:0] rddout_b_0_0;
wire [511:0] rddout_b_0_1;
wire [511:0] rddout_b_0_2;
wire [511:0] rddout_b_0_3;
wire [511:0] rddout_b_0_4;
wire [511:0] rddout_b_0_5;
wire [511:0] rddout_b_0_6;
wire [511:0] rddout_b_0_7;




wire [511:0] val_i_x_0_a;
wire [511:0] val_i_x_1_a;
wire [511:0] val_i_x_2_a;
wire [511:0] val_i_x_3_a;
wire [511:0] val_i_x_4_a;
wire [511:0] val_i_x_5_a;
wire [511:0] val_i_x_6_a;
wire [511:0] val_i_x_7_a;
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


wire [511:0] val_i_x_0_b;
wire [511:0] val_i_x_1_b;
wire [511:0] val_i_x_2_b;
wire [511:0] val_i_x_3_b;
wire [511:0] val_i_x_4_b;
wire [511:0] val_i_x_5_b;
wire [511:0] val_i_x_6_b;
wire [511:0] val_i_x_7_b;
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
	


assign val_i_x_0_a[511:0] = rddout_a_0_0[511:0];
assign val_i_x_1_a[511:0] = rddout_a_0_1[511:0];
assign val_i_x_2_a[511:0] = rddout_a_0_2[511:0];
assign val_i_x_3_a[511:0] = rddout_a_0_3[511:0];
assign val_i_x_4_a[511:0] = rddout_a_0_4[511:0];
assign val_i_x_5_a[511:0] = rddout_a_0_5[511:0];
assign val_i_x_6_a[511:0] = rddout_a_0_6[511:0];
assign val_i_x_7_a[511:0] = rddout_a_0_7[511:0];


assign val_i_x_0_b[511:0] = rddout_b_0_0[511:0];
assign val_i_x_1_b[511:0] = rddout_b_0_1[511:0];
assign val_i_x_2_b[511:0] = rddout_b_0_2[511:0];
assign val_i_x_3_b[511:0] = rddout_b_0_3[511:0];
assign val_i_x_4_b[511:0] = rddout_b_0_4[511:0];
assign val_i_x_5_b[511:0] = rddout_b_0_5[511:0];
assign val_i_x_6_b[511:0] = rddout_b_0_6[511:0];
assign val_i_x_7_b[511:0] = rddout_b_0_7[511:0];


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
