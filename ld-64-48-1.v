	
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


module popcnt_bram_dsp_64_6 (
input clk,
input	[63:0] input_v,
output	[6:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] lut8_BRAM_i_0;
wire [7:0] lut8_BRAM_i_1;
wire [7:0] lut8_BRAM_i_2;
wire [7:0] lut8_BRAM_i_3;
wire [7:0] lut8_BRAM_i_4;
wire [7:0] lut8_BRAM_i_5;
wire [7:0] lut8_BRAM_i_6;
wire [7:0] lut8_BRAM_i_7;
wire [3:0] lut8_BRAM_o_0;
wire [3:0] lut8_BRAM_o_1;
wire [3:0] lut8_BRAM_o_2;
wire [3:0] lut8_BRAM_o_3;
wire [3:0] lut8_BRAM_o_4;
wire [3:0] lut8_BRAM_o_5;
wire [3:0] lut8_BRAM_o_6;
wire [3:0] lut8_BRAM_o_7;
assign input_0[7:0] = input_v[63:56];
assign input_1[7:0] = input_v[55:48];
assign input_2[7:0] = input_v[47:40];
assign input_3[7:0] = input_v[39:32];
assign input_4[7:0] = input_v[31:24];
assign input_5[7:0] = input_v[23:16];
assign input_6[7:0] = input_v[15:8];
assign input_7[7:0] = input_v[7:0];
assign lut8_BRAM_i_0[7:0] = input_0[7:0];
assign lut8_BRAM_i_1[7:0] = input_1[7:0];
assign lut8_BRAM_i_2[7:0] = input_2[7:0];
assign lut8_BRAM_i_3[7:0] = input_3[7:0];
assign lut8_BRAM_i_4[7:0] = input_4[7:0];
assign lut8_BRAM_i_5[7:0] = input_5[7:0];
assign lut8_BRAM_i_6[7:0] = input_6[7:0];
assign lut8_BRAM_i_7[7:0] = input_7[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
lut8_BRAM lut8_BRAM_1 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_2),.lut8_BRAM_i_1(lut8_BRAM_i_3),.lut8_BRAM_o_0(lut8_BRAM_o_2),.lut8_BRAM_o_1(lut8_BRAM_o_3) );
lut8_BRAM lut8_BRAM_2 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_4),.lut8_BRAM_i_1(lut8_BRAM_i_5),.lut8_BRAM_o_0(lut8_BRAM_o_4),.lut8_BRAM_o_1(lut8_BRAM_o_5) );
lut8_BRAM lut8_BRAM_3 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_6),.lut8_BRAM_i_1(lut8_BRAM_i_7),.lut8_BRAM_o_0(lut8_BRAM_o_6),.lut8_BRAM_o_1(lut8_BRAM_o_7) );
wire [3:0] add_2_i_0_0;
wire [3:0] add_2_i_0_1;
wire [3:0] add_2_i_2_0;
wire [3:0] add_2_i_2_1;
wire [4:0] add_2_o_0_0;
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [4:0] add_1_i_0_0;
wire [4:0] add_1_i_0_1;
wire [4:0] add_1_i_1_0;
wire [4:0] add_1_i_1_1;
wire [5:0] add_1_o_0_0;
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
wire [5:0] add_0_i_0_0;
wire [5:0] add_0_i_0_1;
wire [5:0] add_0_i_1_0;
wire [5:0] add_0_i_1_1;
wire [6:0] add_0_o_0_0;
wire [6:0] add_0_o_1_0;
assign add_2_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_2_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
assign add_3_i_0_0[3:0] = lut8_BRAM_o_2[3:0];
assign add_3_i_0_1[3:0] = lut8_BRAM_o_3[3:0];
assign add_5_i_0_0[3:0] = lut8_BRAM_o_4[3:0];
assign add_5_i_0_1[3:0] = lut8_BRAM_o_5[3:0];
assign add_6_i_0_0[3:0] = lut8_BRAM_o_6[3:0];
assign add_6_i_0_1[3:0] = lut8_BRAM_o_7[3:0];
assign add_1_i_0_0[4:0] = add_2_o_0_0[4:0];
assign add_1_i_0_1[4:0] = add_3_o_0_0[4:0];
assign add_0_i_0_0[5:0] = add_1_o_0_0[5:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_0_i_0_1[5:0] = add_4_o_0_0[5:0];
du #(4, 2) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_0),.din(add_2_i_0_0));
du #(4, 2) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_2_0 + add_2_i_2_1;

du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(5, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(5, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(6, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(6, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(7, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_bram_nodsp_64_6 (
input clk,
input	[63:0] input_v,
output	[6:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] lut8_BRAM_i_0;
wire [7:0] lut8_BRAM_i_1;
wire [7:0] lut8_BRAM_i_2;
wire [7:0] lut8_BRAM_i_3;
wire [7:0] lut8_BRAM_i_4;
wire [7:0] lut8_BRAM_i_5;
wire [7:0] lut8_BRAM_i_6;
wire [7:0] lut8_BRAM_i_7;
wire [3:0] lut8_BRAM_o_0;
wire [3:0] lut8_BRAM_o_1;
wire [3:0] lut8_BRAM_o_2;
wire [3:0] lut8_BRAM_o_3;
wire [3:0] lut8_BRAM_o_4;
wire [3:0] lut8_BRAM_o_5;
wire [3:0] lut8_BRAM_o_6;
wire [3:0] lut8_BRAM_o_7;
assign input_0[7:0] = input_v[63:56];
assign input_1[7:0] = input_v[55:48];
assign input_2[7:0] = input_v[47:40];
assign input_3[7:0] = input_v[39:32];
assign input_4[7:0] = input_v[31:24];
assign input_5[7:0] = input_v[23:16];
assign input_6[7:0] = input_v[15:8];
assign input_7[7:0] = input_v[7:0];
assign lut8_BRAM_i_0[7:0] = input_0[7:0];
assign lut8_BRAM_i_1[7:0] = input_1[7:0];
assign lut8_BRAM_i_2[7:0] = input_2[7:0];
assign lut8_BRAM_i_3[7:0] = input_3[7:0];
assign lut8_BRAM_i_4[7:0] = input_4[7:0];
assign lut8_BRAM_i_5[7:0] = input_5[7:0];
assign lut8_BRAM_i_6[7:0] = input_6[7:0];
assign lut8_BRAM_i_7[7:0] = input_7[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
lut8_BRAM lut8_BRAM_1 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_2),.lut8_BRAM_i_1(lut8_BRAM_i_3),.lut8_BRAM_o_0(lut8_BRAM_o_2),.lut8_BRAM_o_1(lut8_BRAM_o_3) );
lut8_BRAM lut8_BRAM_2 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_4),.lut8_BRAM_i_1(lut8_BRAM_i_5),.lut8_BRAM_o_0(lut8_BRAM_o_4),.lut8_BRAM_o_1(lut8_BRAM_o_5) );
lut8_BRAM lut8_BRAM_3 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_6),.lut8_BRAM_i_1(lut8_BRAM_i_7),.lut8_BRAM_o_0(lut8_BRAM_o_6),.lut8_BRAM_o_1(lut8_BRAM_o_7) );
wire [3:0] add_2_i_0_0;
wire [3:0] add_2_i_0_1;
wire [3:0] add_2_i_2_0;
wire [3:0] add_2_i_2_1;
wire [4:0] add_2_o_0_0;
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [4:0] add_1_i_0_0;
wire [4:0] add_1_i_0_1;
wire [4:0] add_1_i_1_0;
wire [4:0] add_1_i_1_1;
wire [5:0] add_1_o_0_0;
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
wire [5:0] add_0_i_0_0;
wire [5:0] add_0_i_0_1;
wire [5:0] add_0_i_1_0;
wire [5:0] add_0_i_1_1;
wire [6:0] add_0_o_0_0;
wire [6:0] add_0_o_1_0;
assign add_2_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_2_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
assign add_3_i_0_0[3:0] = lut8_BRAM_o_2[3:0];
assign add_3_i_0_1[3:0] = lut8_BRAM_o_3[3:0];
assign add_5_i_0_0[3:0] = lut8_BRAM_o_4[3:0];
assign add_5_i_0_1[3:0] = lut8_BRAM_o_5[3:0];
assign add_6_i_0_0[3:0] = lut8_BRAM_o_6[3:0];
assign add_6_i_0_1[3:0] = lut8_BRAM_o_7[3:0];
assign add_1_i_0_0[4:0] = add_2_o_0_0[4:0];
assign add_1_i_0_1[4:0] = add_3_o_0_0[4:0];
assign add_0_i_0_0[5:0] = add_1_o_0_0[5:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_0_i_0_1[5:0] = add_4_o_0_0[5:0];
du #(4, 2) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_0),.din(add_2_i_0_0));
du #(4, 2) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_2_0 + add_2_i_2_1;

du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(5, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(5, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(6, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(6, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(7, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_dist_dsp_64_6 (
input clk,
input	[63:0] input_v,
output	[6:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] lut8_DIST_i_0;
wire [7:0] lut8_DIST_i_1;
wire [7:0] lut8_DIST_i_2;
wire [7:0] lut8_DIST_i_3;
wire [7:0] lut8_DIST_i_4;
wire [7:0] lut8_DIST_i_5;
wire [7:0] lut8_DIST_i_6;
wire [7:0] lut8_DIST_i_7;
wire [3:0] lut8_DIST_o_0;
wire [3:0] lut8_DIST_o_1;
wire [3:0] lut8_DIST_o_2;
wire [3:0] lut8_DIST_o_3;
wire [3:0] lut8_DIST_o_4;
wire [3:0] lut8_DIST_o_5;
wire [3:0] lut8_DIST_o_6;
wire [3:0] lut8_DIST_o_7;
assign input_0[7:0] = input_v[63:56];
assign input_1[7:0] = input_v[55:48];
assign input_2[7:0] = input_v[47:40];
assign input_3[7:0] = input_v[39:32];
assign input_4[7:0] = input_v[31:24];
assign input_5[7:0] = input_v[23:16];
assign input_6[7:0] = input_v[15:8];
assign input_7[7:0] = input_v[7:0];
assign lut8_DIST_i_0[7:0] = input_0[7:0];
assign lut8_DIST_i_1[7:0] = input_1[7:0];
assign lut8_DIST_i_2[7:0] = input_2[7:0];
assign lut8_DIST_i_3[7:0] = input_3[7:0];
assign lut8_DIST_i_4[7:0] = input_4[7:0];
assign lut8_DIST_i_5[7:0] = input_5[7:0];
assign lut8_DIST_i_6[7:0] = input_6[7:0];
assign lut8_DIST_i_7[7:0] = input_7[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
lut8_DIST lut8_DIST_1 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_2),.lut8_DIST_i_1(lut8_DIST_i_3),.lut8_DIST_o_0(lut8_DIST_o_2),.lut8_DIST_o_1(lut8_DIST_o_3) );
lut8_DIST lut8_DIST_2 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_4),.lut8_DIST_i_1(lut8_DIST_i_5),.lut8_DIST_o_0(lut8_DIST_o_4),.lut8_DIST_o_1(lut8_DIST_o_5) );
lut8_DIST lut8_DIST_3 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_6),.lut8_DIST_i_1(lut8_DIST_i_7),.lut8_DIST_o_0(lut8_DIST_o_6),.lut8_DIST_o_1(lut8_DIST_o_7) );
wire [3:0] add_2_i_0_0;
wire [3:0] add_2_i_0_1;
wire [3:0] add_2_i_2_0;
wire [3:0] add_2_i_2_1;
wire [4:0] add_2_o_0_0;
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [4:0] add_1_i_0_0;
wire [4:0] add_1_i_0_1;
wire [4:0] add_1_i_1_0;
wire [4:0] add_1_i_1_1;
wire [5:0] add_1_o_0_0;
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
wire [5:0] add_0_i_0_0;
wire [5:0] add_0_i_0_1;
wire [5:0] add_0_i_1_0;
wire [5:0] add_0_i_1_1;
wire [6:0] add_0_o_0_0;
wire [6:0] add_0_o_1_0;
assign add_2_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_2_i_0_1[3:0] = lut8_DIST_o_1[3:0];
assign add_3_i_0_0[3:0] = lut8_DIST_o_2[3:0];
assign add_3_i_0_1[3:0] = lut8_DIST_o_3[3:0];
assign add_5_i_0_0[3:0] = lut8_DIST_o_4[3:0];
assign add_5_i_0_1[3:0] = lut8_DIST_o_5[3:0];
assign add_6_i_0_0[3:0] = lut8_DIST_o_6[3:0];
assign add_6_i_0_1[3:0] = lut8_DIST_o_7[3:0];
assign add_1_i_0_0[4:0] = add_2_o_0_0[4:0];
assign add_1_i_0_1[4:0] = add_3_o_0_0[4:0];
assign add_0_i_0_0[5:0] = add_1_o_0_0[5:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_0_i_0_1[5:0] = add_4_o_0_0[5:0];
du #(4, 2) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_0),.din(add_2_i_0_0));
du #(4, 2) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_2_0 + add_2_i_2_1;

du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(5, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(5, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(6, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(6, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(7, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

assign output_v = add_0_o_1_0;

endmodule
module popcnt_dist_nodsp_64_6 (
input clk,
input	[63:0] input_v,
output	[6:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] input_2;
wire [7:0] input_3;
wire [7:0] input_4;
wire [7:0] input_5;
wire [7:0] input_6;
wire [7:0] input_7;
wire [7:0] lut8_DIST_i_0;
wire [7:0] lut8_DIST_i_1;
wire [7:0] lut8_DIST_i_2;
wire [7:0] lut8_DIST_i_3;
wire [7:0] lut8_DIST_i_4;
wire [7:0] lut8_DIST_i_5;
wire [7:0] lut8_DIST_i_6;
wire [7:0] lut8_DIST_i_7;
wire [3:0] lut8_DIST_o_0;
wire [3:0] lut8_DIST_o_1;
wire [3:0] lut8_DIST_o_2;
wire [3:0] lut8_DIST_o_3;
wire [3:0] lut8_DIST_o_4;
wire [3:0] lut8_DIST_o_5;
wire [3:0] lut8_DIST_o_6;
wire [3:0] lut8_DIST_o_7;
assign input_0[7:0] = input_v[63:56];
assign input_1[7:0] = input_v[55:48];
assign input_2[7:0] = input_v[47:40];
assign input_3[7:0] = input_v[39:32];
assign input_4[7:0] = input_v[31:24];
assign input_5[7:0] = input_v[23:16];
assign input_6[7:0] = input_v[15:8];
assign input_7[7:0] = input_v[7:0];
assign lut8_DIST_i_0[7:0] = input_0[7:0];
assign lut8_DIST_i_1[7:0] = input_1[7:0];
assign lut8_DIST_i_2[7:0] = input_2[7:0];
assign lut8_DIST_i_3[7:0] = input_3[7:0];
assign lut8_DIST_i_4[7:0] = input_4[7:0];
assign lut8_DIST_i_5[7:0] = input_5[7:0];
assign lut8_DIST_i_6[7:0] = input_6[7:0];
assign lut8_DIST_i_7[7:0] = input_7[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
lut8_DIST lut8_DIST_1 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_2),.lut8_DIST_i_1(lut8_DIST_i_3),.lut8_DIST_o_0(lut8_DIST_o_2),.lut8_DIST_o_1(lut8_DIST_o_3) );
lut8_DIST lut8_DIST_2 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_4),.lut8_DIST_i_1(lut8_DIST_i_5),.lut8_DIST_o_0(lut8_DIST_o_4),.lut8_DIST_o_1(lut8_DIST_o_5) );
lut8_DIST lut8_DIST_3 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_6),.lut8_DIST_i_1(lut8_DIST_i_7),.lut8_DIST_o_0(lut8_DIST_o_6),.lut8_DIST_o_1(lut8_DIST_o_7) );
wire [3:0] add_2_i_0_0;
wire [3:0] add_2_i_0_1;
wire [3:0] add_2_i_2_0;
wire [3:0] add_2_i_2_1;
wire [4:0] add_2_o_0_0;
wire [3:0] add_3_i_0_0;
wire [3:0] add_3_i_0_1;
wire [3:0] add_3_i_2_0;
wire [3:0] add_3_i_2_1;
wire [4:0] add_3_o_0_0;
wire [4:0] add_1_i_0_0;
wire [4:0] add_1_i_0_1;
wire [4:0] add_1_i_1_0;
wire [4:0] add_1_i_1_1;
wire [5:0] add_1_o_0_0;
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
wire [5:0] add_0_i_0_0;
wire [5:0] add_0_i_0_1;
wire [5:0] add_0_i_1_0;
wire [5:0] add_0_i_1_1;
wire [6:0] add_0_o_0_0;
wire [6:0] add_0_o_1_0;
assign add_2_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_2_i_0_1[3:0] = lut8_DIST_o_1[3:0];
assign add_3_i_0_0[3:0] = lut8_DIST_o_2[3:0];
assign add_3_i_0_1[3:0] = lut8_DIST_o_3[3:0];
assign add_5_i_0_0[3:0] = lut8_DIST_o_4[3:0];
assign add_5_i_0_1[3:0] = lut8_DIST_o_5[3:0];
assign add_6_i_0_0[3:0] = lut8_DIST_o_6[3:0];
assign add_6_i_0_1[3:0] = lut8_DIST_o_7[3:0];
assign add_1_i_0_0[4:0] = add_2_o_0_0[4:0];
assign add_1_i_0_1[4:0] = add_3_o_0_0[4:0];
assign add_0_i_0_0[5:0] = add_1_o_0_0[5:0];
assign add_4_i_0_0[4:0] = add_5_o_0_0[4:0];
assign add_4_i_0_1[4:0] = add_6_o_0_0[4:0];
assign add_0_i_0_1[5:0] = add_4_o_0_0[5:0];
du #(4, 2) du_2_0 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_0),.din(add_2_i_0_0));
du #(4, 2) du_2_1 (.clk(clk), .stall(1'b0), .dout(add_2_i_2_1),.din(add_2_i_0_1));

assign add_2_o_0_0 = add_2_i_2_0 + add_2_i_2_1;

du #(4, 2) du_3_0 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_0),.din(add_3_i_0_0));
du #(4, 2) du_3_1 (.clk(clk), .stall(1'b0), .dout(add_3_i_2_1),.din(add_3_i_0_1));

assign add_3_o_0_0 = add_3_i_2_0 + add_3_i_2_1;

du #(5, 1) du_1_0 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_0),.din(add_1_i_0_0));
du #(5, 1) du_1_1 (.clk(clk), .stall(1'b0), .dout(add_1_i_1_1),.din(add_1_i_0_1));

assign add_1_o_0_0 = add_1_i_1_0 + add_1_i_1_1;

du #(4, 2) du_5_0 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_0),.din(add_5_i_0_0));
du #(4, 2) du_5_1 (.clk(clk), .stall(1'b0), .dout(add_5_i_2_1),.din(add_5_i_0_1));

assign add_5_o_0_0 = add_5_i_2_0 + add_5_i_2_1;

du #(4, 2) du_6_0 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_0),.din(add_6_i_0_0));
du #(4, 2) du_6_1 (.clk(clk), .stall(1'b0), .dout(add_6_i_2_1),.din(add_6_i_0_1));

assign add_6_o_0_0 = add_6_i_2_0 + add_6_i_2_1;

du #(5, 1) du_4_0 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_0),.din(add_4_i_0_0));
du #(5, 1) du_4_1 (.clk(clk), .stall(1'b0), .dout(add_4_i_1_1),.din(add_4_i_0_1));

assign add_4_o_0_0 = add_4_i_1_0 + add_4_i_1_1;

du #(6, 1) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_0),.din(add_0_i_0_0));
du #(6, 1) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_1_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_1_0 + add_0_i_1_1;

du #(7, 1) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_1_0),.din(add_0_o_0_0));

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
	parameter IPORT_WIDTH = 64,					
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
	wire [6:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_0 #(IPORT_WIDTH) l_inst (.vld_in(vld_in), .val_i(val_i), .val_o(t_l_val_o));					
	assign val_i_t = t_l_val_o;				   	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_64_6 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(7, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1			#(					
	parameter IPORT_WIDTH = 64,					
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
	wire [6:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_64_6 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(7, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1_special			#(					
	parameter IPORT_WIDTH = 64,					
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
	wire [6:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_64_6 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(7, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule



module pw_sim_48_1_64_6 (
input clk,
input rst,
input vld_in,
input	[63:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[63:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[63:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[63:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[63:0] val_i_x_3,
output	[31:0] x_cnt_3,
input	[63:0] val_i_x_4,
output	[31:0] x_cnt_4,
input	[63:0] val_i_x_5,
output	[31:0] x_cnt_5,
input	[63:0] val_i_x_6,
output	[31:0] x_cnt_6,
input	[63:0] val_i_x_7,
output	[31:0] x_cnt_7,
input	[63:0] val_i_x_8,
output	[31:0] x_cnt_8,
input	[63:0] val_i_x_9,
output	[31:0] x_cnt_9,
input	[63:0] val_i_x_10,
output	[31:0] x_cnt_10,
input	[63:0] val_i_x_11,
output	[31:0] x_cnt_11,
input	[63:0] val_i_x_12,
output	[31:0] x_cnt_12,
input	[63:0] val_i_x_13,
output	[31:0] x_cnt_13,
input	[63:0] val_i_x_14,
output	[31:0] x_cnt_14,
input	[63:0] val_i_x_15,
output	[31:0] x_cnt_15,
input	[63:0] val_i_x_16,
output	[31:0] x_cnt_16,
input	[63:0] val_i_x_17,
output	[31:0] x_cnt_17,
input	[63:0] val_i_x_18,
output	[31:0] x_cnt_18,
input	[63:0] val_i_x_19,
output	[31:0] x_cnt_19,
input	[63:0] val_i_x_20,
output	[31:0] x_cnt_20,
input	[63:0] val_i_x_21,
output	[31:0] x_cnt_21,
input	[63:0] val_i_x_22,
output	[31:0] x_cnt_22,
input	[63:0] val_i_x_23,
output	[31:0] x_cnt_23,
input	[63:0] val_i_x_24,
output	[31:0] x_cnt_24,
input	[63:0] val_i_x_25,
output	[31:0] x_cnt_25,
input	[63:0] val_i_x_26,
output	[31:0] x_cnt_26,
input	[63:0] val_i_x_27,
output	[31:0] x_cnt_27,
input	[63:0] val_i_x_28,
output	[31:0] x_cnt_28,
input	[63:0] val_i_x_29,
output	[31:0] x_cnt_29,
input	[63:0] val_i_x_30,
output	[31:0] x_cnt_30,
input	[63:0] val_i_x_31,
output	[31:0] x_cnt_31,
input	[63:0] val_i_x_32,
output	[31:0] x_cnt_32,
input	[63:0] val_i_x_33,
output	[31:0] x_cnt_33,
input	[63:0] val_i_x_34,
output	[31:0] x_cnt_34,
input	[63:0] val_i_x_35,
output	[31:0] x_cnt_35,
input	[63:0] val_i_x_36,
output	[31:0] x_cnt_36,
input	[63:0] val_i_x_37,
output	[31:0] x_cnt_37,
input	[63:0] val_i_x_38,
output	[31:0] x_cnt_38,
input	[63:0] val_i_x_39,
output	[31:0] x_cnt_39,
input	[63:0] val_i_x_40,
output	[31:0] x_cnt_40,
input	[63:0] val_i_x_41,
output	[31:0] x_cnt_41,
input	[63:0] val_i_x_42,
output	[31:0] x_cnt_42,
input	[63:0] val_i_x_43,
output	[31:0] x_cnt_43,
input	[63:0] val_i_x_44,
output	[31:0] x_cnt_44,
input	[63:0] val_i_x_45,
output	[31:0] x_cnt_45,
input	[63:0] val_i_x_46,
output	[31:0] x_cnt_46,
input	[63:0] val_i_x_47,
output	[31:0] x_cnt_47,
output	[31:0] xy_cnt_0_0,
output	[31:0] xy_cnt_0_1,
output	[31:0] xy_cnt_0_2,
output	[31:0] xy_cnt_0_3,
output	[31:0] xy_cnt_0_4,
output	[31:0] xy_cnt_0_5,
output	[31:0] xy_cnt_0_6,
output	[31:0] xy_cnt_0_7,
output	[31:0] xy_cnt_0_8,
output	[31:0] xy_cnt_0_9,
output	[31:0] xy_cnt_0_10,
output	[31:0] xy_cnt_0_11,
output	[31:0] xy_cnt_0_12,
output	[31:0] xy_cnt_0_13,
output	[31:0] xy_cnt_0_14,
output	[31:0] xy_cnt_0_15,
output	[31:0] xy_cnt_0_16,
output	[31:0] xy_cnt_0_17,
output	[31:0] xy_cnt_0_18,
output	[31:0] xy_cnt_0_19,
output	[31:0] xy_cnt_0_20,
output	[31:0] xy_cnt_0_21,
output	[31:0] xy_cnt_0_22,
output	[31:0] xy_cnt_0_23,
output	[31:0] xy_cnt_0_24,
output	[31:0] xy_cnt_0_25,
output	[31:0] xy_cnt_0_26,
output	[31:0] xy_cnt_0_27,
output	[31:0] xy_cnt_0_28,
output	[31:0] xy_cnt_0_29,
output	[31:0] xy_cnt_0_30,
output	[31:0] xy_cnt_0_31,
output	[31:0] xy_cnt_0_32,
output	[31:0] xy_cnt_0_33,
output	[31:0] xy_cnt_0_34,
output	[31:0] xy_cnt_0_35,
output	[31:0] xy_cnt_0_36,
output	[31:0] xy_cnt_0_37,
output	[31:0] xy_cnt_0_38,
output	[31:0] xy_cnt_0_39,
output	[31:0] xy_cnt_0_40,
output	[31:0] xy_cnt_0_41,
output	[31:0] xy_cnt_0_42,
output	[31:0] xy_cnt_0_43,
output	[31:0] xy_cnt_0_44,
output	[31:0] xy_cnt_0_45,
output	[31:0] xy_cnt_0_46,
output	[31:0] xy_cnt_0_47
);

wire [63:0] val_i_y_t_0;

wire [63:0] val_i_x_t_0;
wire [63:0] val_i_x_t_1;
wire [63:0] val_i_x_t_2;
wire [63:0] val_i_x_t_3;
wire [63:0] val_i_x_t_4;
wire [63:0] val_i_x_t_5;
wire [63:0] val_i_x_t_6;
wire [63:0] val_i_x_t_7;
wire [63:0] val_i_x_t_8;
wire [63:0] val_i_x_t_9;
wire [63:0] val_i_x_t_10;
wire [63:0] val_i_x_t_11;
wire [63:0] val_i_x_t_12;
wire [63:0] val_i_x_t_13;
wire [63:0] val_i_x_t_14;
wire [63:0] val_i_x_t_15;
wire [63:0] val_i_x_t_16;
wire [63:0] val_i_x_t_17;
wire [63:0] val_i_x_t_18;
wire [63:0] val_i_x_t_19;
wire [63:0] val_i_x_t_20;
wire [63:0] val_i_x_t_21;
wire [63:0] val_i_x_t_22;
wire [63:0] val_i_x_t_23;
wire [63:0] val_i_x_t_24;
wire [63:0] val_i_x_t_25;
wire [63:0] val_i_x_t_26;
wire [63:0] val_i_x_t_27;
wire [63:0] val_i_x_t_28;
wire [63:0] val_i_x_t_29;
wire [63:0] val_i_x_t_30;
wire [63:0] val_i_x_t_31;
wire [63:0] val_i_x_t_32;
wire [63:0] val_i_x_t_33;
wire [63:0] val_i_x_t_34;
wire [63:0] val_i_x_t_35;
wire [63:0] val_i_x_t_36;
wire [63:0] val_i_x_t_37;
wire [63:0] val_i_x_t_38;
wire [63:0] val_i_x_t_39;
wire [63:0] val_i_x_t_40;
wire [63:0] val_i_x_t_41;
wire [63:0] val_i_x_t_42;
wire [63:0] val_i_x_t_43;
wire [63:0] val_i_x_t_44;
wire [63:0] val_i_x_t_45;
wire [63:0] val_i_x_t_46;
wire [63:0] val_i_x_t_47;

gblock_0 #(64, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(64, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(64, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(64, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));
gblock_0 #(64, 32) gblock_0_x_4 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_4), .val_o(x_cnt_4), .val_i_t(val_i_x_t_4));
gblock_0 #(64, 32) gblock_0_x_5 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_5), .val_o(x_cnt_5), .val_i_t(val_i_x_t_5));
gblock_0 #(64, 32) gblock_0_x_6 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_6), .val_o(x_cnt_6), .val_i_t(val_i_x_t_6));
gblock_0 #(64, 32) gblock_0_x_7 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_7), .val_o(x_cnt_7), .val_i_t(val_i_x_t_7));
gblock_0 #(64, 32) gblock_0_x_8 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_8), .val_o(x_cnt_8), .val_i_t(val_i_x_t_8));
gblock_0 #(64, 32) gblock_0_x_9 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_9), .val_o(x_cnt_9), .val_i_t(val_i_x_t_9));
gblock_0 #(64, 32) gblock_0_x_10 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_10), .val_o(x_cnt_10), .val_i_t(val_i_x_t_10));
gblock_0 #(64, 32) gblock_0_x_11 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_11), .val_o(x_cnt_11), .val_i_t(val_i_x_t_11));
gblock_0 #(64, 32) gblock_0_x_12 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_12), .val_o(x_cnt_12), .val_i_t(val_i_x_t_12));
gblock_0 #(64, 32) gblock_0_x_13 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_13), .val_o(x_cnt_13), .val_i_t(val_i_x_t_13));
gblock_0 #(64, 32) gblock_0_x_14 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_14), .val_o(x_cnt_14), .val_i_t(val_i_x_t_14));
gblock_0 #(64, 32) gblock_0_x_15 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_15), .val_o(x_cnt_15), .val_i_t(val_i_x_t_15));
gblock_0 #(64, 32) gblock_0_x_16 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_16), .val_o(x_cnt_16), .val_i_t(val_i_x_t_16));
gblock_0 #(64, 32) gblock_0_x_17 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_17), .val_o(x_cnt_17), .val_i_t(val_i_x_t_17));
gblock_0 #(64, 32) gblock_0_x_18 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_18), .val_o(x_cnt_18), .val_i_t(val_i_x_t_18));
gblock_0 #(64, 32) gblock_0_x_19 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_19), .val_o(x_cnt_19), .val_i_t(val_i_x_t_19));
gblock_0 #(64, 32) gblock_0_x_20 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_20), .val_o(x_cnt_20), .val_i_t(val_i_x_t_20));
gblock_0 #(64, 32) gblock_0_x_21 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_21), .val_o(x_cnt_21), .val_i_t(val_i_x_t_21));
gblock_0 #(64, 32) gblock_0_x_22 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_22), .val_o(x_cnt_22), .val_i_t(val_i_x_t_22));
gblock_0 #(64, 32) gblock_0_x_23 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_23), .val_o(x_cnt_23), .val_i_t(val_i_x_t_23));
gblock_0 #(64, 32) gblock_0_x_24 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_24), .val_o(x_cnt_24), .val_i_t(val_i_x_t_24));
gblock_0 #(64, 32) gblock_0_x_25 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_25), .val_o(x_cnt_25), .val_i_t(val_i_x_t_25));
gblock_0 #(64, 32) gblock_0_x_26 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_26), .val_o(x_cnt_26), .val_i_t(val_i_x_t_26));
gblock_0 #(64, 32) gblock_0_x_27 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_27), .val_o(x_cnt_27), .val_i_t(val_i_x_t_27));
gblock_0 #(64, 32) gblock_0_x_28 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_28), .val_o(x_cnt_28), .val_i_t(val_i_x_t_28));
gblock_0 #(64, 32) gblock_0_x_29 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_29), .val_o(x_cnt_29), .val_i_t(val_i_x_t_29));
gblock_0 #(64, 32) gblock_0_x_30 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_30), .val_o(x_cnt_30), .val_i_t(val_i_x_t_30));
gblock_0 #(64, 32) gblock_0_x_31 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_31), .val_o(x_cnt_31), .val_i_t(val_i_x_t_31));
gblock_0 #(64, 32) gblock_0_x_32 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_32), .val_o(x_cnt_32), .val_i_t(val_i_x_t_32));
gblock_0 #(64, 32) gblock_0_x_33 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_33), .val_o(x_cnt_33), .val_i_t(val_i_x_t_33));
gblock_0 #(64, 32) gblock_0_x_34 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_34), .val_o(x_cnt_34), .val_i_t(val_i_x_t_34));
gblock_0 #(64, 32) gblock_0_x_35 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_35), .val_o(x_cnt_35), .val_i_t(val_i_x_t_35));
gblock_0 #(64, 32) gblock_0_x_36 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_36), .val_o(x_cnt_36), .val_i_t(val_i_x_t_36));
gblock_0 #(64, 32) gblock_0_x_37 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_37), .val_o(x_cnt_37), .val_i_t(val_i_x_t_37));
gblock_0 #(64, 32) gblock_0_x_38 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_38), .val_o(x_cnt_38), .val_i_t(val_i_x_t_38));
gblock_0 #(64, 32) gblock_0_x_39 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_39), .val_o(x_cnt_39), .val_i_t(val_i_x_t_39));
gblock_0 #(64, 32) gblock_0_x_40 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_40), .val_o(x_cnt_40), .val_i_t(val_i_x_t_40));
gblock_0 #(64, 32) gblock_0_x_41 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_41), .val_o(x_cnt_41), .val_i_t(val_i_x_t_41));
gblock_0 #(64, 32) gblock_0_x_42 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_42), .val_o(x_cnt_42), .val_i_t(val_i_x_t_42));
gblock_0 #(64, 32) gblock_0_x_43 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_43), .val_o(x_cnt_43), .val_i_t(val_i_x_t_43));
gblock_0 #(64, 32) gblock_0_x_44 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_44), .val_o(x_cnt_44), .val_i_t(val_i_x_t_44));
gblock_0 #(64, 32) gblock_0_x_45 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_45), .val_o(x_cnt_45), .val_i_t(val_i_x_t_45));
gblock_0 #(64, 32) gblock_0_x_46 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_46), .val_o(x_cnt_46), .val_i_t(val_i_x_t_46));
gblock_0 #(64, 32) gblock_0_x_47 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_47), .val_o(x_cnt_47), .val_i_t(val_i_x_t_47));

gblock_0 #(64, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1 #(64, 32) gblock_1_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1 #(64, 32) gblock_1_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1 #(64, 32) gblock_1_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1 #(64, 32) gblock_1_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));
gblock_1 #(64, 32) gblock_1_0_4 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_4), .val_o(xy_cnt_0_4));
gblock_1 #(64, 32) gblock_1_0_5 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_5), .val_o(xy_cnt_0_5));
gblock_1 #(64, 32) gblock_1_0_6 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_6), .val_o(xy_cnt_0_6));
gblock_1 #(64, 32) gblock_1_0_7 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_7), .val_o(xy_cnt_0_7));
gblock_1 #(64, 32) gblock_1_0_8 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_8), .val_o(xy_cnt_0_8));
gblock_1 #(64, 32) gblock_1_0_9 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_9), .val_o(xy_cnt_0_9));
gblock_1 #(64, 32) gblock_1_0_10 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_10), .val_o(xy_cnt_0_10));
gblock_1 #(64, 32) gblock_1_0_11 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_11), .val_o(xy_cnt_0_11));
gblock_1 #(64, 32) gblock_1_0_12 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_12), .val_o(xy_cnt_0_12));
gblock_1 #(64, 32) gblock_1_0_13 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_13), .val_o(xy_cnt_0_13));
gblock_1 #(64, 32) gblock_1_0_14 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_14), .val_o(xy_cnt_0_14));
gblock_1 #(64, 32) gblock_1_0_15 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_15), .val_o(xy_cnt_0_15));
gblock_1 #(64, 32) gblock_1_0_16 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_16), .val_o(xy_cnt_0_16));
gblock_1 #(64, 32) gblock_1_0_17 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_17), .val_o(xy_cnt_0_17));
gblock_1 #(64, 32) gblock_1_0_18 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_18), .val_o(xy_cnt_0_18));
gblock_1 #(64, 32) gblock_1_0_19 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_19), .val_o(xy_cnt_0_19));
gblock_1 #(64, 32) gblock_1_0_20 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_20), .val_o(xy_cnt_0_20));
gblock_1 #(64, 32) gblock_1_0_21 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_21), .val_o(xy_cnt_0_21));
gblock_1 #(64, 32) gblock_1_0_22 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_22), .val_o(xy_cnt_0_22));
gblock_1 #(64, 32) gblock_1_0_23 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_23), .val_o(xy_cnt_0_23));
gblock_1 #(64, 32) gblock_1_0_24 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_24), .val_o(xy_cnt_0_24));
gblock_1 #(64, 32) gblock_1_0_25 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_25), .val_o(xy_cnt_0_25));
gblock_1 #(64, 32) gblock_1_0_26 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_26), .val_o(xy_cnt_0_26));
gblock_1 #(64, 32) gblock_1_0_27 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_27), .val_o(xy_cnt_0_27));
gblock_1 #(64, 32) gblock_1_0_28 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_28), .val_o(xy_cnt_0_28));
gblock_1 #(64, 32) gblock_1_0_29 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_29), .val_o(xy_cnt_0_29));
gblock_1 #(64, 32) gblock_1_0_30 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_30), .val_o(xy_cnt_0_30));
gblock_1 #(64, 32) gblock_1_0_31 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_31), .val_o(xy_cnt_0_31));
gblock_1 #(64, 32) gblock_1_0_32 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_32), .val_o(xy_cnt_0_32));
gblock_1 #(64, 32) gblock_1_0_33 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_33), .val_o(xy_cnt_0_33));
gblock_1 #(64, 32) gblock_1_0_34 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_34), .val_o(xy_cnt_0_34));
gblock_1 #(64, 32) gblock_1_0_35 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_35), .val_o(xy_cnt_0_35));
gblock_1 #(64, 32) gblock_1_0_36 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_36), .val_o(xy_cnt_0_36));
gblock_1 #(64, 32) gblock_1_0_37 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_37), .val_o(xy_cnt_0_37));
gblock_1 #(64, 32) gblock_1_0_38 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_38), .val_o(xy_cnt_0_38));
gblock_1 #(64, 32) gblock_1_0_39 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_39), .val_o(xy_cnt_0_39));
gblock_1 #(64, 32) gblock_1_0_40 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_40), .val_o(xy_cnt_0_40));
gblock_1 #(64, 32) gblock_1_0_41 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_41), .val_o(xy_cnt_0_41));
gblock_1 #(64, 32) gblock_1_0_42 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_42), .val_o(xy_cnt_0_42));
gblock_1 #(64, 32) gblock_1_0_43 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_43), .val_o(xy_cnt_0_43));
gblock_1 #(64, 32) gblock_1_0_44 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_44), .val_o(xy_cnt_0_44));
gblock_1 #(64, 32) gblock_1_0_45 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_45), .val_o(xy_cnt_0_45));
gblock_1 #(64, 32) gblock_1_0_46 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_46), .val_o(xy_cnt_0_46));
gblock_1 #(64, 32) gblock_1_0_47 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_47), .val_o(xy_cnt_0_47));


endmodule


module pw_sim_special_48_1_64_6 (
input clk,
input rst,
input vld_in,
input	[63:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[63:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[63:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[63:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[63:0] val_i_x_3,
output	[31:0] x_cnt_3,
input	[63:0] val_i_x_4,
output	[31:0] x_cnt_4,
input	[63:0] val_i_x_5,
output	[31:0] x_cnt_5,
input	[63:0] val_i_x_6,
output	[31:0] x_cnt_6,
input	[63:0] val_i_x_7,
output	[31:0] x_cnt_7,
input	[63:0] val_i_x_8,
output	[31:0] x_cnt_8,
input	[63:0] val_i_x_9,
output	[31:0] x_cnt_9,
input	[63:0] val_i_x_10,
output	[31:0] x_cnt_10,
input	[63:0] val_i_x_11,
output	[31:0] x_cnt_11,
input	[63:0] val_i_x_12,
output	[31:0] x_cnt_12,
input	[63:0] val_i_x_13,
output	[31:0] x_cnt_13,
input	[63:0] val_i_x_14,
output	[31:0] x_cnt_14,
input	[63:0] val_i_x_15,
output	[31:0] x_cnt_15,
input	[63:0] val_i_x_16,
output	[31:0] x_cnt_16,
input	[63:0] val_i_x_17,
output	[31:0] x_cnt_17,
input	[63:0] val_i_x_18,
output	[31:0] x_cnt_18,
input	[63:0] val_i_x_19,
output	[31:0] x_cnt_19,
input	[63:0] val_i_x_20,
output	[31:0] x_cnt_20,
input	[63:0] val_i_x_21,
output	[31:0] x_cnt_21,
input	[63:0] val_i_x_22,
output	[31:0] x_cnt_22,
input	[63:0] val_i_x_23,
output	[31:0] x_cnt_23,
input	[63:0] val_i_x_24,
output	[31:0] x_cnt_24,
input	[63:0] val_i_x_25,
output	[31:0] x_cnt_25,
input	[63:0] val_i_x_26,
output	[31:0] x_cnt_26,
input	[63:0] val_i_x_27,
output	[31:0] x_cnt_27,
input	[63:0] val_i_x_28,
output	[31:0] x_cnt_28,
input	[63:0] val_i_x_29,
output	[31:0] x_cnt_29,
input	[63:0] val_i_x_30,
output	[31:0] x_cnt_30,
input	[63:0] val_i_x_31,
output	[31:0] x_cnt_31,
input	[63:0] val_i_x_32,
output	[31:0] x_cnt_32,
input	[63:0] val_i_x_33,
output	[31:0] x_cnt_33,
input	[63:0] val_i_x_34,
output	[31:0] x_cnt_34,
input	[63:0] val_i_x_35,
output	[31:0] x_cnt_35,
input	[63:0] val_i_x_36,
output	[31:0] x_cnt_36,
input	[63:0] val_i_x_37,
output	[31:0] x_cnt_37,
input	[63:0] val_i_x_38,
output	[31:0] x_cnt_38,
input	[63:0] val_i_x_39,
output	[31:0] x_cnt_39,
input	[63:0] val_i_x_40,
output	[31:0] x_cnt_40,
input	[63:0] val_i_x_41,
output	[31:0] x_cnt_41,
input	[63:0] val_i_x_42,
output	[31:0] x_cnt_42,
input	[63:0] val_i_x_43,
output	[31:0] x_cnt_43,
input	[63:0] val_i_x_44,
output	[31:0] x_cnt_44,
input	[63:0] val_i_x_45,
output	[31:0] x_cnt_45,
input	[63:0] val_i_x_46,
output	[31:0] x_cnt_46,
input	[63:0] val_i_x_47,
output	[31:0] x_cnt_47,
output	[31:0] xy_cnt_0_0,
output	[31:0] xy_cnt_0_1,
output	[31:0] xy_cnt_0_2,
output	[31:0] xy_cnt_0_3,
output	[31:0] xy_cnt_0_4,
output	[31:0] xy_cnt_0_5,
output	[31:0] xy_cnt_0_6,
output	[31:0] xy_cnt_0_7,
output	[31:0] xy_cnt_0_8,
output	[31:0] xy_cnt_0_9,
output	[31:0] xy_cnt_0_10,
output	[31:0] xy_cnt_0_11,
output	[31:0] xy_cnt_0_12,
output	[31:0] xy_cnt_0_13,
output	[31:0] xy_cnt_0_14,
output	[31:0] xy_cnt_0_15,
output	[31:0] xy_cnt_0_16,
output	[31:0] xy_cnt_0_17,
output	[31:0] xy_cnt_0_18,
output	[31:0] xy_cnt_0_19,
output	[31:0] xy_cnt_0_20,
output	[31:0] xy_cnt_0_21,
output	[31:0] xy_cnt_0_22,
output	[31:0] xy_cnt_0_23,
output	[31:0] xy_cnt_0_24,
output	[31:0] xy_cnt_0_25,
output	[31:0] xy_cnt_0_26,
output	[31:0] xy_cnt_0_27,
output	[31:0] xy_cnt_0_28,
output	[31:0] xy_cnt_0_29,
output	[31:0] xy_cnt_0_30,
output	[31:0] xy_cnt_0_31,
output	[31:0] xy_cnt_0_32,
output	[31:0] xy_cnt_0_33,
output	[31:0] xy_cnt_0_34,
output	[31:0] xy_cnt_0_35,
output	[31:0] xy_cnt_0_36,
output	[31:0] xy_cnt_0_37,
output	[31:0] xy_cnt_0_38,
output	[31:0] xy_cnt_0_39,
output	[31:0] xy_cnt_0_40,
output	[31:0] xy_cnt_0_41,
output	[31:0] xy_cnt_0_42,
output	[31:0] xy_cnt_0_43,
output	[31:0] xy_cnt_0_44,
output	[31:0] xy_cnt_0_45,
output	[31:0] xy_cnt_0_46,
output	[31:0] xy_cnt_0_47
);

wire [63:0] val_i_y_t_0;

wire [63:0] val_i_x_t_0;
wire [63:0] val_i_x_t_1;
wire [63:0] val_i_x_t_2;
wire [63:0] val_i_x_t_3;
wire [63:0] val_i_x_t_4;
wire [63:0] val_i_x_t_5;
wire [63:0] val_i_x_t_6;
wire [63:0] val_i_x_t_7;
wire [63:0] val_i_x_t_8;
wire [63:0] val_i_x_t_9;
wire [63:0] val_i_x_t_10;
wire [63:0] val_i_x_t_11;
wire [63:0] val_i_x_t_12;
wire [63:0] val_i_x_t_13;
wire [63:0] val_i_x_t_14;
wire [63:0] val_i_x_t_15;
wire [63:0] val_i_x_t_16;
wire [63:0] val_i_x_t_17;
wire [63:0] val_i_x_t_18;
wire [63:0] val_i_x_t_19;
wire [63:0] val_i_x_t_20;
wire [63:0] val_i_x_t_21;
wire [63:0] val_i_x_t_22;
wire [63:0] val_i_x_t_23;
wire [63:0] val_i_x_t_24;
wire [63:0] val_i_x_t_25;
wire [63:0] val_i_x_t_26;
wire [63:0] val_i_x_t_27;
wire [63:0] val_i_x_t_28;
wire [63:0] val_i_x_t_29;
wire [63:0] val_i_x_t_30;
wire [63:0] val_i_x_t_31;
wire [63:0] val_i_x_t_32;
wire [63:0] val_i_x_t_33;
wire [63:0] val_i_x_t_34;
wire [63:0] val_i_x_t_35;
wire [63:0] val_i_x_t_36;
wire [63:0] val_i_x_t_37;
wire [63:0] val_i_x_t_38;
wire [63:0] val_i_x_t_39;
wire [63:0] val_i_x_t_40;
wire [63:0] val_i_x_t_41;
wire [63:0] val_i_x_t_42;
wire [63:0] val_i_x_t_43;
wire [63:0] val_i_x_t_44;
wire [63:0] val_i_x_t_45;
wire [63:0] val_i_x_t_46;
wire [63:0] val_i_x_t_47;

gblock_0 #(64, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(64, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(64, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(64, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));
gblock_0 #(64, 32) gblock_0_x_4 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_4), .val_o(x_cnt_4), .val_i_t(val_i_x_t_4));
gblock_0 #(64, 32) gblock_0_x_5 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_5), .val_o(x_cnt_5), .val_i_t(val_i_x_t_5));
gblock_0 #(64, 32) gblock_0_x_6 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_6), .val_o(x_cnt_6), .val_i_t(val_i_x_t_6));
gblock_0 #(64, 32) gblock_0_x_7 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_7), .val_o(x_cnt_7), .val_i_t(val_i_x_t_7));
gblock_0 #(64, 32) gblock_0_x_8 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_8), .val_o(x_cnt_8), .val_i_t(val_i_x_t_8));
gblock_0 #(64, 32) gblock_0_x_9 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_9), .val_o(x_cnt_9), .val_i_t(val_i_x_t_9));
gblock_0 #(64, 32) gblock_0_x_10 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_10), .val_o(x_cnt_10), .val_i_t(val_i_x_t_10));
gblock_0 #(64, 32) gblock_0_x_11 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_11), .val_o(x_cnt_11), .val_i_t(val_i_x_t_11));
gblock_0 #(64, 32) gblock_0_x_12 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_12), .val_o(x_cnt_12), .val_i_t(val_i_x_t_12));
gblock_0 #(64, 32) gblock_0_x_13 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_13), .val_o(x_cnt_13), .val_i_t(val_i_x_t_13));
gblock_0 #(64, 32) gblock_0_x_14 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_14), .val_o(x_cnt_14), .val_i_t(val_i_x_t_14));
gblock_0 #(64, 32) gblock_0_x_15 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_15), .val_o(x_cnt_15), .val_i_t(val_i_x_t_15));
gblock_0 #(64, 32) gblock_0_x_16 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_16), .val_o(x_cnt_16), .val_i_t(val_i_x_t_16));
gblock_0 #(64, 32) gblock_0_x_17 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_17), .val_o(x_cnt_17), .val_i_t(val_i_x_t_17));
gblock_0 #(64, 32) gblock_0_x_18 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_18), .val_o(x_cnt_18), .val_i_t(val_i_x_t_18));
gblock_0 #(64, 32) gblock_0_x_19 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_19), .val_o(x_cnt_19), .val_i_t(val_i_x_t_19));
gblock_0 #(64, 32) gblock_0_x_20 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_20), .val_o(x_cnt_20), .val_i_t(val_i_x_t_20));
gblock_0 #(64, 32) gblock_0_x_21 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_21), .val_o(x_cnt_21), .val_i_t(val_i_x_t_21));
gblock_0 #(64, 32) gblock_0_x_22 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_22), .val_o(x_cnt_22), .val_i_t(val_i_x_t_22));
gblock_0 #(64, 32) gblock_0_x_23 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_23), .val_o(x_cnt_23), .val_i_t(val_i_x_t_23));
gblock_0 #(64, 32) gblock_0_x_24 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_24), .val_o(x_cnt_24), .val_i_t(val_i_x_t_24));
gblock_0 #(64, 32) gblock_0_x_25 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_25), .val_o(x_cnt_25), .val_i_t(val_i_x_t_25));
gblock_0 #(64, 32) gblock_0_x_26 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_26), .val_o(x_cnt_26), .val_i_t(val_i_x_t_26));
gblock_0 #(64, 32) gblock_0_x_27 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_27), .val_o(x_cnt_27), .val_i_t(val_i_x_t_27));
gblock_0 #(64, 32) gblock_0_x_28 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_28), .val_o(x_cnt_28), .val_i_t(val_i_x_t_28));
gblock_0 #(64, 32) gblock_0_x_29 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_29), .val_o(x_cnt_29), .val_i_t(val_i_x_t_29));
gblock_0 #(64, 32) gblock_0_x_30 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_30), .val_o(x_cnt_30), .val_i_t(val_i_x_t_30));
gblock_0 #(64, 32) gblock_0_x_31 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_31), .val_o(x_cnt_31), .val_i_t(val_i_x_t_31));
gblock_0 #(64, 32) gblock_0_x_32 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_32), .val_o(x_cnt_32), .val_i_t(val_i_x_t_32));
gblock_0 #(64, 32) gblock_0_x_33 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_33), .val_o(x_cnt_33), .val_i_t(val_i_x_t_33));
gblock_0 #(64, 32) gblock_0_x_34 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_34), .val_o(x_cnt_34), .val_i_t(val_i_x_t_34));
gblock_0 #(64, 32) gblock_0_x_35 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_35), .val_o(x_cnt_35), .val_i_t(val_i_x_t_35));
gblock_0 #(64, 32) gblock_0_x_36 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_36), .val_o(x_cnt_36), .val_i_t(val_i_x_t_36));
gblock_0 #(64, 32) gblock_0_x_37 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_37), .val_o(x_cnt_37), .val_i_t(val_i_x_t_37));
gblock_0 #(64, 32) gblock_0_x_38 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_38), .val_o(x_cnt_38), .val_i_t(val_i_x_t_38));
gblock_0 #(64, 32) gblock_0_x_39 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_39), .val_o(x_cnt_39), .val_i_t(val_i_x_t_39));
gblock_0 #(64, 32) gblock_0_x_40 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_40), .val_o(x_cnt_40), .val_i_t(val_i_x_t_40));
gblock_0 #(64, 32) gblock_0_x_41 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_41), .val_o(x_cnt_41), .val_i_t(val_i_x_t_41));
gblock_0 #(64, 32) gblock_0_x_42 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_42), .val_o(x_cnt_42), .val_i_t(val_i_x_t_42));
gblock_0 #(64, 32) gblock_0_x_43 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_43), .val_o(x_cnt_43), .val_i_t(val_i_x_t_43));
gblock_0 #(64, 32) gblock_0_x_44 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_44), .val_o(x_cnt_44), .val_i_t(val_i_x_t_44));
gblock_0 #(64, 32) gblock_0_x_45 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_45), .val_o(x_cnt_45), .val_i_t(val_i_x_t_45));
gblock_0 #(64, 32) gblock_0_x_46 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_46), .val_o(x_cnt_46), .val_i_t(val_i_x_t_46));
gblock_0 #(64, 32) gblock_0_x_47 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_47), .val_o(x_cnt_47), .val_i_t(val_i_x_t_47));

gblock_0 #(64, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1_special #(64, 32) gblock_1_special_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1_special #(64, 32) gblock_1_special_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1_special #(64, 32) gblock_1_special_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1_special #(64, 32) gblock_1_special_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));
gblock_1_special #(64, 32) gblock_1_special_0_4 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_4), .val_o(xy_cnt_0_4));
gblock_1_special #(64, 32) gblock_1_special_0_5 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_5), .val_o(xy_cnt_0_5));
gblock_1_special #(64, 32) gblock_1_special_0_6 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_6), .val_o(xy_cnt_0_6));
gblock_1_special #(64, 32) gblock_1_special_0_7 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_7), .val_o(xy_cnt_0_7));
gblock_1_special #(64, 32) gblock_1_special_0_8 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_8), .val_o(xy_cnt_0_8));
gblock_1_special #(64, 32) gblock_1_special_0_9 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_9), .val_o(xy_cnt_0_9));
gblock_1_special #(64, 32) gblock_1_special_0_10 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_10), .val_o(xy_cnt_0_10));
gblock_1_special #(64, 32) gblock_1_special_0_11 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_11), .val_o(xy_cnt_0_11));
gblock_1_special #(64, 32) gblock_1_special_0_12 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_12), .val_o(xy_cnt_0_12));
gblock_1_special #(64, 32) gblock_1_special_0_13 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_13), .val_o(xy_cnt_0_13));
gblock_1_special #(64, 32) gblock_1_special_0_14 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_14), .val_o(xy_cnt_0_14));
gblock_1_special #(64, 32) gblock_1_special_0_15 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_15), .val_o(xy_cnt_0_15));
gblock_1_special #(64, 32) gblock_1_special_0_16 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_16), .val_o(xy_cnt_0_16));
gblock_1_special #(64, 32) gblock_1_special_0_17 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_17), .val_o(xy_cnt_0_17));
gblock_1_special #(64, 32) gblock_1_special_0_18 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_18), .val_o(xy_cnt_0_18));
gblock_1_special #(64, 32) gblock_1_special_0_19 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_19), .val_o(xy_cnt_0_19));
gblock_1_special #(64, 32) gblock_1_special_0_20 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_20), .val_o(xy_cnt_0_20));
gblock_1_special #(64, 32) gblock_1_special_0_21 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_21), .val_o(xy_cnt_0_21));
gblock_1_special #(64, 32) gblock_1_special_0_22 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_22), .val_o(xy_cnt_0_22));
gblock_1_special #(64, 32) gblock_1_special_0_23 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_23), .val_o(xy_cnt_0_23));
gblock_1_special #(64, 32) gblock_1_special_0_24 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_24), .val_o(xy_cnt_0_24));
gblock_1_special #(64, 32) gblock_1_special_0_25 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_25), .val_o(xy_cnt_0_25));
gblock_1_special #(64, 32) gblock_1_special_0_26 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_26), .val_o(xy_cnt_0_26));
gblock_1_special #(64, 32) gblock_1_special_0_27 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_27), .val_o(xy_cnt_0_27));
gblock_1_special #(64, 32) gblock_1_special_0_28 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_28), .val_o(xy_cnt_0_28));
gblock_1_special #(64, 32) gblock_1_special_0_29 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_29), .val_o(xy_cnt_0_29));
gblock_1_special #(64, 32) gblock_1_special_0_30 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_30), .val_o(xy_cnt_0_30));
gblock_1_special #(64, 32) gblock_1_special_0_31 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_31), .val_o(xy_cnt_0_31));
gblock_1_special #(64, 32) gblock_1_special_0_32 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_32), .val_o(xy_cnt_0_32));
gblock_1_special #(64, 32) gblock_1_special_0_33 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_33), .val_o(xy_cnt_0_33));
gblock_1_special #(64, 32) gblock_1_special_0_34 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_34), .val_o(xy_cnt_0_34));
gblock_1_special #(64, 32) gblock_1_special_0_35 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_35), .val_o(xy_cnt_0_35));
gblock_1_special #(64, 32) gblock_1_special_0_36 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_36), .val_o(xy_cnt_0_36));
gblock_1_special #(64, 32) gblock_1_special_0_37 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_37), .val_o(xy_cnt_0_37));
gblock_1_special #(64, 32) gblock_1_special_0_38 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_38), .val_o(xy_cnt_0_38));
gblock_1_special #(64, 32) gblock_1_special_0_39 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_39), .val_o(xy_cnt_0_39));
gblock_1_special #(64, 32) gblock_1_special_0_40 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_40), .val_o(xy_cnt_0_40));
gblock_1_special #(64, 32) gblock_1_special_0_41 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_41), .val_o(xy_cnt_0_41));
gblock_1_special #(64, 32) gblock_1_special_0_42 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_42), .val_o(xy_cnt_0_42));
gblock_1_special #(64, 32) gblock_1_special_0_43 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_43), .val_o(xy_cnt_0_43));
gblock_1_special #(64, 32) gblock_1_special_0_44 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_44), .val_o(xy_cnt_0_44));
gblock_1_special #(64, 32) gblock_1_special_0_45 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_45), .val_o(xy_cnt_0_45));
gblock_1_special #(64, 32) gblock_1_special_0_46 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_46), .val_o(xy_cnt_0_46));
gblock_1_special #(64, 32) gblock_1_special_0_47 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_47), .val_o(xy_cnt_0_47));


endmodule


module pw_coefficients_wrapper_48_1_64_6 (
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
input	[31:0] x_cnt_8,
input	[31:0] x_cnt_9,
input	[31:0] x_cnt_10,
input	[31:0] x_cnt_11,
input	[31:0] x_cnt_12,
input	[31:0] x_cnt_13,
input	[31:0] x_cnt_14,
input	[31:0] x_cnt_15,
input	[31:0] x_cnt_16,
input	[31:0] x_cnt_17,
input	[31:0] x_cnt_18,
input	[31:0] x_cnt_19,
input	[31:0] x_cnt_20,
input	[31:0] x_cnt_21,
input	[31:0] x_cnt_22,
input	[31:0] x_cnt_23,
input	[31:0] x_cnt_24,
input	[31:0] x_cnt_25,
input	[31:0] x_cnt_26,
input	[31:0] x_cnt_27,
input	[31:0] x_cnt_28,
input	[31:0] x_cnt_29,
input	[31:0] x_cnt_30,
input	[31:0] x_cnt_31,
input	[31:0] x_cnt_32,
input	[31:0] x_cnt_33,
input	[31:0] x_cnt_34,
input	[31:0] x_cnt_35,
input	[31:0] x_cnt_36,
input	[31:0] x_cnt_37,
input	[31:0] x_cnt_38,
input	[31:0] x_cnt_39,
input	[31:0] x_cnt_40,
input	[31:0] x_cnt_41,
input	[31:0] x_cnt_42,
input	[31:0] x_cnt_43,
input	[31:0] x_cnt_44,
input	[31:0] x_cnt_45,
input	[31:0] x_cnt_46,
input	[31:0] x_cnt_47,
input	[31:0] xy_cnt_0_0,
input	[31:0] xy_cnt_0_1,
input	[31:0] xy_cnt_0_2,
input	[31:0] xy_cnt_0_3,
input	[31:0] xy_cnt_0_4,
input	[31:0] xy_cnt_0_5,
input	[31:0] xy_cnt_0_6,
input	[31:0] xy_cnt_0_7,
input	[31:0] xy_cnt_0_8,
input	[31:0] xy_cnt_0_9,
input	[31:0] xy_cnt_0_10,
input	[31:0] xy_cnt_0_11,
input	[31:0] xy_cnt_0_12,
input	[31:0] xy_cnt_0_13,
input	[31:0] xy_cnt_0_14,
input	[31:0] xy_cnt_0_15,
input	[31:0] xy_cnt_0_16,
input	[31:0] xy_cnt_0_17,
input	[31:0] xy_cnt_0_18,
input	[31:0] xy_cnt_0_19,
input	[31:0] xy_cnt_0_20,
input	[31:0] xy_cnt_0_21,
input	[31:0] xy_cnt_0_22,
input	[31:0] xy_cnt_0_23,
input	[31:0] xy_cnt_0_24,
input	[31:0] xy_cnt_0_25,
input	[31:0] xy_cnt_0_26,
input	[31:0] xy_cnt_0_27,
input	[31:0] xy_cnt_0_28,
input	[31:0] xy_cnt_0_29,
input	[31:0] xy_cnt_0_30,
input	[31:0] xy_cnt_0_31,
input	[31:0] xy_cnt_0_32,
input	[31:0] xy_cnt_0_33,
input	[31:0] xy_cnt_0_34,
input	[31:0] xy_cnt_0_35,
input	[31:0] xy_cnt_0_36,
input	[31:0] xy_cnt_0_37,
input	[31:0] xy_cnt_0_38,
input	[31:0] xy_cnt_0_39,
input	[31:0] xy_cnt_0_40,
input	[31:0] xy_cnt_0_41,
input	[31:0] xy_cnt_0_42,
input	[31:0] xy_cnt_0_43,
input	[31:0] xy_cnt_0_44,
input	[31:0] xy_cnt_0_45,
input	[31:0] xy_cnt_0_46,
input	[31:0] xy_cnt_0_47,
output linkdiseq_wren_0_0,
output linkdiseq_wren_0_1,
output linkdiseq_wren_0_2,
output linkdiseq_wren_0_3,
output linkdiseq_wren_0_4,
output linkdiseq_wren_0_5,
output linkdiseq_wren_0_6,
output linkdiseq_wren_0_7,
output linkdiseq_wren_0_8,
output linkdiseq_wren_0_9,
output linkdiseq_wren_0_10,
output linkdiseq_wren_0_11,
output linkdiseq_wren_0_12,
output linkdiseq_wren_0_13,
output linkdiseq_wren_0_14,
output linkdiseq_wren_0_15,
output linkdiseq_wren_0_16,
output linkdiseq_wren_0_17,
output linkdiseq_wren_0_18,
output linkdiseq_wren_0_19,
output linkdiseq_wren_0_20,
output linkdiseq_wren_0_21,
output linkdiseq_wren_0_22,
output linkdiseq_wren_0_23,
output linkdiseq_wren_0_24,
output linkdiseq_wren_0_25,
output linkdiseq_wren_0_26,
output linkdiseq_wren_0_27,
output linkdiseq_wren_0_28,
output linkdiseq_wren_0_29,
output linkdiseq_wren_0_30,
output linkdiseq_wren_0_31,
output linkdiseq_wren_0_32,
output linkdiseq_wren_0_33,
output linkdiseq_wren_0_34,
output linkdiseq_wren_0_35,
output linkdiseq_wren_0_36,
output linkdiseq_wren_0_37,
output linkdiseq_wren_0_38,
output linkdiseq_wren_0_39,
output linkdiseq_wren_0_40,
output linkdiseq_wren_0_41,
output linkdiseq_wren_0_42,
output linkdiseq_wren_0_43,
output linkdiseq_wren_0_44,
output linkdiseq_wren_0_45,
output linkdiseq_wren_0_46,
output linkdiseq_wren_0_47,
output	[31:0] linkdiseq_0_0,
output	[31:0] linkdiseq_0_1,
output	[31:0] linkdiseq_0_2,
output	[31:0] linkdiseq_0_3,
output	[31:0] linkdiseq_0_4,
output	[31:0] linkdiseq_0_5,
output	[31:0] linkdiseq_0_6,
output	[31:0] linkdiseq_0_7,
output	[31:0] linkdiseq_0_8,
output	[31:0] linkdiseq_0_9,
output	[31:0] linkdiseq_0_10,
output	[31:0] linkdiseq_0_11,
output	[31:0] linkdiseq_0_12,
output	[31:0] linkdiseq_0_13,
output	[31:0] linkdiseq_0_14,
output	[31:0] linkdiseq_0_15,
output	[31:0] linkdiseq_0_16,
output	[31:0] linkdiseq_0_17,
output	[31:0] linkdiseq_0_18,
output	[31:0] linkdiseq_0_19,
output	[31:0] linkdiseq_0_20,
output	[31:0] linkdiseq_0_21,
output	[31:0] linkdiseq_0_22,
output	[31:0] linkdiseq_0_23,
output	[31:0] linkdiseq_0_24,
output	[31:0] linkdiseq_0_25,
output	[31:0] linkdiseq_0_26,
output	[31:0] linkdiseq_0_27,
output	[31:0] linkdiseq_0_28,
output	[31:0] linkdiseq_0_29,
output	[31:0] linkdiseq_0_30,
output	[31:0] linkdiseq_0_31,
output	[31:0] linkdiseq_0_32,
output	[31:0] linkdiseq_0_33,
output	[31:0] linkdiseq_0_34,
output	[31:0] linkdiseq_0_35,
output	[31:0] linkdiseq_0_36,
output	[31:0] linkdiseq_0_37,
output	[31:0] linkdiseq_0_38,
output	[31:0] linkdiseq_0_39,
output	[31:0] linkdiseq_0_40,
output	[31:0] linkdiseq_0_41,
output	[31:0] linkdiseq_0_42,
output	[31:0] linkdiseq_0_43,
output	[31:0] linkdiseq_0_44,
output	[31:0] linkdiseq_0_45,
output	[31:0] linkdiseq_0_46,
output	[31:0] linkdiseq_0_47
);

similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_0 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_0), .c(xy_cnt_0_0), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_0), .linkdiseq_coefficient(linkdiseq_0_0));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_1 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_1), .c(xy_cnt_0_1), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_1), .linkdiseq_coefficient(linkdiseq_0_1));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_2 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_2), .c(xy_cnt_0_2), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_2), .linkdiseq_coefficient(linkdiseq_0_2));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_3 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_3), .c(xy_cnt_0_3), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_3), .linkdiseq_coefficient(linkdiseq_0_3));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_4 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_4), .c(xy_cnt_0_4), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_4), .linkdiseq_coefficient(linkdiseq_0_4));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_5 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_5), .c(xy_cnt_0_5), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_5), .linkdiseq_coefficient(linkdiseq_0_5));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_6 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_6), .c(xy_cnt_0_6), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_6), .linkdiseq_coefficient(linkdiseq_0_6));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_7 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_7), .c(xy_cnt_0_7), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_7), .linkdiseq_coefficient(linkdiseq_0_7));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_8 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_8), .c(xy_cnt_0_8), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_8), .linkdiseq_coefficient(linkdiseq_0_8));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_9 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_9), .c(xy_cnt_0_9), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_9), .linkdiseq_coefficient(linkdiseq_0_9));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_10 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_10), .c(xy_cnt_0_10), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_10), .linkdiseq_coefficient(linkdiseq_0_10));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_11 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_11), .c(xy_cnt_0_11), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_11), .linkdiseq_coefficient(linkdiseq_0_11));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_12 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_12), .c(xy_cnt_0_12), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_12), .linkdiseq_coefficient(linkdiseq_0_12));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_13 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_13), .c(xy_cnt_0_13), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_13), .linkdiseq_coefficient(linkdiseq_0_13));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_14 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_14), .c(xy_cnt_0_14), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_14), .linkdiseq_coefficient(linkdiseq_0_14));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_15 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_15), .c(xy_cnt_0_15), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_15), .linkdiseq_coefficient(linkdiseq_0_15));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_16 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_16), .c(xy_cnt_0_16), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_16), .linkdiseq_coefficient(linkdiseq_0_16));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_17 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_17), .c(xy_cnt_0_17), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_17), .linkdiseq_coefficient(linkdiseq_0_17));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_18 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_18), .c(xy_cnt_0_18), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_18), .linkdiseq_coefficient(linkdiseq_0_18));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_19 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_19), .c(xy_cnt_0_19), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_19), .linkdiseq_coefficient(linkdiseq_0_19));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_20 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_20), .c(xy_cnt_0_20), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_20), .linkdiseq_coefficient(linkdiseq_0_20));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_21 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_21), .c(xy_cnt_0_21), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_21), .linkdiseq_coefficient(linkdiseq_0_21));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_22 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_22), .c(xy_cnt_0_22), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_22), .linkdiseq_coefficient(linkdiseq_0_22));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_23 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_23), .c(xy_cnt_0_23), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_23), .linkdiseq_coefficient(linkdiseq_0_23));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_24 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_24), .c(xy_cnt_0_24), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_24), .linkdiseq_coefficient(linkdiseq_0_24));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_25 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_25), .c(xy_cnt_0_25), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_25), .linkdiseq_coefficient(linkdiseq_0_25));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_26 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_26), .c(xy_cnt_0_26), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_26), .linkdiseq_coefficient(linkdiseq_0_26));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_27 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_27), .c(xy_cnt_0_27), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_27), .linkdiseq_coefficient(linkdiseq_0_27));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_28 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_28), .c(xy_cnt_0_28), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_28), .linkdiseq_coefficient(linkdiseq_0_28));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_29 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_29), .c(xy_cnt_0_29), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_29), .linkdiseq_coefficient(linkdiseq_0_29));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_30 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_30), .c(xy_cnt_0_30), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_30), .linkdiseq_coefficient(linkdiseq_0_30));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_31 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_31), .c(xy_cnt_0_31), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_31), .linkdiseq_coefficient(linkdiseq_0_31));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_32 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_32), .c(xy_cnt_0_32), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_32), .linkdiseq_coefficient(linkdiseq_0_32));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_33 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_33), .c(xy_cnt_0_33), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_33), .linkdiseq_coefficient(linkdiseq_0_33));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_34 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_34), .c(xy_cnt_0_34), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_34), .linkdiseq_coefficient(linkdiseq_0_34));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_35 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_35), .c(xy_cnt_0_35), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_35), .linkdiseq_coefficient(linkdiseq_0_35));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_36 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_36), .c(xy_cnt_0_36), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_36), .linkdiseq_coefficient(linkdiseq_0_36));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_37 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_37), .c(xy_cnt_0_37), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_37), .linkdiseq_coefficient(linkdiseq_0_37));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_38 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_38), .c(xy_cnt_0_38), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_38), .linkdiseq_coefficient(linkdiseq_0_38));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_39 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_39), .c(xy_cnt_0_39), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_39), .linkdiseq_coefficient(linkdiseq_0_39));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_40 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_40), .c(xy_cnt_0_40), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_40), .linkdiseq_coefficient(linkdiseq_0_40));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_41 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_41), .c(xy_cnt_0_41), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_41), .linkdiseq_coefficient(linkdiseq_0_41));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_42 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_42), .c(xy_cnt_0_42), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_42), .linkdiseq_coefficient(linkdiseq_0_42));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_43 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_43), .c(xy_cnt_0_43), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_43), .linkdiseq_coefficient(linkdiseq_0_43));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_44 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_44), .c(xy_cnt_0_44), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_44), .linkdiseq_coefficient(linkdiseq_0_44));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_45 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_45), .c(xy_cnt_0_45), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_45), .linkdiseq_coefficient(linkdiseq_0_45));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_46 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_46), .c(xy_cnt_0_46), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_46), .linkdiseq_coefficient(linkdiseq_0_46));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_47 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_47), .c(xy_cnt_0_47), .sample_size (sample_size), .vld_out(linkdiseq_wren_0_47), .linkdiseq_coefficient(linkdiseq_0_47));


endmodule




module pru_array (
input clk,
input rst,
input	[31:0] element_length_t,
input	[31:0] sample_size,
input vld_in,
input	[63:0] val_i_y_0,
input	[63:0] val_i_x_0,
input	[63:0] val_i_x_1,
input	[63:0] val_i_x_2,
input	[63:0] val_i_x_3,
input	[63:0] val_i_x_4,
input	[63:0] val_i_x_5,
input	[63:0] val_i_x_6,
input	[63:0] val_i_x_7,
input	[63:0] val_i_x_8,
input	[63:0] val_i_x_9,
input	[63:0] val_i_x_10,
input	[63:0] val_i_x_11,
input	[63:0] val_i_x_12,
input	[63:0] val_i_x_13,
input	[63:0] val_i_x_14,
input	[63:0] val_i_x_15,
input	[63:0] val_i_x_16,
input	[63:0] val_i_x_17,
input	[63:0] val_i_x_18,
input	[63:0] val_i_x_19,
input	[63:0] val_i_x_20,
input	[63:0] val_i_x_21,
input	[63:0] val_i_x_22,
input	[63:0] val_i_x_23,
input	[63:0] val_i_x_24,
input	[63:0] val_i_x_25,
input	[63:0] val_i_x_26,
input	[63:0] val_i_x_27,
input	[63:0] val_i_x_28,
input	[63:0] val_i_x_29,
input	[63:0] val_i_x_30,
input	[63:0] val_i_x_31,
input	[63:0] val_i_x_32,
input	[63:0] val_i_x_33,
input	[63:0] val_i_x_34,
input	[63:0] val_i_x_35,
input	[63:0] val_i_x_36,
input	[63:0] val_i_x_37,
input	[63:0] val_i_x_38,
input	[63:0] val_i_x_39,
input	[63:0] val_i_x_40,
input	[63:0] val_i_x_41,
input	[63:0] val_i_x_42,
input	[63:0] val_i_x_43,
input	[63:0] val_i_x_44,
input	[63:0] val_i_x_45,
input	[63:0] val_i_x_46,
input	[63:0] val_i_x_47,
output linkdiseq_wren_0_0,
output linkdiseq_wren_0_1,
output linkdiseq_wren_0_2,
output linkdiseq_wren_0_3,
output linkdiseq_wren_0_4,
output linkdiseq_wren_0_5,
output linkdiseq_wren_0_6,
output linkdiseq_wren_0_7,
output linkdiseq_wren_0_8,
output linkdiseq_wren_0_9,
output linkdiseq_wren_0_10,
output linkdiseq_wren_0_11,
output linkdiseq_wren_0_12,
output linkdiseq_wren_0_13,
output linkdiseq_wren_0_14,
output linkdiseq_wren_0_15,
output linkdiseq_wren_0_16,
output linkdiseq_wren_0_17,
output linkdiseq_wren_0_18,
output linkdiseq_wren_0_19,
output linkdiseq_wren_0_20,
output linkdiseq_wren_0_21,
output linkdiseq_wren_0_22,
output linkdiseq_wren_0_23,
output linkdiseq_wren_0_24,
output linkdiseq_wren_0_25,
output linkdiseq_wren_0_26,
output linkdiseq_wren_0_27,
output linkdiseq_wren_0_28,
output linkdiseq_wren_0_29,
output linkdiseq_wren_0_30,
output linkdiseq_wren_0_31,
output linkdiseq_wren_0_32,
output linkdiseq_wren_0_33,
output linkdiseq_wren_0_34,
output linkdiseq_wren_0_35,
output linkdiseq_wren_0_36,
output linkdiseq_wren_0_37,
output linkdiseq_wren_0_38,
output linkdiseq_wren_0_39,
output linkdiseq_wren_0_40,
output linkdiseq_wren_0_41,
output linkdiseq_wren_0_42,
output linkdiseq_wren_0_43,
output linkdiseq_wren_0_44,
output linkdiseq_wren_0_45,
output linkdiseq_wren_0_46,
output linkdiseq_wren_0_47,
output	[31:0] linkdiseq_0_0,
output	[31:0] linkdiseq_0_1,
output	[31:0] linkdiseq_0_2,
output	[31:0] linkdiseq_0_3,
output	[31:0] linkdiseq_0_4,
output	[31:0] linkdiseq_0_5,
output	[31:0] linkdiseq_0_6,
output	[31:0] linkdiseq_0_7,
output	[31:0] linkdiseq_0_8,
output	[31:0] linkdiseq_0_9,
output	[31:0] linkdiseq_0_10,
output	[31:0] linkdiseq_0_11,
output	[31:0] linkdiseq_0_12,
output	[31:0] linkdiseq_0_13,
output	[31:0] linkdiseq_0_14,
output	[31:0] linkdiseq_0_15,
output	[31:0] linkdiseq_0_16,
output	[31:0] linkdiseq_0_17,
output	[31:0] linkdiseq_0_18,
output	[31:0] linkdiseq_0_19,
output	[31:0] linkdiseq_0_20,
output	[31:0] linkdiseq_0_21,
output	[31:0] linkdiseq_0_22,
output	[31:0] linkdiseq_0_23,
output	[31:0] linkdiseq_0_24,
output	[31:0] linkdiseq_0_25,
output	[31:0] linkdiseq_0_26,
output	[31:0] linkdiseq_0_27,
output	[31:0] linkdiseq_0_28,
output	[31:0] linkdiseq_0_29,
output	[31:0] linkdiseq_0_30,
output	[31:0] linkdiseq_0_31,
output	[31:0] linkdiseq_0_32,
output	[31:0] linkdiseq_0_33,
output	[31:0] linkdiseq_0_34,
output	[31:0] linkdiseq_0_35,
output	[31:0] linkdiseq_0_36,
output	[31:0] linkdiseq_0_37,
output	[31:0] linkdiseq_0_38,
output	[31:0] linkdiseq_0_39,
output	[31:0] linkdiseq_0_40,
output	[31:0] linkdiseq_0_41,
output	[31:0] linkdiseq_0_42,
output	[31:0] linkdiseq_0_43,
output	[31:0] linkdiseq_0_44,
output	[31:0] linkdiseq_0_45,
output	[31:0] linkdiseq_0_46,
output	[31:0] linkdiseq_0_47
);

wire vld_in_t;
du #(1, 6) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
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
wire [31:0] t_x_cnt_8;
wire [31:0] t_x_cnt_9;
wire [31:0] t_x_cnt_10;
wire [31:0] t_x_cnt_11;
wire [31:0] t_x_cnt_12;
wire [31:0] t_x_cnt_13;
wire [31:0] t_x_cnt_14;
wire [31:0] t_x_cnt_15;
wire [31:0] t_x_cnt_16;
wire [31:0] t_x_cnt_17;
wire [31:0] t_x_cnt_18;
wire [31:0] t_x_cnt_19;
wire [31:0] t_x_cnt_20;
wire [31:0] t_x_cnt_21;
wire [31:0] t_x_cnt_22;
wire [31:0] t_x_cnt_23;
wire [31:0] t_x_cnt_24;
wire [31:0] t_x_cnt_25;
wire [31:0] t_x_cnt_26;
wire [31:0] t_x_cnt_27;
wire [31:0] t_x_cnt_28;
wire [31:0] t_x_cnt_29;
wire [31:0] t_x_cnt_30;
wire [31:0] t_x_cnt_31;
wire [31:0] t_x_cnt_32;
wire [31:0] t_x_cnt_33;
wire [31:0] t_x_cnt_34;
wire [31:0] t_x_cnt_35;
wire [31:0] t_x_cnt_36;
wire [31:0] t_x_cnt_37;
wire [31:0] t_x_cnt_38;
wire [31:0] t_x_cnt_39;
wire [31:0] t_x_cnt_40;
wire [31:0] t_x_cnt_41;
wire [31:0] t_x_cnt_42;
wire [31:0] t_x_cnt_43;
wire [31:0] t_x_cnt_44;
wire [31:0] t_x_cnt_45;
wire [31:0] t_x_cnt_46;
wire [31:0] t_x_cnt_47;

wire [31:0] t_xy_cnt_0_0;
wire [31:0] t_xy_cnt_0_1;
wire [31:0] t_xy_cnt_0_2;
wire [31:0] t_xy_cnt_0_3;
wire [31:0] t_xy_cnt_0_4;
wire [31:0] t_xy_cnt_0_5;
wire [31:0] t_xy_cnt_0_6;
wire [31:0] t_xy_cnt_0_7;
wire [31:0] t_xy_cnt_0_8;
wire [31:0] t_xy_cnt_0_9;
wire [31:0] t_xy_cnt_0_10;
wire [31:0] t_xy_cnt_0_11;
wire [31:0] t_xy_cnt_0_12;
wire [31:0] t_xy_cnt_0_13;
wire [31:0] t_xy_cnt_0_14;
wire [31:0] t_xy_cnt_0_15;
wire [31:0] t_xy_cnt_0_16;
wire [31:0] t_xy_cnt_0_17;
wire [31:0] t_xy_cnt_0_18;
wire [31:0] t_xy_cnt_0_19;
wire [31:0] t_xy_cnt_0_20;
wire [31:0] t_xy_cnt_0_21;
wire [31:0] t_xy_cnt_0_22;
wire [31:0] t_xy_cnt_0_23;
wire [31:0] t_xy_cnt_0_24;
wire [31:0] t_xy_cnt_0_25;
wire [31:0] t_xy_cnt_0_26;
wire [31:0] t_xy_cnt_0_27;
wire [31:0] t_xy_cnt_0_28;
wire [31:0] t_xy_cnt_0_29;
wire [31:0] t_xy_cnt_0_30;
wire [31:0] t_xy_cnt_0_31;
wire [31:0] t_xy_cnt_0_32;
wire [31:0] t_xy_cnt_0_33;
wire [31:0] t_xy_cnt_0_34;
wire [31:0] t_xy_cnt_0_35;
wire [31:0] t_xy_cnt_0_36;
wire [31:0] t_xy_cnt_0_37;
wire [31:0] t_xy_cnt_0_38;
wire [31:0] t_xy_cnt_0_39;
wire [31:0] t_xy_cnt_0_40;
wire [31:0] t_xy_cnt_0_41;
wire [31:0] t_xy_cnt_0_42;
wire [31:0] t_xy_cnt_0_43;
wire [31:0] t_xy_cnt_0_44;
wire [31:0] t_xy_cnt_0_45;
wire [31:0] t_xy_cnt_0_46;
wire [31:0] t_xy_cnt_0_47;
pw_sim_48_1_64_6 pw_sim_48_1_64_6_0(
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
 .val_i_x_8(val_i_x_8),
 .x_cnt_8(t_x_cnt_8),
 .val_i_x_9(val_i_x_9),
 .x_cnt_9(t_x_cnt_9),
 .val_i_x_10(val_i_x_10),
 .x_cnt_10(t_x_cnt_10),
 .val_i_x_11(val_i_x_11),
 .x_cnt_11(t_x_cnt_11),
 .val_i_x_12(val_i_x_12),
 .x_cnt_12(t_x_cnt_12),
 .val_i_x_13(val_i_x_13),
 .x_cnt_13(t_x_cnt_13),
 .val_i_x_14(val_i_x_14),
 .x_cnt_14(t_x_cnt_14),
 .val_i_x_15(val_i_x_15),
 .x_cnt_15(t_x_cnt_15),
 .val_i_x_16(val_i_x_16),
 .x_cnt_16(t_x_cnt_16),
 .val_i_x_17(val_i_x_17),
 .x_cnt_17(t_x_cnt_17),
 .val_i_x_18(val_i_x_18),
 .x_cnt_18(t_x_cnt_18),
 .val_i_x_19(val_i_x_19),
 .x_cnt_19(t_x_cnt_19),
 .val_i_x_20(val_i_x_20),
 .x_cnt_20(t_x_cnt_20),
 .val_i_x_21(val_i_x_21),
 .x_cnt_21(t_x_cnt_21),
 .val_i_x_22(val_i_x_22),
 .x_cnt_22(t_x_cnt_22),
 .val_i_x_23(val_i_x_23),
 .x_cnt_23(t_x_cnt_23),
 .val_i_x_24(val_i_x_24),
 .x_cnt_24(t_x_cnt_24),
 .val_i_x_25(val_i_x_25),
 .x_cnt_25(t_x_cnt_25),
 .val_i_x_26(val_i_x_26),
 .x_cnt_26(t_x_cnt_26),
 .val_i_x_27(val_i_x_27),
 .x_cnt_27(t_x_cnt_27),
 .val_i_x_28(val_i_x_28),
 .x_cnt_28(t_x_cnt_28),
 .val_i_x_29(val_i_x_29),
 .x_cnt_29(t_x_cnt_29),
 .val_i_x_30(val_i_x_30),
 .x_cnt_30(t_x_cnt_30),
 .val_i_x_31(val_i_x_31),
 .x_cnt_31(t_x_cnt_31),
 .val_i_x_32(val_i_x_32),
 .x_cnt_32(t_x_cnt_32),
 .val_i_x_33(val_i_x_33),
 .x_cnt_33(t_x_cnt_33),
 .val_i_x_34(val_i_x_34),
 .x_cnt_34(t_x_cnt_34),
 .val_i_x_35(val_i_x_35),
 .x_cnt_35(t_x_cnt_35),
 .val_i_x_36(val_i_x_36),
 .x_cnt_36(t_x_cnt_36),
 .val_i_x_37(val_i_x_37),
 .x_cnt_37(t_x_cnt_37),
 .val_i_x_38(val_i_x_38),
 .x_cnt_38(t_x_cnt_38),
 .val_i_x_39(val_i_x_39),
 .x_cnt_39(t_x_cnt_39),
 .val_i_x_40(val_i_x_40),
 .x_cnt_40(t_x_cnt_40),
 .val_i_x_41(val_i_x_41),
 .x_cnt_41(t_x_cnt_41),
 .val_i_x_42(val_i_x_42),
 .x_cnt_42(t_x_cnt_42),
 .val_i_x_43(val_i_x_43),
 .x_cnt_43(t_x_cnt_43),
 .val_i_x_44(val_i_x_44),
 .x_cnt_44(t_x_cnt_44),
 .val_i_x_45(val_i_x_45),
 .x_cnt_45(t_x_cnt_45),
 .val_i_x_46(val_i_x_46),
 .x_cnt_46(t_x_cnt_46),
 .val_i_x_47(val_i_x_47),
 .x_cnt_47(t_x_cnt_47),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7),
 .xy_cnt_0_8(t_xy_cnt_0_8),
 .xy_cnt_0_9(t_xy_cnt_0_9),
 .xy_cnt_0_10(t_xy_cnt_0_10),
 .xy_cnt_0_11(t_xy_cnt_0_11),
 .xy_cnt_0_12(t_xy_cnt_0_12),
 .xy_cnt_0_13(t_xy_cnt_0_13),
 .xy_cnt_0_14(t_xy_cnt_0_14),
 .xy_cnt_0_15(t_xy_cnt_0_15),
 .xy_cnt_0_16(t_xy_cnt_0_16),
 .xy_cnt_0_17(t_xy_cnt_0_17),
 .xy_cnt_0_18(t_xy_cnt_0_18),
 .xy_cnt_0_19(t_xy_cnt_0_19),
 .xy_cnt_0_20(t_xy_cnt_0_20),
 .xy_cnt_0_21(t_xy_cnt_0_21),
 .xy_cnt_0_22(t_xy_cnt_0_22),
 .xy_cnt_0_23(t_xy_cnt_0_23),
 .xy_cnt_0_24(t_xy_cnt_0_24),
 .xy_cnt_0_25(t_xy_cnt_0_25),
 .xy_cnt_0_26(t_xy_cnt_0_26),
 .xy_cnt_0_27(t_xy_cnt_0_27),
 .xy_cnt_0_28(t_xy_cnt_0_28),
 .xy_cnt_0_29(t_xy_cnt_0_29),
 .xy_cnt_0_30(t_xy_cnt_0_30),
 .xy_cnt_0_31(t_xy_cnt_0_31),
 .xy_cnt_0_32(t_xy_cnt_0_32),
 .xy_cnt_0_33(t_xy_cnt_0_33),
 .xy_cnt_0_34(t_xy_cnt_0_34),
 .xy_cnt_0_35(t_xy_cnt_0_35),
 .xy_cnt_0_36(t_xy_cnt_0_36),
 .xy_cnt_0_37(t_xy_cnt_0_37),
 .xy_cnt_0_38(t_xy_cnt_0_38),
 .xy_cnt_0_39(t_xy_cnt_0_39),
 .xy_cnt_0_40(t_xy_cnt_0_40),
 .xy_cnt_0_41(t_xy_cnt_0_41),
 .xy_cnt_0_42(t_xy_cnt_0_42),
 .xy_cnt_0_43(t_xy_cnt_0_43),
 .xy_cnt_0_44(t_xy_cnt_0_44),
 .xy_cnt_0_45(t_xy_cnt_0_45),
 .xy_cnt_0_46(t_xy_cnt_0_46),
 .xy_cnt_0_47(t_xy_cnt_0_47)
);
pw_coefficients_wrapper_48_1_64_6 pw_coefficients_wrapper_48_1_64_6_0(
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
 .x_cnt_8(t_x_cnt_8),
 .x_cnt_9(t_x_cnt_9),
 .x_cnt_10(t_x_cnt_10),
 .x_cnt_11(t_x_cnt_11),
 .x_cnt_12(t_x_cnt_12),
 .x_cnt_13(t_x_cnt_13),
 .x_cnt_14(t_x_cnt_14),
 .x_cnt_15(t_x_cnt_15),
 .x_cnt_16(t_x_cnt_16),
 .x_cnt_17(t_x_cnt_17),
 .x_cnt_18(t_x_cnt_18),
 .x_cnt_19(t_x_cnt_19),
 .x_cnt_20(t_x_cnt_20),
 .x_cnt_21(t_x_cnt_21),
 .x_cnt_22(t_x_cnt_22),
 .x_cnt_23(t_x_cnt_23),
 .x_cnt_24(t_x_cnt_24),
 .x_cnt_25(t_x_cnt_25),
 .x_cnt_26(t_x_cnt_26),
 .x_cnt_27(t_x_cnt_27),
 .x_cnt_28(t_x_cnt_28),
 .x_cnt_29(t_x_cnt_29),
 .x_cnt_30(t_x_cnt_30),
 .x_cnt_31(t_x_cnt_31),
 .x_cnt_32(t_x_cnt_32),
 .x_cnt_33(t_x_cnt_33),
 .x_cnt_34(t_x_cnt_34),
 .x_cnt_35(t_x_cnt_35),
 .x_cnt_36(t_x_cnt_36),
 .x_cnt_37(t_x_cnt_37),
 .x_cnt_38(t_x_cnt_38),
 .x_cnt_39(t_x_cnt_39),
 .x_cnt_40(t_x_cnt_40),
 .x_cnt_41(t_x_cnt_41),
 .x_cnt_42(t_x_cnt_42),
 .x_cnt_43(t_x_cnt_43),
 .x_cnt_44(t_x_cnt_44),
 .x_cnt_45(t_x_cnt_45),
 .x_cnt_46(t_x_cnt_46),
 .x_cnt_47(t_x_cnt_47),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7),
 .xy_cnt_0_8(t_xy_cnt_0_8),
 .xy_cnt_0_9(t_xy_cnt_0_9),
 .xy_cnt_0_10(t_xy_cnt_0_10),
 .xy_cnt_0_11(t_xy_cnt_0_11),
 .xy_cnt_0_12(t_xy_cnt_0_12),
 .xy_cnt_0_13(t_xy_cnt_0_13),
 .xy_cnt_0_14(t_xy_cnt_0_14),
 .xy_cnt_0_15(t_xy_cnt_0_15),
 .xy_cnt_0_16(t_xy_cnt_0_16),
 .xy_cnt_0_17(t_xy_cnt_0_17),
 .xy_cnt_0_18(t_xy_cnt_0_18),
 .xy_cnt_0_19(t_xy_cnt_0_19),
 .xy_cnt_0_20(t_xy_cnt_0_20),
 .xy_cnt_0_21(t_xy_cnt_0_21),
 .xy_cnt_0_22(t_xy_cnt_0_22),
 .xy_cnt_0_23(t_xy_cnt_0_23),
 .xy_cnt_0_24(t_xy_cnt_0_24),
 .xy_cnt_0_25(t_xy_cnt_0_25),
 .xy_cnt_0_26(t_xy_cnt_0_26),
 .xy_cnt_0_27(t_xy_cnt_0_27),
 .xy_cnt_0_28(t_xy_cnt_0_28),
 .xy_cnt_0_29(t_xy_cnt_0_29),
 .xy_cnt_0_30(t_xy_cnt_0_30),
 .xy_cnt_0_31(t_xy_cnt_0_31),
 .xy_cnt_0_32(t_xy_cnt_0_32),
 .xy_cnt_0_33(t_xy_cnt_0_33),
 .xy_cnt_0_34(t_xy_cnt_0_34),
 .xy_cnt_0_35(t_xy_cnt_0_35),
 .xy_cnt_0_36(t_xy_cnt_0_36),
 .xy_cnt_0_37(t_xy_cnt_0_37),
 .xy_cnt_0_38(t_xy_cnt_0_38),
 .xy_cnt_0_39(t_xy_cnt_0_39),
 .xy_cnt_0_40(t_xy_cnt_0_40),
 .xy_cnt_0_41(t_xy_cnt_0_41),
 .xy_cnt_0_42(t_xy_cnt_0_42),
 .xy_cnt_0_43(t_xy_cnt_0_43),
 .xy_cnt_0_44(t_xy_cnt_0_44),
 .xy_cnt_0_45(t_xy_cnt_0_45),
 .xy_cnt_0_46(t_xy_cnt_0_46),
 .xy_cnt_0_47(t_xy_cnt_0_47),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7),
 .linkdiseq_wren_0_8(linkdiseq_wren_0_8),
 .linkdiseq_wren_0_9(linkdiseq_wren_0_9),
 .linkdiseq_wren_0_10(linkdiseq_wren_0_10),
 .linkdiseq_wren_0_11(linkdiseq_wren_0_11),
 .linkdiseq_wren_0_12(linkdiseq_wren_0_12),
 .linkdiseq_wren_0_13(linkdiseq_wren_0_13),
 .linkdiseq_wren_0_14(linkdiseq_wren_0_14),
 .linkdiseq_wren_0_15(linkdiseq_wren_0_15),
 .linkdiseq_wren_0_16(linkdiseq_wren_0_16),
 .linkdiseq_wren_0_17(linkdiseq_wren_0_17),
 .linkdiseq_wren_0_18(linkdiseq_wren_0_18),
 .linkdiseq_wren_0_19(linkdiseq_wren_0_19),
 .linkdiseq_wren_0_20(linkdiseq_wren_0_20),
 .linkdiseq_wren_0_21(linkdiseq_wren_0_21),
 .linkdiseq_wren_0_22(linkdiseq_wren_0_22),
 .linkdiseq_wren_0_23(linkdiseq_wren_0_23),
 .linkdiseq_wren_0_24(linkdiseq_wren_0_24),
 .linkdiseq_wren_0_25(linkdiseq_wren_0_25),
 .linkdiseq_wren_0_26(linkdiseq_wren_0_26),
 .linkdiseq_wren_0_27(linkdiseq_wren_0_27),
 .linkdiseq_wren_0_28(linkdiseq_wren_0_28),
 .linkdiseq_wren_0_29(linkdiseq_wren_0_29),
 .linkdiseq_wren_0_30(linkdiseq_wren_0_30),
 .linkdiseq_wren_0_31(linkdiseq_wren_0_31),
 .linkdiseq_wren_0_32(linkdiseq_wren_0_32),
 .linkdiseq_wren_0_33(linkdiseq_wren_0_33),
 .linkdiseq_wren_0_34(linkdiseq_wren_0_34),
 .linkdiseq_wren_0_35(linkdiseq_wren_0_35),
 .linkdiseq_wren_0_36(linkdiseq_wren_0_36),
 .linkdiseq_wren_0_37(linkdiseq_wren_0_37),
 .linkdiseq_wren_0_38(linkdiseq_wren_0_38),
 .linkdiseq_wren_0_39(linkdiseq_wren_0_39),
 .linkdiseq_wren_0_40(linkdiseq_wren_0_40),
 .linkdiseq_wren_0_41(linkdiseq_wren_0_41),
 .linkdiseq_wren_0_42(linkdiseq_wren_0_42),
 .linkdiseq_wren_0_43(linkdiseq_wren_0_43),
 .linkdiseq_wren_0_44(linkdiseq_wren_0_44),
 .linkdiseq_wren_0_45(linkdiseq_wren_0_45),
 .linkdiseq_wren_0_46(linkdiseq_wren_0_46),
 .linkdiseq_wren_0_47(linkdiseq_wren_0_47),
 .linkdiseq_0_0(linkdiseq_0_0),
 .linkdiseq_0_1(linkdiseq_0_1),
 .linkdiseq_0_2(linkdiseq_0_2),
 .linkdiseq_0_3(linkdiseq_0_3),
 .linkdiseq_0_4(linkdiseq_0_4),
 .linkdiseq_0_5(linkdiseq_0_5),
 .linkdiseq_0_6(linkdiseq_0_6),
 .linkdiseq_0_7(linkdiseq_0_7),
 .linkdiseq_0_8(linkdiseq_0_8),
 .linkdiseq_0_9(linkdiseq_0_9),
 .linkdiseq_0_10(linkdiseq_0_10),
 .linkdiseq_0_11(linkdiseq_0_11),
 .linkdiseq_0_12(linkdiseq_0_12),
 .linkdiseq_0_13(linkdiseq_0_13),
 .linkdiseq_0_14(linkdiseq_0_14),
 .linkdiseq_0_15(linkdiseq_0_15),
 .linkdiseq_0_16(linkdiseq_0_16),
 .linkdiseq_0_17(linkdiseq_0_17),
 .linkdiseq_0_18(linkdiseq_0_18),
 .linkdiseq_0_19(linkdiseq_0_19),
 .linkdiseq_0_20(linkdiseq_0_20),
 .linkdiseq_0_21(linkdiseq_0_21),
 .linkdiseq_0_22(linkdiseq_0_22),
 .linkdiseq_0_23(linkdiseq_0_23),
 .linkdiseq_0_24(linkdiseq_0_24),
 .linkdiseq_0_25(linkdiseq_0_25),
 .linkdiseq_0_26(linkdiseq_0_26),
 .linkdiseq_0_27(linkdiseq_0_27),
 .linkdiseq_0_28(linkdiseq_0_28),
 .linkdiseq_0_29(linkdiseq_0_29),
 .linkdiseq_0_30(linkdiseq_0_30),
 .linkdiseq_0_31(linkdiseq_0_31),
 .linkdiseq_0_32(linkdiseq_0_32),
 .linkdiseq_0_33(linkdiseq_0_33),
 .linkdiseq_0_34(linkdiseq_0_34),
 .linkdiseq_0_35(linkdiseq_0_35),
 .linkdiseq_0_36(linkdiseq_0_36),
 .linkdiseq_0_37(linkdiseq_0_37),
 .linkdiseq_0_38(linkdiseq_0_38),
 .linkdiseq_0_39(linkdiseq_0_39),
 .linkdiseq_0_40(linkdiseq_0_40),
 .linkdiseq_0_41(linkdiseq_0_41),
 .linkdiseq_0_42(linkdiseq_0_42),
 .linkdiseq_0_43(linkdiseq_0_43),
 .linkdiseq_0_44(linkdiseq_0_44),
 .linkdiseq_0_45(linkdiseq_0_45),
 .linkdiseq_0_46(linkdiseq_0_46),
 .linkdiseq_0_47(linkdiseq_0_47)
);
endmodule


module pru_array_special (
input clk,
input rst,
input	[31:0] element_length_t,
input	[31:0] sample_size,
input vld_in,
input	[63:0] val_i_y_0,
input	[63:0] val_i_x_0,
input	[63:0] val_i_x_1,
input	[63:0] val_i_x_2,
input	[63:0] val_i_x_3,
input	[63:0] val_i_x_4,
input	[63:0] val_i_x_5,
input	[63:0] val_i_x_6,
input	[63:0] val_i_x_7,
input	[63:0] val_i_x_8,
input	[63:0] val_i_x_9,
input	[63:0] val_i_x_10,
input	[63:0] val_i_x_11,
input	[63:0] val_i_x_12,
input	[63:0] val_i_x_13,
input	[63:0] val_i_x_14,
input	[63:0] val_i_x_15,
input	[63:0] val_i_x_16,
input	[63:0] val_i_x_17,
input	[63:0] val_i_x_18,
input	[63:0] val_i_x_19,
input	[63:0] val_i_x_20,
input	[63:0] val_i_x_21,
input	[63:0] val_i_x_22,
input	[63:0] val_i_x_23,
input	[63:0] val_i_x_24,
input	[63:0] val_i_x_25,
input	[63:0] val_i_x_26,
input	[63:0] val_i_x_27,
input	[63:0] val_i_x_28,
input	[63:0] val_i_x_29,
input	[63:0] val_i_x_30,
input	[63:0] val_i_x_31,
input	[63:0] val_i_x_32,
input	[63:0] val_i_x_33,
input	[63:0] val_i_x_34,
input	[63:0] val_i_x_35,
input	[63:0] val_i_x_36,
input	[63:0] val_i_x_37,
input	[63:0] val_i_x_38,
input	[63:0] val_i_x_39,
input	[63:0] val_i_x_40,
input	[63:0] val_i_x_41,
input	[63:0] val_i_x_42,
input	[63:0] val_i_x_43,
input	[63:0] val_i_x_44,
input	[63:0] val_i_x_45,
input	[63:0] val_i_x_46,
input	[63:0] val_i_x_47,
output linkdiseq_wren_0_0,
output linkdiseq_wren_0_1,
output linkdiseq_wren_0_2,
output linkdiseq_wren_0_3,
output linkdiseq_wren_0_4,
output linkdiseq_wren_0_5,
output linkdiseq_wren_0_6,
output linkdiseq_wren_0_7,
output linkdiseq_wren_0_8,
output linkdiseq_wren_0_9,
output linkdiseq_wren_0_10,
output linkdiseq_wren_0_11,
output linkdiseq_wren_0_12,
output linkdiseq_wren_0_13,
output linkdiseq_wren_0_14,
output linkdiseq_wren_0_15,
output linkdiseq_wren_0_16,
output linkdiseq_wren_0_17,
output linkdiseq_wren_0_18,
output linkdiseq_wren_0_19,
output linkdiseq_wren_0_20,
output linkdiseq_wren_0_21,
output linkdiseq_wren_0_22,
output linkdiseq_wren_0_23,
output linkdiseq_wren_0_24,
output linkdiseq_wren_0_25,
output linkdiseq_wren_0_26,
output linkdiseq_wren_0_27,
output linkdiseq_wren_0_28,
output linkdiseq_wren_0_29,
output linkdiseq_wren_0_30,
output linkdiseq_wren_0_31,
output linkdiseq_wren_0_32,
output linkdiseq_wren_0_33,
output linkdiseq_wren_0_34,
output linkdiseq_wren_0_35,
output linkdiseq_wren_0_36,
output linkdiseq_wren_0_37,
output linkdiseq_wren_0_38,
output linkdiseq_wren_0_39,
output linkdiseq_wren_0_40,
output linkdiseq_wren_0_41,
output linkdiseq_wren_0_42,
output linkdiseq_wren_0_43,
output linkdiseq_wren_0_44,
output linkdiseq_wren_0_45,
output linkdiseq_wren_0_46,
output linkdiseq_wren_0_47,
output	[31:0] linkdiseq_0_0,
output	[31:0] linkdiseq_0_1,
output	[31:0] linkdiseq_0_2,
output	[31:0] linkdiseq_0_3,
output	[31:0] linkdiseq_0_4,
output	[31:0] linkdiseq_0_5,
output	[31:0] linkdiseq_0_6,
output	[31:0] linkdiseq_0_7,
output	[31:0] linkdiseq_0_8,
output	[31:0] linkdiseq_0_9,
output	[31:0] linkdiseq_0_10,
output	[31:0] linkdiseq_0_11,
output	[31:0] linkdiseq_0_12,
output	[31:0] linkdiseq_0_13,
output	[31:0] linkdiseq_0_14,
output	[31:0] linkdiseq_0_15,
output	[31:0] linkdiseq_0_16,
output	[31:0] linkdiseq_0_17,
output	[31:0] linkdiseq_0_18,
output	[31:0] linkdiseq_0_19,
output	[31:0] linkdiseq_0_20,
output	[31:0] linkdiseq_0_21,
output	[31:0] linkdiseq_0_22,
output	[31:0] linkdiseq_0_23,
output	[31:0] linkdiseq_0_24,
output	[31:0] linkdiseq_0_25,
output	[31:0] linkdiseq_0_26,
output	[31:0] linkdiseq_0_27,
output	[31:0] linkdiseq_0_28,
output	[31:0] linkdiseq_0_29,
output	[31:0] linkdiseq_0_30,
output	[31:0] linkdiseq_0_31,
output	[31:0] linkdiseq_0_32,
output	[31:0] linkdiseq_0_33,
output	[31:0] linkdiseq_0_34,
output	[31:0] linkdiseq_0_35,
output	[31:0] linkdiseq_0_36,
output	[31:0] linkdiseq_0_37,
output	[31:0] linkdiseq_0_38,
output	[31:0] linkdiseq_0_39,
output	[31:0] linkdiseq_0_40,
output	[31:0] linkdiseq_0_41,
output	[31:0] linkdiseq_0_42,
output	[31:0] linkdiseq_0_43,
output	[31:0] linkdiseq_0_44,
output	[31:0] linkdiseq_0_45,
output	[31:0] linkdiseq_0_46,
output	[31:0] linkdiseq_0_47
);

wire vld_in_t;
du #(1, 6) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
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
wire [31:0] t_x_cnt_8;
wire [31:0] t_x_cnt_9;
wire [31:0] t_x_cnt_10;
wire [31:0] t_x_cnt_11;
wire [31:0] t_x_cnt_12;
wire [31:0] t_x_cnt_13;
wire [31:0] t_x_cnt_14;
wire [31:0] t_x_cnt_15;
wire [31:0] t_x_cnt_16;
wire [31:0] t_x_cnt_17;
wire [31:0] t_x_cnt_18;
wire [31:0] t_x_cnt_19;
wire [31:0] t_x_cnt_20;
wire [31:0] t_x_cnt_21;
wire [31:0] t_x_cnt_22;
wire [31:0] t_x_cnt_23;
wire [31:0] t_x_cnt_24;
wire [31:0] t_x_cnt_25;
wire [31:0] t_x_cnt_26;
wire [31:0] t_x_cnt_27;
wire [31:0] t_x_cnt_28;
wire [31:0] t_x_cnt_29;
wire [31:0] t_x_cnt_30;
wire [31:0] t_x_cnt_31;
wire [31:0] t_x_cnt_32;
wire [31:0] t_x_cnt_33;
wire [31:0] t_x_cnt_34;
wire [31:0] t_x_cnt_35;
wire [31:0] t_x_cnt_36;
wire [31:0] t_x_cnt_37;
wire [31:0] t_x_cnt_38;
wire [31:0] t_x_cnt_39;
wire [31:0] t_x_cnt_40;
wire [31:0] t_x_cnt_41;
wire [31:0] t_x_cnt_42;
wire [31:0] t_x_cnt_43;
wire [31:0] t_x_cnt_44;
wire [31:0] t_x_cnt_45;
wire [31:0] t_x_cnt_46;
wire [31:0] t_x_cnt_47;

wire [31:0] t_xy_cnt_0_0;
wire [31:0] t_xy_cnt_0_1;
wire [31:0] t_xy_cnt_0_2;
wire [31:0] t_xy_cnt_0_3;
wire [31:0] t_xy_cnt_0_4;
wire [31:0] t_xy_cnt_0_5;
wire [31:0] t_xy_cnt_0_6;
wire [31:0] t_xy_cnt_0_7;
wire [31:0] t_xy_cnt_0_8;
wire [31:0] t_xy_cnt_0_9;
wire [31:0] t_xy_cnt_0_10;
wire [31:0] t_xy_cnt_0_11;
wire [31:0] t_xy_cnt_0_12;
wire [31:0] t_xy_cnt_0_13;
wire [31:0] t_xy_cnt_0_14;
wire [31:0] t_xy_cnt_0_15;
wire [31:0] t_xy_cnt_0_16;
wire [31:0] t_xy_cnt_0_17;
wire [31:0] t_xy_cnt_0_18;
wire [31:0] t_xy_cnt_0_19;
wire [31:0] t_xy_cnt_0_20;
wire [31:0] t_xy_cnt_0_21;
wire [31:0] t_xy_cnt_0_22;
wire [31:0] t_xy_cnt_0_23;
wire [31:0] t_xy_cnt_0_24;
wire [31:0] t_xy_cnt_0_25;
wire [31:0] t_xy_cnt_0_26;
wire [31:0] t_xy_cnt_0_27;
wire [31:0] t_xy_cnt_0_28;
wire [31:0] t_xy_cnt_0_29;
wire [31:0] t_xy_cnt_0_30;
wire [31:0] t_xy_cnt_0_31;
wire [31:0] t_xy_cnt_0_32;
wire [31:0] t_xy_cnt_0_33;
wire [31:0] t_xy_cnt_0_34;
wire [31:0] t_xy_cnt_0_35;
wire [31:0] t_xy_cnt_0_36;
wire [31:0] t_xy_cnt_0_37;
wire [31:0] t_xy_cnt_0_38;
wire [31:0] t_xy_cnt_0_39;
wire [31:0] t_xy_cnt_0_40;
wire [31:0] t_xy_cnt_0_41;
wire [31:0] t_xy_cnt_0_42;
wire [31:0] t_xy_cnt_0_43;
wire [31:0] t_xy_cnt_0_44;
wire [31:0] t_xy_cnt_0_45;
wire [31:0] t_xy_cnt_0_46;
wire [31:0] t_xy_cnt_0_47;
pw_sim_special_48_1_64_6 pw_sim_special_48_1_64_6_0(
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
 .val_i_x_8(val_i_x_8),
 .x_cnt_8(t_x_cnt_8),
 .val_i_x_9(val_i_x_9),
 .x_cnt_9(t_x_cnt_9),
 .val_i_x_10(val_i_x_10),
 .x_cnt_10(t_x_cnt_10),
 .val_i_x_11(val_i_x_11),
 .x_cnt_11(t_x_cnt_11),
 .val_i_x_12(val_i_x_12),
 .x_cnt_12(t_x_cnt_12),
 .val_i_x_13(val_i_x_13),
 .x_cnt_13(t_x_cnt_13),
 .val_i_x_14(val_i_x_14),
 .x_cnt_14(t_x_cnt_14),
 .val_i_x_15(val_i_x_15),
 .x_cnt_15(t_x_cnt_15),
 .val_i_x_16(val_i_x_16),
 .x_cnt_16(t_x_cnt_16),
 .val_i_x_17(val_i_x_17),
 .x_cnt_17(t_x_cnt_17),
 .val_i_x_18(val_i_x_18),
 .x_cnt_18(t_x_cnt_18),
 .val_i_x_19(val_i_x_19),
 .x_cnt_19(t_x_cnt_19),
 .val_i_x_20(val_i_x_20),
 .x_cnt_20(t_x_cnt_20),
 .val_i_x_21(val_i_x_21),
 .x_cnt_21(t_x_cnt_21),
 .val_i_x_22(val_i_x_22),
 .x_cnt_22(t_x_cnt_22),
 .val_i_x_23(val_i_x_23),
 .x_cnt_23(t_x_cnt_23),
 .val_i_x_24(val_i_x_24),
 .x_cnt_24(t_x_cnt_24),
 .val_i_x_25(val_i_x_25),
 .x_cnt_25(t_x_cnt_25),
 .val_i_x_26(val_i_x_26),
 .x_cnt_26(t_x_cnt_26),
 .val_i_x_27(val_i_x_27),
 .x_cnt_27(t_x_cnt_27),
 .val_i_x_28(val_i_x_28),
 .x_cnt_28(t_x_cnt_28),
 .val_i_x_29(val_i_x_29),
 .x_cnt_29(t_x_cnt_29),
 .val_i_x_30(val_i_x_30),
 .x_cnt_30(t_x_cnt_30),
 .val_i_x_31(val_i_x_31),
 .x_cnt_31(t_x_cnt_31),
 .val_i_x_32(val_i_x_32),
 .x_cnt_32(t_x_cnt_32),
 .val_i_x_33(val_i_x_33),
 .x_cnt_33(t_x_cnt_33),
 .val_i_x_34(val_i_x_34),
 .x_cnt_34(t_x_cnt_34),
 .val_i_x_35(val_i_x_35),
 .x_cnt_35(t_x_cnt_35),
 .val_i_x_36(val_i_x_36),
 .x_cnt_36(t_x_cnt_36),
 .val_i_x_37(val_i_x_37),
 .x_cnt_37(t_x_cnt_37),
 .val_i_x_38(val_i_x_38),
 .x_cnt_38(t_x_cnt_38),
 .val_i_x_39(val_i_x_39),
 .x_cnt_39(t_x_cnt_39),
 .val_i_x_40(val_i_x_40),
 .x_cnt_40(t_x_cnt_40),
 .val_i_x_41(val_i_x_41),
 .x_cnt_41(t_x_cnt_41),
 .val_i_x_42(val_i_x_42),
 .x_cnt_42(t_x_cnt_42),
 .val_i_x_43(val_i_x_43),
 .x_cnt_43(t_x_cnt_43),
 .val_i_x_44(val_i_x_44),
 .x_cnt_44(t_x_cnt_44),
 .val_i_x_45(val_i_x_45),
 .x_cnt_45(t_x_cnt_45),
 .val_i_x_46(val_i_x_46),
 .x_cnt_46(t_x_cnt_46),
 .val_i_x_47(val_i_x_47),
 .x_cnt_47(t_x_cnt_47),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7),
 .xy_cnt_0_8(t_xy_cnt_0_8),
 .xy_cnt_0_9(t_xy_cnt_0_9),
 .xy_cnt_0_10(t_xy_cnt_0_10),
 .xy_cnt_0_11(t_xy_cnt_0_11),
 .xy_cnt_0_12(t_xy_cnt_0_12),
 .xy_cnt_0_13(t_xy_cnt_0_13),
 .xy_cnt_0_14(t_xy_cnt_0_14),
 .xy_cnt_0_15(t_xy_cnt_0_15),
 .xy_cnt_0_16(t_xy_cnt_0_16),
 .xy_cnt_0_17(t_xy_cnt_0_17),
 .xy_cnt_0_18(t_xy_cnt_0_18),
 .xy_cnt_0_19(t_xy_cnt_0_19),
 .xy_cnt_0_20(t_xy_cnt_0_20),
 .xy_cnt_0_21(t_xy_cnt_0_21),
 .xy_cnt_0_22(t_xy_cnt_0_22),
 .xy_cnt_0_23(t_xy_cnt_0_23),
 .xy_cnt_0_24(t_xy_cnt_0_24),
 .xy_cnt_0_25(t_xy_cnt_0_25),
 .xy_cnt_0_26(t_xy_cnt_0_26),
 .xy_cnt_0_27(t_xy_cnt_0_27),
 .xy_cnt_0_28(t_xy_cnt_0_28),
 .xy_cnt_0_29(t_xy_cnt_0_29),
 .xy_cnt_0_30(t_xy_cnt_0_30),
 .xy_cnt_0_31(t_xy_cnt_0_31),
 .xy_cnt_0_32(t_xy_cnt_0_32),
 .xy_cnt_0_33(t_xy_cnt_0_33),
 .xy_cnt_0_34(t_xy_cnt_0_34),
 .xy_cnt_0_35(t_xy_cnt_0_35),
 .xy_cnt_0_36(t_xy_cnt_0_36),
 .xy_cnt_0_37(t_xy_cnt_0_37),
 .xy_cnt_0_38(t_xy_cnt_0_38),
 .xy_cnt_0_39(t_xy_cnt_0_39),
 .xy_cnt_0_40(t_xy_cnt_0_40),
 .xy_cnt_0_41(t_xy_cnt_0_41),
 .xy_cnt_0_42(t_xy_cnt_0_42),
 .xy_cnt_0_43(t_xy_cnt_0_43),
 .xy_cnt_0_44(t_xy_cnt_0_44),
 .xy_cnt_0_45(t_xy_cnt_0_45),
 .xy_cnt_0_46(t_xy_cnt_0_46),
 .xy_cnt_0_47(t_xy_cnt_0_47)
);
pw_coefficients_wrapper_48_1_64_6 pw_coefficients_wrapper_48_1_64_6_0(
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
 .x_cnt_8(t_x_cnt_8),
 .x_cnt_9(t_x_cnt_9),
 .x_cnt_10(t_x_cnt_10),
 .x_cnt_11(t_x_cnt_11),
 .x_cnt_12(t_x_cnt_12),
 .x_cnt_13(t_x_cnt_13),
 .x_cnt_14(t_x_cnt_14),
 .x_cnt_15(t_x_cnt_15),
 .x_cnt_16(t_x_cnt_16),
 .x_cnt_17(t_x_cnt_17),
 .x_cnt_18(t_x_cnt_18),
 .x_cnt_19(t_x_cnt_19),
 .x_cnt_20(t_x_cnt_20),
 .x_cnt_21(t_x_cnt_21),
 .x_cnt_22(t_x_cnt_22),
 .x_cnt_23(t_x_cnt_23),
 .x_cnt_24(t_x_cnt_24),
 .x_cnt_25(t_x_cnt_25),
 .x_cnt_26(t_x_cnt_26),
 .x_cnt_27(t_x_cnt_27),
 .x_cnt_28(t_x_cnt_28),
 .x_cnt_29(t_x_cnt_29),
 .x_cnt_30(t_x_cnt_30),
 .x_cnt_31(t_x_cnt_31),
 .x_cnt_32(t_x_cnt_32),
 .x_cnt_33(t_x_cnt_33),
 .x_cnt_34(t_x_cnt_34),
 .x_cnt_35(t_x_cnt_35),
 .x_cnt_36(t_x_cnt_36),
 .x_cnt_37(t_x_cnt_37),
 .x_cnt_38(t_x_cnt_38),
 .x_cnt_39(t_x_cnt_39),
 .x_cnt_40(t_x_cnt_40),
 .x_cnt_41(t_x_cnt_41),
 .x_cnt_42(t_x_cnt_42),
 .x_cnt_43(t_x_cnt_43),
 .x_cnt_44(t_x_cnt_44),
 .x_cnt_45(t_x_cnt_45),
 .x_cnt_46(t_x_cnt_46),
 .x_cnt_47(t_x_cnt_47),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .xy_cnt_0_4(t_xy_cnt_0_4),
 .xy_cnt_0_5(t_xy_cnt_0_5),
 .xy_cnt_0_6(t_xy_cnt_0_6),
 .xy_cnt_0_7(t_xy_cnt_0_7),
 .xy_cnt_0_8(t_xy_cnt_0_8),
 .xy_cnt_0_9(t_xy_cnt_0_9),
 .xy_cnt_0_10(t_xy_cnt_0_10),
 .xy_cnt_0_11(t_xy_cnt_0_11),
 .xy_cnt_0_12(t_xy_cnt_0_12),
 .xy_cnt_0_13(t_xy_cnt_0_13),
 .xy_cnt_0_14(t_xy_cnt_0_14),
 .xy_cnt_0_15(t_xy_cnt_0_15),
 .xy_cnt_0_16(t_xy_cnt_0_16),
 .xy_cnt_0_17(t_xy_cnt_0_17),
 .xy_cnt_0_18(t_xy_cnt_0_18),
 .xy_cnt_0_19(t_xy_cnt_0_19),
 .xy_cnt_0_20(t_xy_cnt_0_20),
 .xy_cnt_0_21(t_xy_cnt_0_21),
 .xy_cnt_0_22(t_xy_cnt_0_22),
 .xy_cnt_0_23(t_xy_cnt_0_23),
 .xy_cnt_0_24(t_xy_cnt_0_24),
 .xy_cnt_0_25(t_xy_cnt_0_25),
 .xy_cnt_0_26(t_xy_cnt_0_26),
 .xy_cnt_0_27(t_xy_cnt_0_27),
 .xy_cnt_0_28(t_xy_cnt_0_28),
 .xy_cnt_0_29(t_xy_cnt_0_29),
 .xy_cnt_0_30(t_xy_cnt_0_30),
 .xy_cnt_0_31(t_xy_cnt_0_31),
 .xy_cnt_0_32(t_xy_cnt_0_32),
 .xy_cnt_0_33(t_xy_cnt_0_33),
 .xy_cnt_0_34(t_xy_cnt_0_34),
 .xy_cnt_0_35(t_xy_cnt_0_35),
 .xy_cnt_0_36(t_xy_cnt_0_36),
 .xy_cnt_0_37(t_xy_cnt_0_37),
 .xy_cnt_0_38(t_xy_cnt_0_38),
 .xy_cnt_0_39(t_xy_cnt_0_39),
 .xy_cnt_0_40(t_xy_cnt_0_40),
 .xy_cnt_0_41(t_xy_cnt_0_41),
 .xy_cnt_0_42(t_xy_cnt_0_42),
 .xy_cnt_0_43(t_xy_cnt_0_43),
 .xy_cnt_0_44(t_xy_cnt_0_44),
 .xy_cnt_0_45(t_xy_cnt_0_45),
 .xy_cnt_0_46(t_xy_cnt_0_46),
 .xy_cnt_0_47(t_xy_cnt_0_47),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7),
 .linkdiseq_wren_0_8(linkdiseq_wren_0_8),
 .linkdiseq_wren_0_9(linkdiseq_wren_0_9),
 .linkdiseq_wren_0_10(linkdiseq_wren_0_10),
 .linkdiseq_wren_0_11(linkdiseq_wren_0_11),
 .linkdiseq_wren_0_12(linkdiseq_wren_0_12),
 .linkdiseq_wren_0_13(linkdiseq_wren_0_13),
 .linkdiseq_wren_0_14(linkdiseq_wren_0_14),
 .linkdiseq_wren_0_15(linkdiseq_wren_0_15),
 .linkdiseq_wren_0_16(linkdiseq_wren_0_16),
 .linkdiseq_wren_0_17(linkdiseq_wren_0_17),
 .linkdiseq_wren_0_18(linkdiseq_wren_0_18),
 .linkdiseq_wren_0_19(linkdiseq_wren_0_19),
 .linkdiseq_wren_0_20(linkdiseq_wren_0_20),
 .linkdiseq_wren_0_21(linkdiseq_wren_0_21),
 .linkdiseq_wren_0_22(linkdiseq_wren_0_22),
 .linkdiseq_wren_0_23(linkdiseq_wren_0_23),
 .linkdiseq_wren_0_24(linkdiseq_wren_0_24),
 .linkdiseq_wren_0_25(linkdiseq_wren_0_25),
 .linkdiseq_wren_0_26(linkdiseq_wren_0_26),
 .linkdiseq_wren_0_27(linkdiseq_wren_0_27),
 .linkdiseq_wren_0_28(linkdiseq_wren_0_28),
 .linkdiseq_wren_0_29(linkdiseq_wren_0_29),
 .linkdiseq_wren_0_30(linkdiseq_wren_0_30),
 .linkdiseq_wren_0_31(linkdiseq_wren_0_31),
 .linkdiseq_wren_0_32(linkdiseq_wren_0_32),
 .linkdiseq_wren_0_33(linkdiseq_wren_0_33),
 .linkdiseq_wren_0_34(linkdiseq_wren_0_34),
 .linkdiseq_wren_0_35(linkdiseq_wren_0_35),
 .linkdiseq_wren_0_36(linkdiseq_wren_0_36),
 .linkdiseq_wren_0_37(linkdiseq_wren_0_37),
 .linkdiseq_wren_0_38(linkdiseq_wren_0_38),
 .linkdiseq_wren_0_39(linkdiseq_wren_0_39),
 .linkdiseq_wren_0_40(linkdiseq_wren_0_40),
 .linkdiseq_wren_0_41(linkdiseq_wren_0_41),
 .linkdiseq_wren_0_42(linkdiseq_wren_0_42),
 .linkdiseq_wren_0_43(linkdiseq_wren_0_43),
 .linkdiseq_wren_0_44(linkdiseq_wren_0_44),
 .linkdiseq_wren_0_45(linkdiseq_wren_0_45),
 .linkdiseq_wren_0_46(linkdiseq_wren_0_46),
 .linkdiseq_wren_0_47(linkdiseq_wren_0_47),
 .linkdiseq_0_0(linkdiseq_0_0),
 .linkdiseq_0_1(linkdiseq_0_1),
 .linkdiseq_0_2(linkdiseq_0_2),
 .linkdiseq_0_3(linkdiseq_0_3),
 .linkdiseq_0_4(linkdiseq_0_4),
 .linkdiseq_0_5(linkdiseq_0_5),
 .linkdiseq_0_6(linkdiseq_0_6),
 .linkdiseq_0_7(linkdiseq_0_7),
 .linkdiseq_0_8(linkdiseq_0_8),
 .linkdiseq_0_9(linkdiseq_0_9),
 .linkdiseq_0_10(linkdiseq_0_10),
 .linkdiseq_0_11(linkdiseq_0_11),
 .linkdiseq_0_12(linkdiseq_0_12),
 .linkdiseq_0_13(linkdiseq_0_13),
 .linkdiseq_0_14(linkdiseq_0_14),
 .linkdiseq_0_15(linkdiseq_0_15),
 .linkdiseq_0_16(linkdiseq_0_16),
 .linkdiseq_0_17(linkdiseq_0_17),
 .linkdiseq_0_18(linkdiseq_0_18),
 .linkdiseq_0_19(linkdiseq_0_19),
 .linkdiseq_0_20(linkdiseq_0_20),
 .linkdiseq_0_21(linkdiseq_0_21),
 .linkdiseq_0_22(linkdiseq_0_22),
 .linkdiseq_0_23(linkdiseq_0_23),
 .linkdiseq_0_24(linkdiseq_0_24),
 .linkdiseq_0_25(linkdiseq_0_25),
 .linkdiseq_0_26(linkdiseq_0_26),
 .linkdiseq_0_27(linkdiseq_0_27),
 .linkdiseq_0_28(linkdiseq_0_28),
 .linkdiseq_0_29(linkdiseq_0_29),
 .linkdiseq_0_30(linkdiseq_0_30),
 .linkdiseq_0_31(linkdiseq_0_31),
 .linkdiseq_0_32(linkdiseq_0_32),
 .linkdiseq_0_33(linkdiseq_0_33),
 .linkdiseq_0_34(linkdiseq_0_34),
 .linkdiseq_0_35(linkdiseq_0_35),
 .linkdiseq_0_36(linkdiseq_0_36),
 .linkdiseq_0_37(linkdiseq_0_37),
 .linkdiseq_0_38(linkdiseq_0_38),
 .linkdiseq_0_39(linkdiseq_0_39),
 .linkdiseq_0_40(linkdiseq_0_40),
 .linkdiseq_0_41(linkdiseq_0_41),
 .linkdiseq_0_42(linkdiseq_0_42),
 .linkdiseq_0_43(linkdiseq_0_43),
 .linkdiseq_0_44(linkdiseq_0_44),
 .linkdiseq_0_45(linkdiseq_0_45),
 .linkdiseq_0_46(linkdiseq_0_46),
 .linkdiseq_0_47(linkdiseq_0_47)
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
  input [5:0] bank_id,
  input wren_q,
  input [7:0]wraddr_q,
  input [5:0]wrid_q,
  input [63:0]wrdin_q,
  input [7:0]rdaddr_a,
  output [63:0]rddout_a,
  input [7:0]rdaddr_b,
  output [63:0]rddout_b
);

wire wren_a;
assign wren_a = (wren_q==1) & (wrid_q==bank_id);
wire [7:0] addr_a;
assign addr_a = (wren_a==1)?wraddr_q:rdaddr_a;
bram_tdp #(64,8) umem ( .a_clk(clk), .a_wr(wren_a), .a_addr(addr_a), .a_din(wrdin_q), .a_dout(rddout_a), 
  .b_clk(clk), .b_wr(1'b0), .b_addr(rdaddr_b), .b_din(64'b0), .b_dout(rddout_b)); 
endmodule
module omem_bank (
  input clk,
  input [5:0] bank_id,
  input wren,
  input [10:0]wraddr,
  input [31:0]wrdin,
  input rden_q,
  input [10:0]rdaddr_q,
  input [5:0]rdid_q,
  output [31:0]rddout_q
);

reg rden_q_1c_r;
wire rden_q_1c_w;
reg [5:0]rdid_q_1c_r;
wire [5:0]rdid_q_1c_w;
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
input	[5:0] wrid_q,
input	[63:0] wrdin_q,
input	[7:0] rdaddr_a,
input	[7:0] rdaddr_b,
output	[63:0] rddout_a_0_0,
output	[63:0] rddout_b_0_0,
output	[63:0] rddout_a_0_1,
output	[63:0] rddout_b_0_1,
output	[63:0] rddout_a_0_2,
output	[63:0] rddout_b_0_2,
output	[63:0] rddout_a_0_3,
output	[63:0] rddout_b_0_3,
output	[63:0] rddout_a_0_4,
output	[63:0] rddout_b_0_4,
output	[63:0] rddout_a_0_5,
output	[63:0] rddout_b_0_5,
output	[63:0] rddout_a_0_6,
output	[63:0] rddout_b_0_6,
output	[63:0] rddout_a_0_7,
output	[63:0] rddout_b_0_7,
output	[63:0] rddout_a_0_8,
output	[63:0] rddout_b_0_8,
output	[63:0] rddout_a_0_9,
output	[63:0] rddout_b_0_9,
output	[63:0] rddout_a_0_10,
output	[63:0] rddout_b_0_10,
output	[63:0] rddout_a_0_11,
output	[63:0] rddout_b_0_11,
output	[63:0] rddout_a_0_12,
output	[63:0] rddout_b_0_12,
output	[63:0] rddout_a_0_13,
output	[63:0] rddout_b_0_13,
output	[63:0] rddout_a_0_14,
output	[63:0] rddout_b_0_14,
output	[63:0] rddout_a_0_15,
output	[63:0] rddout_b_0_15,
output	[63:0] rddout_a_0_16,
output	[63:0] rddout_b_0_16,
output	[63:0] rddout_a_0_17,
output	[63:0] rddout_b_0_17,
output	[63:0] rddout_a_0_18,
output	[63:0] rddout_b_0_18,
output	[63:0] rddout_a_0_19,
output	[63:0] rddout_b_0_19,
output	[63:0] rddout_a_0_20,
output	[63:0] rddout_b_0_20,
output	[63:0] rddout_a_0_21,
output	[63:0] rddout_b_0_21,
output	[63:0] rddout_a_0_22,
output	[63:0] rddout_b_0_22,
output	[63:0] rddout_a_0_23,
output	[63:0] rddout_b_0_23,
output	[63:0] rddout_a_0_24,
output	[63:0] rddout_b_0_24,
output	[63:0] rddout_a_0_25,
output	[63:0] rddout_b_0_25,
output	[63:0] rddout_a_0_26,
output	[63:0] rddout_b_0_26,
output	[63:0] rddout_a_0_27,
output	[63:0] rddout_b_0_27,
output	[63:0] rddout_a_0_28,
output	[63:0] rddout_b_0_28,
output	[63:0] rddout_a_0_29,
output	[63:0] rddout_b_0_29,
output	[63:0] rddout_a_0_30,
output	[63:0] rddout_b_0_30,
output	[63:0] rddout_a_0_31,
output	[63:0] rddout_b_0_31,
output	[63:0] rddout_a_0_32,
output	[63:0] rddout_b_0_32,
output	[63:0] rddout_a_0_33,
output	[63:0] rddout_b_0_33,
output	[63:0] rddout_a_0_34,
output	[63:0] rddout_b_0_34,
output	[63:0] rddout_a_0_35,
output	[63:0] rddout_b_0_35,
output	[63:0] rddout_a_0_36,
output	[63:0] rddout_b_0_36,
output	[63:0] rddout_a_0_37,
output	[63:0] rddout_b_0_37,
output	[63:0] rddout_a_0_38,
output	[63:0] rddout_b_0_38,
output	[63:0] rddout_a_0_39,
output	[63:0] rddout_b_0_39,
output	[63:0] rddout_a_0_40,
output	[63:0] rddout_b_0_40,
output	[63:0] rddout_a_0_41,
output	[63:0] rddout_b_0_41,
output	[63:0] rddout_a_0_42,
output	[63:0] rddout_b_0_42,
output	[63:0] rddout_a_0_43,
output	[63:0] rddout_b_0_43,
output	[63:0] rddout_a_0_44,
output	[63:0] rddout_b_0_44,
output	[63:0] rddout_a_0_45,
output	[63:0] rddout_b_0_45,
output	[63:0] rddout_a_0_46,
output	[63:0] rddout_b_0_46,
output	[63:0] rddout_a_0_47,
output	[63:0] rddout_b_0_47
);

wire [5:0] bank_id_0_0;
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

wire [5:0] bank_id_0_1;
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

wire [5:0] bank_id_0_2;
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

wire [5:0] bank_id_0_3;
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

wire [5:0] bank_id_0_4;
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

wire [5:0] bank_id_0_5;
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

wire [5:0] bank_id_0_6;
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

wire [5:0] bank_id_0_7;
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

wire [5:0] bank_id_0_8;
assign bank_id_0_8 = 8;
imem_bank mem_bank_0_8 (
  .clk(clk),
  .bank_id(bank_id_0_8),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_8),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_8)
);

wire [5:0] bank_id_0_9;
assign bank_id_0_9 = 9;
imem_bank mem_bank_0_9 (
  .clk(clk),
  .bank_id(bank_id_0_9),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_9),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_9)
);

wire [5:0] bank_id_0_10;
assign bank_id_0_10 = 10;
imem_bank mem_bank_0_10 (
  .clk(clk),
  .bank_id(bank_id_0_10),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_10),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_10)
);

wire [5:0] bank_id_0_11;
assign bank_id_0_11 = 11;
imem_bank mem_bank_0_11 (
  .clk(clk),
  .bank_id(bank_id_0_11),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_11),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_11)
);

wire [5:0] bank_id_0_12;
assign bank_id_0_12 = 12;
imem_bank mem_bank_0_12 (
  .clk(clk),
  .bank_id(bank_id_0_12),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_12),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_12)
);

wire [5:0] bank_id_0_13;
assign bank_id_0_13 = 13;
imem_bank mem_bank_0_13 (
  .clk(clk),
  .bank_id(bank_id_0_13),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_13),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_13)
);

wire [5:0] bank_id_0_14;
assign bank_id_0_14 = 14;
imem_bank mem_bank_0_14 (
  .clk(clk),
  .bank_id(bank_id_0_14),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_14),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_14)
);

wire [5:0] bank_id_0_15;
assign bank_id_0_15 = 15;
imem_bank mem_bank_0_15 (
  .clk(clk),
  .bank_id(bank_id_0_15),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_15),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_15)
);

wire [5:0] bank_id_0_16;
assign bank_id_0_16 = 16;
imem_bank mem_bank_0_16 (
  .clk(clk),
  .bank_id(bank_id_0_16),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_16),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_16)
);

wire [5:0] bank_id_0_17;
assign bank_id_0_17 = 17;
imem_bank mem_bank_0_17 (
  .clk(clk),
  .bank_id(bank_id_0_17),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_17),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_17)
);

wire [5:0] bank_id_0_18;
assign bank_id_0_18 = 18;
imem_bank mem_bank_0_18 (
  .clk(clk),
  .bank_id(bank_id_0_18),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_18),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_18)
);

wire [5:0] bank_id_0_19;
assign bank_id_0_19 = 19;
imem_bank mem_bank_0_19 (
  .clk(clk),
  .bank_id(bank_id_0_19),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_19),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_19)
);

wire [5:0] bank_id_0_20;
assign bank_id_0_20 = 20;
imem_bank mem_bank_0_20 (
  .clk(clk),
  .bank_id(bank_id_0_20),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_20),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_20)
);

wire [5:0] bank_id_0_21;
assign bank_id_0_21 = 21;
imem_bank mem_bank_0_21 (
  .clk(clk),
  .bank_id(bank_id_0_21),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_21),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_21)
);

wire [5:0] bank_id_0_22;
assign bank_id_0_22 = 22;
imem_bank mem_bank_0_22 (
  .clk(clk),
  .bank_id(bank_id_0_22),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_22),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_22)
);

wire [5:0] bank_id_0_23;
assign bank_id_0_23 = 23;
imem_bank mem_bank_0_23 (
  .clk(clk),
  .bank_id(bank_id_0_23),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_23),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_23)
);

wire [5:0] bank_id_0_24;
assign bank_id_0_24 = 24;
imem_bank mem_bank_0_24 (
  .clk(clk),
  .bank_id(bank_id_0_24),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_24),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_24)
);

wire [5:0] bank_id_0_25;
assign bank_id_0_25 = 25;
imem_bank mem_bank_0_25 (
  .clk(clk),
  .bank_id(bank_id_0_25),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_25),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_25)
);

wire [5:0] bank_id_0_26;
assign bank_id_0_26 = 26;
imem_bank mem_bank_0_26 (
  .clk(clk),
  .bank_id(bank_id_0_26),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_26),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_26)
);

wire [5:0] bank_id_0_27;
assign bank_id_0_27 = 27;
imem_bank mem_bank_0_27 (
  .clk(clk),
  .bank_id(bank_id_0_27),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_27),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_27)
);

wire [5:0] bank_id_0_28;
assign bank_id_0_28 = 28;
imem_bank mem_bank_0_28 (
  .clk(clk),
  .bank_id(bank_id_0_28),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_28),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_28)
);

wire [5:0] bank_id_0_29;
assign bank_id_0_29 = 29;
imem_bank mem_bank_0_29 (
  .clk(clk),
  .bank_id(bank_id_0_29),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_29),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_29)
);

wire [5:0] bank_id_0_30;
assign bank_id_0_30 = 30;
imem_bank mem_bank_0_30 (
  .clk(clk),
  .bank_id(bank_id_0_30),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_30),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_30)
);

wire [5:0] bank_id_0_31;
assign bank_id_0_31 = 31;
imem_bank mem_bank_0_31 (
  .clk(clk),
  .bank_id(bank_id_0_31),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_31),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_31)
);

wire [5:0] bank_id_0_32;
assign bank_id_0_32 = 32;
imem_bank mem_bank_0_32 (
  .clk(clk),
  .bank_id(bank_id_0_32),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_32),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_32)
);

wire [5:0] bank_id_0_33;
assign bank_id_0_33 = 33;
imem_bank mem_bank_0_33 (
  .clk(clk),
  .bank_id(bank_id_0_33),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_33),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_33)
);

wire [5:0] bank_id_0_34;
assign bank_id_0_34 = 34;
imem_bank mem_bank_0_34 (
  .clk(clk),
  .bank_id(bank_id_0_34),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_34),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_34)
);

wire [5:0] bank_id_0_35;
assign bank_id_0_35 = 35;
imem_bank mem_bank_0_35 (
  .clk(clk),
  .bank_id(bank_id_0_35),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_35),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_35)
);

wire [5:0] bank_id_0_36;
assign bank_id_0_36 = 36;
imem_bank mem_bank_0_36 (
  .clk(clk),
  .bank_id(bank_id_0_36),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_36),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_36)
);

wire [5:0] bank_id_0_37;
assign bank_id_0_37 = 37;
imem_bank mem_bank_0_37 (
  .clk(clk),
  .bank_id(bank_id_0_37),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_37),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_37)
);

wire [5:0] bank_id_0_38;
assign bank_id_0_38 = 38;
imem_bank mem_bank_0_38 (
  .clk(clk),
  .bank_id(bank_id_0_38),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_38),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_38)
);

wire [5:0] bank_id_0_39;
assign bank_id_0_39 = 39;
imem_bank mem_bank_0_39 (
  .clk(clk),
  .bank_id(bank_id_0_39),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_39),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_39)
);

wire [5:0] bank_id_0_40;
assign bank_id_0_40 = 40;
imem_bank mem_bank_0_40 (
  .clk(clk),
  .bank_id(bank_id_0_40),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_40),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_40)
);

wire [5:0] bank_id_0_41;
assign bank_id_0_41 = 41;
imem_bank mem_bank_0_41 (
  .clk(clk),
  .bank_id(bank_id_0_41),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_41),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_41)
);

wire [5:0] bank_id_0_42;
assign bank_id_0_42 = 42;
imem_bank mem_bank_0_42 (
  .clk(clk),
  .bank_id(bank_id_0_42),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_42),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_42)
);

wire [5:0] bank_id_0_43;
assign bank_id_0_43 = 43;
imem_bank mem_bank_0_43 (
  .clk(clk),
  .bank_id(bank_id_0_43),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_43),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_43)
);

wire [5:0] bank_id_0_44;
assign bank_id_0_44 = 44;
imem_bank mem_bank_0_44 (
  .clk(clk),
  .bank_id(bank_id_0_44),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_44),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_44)
);

wire [5:0] bank_id_0_45;
assign bank_id_0_45 = 45;
imem_bank mem_bank_0_45 (
  .clk(clk),
  .bank_id(bank_id_0_45),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_45),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_45)
);

wire [5:0] bank_id_0_46;
assign bank_id_0_46 = 46;
imem_bank mem_bank_0_46 (
  .clk(clk),
  .bank_id(bank_id_0_46),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_46),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_46)
);

wire [5:0] bank_id_0_47;
assign bank_id_0_47 = 47;
imem_bank mem_bank_0_47 (
  .clk(clk),
  .bank_id(bank_id_0_47),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_47),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_47)
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
input wren_a_0_8,
input wren_b_0_8,
input wren_a_0_9,
input wren_b_0_9,
input wren_a_0_10,
input wren_b_0_10,
input wren_a_0_11,
input wren_b_0_11,
input wren_a_0_12,
input wren_b_0_12,
input wren_a_0_13,
input wren_b_0_13,
input wren_a_0_14,
input wren_b_0_14,
input wren_a_0_15,
input wren_b_0_15,
input wren_a_0_16,
input wren_b_0_16,
input wren_a_0_17,
input wren_b_0_17,
input wren_a_0_18,
input wren_b_0_18,
input wren_a_0_19,
input wren_b_0_19,
input wren_a_0_20,
input wren_b_0_20,
input wren_a_0_21,
input wren_b_0_21,
input wren_a_0_22,
input wren_b_0_22,
input wren_a_0_23,
input wren_b_0_23,
input wren_a_0_24,
input wren_b_0_24,
input wren_a_0_25,
input wren_b_0_25,
input wren_a_0_26,
input wren_b_0_26,
input wren_a_0_27,
input wren_b_0_27,
input wren_a_0_28,
input wren_b_0_28,
input wren_a_0_29,
input wren_b_0_29,
input wren_a_0_30,
input wren_b_0_30,
input wren_a_0_31,
input wren_b_0_31,
input wren_a_0_32,
input wren_b_0_32,
input wren_a_0_33,
input wren_b_0_33,
input wren_a_0_34,
input wren_b_0_34,
input wren_a_0_35,
input wren_b_0_35,
input wren_a_0_36,
input wren_b_0_36,
input wren_a_0_37,
input wren_b_0_37,
input wren_a_0_38,
input wren_b_0_38,
input wren_a_0_39,
input wren_b_0_39,
input wren_a_0_40,
input wren_b_0_40,
input wren_a_0_41,
input wren_b_0_41,
input wren_a_0_42,
input wren_b_0_42,
input wren_a_0_43,
input wren_b_0_43,
input wren_a_0_44,
input wren_b_0_44,
input wren_a_0_45,
input wren_b_0_45,
input wren_a_0_46,
input wren_b_0_46,
input wren_a_0_47,
input wren_b_0_47,
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
input	[31:0] wrdin_a_0_8,
input	[31:0] wrdin_b_0_8,
input	[31:0] wrdin_a_0_9,
input	[31:0] wrdin_b_0_9,
input	[31:0] wrdin_a_0_10,
input	[31:0] wrdin_b_0_10,
input	[31:0] wrdin_a_0_11,
input	[31:0] wrdin_b_0_11,
input	[31:0] wrdin_a_0_12,
input	[31:0] wrdin_b_0_12,
input	[31:0] wrdin_a_0_13,
input	[31:0] wrdin_b_0_13,
input	[31:0] wrdin_a_0_14,
input	[31:0] wrdin_b_0_14,
input	[31:0] wrdin_a_0_15,
input	[31:0] wrdin_b_0_15,
input	[31:0] wrdin_a_0_16,
input	[31:0] wrdin_b_0_16,
input	[31:0] wrdin_a_0_17,
input	[31:0] wrdin_b_0_17,
input	[31:0] wrdin_a_0_18,
input	[31:0] wrdin_b_0_18,
input	[31:0] wrdin_a_0_19,
input	[31:0] wrdin_b_0_19,
input	[31:0] wrdin_a_0_20,
input	[31:0] wrdin_b_0_20,
input	[31:0] wrdin_a_0_21,
input	[31:0] wrdin_b_0_21,
input	[31:0] wrdin_a_0_22,
input	[31:0] wrdin_b_0_22,
input	[31:0] wrdin_a_0_23,
input	[31:0] wrdin_b_0_23,
input	[31:0] wrdin_a_0_24,
input	[31:0] wrdin_b_0_24,
input	[31:0] wrdin_a_0_25,
input	[31:0] wrdin_b_0_25,
input	[31:0] wrdin_a_0_26,
input	[31:0] wrdin_b_0_26,
input	[31:0] wrdin_a_0_27,
input	[31:0] wrdin_b_0_27,
input	[31:0] wrdin_a_0_28,
input	[31:0] wrdin_b_0_28,
input	[31:0] wrdin_a_0_29,
input	[31:0] wrdin_b_0_29,
input	[31:0] wrdin_a_0_30,
input	[31:0] wrdin_b_0_30,
input	[31:0] wrdin_a_0_31,
input	[31:0] wrdin_b_0_31,
input	[31:0] wrdin_a_0_32,
input	[31:0] wrdin_b_0_32,
input	[31:0] wrdin_a_0_33,
input	[31:0] wrdin_b_0_33,
input	[31:0] wrdin_a_0_34,
input	[31:0] wrdin_b_0_34,
input	[31:0] wrdin_a_0_35,
input	[31:0] wrdin_b_0_35,
input	[31:0] wrdin_a_0_36,
input	[31:0] wrdin_b_0_36,
input	[31:0] wrdin_a_0_37,
input	[31:0] wrdin_b_0_37,
input	[31:0] wrdin_a_0_38,
input	[31:0] wrdin_b_0_38,
input	[31:0] wrdin_a_0_39,
input	[31:0] wrdin_b_0_39,
input	[31:0] wrdin_a_0_40,
input	[31:0] wrdin_b_0_40,
input	[31:0] wrdin_a_0_41,
input	[31:0] wrdin_b_0_41,
input	[31:0] wrdin_a_0_42,
input	[31:0] wrdin_b_0_42,
input	[31:0] wrdin_a_0_43,
input	[31:0] wrdin_b_0_43,
input	[31:0] wrdin_a_0_44,
input	[31:0] wrdin_b_0_44,
input	[31:0] wrdin_a_0_45,
input	[31:0] wrdin_b_0_45,
input	[31:0] wrdin_a_0_46,
input	[31:0] wrdin_b_0_46,
input	[31:0] wrdin_a_0_47,
input	[31:0] wrdin_b_0_47,
input rden_q,
input	[10:0] rdaddr_q,
input	[5:0] rdid_q,
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
wire [31:0] rddout_q_a_0_8;
wire [31:0] rddout_q_a_0_9;
wire [31:0] rddout_q_a_0_10;
wire [31:0] rddout_q_a_0_11;
wire [31:0] rddout_q_a_0_12;
wire [31:0] rddout_q_a_0_13;
wire [31:0] rddout_q_a_0_14;
wire [31:0] rddout_q_a_0_15;
wire [31:0] rddout_q_a_0_16;
wire [31:0] rddout_q_a_0_17;
wire [31:0] rddout_q_a_0_18;
wire [31:0] rddout_q_a_0_19;
wire [31:0] rddout_q_a_0_20;
wire [31:0] rddout_q_a_0_21;
wire [31:0] rddout_q_a_0_22;
wire [31:0] rddout_q_a_0_23;
wire [31:0] rddout_q_a_0_24;
wire [31:0] rddout_q_a_0_25;
wire [31:0] rddout_q_a_0_26;
wire [31:0] rddout_q_a_0_27;
wire [31:0] rddout_q_a_0_28;
wire [31:0] rddout_q_a_0_29;
wire [31:0] rddout_q_a_0_30;
wire [31:0] rddout_q_a_0_31;
wire [31:0] rddout_q_a_0_32;
wire [31:0] rddout_q_a_0_33;
wire [31:0] rddout_q_a_0_34;
wire [31:0] rddout_q_a_0_35;
wire [31:0] rddout_q_a_0_36;
wire [31:0] rddout_q_a_0_37;
wire [31:0] rddout_q_a_0_38;
wire [31:0] rddout_q_a_0_39;
wire [31:0] rddout_q_a_0_40;
wire [31:0] rddout_q_a_0_41;
wire [31:0] rddout_q_a_0_42;
wire [31:0] rddout_q_a_0_43;
wire [31:0] rddout_q_a_0_44;
wire [31:0] rddout_q_a_0_45;
wire [31:0] rddout_q_a_0_46;
wire [31:0] rddout_q_a_0_47;
wire [31:0] rddout_q_a_0_0_0;
wire [31:0] rddout_q_a_0_1_1;
wire [31:0] rddout_q_a_0_2_2;
wire [31:0] rddout_q_a_0_3_3;
wire [31:0] rddout_q_a_0_4_4;
wire [31:0] rddout_q_a_0_5_5;
wire [31:0] rddout_q_a_0_6_6;
wire [31:0] rddout_q_a_0_7_7;
wire [31:0] rddout_q_a_0_8_8;
wire [31:0] rddout_q_a_0_9_9;
wire [31:0] rddout_q_a_0_10_10;
wire [31:0] rddout_q_a_0_11_11;
wire [31:0] rddout_q_a_0_12_12;
wire [31:0] rddout_q_a_0_13_13;
wire [31:0] rddout_q_a_0_14_14;
wire [31:0] rddout_q_a_0_15_15;
wire [31:0] rddout_q_a_0_16_16;
wire [31:0] rddout_q_a_0_17_17;
wire [31:0] rddout_q_a_0_18_18;
wire [31:0] rddout_q_a_0_19_19;
wire [31:0] rddout_q_a_0_20_20;
wire [31:0] rddout_q_a_0_21_21;
wire [31:0] rddout_q_a_0_22_22;
wire [31:0] rddout_q_a_0_23_23;
wire [31:0] rddout_q_a_0_24_24;
wire [31:0] rddout_q_a_0_25_25;
wire [31:0] rddout_q_a_0_26_26;
wire [31:0] rddout_q_a_0_27_27;
wire [31:0] rddout_q_a_0_28_28;
wire [31:0] rddout_q_a_0_29_29;
wire [31:0] rddout_q_a_0_30_30;
wire [31:0] rddout_q_a_0_31_31;
wire [31:0] rddout_q_a_0_32_32;
wire [31:0] rddout_q_a_0_33_33;
wire [31:0] rddout_q_a_0_34_34;
wire [31:0] rddout_q_a_0_35_35;
wire [31:0] rddout_q_a_0_36_36;
wire [31:0] rddout_q_a_0_37_37;
wire [31:0] rddout_q_a_0_38_38;
wire [31:0] rddout_q_a_0_39_39;
wire [31:0] rddout_q_a_0_40_40;
wire [31:0] rddout_q_a_0_41_41;
wire [31:0] rddout_q_a_0_42_42;
wire [31:0] rddout_q_a_0_43_43;
wire [31:0] rddout_q_a_0_44_44;
wire [31:0] rddout_q_a_0_45_45;
wire [31:0] rddout_q_a_0_46_46;
wire [31:0] rddout_q_a_0_47_47;
assign rddout_q_a_0_0_0[31:0] = rddout_q_a_0_0;
assign rddout_q_a_0_1_1[31:0] = rddout_q_a_0_1;
assign rddout_q_a_0_2_2[31:0] = rddout_q_a_0_2;
assign rddout_q_a_0_3_3[31:0] = rddout_q_a_0_3;
assign rddout_q_a_0_4_4[31:0] = rddout_q_a_0_4;
assign rddout_q_a_0_5_5[31:0] = rddout_q_a_0_5;
assign rddout_q_a_0_6_6[31:0] = rddout_q_a_0_6;
assign rddout_q_a_0_7_7[31:0] = rddout_q_a_0_7;
assign rddout_q_a_0_8_8[31:0] = rddout_q_a_0_8;
assign rddout_q_a_0_9_9[31:0] = rddout_q_a_0_9;
assign rddout_q_a_0_10_10[31:0] = rddout_q_a_0_10;
assign rddout_q_a_0_11_11[31:0] = rddout_q_a_0_11;
assign rddout_q_a_0_12_12[31:0] = rddout_q_a_0_12;
assign rddout_q_a_0_13_13[31:0] = rddout_q_a_0_13;
assign rddout_q_a_0_14_14[31:0] = rddout_q_a_0_14;
assign rddout_q_a_0_15_15[31:0] = rddout_q_a_0_15;
assign rddout_q_a_0_16_16[31:0] = rddout_q_a_0_16;
assign rddout_q_a_0_17_17[31:0] = rddout_q_a_0_17;
assign rddout_q_a_0_18_18[31:0] = rddout_q_a_0_18;
assign rddout_q_a_0_19_19[31:0] = rddout_q_a_0_19;
assign rddout_q_a_0_20_20[31:0] = rddout_q_a_0_20;
assign rddout_q_a_0_21_21[31:0] = rddout_q_a_0_21;
assign rddout_q_a_0_22_22[31:0] = rddout_q_a_0_22;
assign rddout_q_a_0_23_23[31:0] = rddout_q_a_0_23;
assign rddout_q_a_0_24_24[31:0] = rddout_q_a_0_24;
assign rddout_q_a_0_25_25[31:0] = rddout_q_a_0_25;
assign rddout_q_a_0_26_26[31:0] = rddout_q_a_0_26;
assign rddout_q_a_0_27_27[31:0] = rddout_q_a_0_27;
assign rddout_q_a_0_28_28[31:0] = rddout_q_a_0_28;
assign rddout_q_a_0_29_29[31:0] = rddout_q_a_0_29;
assign rddout_q_a_0_30_30[31:0] = rddout_q_a_0_30;
assign rddout_q_a_0_31_31[31:0] = rddout_q_a_0_31;
assign rddout_q_a_0_32_32[31:0] = rddout_q_a_0_32;
assign rddout_q_a_0_33_33[31:0] = rddout_q_a_0_33;
assign rddout_q_a_0_34_34[31:0] = rddout_q_a_0_34;
assign rddout_q_a_0_35_35[31:0] = rddout_q_a_0_35;
assign rddout_q_a_0_36_36[31:0] = rddout_q_a_0_36;
assign rddout_q_a_0_37_37[31:0] = rddout_q_a_0_37;
assign rddout_q_a_0_38_38[31:0] = rddout_q_a_0_38;
assign rddout_q_a_0_39_39[31:0] = rddout_q_a_0_39;
assign rddout_q_a_0_40_40[31:0] = rddout_q_a_0_40;
assign rddout_q_a_0_41_41[31:0] = rddout_q_a_0_41;
assign rddout_q_a_0_42_42[31:0] = rddout_q_a_0_42;
assign rddout_q_a_0_43_43[31:0] = rddout_q_a_0_43;
assign rddout_q_a_0_44_44[31:0] = rddout_q_a_0_44;
assign rddout_q_a_0_45_45[31:0] = rddout_q_a_0_45;
assign rddout_q_a_0_46_46[31:0] = rddout_q_a_0_46;
assign rddout_q_a_0_47_47[31:0] = rddout_q_a_0_47;
wire [31:0] rddout_q_a_0_tmp_0_0;
wire [31:0] rddout_q_a_0_tmp_0_1;
wire [31:0] rddout_q_a_0_tmp_0_2;
wire [31:0] rddout_q_a_0_tmp_0_3;
wire [31:0] rddout_q_a_0_tmp_0_4;
wire [31:0] rddout_q_a_0_tmp_0_5;
wire [31:0] rddout_q_a_0_tmp_0_6;
wire [31:0] rddout_q_a_0_tmp_0_7;
wire [31:0] rddout_q_a_0_tmp_0_8;
wire [31:0] rddout_q_a_0_tmp_0_9;
wire [31:0] rddout_q_a_0_tmp_0_10;
wire [31:0] rddout_q_a_0_tmp_0_11;
wire [31:0] rddout_q_a_0_tmp_0_12;
wire [31:0] rddout_q_a_0_tmp_0_13;
wire [31:0] rddout_q_a_0_tmp_0_14;
wire [31:0] rddout_q_a_0_tmp_0_15;
wire [31:0] rddout_q_a_0_tmp_0_16;
wire [31:0] rddout_q_a_0_tmp_0_17;
wire [31:0] rddout_q_a_0_tmp_0_18;
wire [31:0] rddout_q_a_0_tmp_0_19;
wire [31:0] rddout_q_a_0_tmp_0_20;
wire [31:0] rddout_q_a_0_tmp_0_21;
wire [31:0] rddout_q_a_0_tmp_0_22;
wire [31:0] rddout_q_a_0_tmp_0_23;
wire [31:0] rddout_q_a_0_tmp_d_0_0;
wire [31:0] rddout_q_a_0_tmp_d_0_1;
wire [31:0] rddout_q_a_0_tmp_d_0_2;
wire [31:0] rddout_q_a_0_tmp_d_0_3;
wire [31:0] rddout_q_a_0_tmp_d_0_4;
wire [31:0] rddout_q_a_0_tmp_d_0_5;
wire [31:0] rddout_q_a_0_tmp_d_0_6;
wire [31:0] rddout_q_a_0_tmp_d_0_7;
wire [31:0] rddout_q_a_0_tmp_d_0_8;
wire [31:0] rddout_q_a_0_tmp_d_0_9;
wire [31:0] rddout_q_a_0_tmp_d_0_10;
wire [31:0] rddout_q_a_0_tmp_d_0_11;
wire [31:0] rddout_q_a_0_tmp_d_0_12;
wire [31:0] rddout_q_a_0_tmp_d_0_13;
wire [31:0] rddout_q_a_0_tmp_d_0_14;
wire [31:0] rddout_q_a_0_tmp_d_0_15;
wire [31:0] rddout_q_a_0_tmp_d_0_16;
wire [31:0] rddout_q_a_0_tmp_d_0_17;
wire [31:0] rddout_q_a_0_tmp_d_0_18;
wire [31:0] rddout_q_a_0_tmp_d_0_19;
wire [31:0] rddout_q_a_0_tmp_d_0_20;
wire [31:0] rddout_q_a_0_tmp_d_0_21;
wire [31:0] rddout_q_a_0_tmp_d_0_22;
wire [31:0] rddout_q_a_0_tmp_d_0_23;
wire [31:0] rddout_q_a_0_tmp_1_0;
wire [31:0] rddout_q_a_0_tmp_1_1;
wire [31:0] rddout_q_a_0_tmp_1_2;
wire [31:0] rddout_q_a_0_tmp_1_3;
wire [31:0] rddout_q_a_0_tmp_1_4;
wire [31:0] rddout_q_a_0_tmp_1_5;
wire [31:0] rddout_q_a_0_tmp_1_6;
wire [31:0] rddout_q_a_0_tmp_1_7;
wire [31:0] rddout_q_a_0_tmp_1_8;
wire [31:0] rddout_q_a_0_tmp_1_9;
wire [31:0] rddout_q_a_0_tmp_1_10;
wire [31:0] rddout_q_a_0_tmp_1_11;
wire [31:0] rddout_q_a_0_tmp_d_1_0;
wire [31:0] rddout_q_a_0_tmp_d_1_1;
wire [31:0] rddout_q_a_0_tmp_d_1_2;
wire [31:0] rddout_q_a_0_tmp_d_1_3;
wire [31:0] rddout_q_a_0_tmp_d_1_4;
wire [31:0] rddout_q_a_0_tmp_d_1_5;
wire [31:0] rddout_q_a_0_tmp_d_1_6;
wire [31:0] rddout_q_a_0_tmp_d_1_7;
wire [31:0] rddout_q_a_0_tmp_d_1_8;
wire [31:0] rddout_q_a_0_tmp_d_1_9;
wire [31:0] rddout_q_a_0_tmp_d_1_10;
wire [31:0] rddout_q_a_0_tmp_d_1_11;
wire [31:0] rddout_q_a_0_tmp_2_0;
wire [31:0] rddout_q_a_0_tmp_2_1;
wire [31:0] rddout_q_a_0_tmp_2_2;
wire [31:0] rddout_q_a_0_tmp_2_3;
wire [31:0] rddout_q_a_0_tmp_2_4;
wire [31:0] rddout_q_a_0_tmp_2_5;
wire [31:0] rddout_q_a_0_tmp_d_2_0;
wire [31:0] rddout_q_a_0_tmp_d_2_1;
wire [31:0] rddout_q_a_0_tmp_d_2_2;
wire [31:0] rddout_q_a_0_tmp_d_2_3;
wire [31:0] rddout_q_a_0_tmp_d_2_4;
wire [31:0] rddout_q_a_0_tmp_d_2_5;
wire [31:0] rddout_q_a_0_tmp_3_0;
wire [31:0] rddout_q_a_0_tmp_3_1;
wire [31:0] rddout_q_a_0_tmp_3_2;
wire [31:0] rddout_q_a_0_tmp_d_3_0;
wire [31:0] rddout_q_a_0_tmp_d_3_1;
wire [31:0] rddout_q_a_0_tmp_d_3_2;
wire [31:0] rddout_q_a_0_tmp_4_0;
wire [31:0] rddout_q_a_0_tmp_4_1;
wire [31:0] rddout_q_a_0_tmp_d_4_0;
wire [31:0] rddout_q_a_0_tmp_d_4_1;
wire [31:0] rddout_q_a_0_tmp_5_0;
wire [31:0] rddout_q_a_0_tmp_d_5_0;

 assign rddout_q_a_0_tmp_0_0 = rddout_q_a_0_0_0 | rddout_q_a_0_1_1 ;
 
 assign rddout_q_a_0_tmp_0_1 = rddout_q_a_0_2_2 | rddout_q_a_0_3_3 ;
 
 assign rddout_q_a_0_tmp_0_2 = rddout_q_a_0_4_4 | rddout_q_a_0_5_5 ;
 
 assign rddout_q_a_0_tmp_0_3 = rddout_q_a_0_6_6 | rddout_q_a_0_7_7 ;
 
 assign rddout_q_a_0_tmp_0_4 = rddout_q_a_0_8_8 | rddout_q_a_0_9_9 ;
 
 assign rddout_q_a_0_tmp_0_5 = rddout_q_a_0_10_10 | rddout_q_a_0_11_11 ;
 
 assign rddout_q_a_0_tmp_0_6 = rddout_q_a_0_12_12 | rddout_q_a_0_13_13 ;
 
 assign rddout_q_a_0_tmp_0_7 = rddout_q_a_0_14_14 | rddout_q_a_0_15_15 ;
 
 assign rddout_q_a_0_tmp_0_8 = rddout_q_a_0_16_16 | rddout_q_a_0_17_17 ;
 
 assign rddout_q_a_0_tmp_0_9 = rddout_q_a_0_18_18 | rddout_q_a_0_19_19 ;
 
 assign rddout_q_a_0_tmp_0_10 = rddout_q_a_0_20_20 | rddout_q_a_0_21_21 ;
 
 assign rddout_q_a_0_tmp_0_11 = rddout_q_a_0_22_22 | rddout_q_a_0_23_23 ;
 
 assign rddout_q_a_0_tmp_0_12 = rddout_q_a_0_24_24 | rddout_q_a_0_25_25 ;
 
 assign rddout_q_a_0_tmp_0_13 = rddout_q_a_0_26_26 | rddout_q_a_0_27_27 ;
 
 assign rddout_q_a_0_tmp_0_14 = rddout_q_a_0_28_28 | rddout_q_a_0_29_29 ;
 
 assign rddout_q_a_0_tmp_0_15 = rddout_q_a_0_30_30 | rddout_q_a_0_31_31 ;
 
 assign rddout_q_a_0_tmp_0_16 = rddout_q_a_0_32_32 | rddout_q_a_0_33_33 ;
 
 assign rddout_q_a_0_tmp_0_17 = rddout_q_a_0_34_34 | rddout_q_a_0_35_35 ;
 
 assign rddout_q_a_0_tmp_0_18 = rddout_q_a_0_36_36 | rddout_q_a_0_37_37 ;
 
 assign rddout_q_a_0_tmp_0_19 = rddout_q_a_0_38_38 | rddout_q_a_0_39_39 ;
 
 assign rddout_q_a_0_tmp_0_20 = rddout_q_a_0_40_40 | rddout_q_a_0_41_41 ;
 
 assign rddout_q_a_0_tmp_0_21 = rddout_q_a_0_42_42 | rddout_q_a_0_43_43 ;
 
 assign rddout_q_a_0_tmp_0_22 = rddout_q_a_0_44_44 | rddout_q_a_0_45_45 ;
 
 assign rddout_q_a_0_tmp_0_23 = rddout_q_a_0_46_46 | rddout_q_a_0_47_47 ;
 
 assign rddout_q_a_0_tmp_1_0 = rddout_q_a_0_tmp_d_0_0 | rddout_q_a_0_tmp_d_0_1 ;
 
 assign rddout_q_a_0_tmp_1_1 = rddout_q_a_0_tmp_d_0_2 | rddout_q_a_0_tmp_d_0_3 ;
 
 assign rddout_q_a_0_tmp_1_2 = rddout_q_a_0_tmp_d_0_4 | rddout_q_a_0_tmp_d_0_5 ;
 
 assign rddout_q_a_0_tmp_1_3 = rddout_q_a_0_tmp_d_0_6 | rddout_q_a_0_tmp_d_0_7 ;
 
 assign rddout_q_a_0_tmp_1_4 = rddout_q_a_0_tmp_d_0_8 | rddout_q_a_0_tmp_d_0_9 ;
 
 assign rddout_q_a_0_tmp_1_5 = rddout_q_a_0_tmp_d_0_10 | rddout_q_a_0_tmp_d_0_11 ;
 
 assign rddout_q_a_0_tmp_1_6 = rddout_q_a_0_tmp_d_0_12 | rddout_q_a_0_tmp_d_0_13 ;
 
 assign rddout_q_a_0_tmp_1_7 = rddout_q_a_0_tmp_d_0_14 | rddout_q_a_0_tmp_d_0_15 ;
 
 assign rddout_q_a_0_tmp_1_8 = rddout_q_a_0_tmp_d_0_16 | rddout_q_a_0_tmp_d_0_17 ;
 
 assign rddout_q_a_0_tmp_1_9 = rddout_q_a_0_tmp_d_0_18 | rddout_q_a_0_tmp_d_0_19 ;
 
 assign rddout_q_a_0_tmp_1_10 = rddout_q_a_0_tmp_d_0_20 | rddout_q_a_0_tmp_d_0_21 ;
 
 assign rddout_q_a_0_tmp_1_11 = rddout_q_a_0_tmp_d_0_22 | rddout_q_a_0_tmp_d_0_23 ;
 
 assign rddout_q_a_0_tmp_2_0 = rddout_q_a_0_tmp_d_1_0 | rddout_q_a_0_tmp_d_1_1 ;
 
 assign rddout_q_a_0_tmp_2_1 = rddout_q_a_0_tmp_d_1_2 | rddout_q_a_0_tmp_d_1_3 ;
 
 assign rddout_q_a_0_tmp_2_2 = rddout_q_a_0_tmp_d_1_4 | rddout_q_a_0_tmp_d_1_5 ;
 
 assign rddout_q_a_0_tmp_2_3 = rddout_q_a_0_tmp_d_1_6 | rddout_q_a_0_tmp_d_1_7 ;
 
 assign rddout_q_a_0_tmp_2_4 = rddout_q_a_0_tmp_d_1_8 | rddout_q_a_0_tmp_d_1_9 ;
 
 assign rddout_q_a_0_tmp_2_5 = rddout_q_a_0_tmp_d_1_10 | rddout_q_a_0_tmp_d_1_11 ;
 
 assign rddout_q_a_0_tmp_3_0 = rddout_q_a_0_tmp_d_2_0 | rddout_q_a_0_tmp_d_2_1 ;
 
 assign rddout_q_a_0_tmp_3_1 = rddout_q_a_0_tmp_d_2_2 | rddout_q_a_0_tmp_d_2_3 ;
 
 assign rddout_q_a_0_tmp_3_2 = rddout_q_a_0_tmp_d_2_4 | rddout_q_a_0_tmp_d_2_5 ;
 
 assign rddout_q_a_0_tmp_4_0 = rddout_q_a_0_tmp_d_3_0 | rddout_q_a_0_tmp_d_3_1 ;
 
 assign rddout_q_a_0_tmp_4_1 = rddout_q_a_0_tmp_d_3_2 ;
 
 assign rddout_q_a_0_tmp_5_0 = rddout_q_a_0_tmp_d_4_0 | rddout_q_a_0_tmp_d_4_1 ;
 du_s #(32) du_0_0_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_0),.din(rddout_q_a_0_tmp_0_0));
du_s #(32) du_0_0_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_1),.din(rddout_q_a_0_tmp_0_1));
du_s #(32) du_0_0_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_2),.din(rddout_q_a_0_tmp_0_2));
du_s #(32) du_0_0_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_3),.din(rddout_q_a_0_tmp_0_3));
du_s #(32) du_0_0_4 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_4),.din(rddout_q_a_0_tmp_0_4));
du_s #(32) du_0_0_5 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_5),.din(rddout_q_a_0_tmp_0_5));
du_s #(32) du_0_0_6 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_6),.din(rddout_q_a_0_tmp_0_6));
du_s #(32) du_0_0_7 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_7),.din(rddout_q_a_0_tmp_0_7));
du_s #(32) du_0_0_8 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_8),.din(rddout_q_a_0_tmp_0_8));
du_s #(32) du_0_0_9 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_9),.din(rddout_q_a_0_tmp_0_9));
du_s #(32) du_0_0_10 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_10),.din(rddout_q_a_0_tmp_0_10));
du_s #(32) du_0_0_11 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_11),.din(rddout_q_a_0_tmp_0_11));
du_s #(32) du_0_0_12 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_12),.din(rddout_q_a_0_tmp_0_12));
du_s #(32) du_0_0_13 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_13),.din(rddout_q_a_0_tmp_0_13));
du_s #(32) du_0_0_14 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_14),.din(rddout_q_a_0_tmp_0_14));
du_s #(32) du_0_0_15 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_15),.din(rddout_q_a_0_tmp_0_15));
du_s #(32) du_0_0_16 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_16),.din(rddout_q_a_0_tmp_0_16));
du_s #(32) du_0_0_17 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_17),.din(rddout_q_a_0_tmp_0_17));
du_s #(32) du_0_0_18 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_18),.din(rddout_q_a_0_tmp_0_18));
du_s #(32) du_0_0_19 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_19),.din(rddout_q_a_0_tmp_0_19));
du_s #(32) du_0_0_20 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_20),.din(rddout_q_a_0_tmp_0_20));
du_s #(32) du_0_0_21 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_21),.din(rddout_q_a_0_tmp_0_21));
du_s #(32) du_0_0_22 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_22),.din(rddout_q_a_0_tmp_0_22));
du_s #(32) du_0_0_23 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_23),.din(rddout_q_a_0_tmp_0_23));
du_s #(32) du_0_1_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_0),.din(rddout_q_a_0_tmp_1_0));
du_s #(32) du_0_1_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_1),.din(rddout_q_a_0_tmp_1_1));
du_s #(32) du_0_1_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_2),.din(rddout_q_a_0_tmp_1_2));
du_s #(32) du_0_1_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_3),.din(rddout_q_a_0_tmp_1_3));
du_s #(32) du_0_1_4 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_4),.din(rddout_q_a_0_tmp_1_4));
du_s #(32) du_0_1_5 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_5),.din(rddout_q_a_0_tmp_1_5));
du_s #(32) du_0_1_6 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_6),.din(rddout_q_a_0_tmp_1_6));
du_s #(32) du_0_1_7 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_7),.din(rddout_q_a_0_tmp_1_7));
du_s #(32) du_0_1_8 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_8),.din(rddout_q_a_0_tmp_1_8));
du_s #(32) du_0_1_9 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_9),.din(rddout_q_a_0_tmp_1_9));
du_s #(32) du_0_1_10 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_10),.din(rddout_q_a_0_tmp_1_10));
du_s #(32) du_0_1_11 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_11),.din(rddout_q_a_0_tmp_1_11));
du_s #(32) du_0_2_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_2_0),.din(rddout_q_a_0_tmp_2_0));
du_s #(32) du_0_2_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_2_1),.din(rddout_q_a_0_tmp_2_1));
du_s #(32) du_0_2_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_2_2),.din(rddout_q_a_0_tmp_2_2));
du_s #(32) du_0_2_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_2_3),.din(rddout_q_a_0_tmp_2_3));
du_s #(32) du_0_2_4 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_2_4),.din(rddout_q_a_0_tmp_2_4));
du_s #(32) du_0_2_5 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_2_5),.din(rddout_q_a_0_tmp_2_5));
du_s #(32) du_0_3_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_3_0),.din(rddout_q_a_0_tmp_3_0));
du_s #(32) du_0_3_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_3_1),.din(rddout_q_a_0_tmp_3_1));
du_s #(32) du_0_3_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_3_2),.din(rddout_q_a_0_tmp_3_2));
du_s #(32) du_0_4_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_4_0),.din(rddout_q_a_0_tmp_4_0));
du_s #(32) du_0_4_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_4_1),.din(rddout_q_a_0_tmp_4_1));
du_s #(32) du_0_5_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_5_0),.din(rddout_q_a_0_tmp_5_0));
assign rddout_q_a_0 = rddout_q_a_0_tmp_d_5_0;
wire [31:0] rddout_q_b_0_0;
wire [31:0] rddout_q_b_0_1;
wire [31:0] rddout_q_b_0_2;
wire [31:0] rddout_q_b_0_3;
wire [31:0] rddout_q_b_0_4;
wire [31:0] rddout_q_b_0_5;
wire [31:0] rddout_q_b_0_6;
wire [31:0] rddout_q_b_0_7;
wire [31:0] rddout_q_b_0_8;
wire [31:0] rddout_q_b_0_9;
wire [31:0] rddout_q_b_0_10;
wire [31:0] rddout_q_b_0_11;
wire [31:0] rddout_q_b_0_12;
wire [31:0] rddout_q_b_0_13;
wire [31:0] rddout_q_b_0_14;
wire [31:0] rddout_q_b_0_15;
wire [31:0] rddout_q_b_0_16;
wire [31:0] rddout_q_b_0_17;
wire [31:0] rddout_q_b_0_18;
wire [31:0] rddout_q_b_0_19;
wire [31:0] rddout_q_b_0_20;
wire [31:0] rddout_q_b_0_21;
wire [31:0] rddout_q_b_0_22;
wire [31:0] rddout_q_b_0_23;
wire [31:0] rddout_q_b_0_24;
wire [31:0] rddout_q_b_0_25;
wire [31:0] rddout_q_b_0_26;
wire [31:0] rddout_q_b_0_27;
wire [31:0] rddout_q_b_0_28;
wire [31:0] rddout_q_b_0_29;
wire [31:0] rddout_q_b_0_30;
wire [31:0] rddout_q_b_0_31;
wire [31:0] rddout_q_b_0_32;
wire [31:0] rddout_q_b_0_33;
wire [31:0] rddout_q_b_0_34;
wire [31:0] rddout_q_b_0_35;
wire [31:0] rddout_q_b_0_36;
wire [31:0] rddout_q_b_0_37;
wire [31:0] rddout_q_b_0_38;
wire [31:0] rddout_q_b_0_39;
wire [31:0] rddout_q_b_0_40;
wire [31:0] rddout_q_b_0_41;
wire [31:0] rddout_q_b_0_42;
wire [31:0] rddout_q_b_0_43;
wire [31:0] rddout_q_b_0_44;
wire [31:0] rddout_q_b_0_45;
wire [31:0] rddout_q_b_0_46;
wire [31:0] rddout_q_b_0_47;
wire [31:0] rddout_q_b_0_0_0;
wire [31:0] rddout_q_b_0_1_1;
wire [31:0] rddout_q_b_0_2_2;
wire [31:0] rddout_q_b_0_3_3;
wire [31:0] rddout_q_b_0_4_4;
wire [31:0] rddout_q_b_0_5_5;
wire [31:0] rddout_q_b_0_6_6;
wire [31:0] rddout_q_b_0_7_7;
wire [31:0] rddout_q_b_0_8_8;
wire [31:0] rddout_q_b_0_9_9;
wire [31:0] rddout_q_b_0_10_10;
wire [31:0] rddout_q_b_0_11_11;
wire [31:0] rddout_q_b_0_12_12;
wire [31:0] rddout_q_b_0_13_13;
wire [31:0] rddout_q_b_0_14_14;
wire [31:0] rddout_q_b_0_15_15;
wire [31:0] rddout_q_b_0_16_16;
wire [31:0] rddout_q_b_0_17_17;
wire [31:0] rddout_q_b_0_18_18;
wire [31:0] rddout_q_b_0_19_19;
wire [31:0] rddout_q_b_0_20_20;
wire [31:0] rddout_q_b_0_21_21;
wire [31:0] rddout_q_b_0_22_22;
wire [31:0] rddout_q_b_0_23_23;
wire [31:0] rddout_q_b_0_24_24;
wire [31:0] rddout_q_b_0_25_25;
wire [31:0] rddout_q_b_0_26_26;
wire [31:0] rddout_q_b_0_27_27;
wire [31:0] rddout_q_b_0_28_28;
wire [31:0] rddout_q_b_0_29_29;
wire [31:0] rddout_q_b_0_30_30;
wire [31:0] rddout_q_b_0_31_31;
wire [31:0] rddout_q_b_0_32_32;
wire [31:0] rddout_q_b_0_33_33;
wire [31:0] rddout_q_b_0_34_34;
wire [31:0] rddout_q_b_0_35_35;
wire [31:0] rddout_q_b_0_36_36;
wire [31:0] rddout_q_b_0_37_37;
wire [31:0] rddout_q_b_0_38_38;
wire [31:0] rddout_q_b_0_39_39;
wire [31:0] rddout_q_b_0_40_40;
wire [31:0] rddout_q_b_0_41_41;
wire [31:0] rddout_q_b_0_42_42;
wire [31:0] rddout_q_b_0_43_43;
wire [31:0] rddout_q_b_0_44_44;
wire [31:0] rddout_q_b_0_45_45;
wire [31:0] rddout_q_b_0_46_46;
wire [31:0] rddout_q_b_0_47_47;
assign rddout_q_b_0_0_0[31:0] = rddout_q_b_0_0;
assign rddout_q_b_0_1_1[31:0] = rddout_q_b_0_1;
assign rddout_q_b_0_2_2[31:0] = rddout_q_b_0_2;
assign rddout_q_b_0_3_3[31:0] = rddout_q_b_0_3;
assign rddout_q_b_0_4_4[31:0] = rddout_q_b_0_4;
assign rddout_q_b_0_5_5[31:0] = rddout_q_b_0_5;
assign rddout_q_b_0_6_6[31:0] = rddout_q_b_0_6;
assign rddout_q_b_0_7_7[31:0] = rddout_q_b_0_7;
assign rddout_q_b_0_8_8[31:0] = rddout_q_b_0_8;
assign rddout_q_b_0_9_9[31:0] = rddout_q_b_0_9;
assign rddout_q_b_0_10_10[31:0] = rddout_q_b_0_10;
assign rddout_q_b_0_11_11[31:0] = rddout_q_b_0_11;
assign rddout_q_b_0_12_12[31:0] = rddout_q_b_0_12;
assign rddout_q_b_0_13_13[31:0] = rddout_q_b_0_13;
assign rddout_q_b_0_14_14[31:0] = rddout_q_b_0_14;
assign rddout_q_b_0_15_15[31:0] = rddout_q_b_0_15;
assign rddout_q_b_0_16_16[31:0] = rddout_q_b_0_16;
assign rddout_q_b_0_17_17[31:0] = rddout_q_b_0_17;
assign rddout_q_b_0_18_18[31:0] = rddout_q_b_0_18;
assign rddout_q_b_0_19_19[31:0] = rddout_q_b_0_19;
assign rddout_q_b_0_20_20[31:0] = rddout_q_b_0_20;
assign rddout_q_b_0_21_21[31:0] = rddout_q_b_0_21;
assign rddout_q_b_0_22_22[31:0] = rddout_q_b_0_22;
assign rddout_q_b_0_23_23[31:0] = rddout_q_b_0_23;
assign rddout_q_b_0_24_24[31:0] = rddout_q_b_0_24;
assign rddout_q_b_0_25_25[31:0] = rddout_q_b_0_25;
assign rddout_q_b_0_26_26[31:0] = rddout_q_b_0_26;
assign rddout_q_b_0_27_27[31:0] = rddout_q_b_0_27;
assign rddout_q_b_0_28_28[31:0] = rddout_q_b_0_28;
assign rddout_q_b_0_29_29[31:0] = rddout_q_b_0_29;
assign rddout_q_b_0_30_30[31:0] = rddout_q_b_0_30;
assign rddout_q_b_0_31_31[31:0] = rddout_q_b_0_31;
assign rddout_q_b_0_32_32[31:0] = rddout_q_b_0_32;
assign rddout_q_b_0_33_33[31:0] = rddout_q_b_0_33;
assign rddout_q_b_0_34_34[31:0] = rddout_q_b_0_34;
assign rddout_q_b_0_35_35[31:0] = rddout_q_b_0_35;
assign rddout_q_b_0_36_36[31:0] = rddout_q_b_0_36;
assign rddout_q_b_0_37_37[31:0] = rddout_q_b_0_37;
assign rddout_q_b_0_38_38[31:0] = rddout_q_b_0_38;
assign rddout_q_b_0_39_39[31:0] = rddout_q_b_0_39;
assign rddout_q_b_0_40_40[31:0] = rddout_q_b_0_40;
assign rddout_q_b_0_41_41[31:0] = rddout_q_b_0_41;
assign rddout_q_b_0_42_42[31:0] = rddout_q_b_0_42;
assign rddout_q_b_0_43_43[31:0] = rddout_q_b_0_43;
assign rddout_q_b_0_44_44[31:0] = rddout_q_b_0_44;
assign rddout_q_b_0_45_45[31:0] = rddout_q_b_0_45;
assign rddout_q_b_0_46_46[31:0] = rddout_q_b_0_46;
assign rddout_q_b_0_47_47[31:0] = rddout_q_b_0_47;
wire [31:0] rddout_q_b_0_tmp_0_0;
wire [31:0] rddout_q_b_0_tmp_0_1;
wire [31:0] rddout_q_b_0_tmp_0_2;
wire [31:0] rddout_q_b_0_tmp_0_3;
wire [31:0] rddout_q_b_0_tmp_0_4;
wire [31:0] rddout_q_b_0_tmp_0_5;
wire [31:0] rddout_q_b_0_tmp_0_6;
wire [31:0] rddout_q_b_0_tmp_0_7;
wire [31:0] rddout_q_b_0_tmp_0_8;
wire [31:0] rddout_q_b_0_tmp_0_9;
wire [31:0] rddout_q_b_0_tmp_0_10;
wire [31:0] rddout_q_b_0_tmp_0_11;
wire [31:0] rddout_q_b_0_tmp_0_12;
wire [31:0] rddout_q_b_0_tmp_0_13;
wire [31:0] rddout_q_b_0_tmp_0_14;
wire [31:0] rddout_q_b_0_tmp_0_15;
wire [31:0] rddout_q_b_0_tmp_0_16;
wire [31:0] rddout_q_b_0_tmp_0_17;
wire [31:0] rddout_q_b_0_tmp_0_18;
wire [31:0] rddout_q_b_0_tmp_0_19;
wire [31:0] rddout_q_b_0_tmp_0_20;
wire [31:0] rddout_q_b_0_tmp_0_21;
wire [31:0] rddout_q_b_0_tmp_0_22;
wire [31:0] rddout_q_b_0_tmp_0_23;
wire [31:0] rddout_q_b_0_tmp_d_0_0;
wire [31:0] rddout_q_b_0_tmp_d_0_1;
wire [31:0] rddout_q_b_0_tmp_d_0_2;
wire [31:0] rddout_q_b_0_tmp_d_0_3;
wire [31:0] rddout_q_b_0_tmp_d_0_4;
wire [31:0] rddout_q_b_0_tmp_d_0_5;
wire [31:0] rddout_q_b_0_tmp_d_0_6;
wire [31:0] rddout_q_b_0_tmp_d_0_7;
wire [31:0] rddout_q_b_0_tmp_d_0_8;
wire [31:0] rddout_q_b_0_tmp_d_0_9;
wire [31:0] rddout_q_b_0_tmp_d_0_10;
wire [31:0] rddout_q_b_0_tmp_d_0_11;
wire [31:0] rddout_q_b_0_tmp_d_0_12;
wire [31:0] rddout_q_b_0_tmp_d_0_13;
wire [31:0] rddout_q_b_0_tmp_d_0_14;
wire [31:0] rddout_q_b_0_tmp_d_0_15;
wire [31:0] rddout_q_b_0_tmp_d_0_16;
wire [31:0] rddout_q_b_0_tmp_d_0_17;
wire [31:0] rddout_q_b_0_tmp_d_0_18;
wire [31:0] rddout_q_b_0_tmp_d_0_19;
wire [31:0] rddout_q_b_0_tmp_d_0_20;
wire [31:0] rddout_q_b_0_tmp_d_0_21;
wire [31:0] rddout_q_b_0_tmp_d_0_22;
wire [31:0] rddout_q_b_0_tmp_d_0_23;
wire [31:0] rddout_q_b_0_tmp_1_0;
wire [31:0] rddout_q_b_0_tmp_1_1;
wire [31:0] rddout_q_b_0_tmp_1_2;
wire [31:0] rddout_q_b_0_tmp_1_3;
wire [31:0] rddout_q_b_0_tmp_1_4;
wire [31:0] rddout_q_b_0_tmp_1_5;
wire [31:0] rddout_q_b_0_tmp_1_6;
wire [31:0] rddout_q_b_0_tmp_1_7;
wire [31:0] rddout_q_b_0_tmp_1_8;
wire [31:0] rddout_q_b_0_tmp_1_9;
wire [31:0] rddout_q_b_0_tmp_1_10;
wire [31:0] rddout_q_b_0_tmp_1_11;
wire [31:0] rddout_q_b_0_tmp_d_1_0;
wire [31:0] rddout_q_b_0_tmp_d_1_1;
wire [31:0] rddout_q_b_0_tmp_d_1_2;
wire [31:0] rddout_q_b_0_tmp_d_1_3;
wire [31:0] rddout_q_b_0_tmp_d_1_4;
wire [31:0] rddout_q_b_0_tmp_d_1_5;
wire [31:0] rddout_q_b_0_tmp_d_1_6;
wire [31:0] rddout_q_b_0_tmp_d_1_7;
wire [31:0] rddout_q_b_0_tmp_d_1_8;
wire [31:0] rddout_q_b_0_tmp_d_1_9;
wire [31:0] rddout_q_b_0_tmp_d_1_10;
wire [31:0] rddout_q_b_0_tmp_d_1_11;
wire [31:0] rddout_q_b_0_tmp_2_0;
wire [31:0] rddout_q_b_0_tmp_2_1;
wire [31:0] rddout_q_b_0_tmp_2_2;
wire [31:0] rddout_q_b_0_tmp_2_3;
wire [31:0] rddout_q_b_0_tmp_2_4;
wire [31:0] rddout_q_b_0_tmp_2_5;
wire [31:0] rddout_q_b_0_tmp_d_2_0;
wire [31:0] rddout_q_b_0_tmp_d_2_1;
wire [31:0] rddout_q_b_0_tmp_d_2_2;
wire [31:0] rddout_q_b_0_tmp_d_2_3;
wire [31:0] rddout_q_b_0_tmp_d_2_4;
wire [31:0] rddout_q_b_0_tmp_d_2_5;
wire [31:0] rddout_q_b_0_tmp_3_0;
wire [31:0] rddout_q_b_0_tmp_3_1;
wire [31:0] rddout_q_b_0_tmp_3_2;
wire [31:0] rddout_q_b_0_tmp_d_3_0;
wire [31:0] rddout_q_b_0_tmp_d_3_1;
wire [31:0] rddout_q_b_0_tmp_d_3_2;
wire [31:0] rddout_q_b_0_tmp_4_0;
wire [31:0] rddout_q_b_0_tmp_4_1;
wire [31:0] rddout_q_b_0_tmp_d_4_0;
wire [31:0] rddout_q_b_0_tmp_d_4_1;
wire [31:0] rddout_q_b_0_tmp_5_0;
wire [31:0] rddout_q_b_0_tmp_d_5_0;

 assign rddout_q_b_0_tmp_0_0 = rddout_q_b_0_0_0 | rddout_q_b_0_1_1 ;
 
 assign rddout_q_b_0_tmp_0_1 = rddout_q_b_0_2_2 | rddout_q_b_0_3_3 ;
 
 assign rddout_q_b_0_tmp_0_2 = rddout_q_b_0_4_4 | rddout_q_b_0_5_5 ;
 
 assign rddout_q_b_0_tmp_0_3 = rddout_q_b_0_6_6 | rddout_q_b_0_7_7 ;
 
 assign rddout_q_b_0_tmp_0_4 = rddout_q_b_0_8_8 | rddout_q_b_0_9_9 ;
 
 assign rddout_q_b_0_tmp_0_5 = rddout_q_b_0_10_10 | rddout_q_b_0_11_11 ;
 
 assign rddout_q_b_0_tmp_0_6 = rddout_q_b_0_12_12 | rddout_q_b_0_13_13 ;
 
 assign rddout_q_b_0_tmp_0_7 = rddout_q_b_0_14_14 | rddout_q_b_0_15_15 ;
 
 assign rddout_q_b_0_tmp_0_8 = rddout_q_b_0_16_16 | rddout_q_b_0_17_17 ;
 
 assign rddout_q_b_0_tmp_0_9 = rddout_q_b_0_18_18 | rddout_q_b_0_19_19 ;
 
 assign rddout_q_b_0_tmp_0_10 = rddout_q_b_0_20_20 | rddout_q_b_0_21_21 ;
 
 assign rddout_q_b_0_tmp_0_11 = rddout_q_b_0_22_22 | rddout_q_b_0_23_23 ;
 
 assign rddout_q_b_0_tmp_0_12 = rddout_q_b_0_24_24 | rddout_q_b_0_25_25 ;
 
 assign rddout_q_b_0_tmp_0_13 = rddout_q_b_0_26_26 | rddout_q_b_0_27_27 ;
 
 assign rddout_q_b_0_tmp_0_14 = rddout_q_b_0_28_28 | rddout_q_b_0_29_29 ;
 
 assign rddout_q_b_0_tmp_0_15 = rddout_q_b_0_30_30 | rddout_q_b_0_31_31 ;
 
 assign rddout_q_b_0_tmp_0_16 = rddout_q_b_0_32_32 | rddout_q_b_0_33_33 ;
 
 assign rddout_q_b_0_tmp_0_17 = rddout_q_b_0_34_34 | rddout_q_b_0_35_35 ;
 
 assign rddout_q_b_0_tmp_0_18 = rddout_q_b_0_36_36 | rddout_q_b_0_37_37 ;
 
 assign rddout_q_b_0_tmp_0_19 = rddout_q_b_0_38_38 | rddout_q_b_0_39_39 ;
 
 assign rddout_q_b_0_tmp_0_20 = rddout_q_b_0_40_40 | rddout_q_b_0_41_41 ;
 
 assign rddout_q_b_0_tmp_0_21 = rddout_q_b_0_42_42 | rddout_q_b_0_43_43 ;
 
 assign rddout_q_b_0_tmp_0_22 = rddout_q_b_0_44_44 | rddout_q_b_0_45_45 ;
 
 assign rddout_q_b_0_tmp_0_23 = rddout_q_b_0_46_46 | rddout_q_b_0_47_47 ;
 
 assign rddout_q_b_0_tmp_1_0 = rddout_q_b_0_tmp_d_0_0 | rddout_q_b_0_tmp_d_0_1 ;
 
 assign rddout_q_b_0_tmp_1_1 = rddout_q_b_0_tmp_d_0_2 | rddout_q_b_0_tmp_d_0_3 ;
 
 assign rddout_q_b_0_tmp_1_2 = rddout_q_b_0_tmp_d_0_4 | rddout_q_b_0_tmp_d_0_5 ;
 
 assign rddout_q_b_0_tmp_1_3 = rddout_q_b_0_tmp_d_0_6 | rddout_q_b_0_tmp_d_0_7 ;
 
 assign rddout_q_b_0_tmp_1_4 = rddout_q_b_0_tmp_d_0_8 | rddout_q_b_0_tmp_d_0_9 ;
 
 assign rddout_q_b_0_tmp_1_5 = rddout_q_b_0_tmp_d_0_10 | rddout_q_b_0_tmp_d_0_11 ;
 
 assign rddout_q_b_0_tmp_1_6 = rddout_q_b_0_tmp_d_0_12 | rddout_q_b_0_tmp_d_0_13 ;
 
 assign rddout_q_b_0_tmp_1_7 = rddout_q_b_0_tmp_d_0_14 | rddout_q_b_0_tmp_d_0_15 ;
 
 assign rddout_q_b_0_tmp_1_8 = rddout_q_b_0_tmp_d_0_16 | rddout_q_b_0_tmp_d_0_17 ;
 
 assign rddout_q_b_0_tmp_1_9 = rddout_q_b_0_tmp_d_0_18 | rddout_q_b_0_tmp_d_0_19 ;
 
 assign rddout_q_b_0_tmp_1_10 = rddout_q_b_0_tmp_d_0_20 | rddout_q_b_0_tmp_d_0_21 ;
 
 assign rddout_q_b_0_tmp_1_11 = rddout_q_b_0_tmp_d_0_22 | rddout_q_b_0_tmp_d_0_23 ;
 
 assign rddout_q_b_0_tmp_2_0 = rddout_q_b_0_tmp_d_1_0 | rddout_q_b_0_tmp_d_1_1 ;
 
 assign rddout_q_b_0_tmp_2_1 = rddout_q_b_0_tmp_d_1_2 | rddout_q_b_0_tmp_d_1_3 ;
 
 assign rddout_q_b_0_tmp_2_2 = rddout_q_b_0_tmp_d_1_4 | rddout_q_b_0_tmp_d_1_5 ;
 
 assign rddout_q_b_0_tmp_2_3 = rddout_q_b_0_tmp_d_1_6 | rddout_q_b_0_tmp_d_1_7 ;
 
 assign rddout_q_b_0_tmp_2_4 = rddout_q_b_0_tmp_d_1_8 | rddout_q_b_0_tmp_d_1_9 ;
 
 assign rddout_q_b_0_tmp_2_5 = rddout_q_b_0_tmp_d_1_10 | rddout_q_b_0_tmp_d_1_11 ;
 
 assign rddout_q_b_0_tmp_3_0 = rddout_q_b_0_tmp_d_2_0 | rddout_q_b_0_tmp_d_2_1 ;
 
 assign rddout_q_b_0_tmp_3_1 = rddout_q_b_0_tmp_d_2_2 | rddout_q_b_0_tmp_d_2_3 ;
 
 assign rddout_q_b_0_tmp_3_2 = rddout_q_b_0_tmp_d_2_4 | rddout_q_b_0_tmp_d_2_5 ;
 
 assign rddout_q_b_0_tmp_4_0 = rddout_q_b_0_tmp_d_3_0 | rddout_q_b_0_tmp_d_3_1 ;
 
 assign rddout_q_b_0_tmp_4_1 = rddout_q_b_0_tmp_d_3_2 ;
 
 assign rddout_q_b_0_tmp_5_0 = rddout_q_b_0_tmp_d_4_0 | rddout_q_b_0_tmp_d_4_1 ;
 du_s #(32) du_1_0_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_0),.din(rddout_q_b_0_tmp_0_0));
du_s #(32) du_1_0_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_1),.din(rddout_q_b_0_tmp_0_1));
du_s #(32) du_1_0_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_2),.din(rddout_q_b_0_tmp_0_2));
du_s #(32) du_1_0_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_3),.din(rddout_q_b_0_tmp_0_3));
du_s #(32) du_1_0_4 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_4),.din(rddout_q_b_0_tmp_0_4));
du_s #(32) du_1_0_5 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_5),.din(rddout_q_b_0_tmp_0_5));
du_s #(32) du_1_0_6 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_6),.din(rddout_q_b_0_tmp_0_6));
du_s #(32) du_1_0_7 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_7),.din(rddout_q_b_0_tmp_0_7));
du_s #(32) du_1_0_8 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_8),.din(rddout_q_b_0_tmp_0_8));
du_s #(32) du_1_0_9 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_9),.din(rddout_q_b_0_tmp_0_9));
du_s #(32) du_1_0_10 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_10),.din(rddout_q_b_0_tmp_0_10));
du_s #(32) du_1_0_11 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_11),.din(rddout_q_b_0_tmp_0_11));
du_s #(32) du_1_0_12 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_12),.din(rddout_q_b_0_tmp_0_12));
du_s #(32) du_1_0_13 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_13),.din(rddout_q_b_0_tmp_0_13));
du_s #(32) du_1_0_14 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_14),.din(rddout_q_b_0_tmp_0_14));
du_s #(32) du_1_0_15 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_15),.din(rddout_q_b_0_tmp_0_15));
du_s #(32) du_1_0_16 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_16),.din(rddout_q_b_0_tmp_0_16));
du_s #(32) du_1_0_17 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_17),.din(rddout_q_b_0_tmp_0_17));
du_s #(32) du_1_0_18 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_18),.din(rddout_q_b_0_tmp_0_18));
du_s #(32) du_1_0_19 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_19),.din(rddout_q_b_0_tmp_0_19));
du_s #(32) du_1_0_20 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_20),.din(rddout_q_b_0_tmp_0_20));
du_s #(32) du_1_0_21 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_21),.din(rddout_q_b_0_tmp_0_21));
du_s #(32) du_1_0_22 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_22),.din(rddout_q_b_0_tmp_0_22));
du_s #(32) du_1_0_23 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_23),.din(rddout_q_b_0_tmp_0_23));
du_s #(32) du_1_1_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_0),.din(rddout_q_b_0_tmp_1_0));
du_s #(32) du_1_1_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_1),.din(rddout_q_b_0_tmp_1_1));
du_s #(32) du_1_1_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_2),.din(rddout_q_b_0_tmp_1_2));
du_s #(32) du_1_1_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_3),.din(rddout_q_b_0_tmp_1_3));
du_s #(32) du_1_1_4 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_4),.din(rddout_q_b_0_tmp_1_4));
du_s #(32) du_1_1_5 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_5),.din(rddout_q_b_0_tmp_1_5));
du_s #(32) du_1_1_6 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_6),.din(rddout_q_b_0_tmp_1_6));
du_s #(32) du_1_1_7 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_7),.din(rddout_q_b_0_tmp_1_7));
du_s #(32) du_1_1_8 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_8),.din(rddout_q_b_0_tmp_1_8));
du_s #(32) du_1_1_9 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_9),.din(rddout_q_b_0_tmp_1_9));
du_s #(32) du_1_1_10 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_10),.din(rddout_q_b_0_tmp_1_10));
du_s #(32) du_1_1_11 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_11),.din(rddout_q_b_0_tmp_1_11));
du_s #(32) du_1_2_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_2_0),.din(rddout_q_b_0_tmp_2_0));
du_s #(32) du_1_2_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_2_1),.din(rddout_q_b_0_tmp_2_1));
du_s #(32) du_1_2_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_2_2),.din(rddout_q_b_0_tmp_2_2));
du_s #(32) du_1_2_3 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_2_3),.din(rddout_q_b_0_tmp_2_3));
du_s #(32) du_1_2_4 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_2_4),.din(rddout_q_b_0_tmp_2_4));
du_s #(32) du_1_2_5 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_2_5),.din(rddout_q_b_0_tmp_2_5));
du_s #(32) du_1_3_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_3_0),.din(rddout_q_b_0_tmp_3_0));
du_s #(32) du_1_3_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_3_1),.din(rddout_q_b_0_tmp_3_1));
du_s #(32) du_1_3_2 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_3_2),.din(rddout_q_b_0_tmp_3_2));
du_s #(32) du_1_4_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_4_0),.din(rddout_q_b_0_tmp_4_0));
du_s #(32) du_1_4_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_4_1),.din(rddout_q_b_0_tmp_4_1));
du_s #(32) du_1_5_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_5_0),.din(rddout_q_b_0_tmp_5_0));
assign rddout_q_b_0 = rddout_q_b_0_tmp_d_5_0;
omem_bank mem_bank_0_0_a (
  .clk(clk),
  .bank_id(6'b000000),
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
  .bank_id(6'b000000),
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
  .bank_id(6'b000001),
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
  .bank_id(6'b000001),
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
  .bank_id(6'b000010),
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
  .bank_id(6'b000010),
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
  .bank_id(6'b000011),
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
  .bank_id(6'b000011),
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
  .bank_id(6'b000100),
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
  .bank_id(6'b000100),
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
  .bank_id(6'b000101),
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
  .bank_id(6'b000101),
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
  .bank_id(6'b000110),
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
  .bank_id(6'b000110),
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
  .bank_id(6'b000111),
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
  .bank_id(6'b000111),
  .wren(wren_b_0_7),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_7),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_7)
);

omem_bank mem_bank_0_8_a (
  .clk(clk),
  .bank_id(6'b001000),
  .wren(wren_a_0_8),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_8),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_8)
);

omem_bank mem_bank_0_8_b (
  .clk(clk),
  .bank_id(6'b001000),
  .wren(wren_b_0_8),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_8),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_8)
);

omem_bank mem_bank_0_9_a (
  .clk(clk),
  .bank_id(6'b001001),
  .wren(wren_a_0_9),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_9),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_9)
);

omem_bank mem_bank_0_9_b (
  .clk(clk),
  .bank_id(6'b001001),
  .wren(wren_b_0_9),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_9),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_9)
);

omem_bank mem_bank_0_10_a (
  .clk(clk),
  .bank_id(6'b001010),
  .wren(wren_a_0_10),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_10),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_10)
);

omem_bank mem_bank_0_10_b (
  .clk(clk),
  .bank_id(6'b001010),
  .wren(wren_b_0_10),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_10),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_10)
);

omem_bank mem_bank_0_11_a (
  .clk(clk),
  .bank_id(6'b001011),
  .wren(wren_a_0_11),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_11),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_11)
);

omem_bank mem_bank_0_11_b (
  .clk(clk),
  .bank_id(6'b001011),
  .wren(wren_b_0_11),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_11),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_11)
);

omem_bank mem_bank_0_12_a (
  .clk(clk),
  .bank_id(6'b001100),
  .wren(wren_a_0_12),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_12),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_12)
);

omem_bank mem_bank_0_12_b (
  .clk(clk),
  .bank_id(6'b001100),
  .wren(wren_b_0_12),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_12),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_12)
);

omem_bank mem_bank_0_13_a (
  .clk(clk),
  .bank_id(6'b001101),
  .wren(wren_a_0_13),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_13),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_13)
);

omem_bank mem_bank_0_13_b (
  .clk(clk),
  .bank_id(6'b001101),
  .wren(wren_b_0_13),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_13),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_13)
);

omem_bank mem_bank_0_14_a (
  .clk(clk),
  .bank_id(6'b001110),
  .wren(wren_a_0_14),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_14),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_14)
);

omem_bank mem_bank_0_14_b (
  .clk(clk),
  .bank_id(6'b001110),
  .wren(wren_b_0_14),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_14),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_14)
);

omem_bank mem_bank_0_15_a (
  .clk(clk),
  .bank_id(6'b001111),
  .wren(wren_a_0_15),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_15),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_15)
);

omem_bank mem_bank_0_15_b (
  .clk(clk),
  .bank_id(6'b001111),
  .wren(wren_b_0_15),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_15),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_15)
);

omem_bank mem_bank_0_16_a (
  .clk(clk),
  .bank_id(6'b010000),
  .wren(wren_a_0_16),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_16),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_16)
);

omem_bank mem_bank_0_16_b (
  .clk(clk),
  .bank_id(6'b010000),
  .wren(wren_b_0_16),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_16),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_16)
);

omem_bank mem_bank_0_17_a (
  .clk(clk),
  .bank_id(6'b010001),
  .wren(wren_a_0_17),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_17),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_17)
);

omem_bank mem_bank_0_17_b (
  .clk(clk),
  .bank_id(6'b010001),
  .wren(wren_b_0_17),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_17),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_17)
);

omem_bank mem_bank_0_18_a (
  .clk(clk),
  .bank_id(6'b010010),
  .wren(wren_a_0_18),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_18),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_18)
);

omem_bank mem_bank_0_18_b (
  .clk(clk),
  .bank_id(6'b010010),
  .wren(wren_b_0_18),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_18),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_18)
);

omem_bank mem_bank_0_19_a (
  .clk(clk),
  .bank_id(6'b010011),
  .wren(wren_a_0_19),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_19),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_19)
);

omem_bank mem_bank_0_19_b (
  .clk(clk),
  .bank_id(6'b010011),
  .wren(wren_b_0_19),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_19),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_19)
);

omem_bank mem_bank_0_20_a (
  .clk(clk),
  .bank_id(6'b010100),
  .wren(wren_a_0_20),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_20),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_20)
);

omem_bank mem_bank_0_20_b (
  .clk(clk),
  .bank_id(6'b010100),
  .wren(wren_b_0_20),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_20),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_20)
);

omem_bank mem_bank_0_21_a (
  .clk(clk),
  .bank_id(6'b010101),
  .wren(wren_a_0_21),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_21),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_21)
);

omem_bank mem_bank_0_21_b (
  .clk(clk),
  .bank_id(6'b010101),
  .wren(wren_b_0_21),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_21),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_21)
);

omem_bank mem_bank_0_22_a (
  .clk(clk),
  .bank_id(6'b010110),
  .wren(wren_a_0_22),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_22),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_22)
);

omem_bank mem_bank_0_22_b (
  .clk(clk),
  .bank_id(6'b010110),
  .wren(wren_b_0_22),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_22),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_22)
);

omem_bank mem_bank_0_23_a (
  .clk(clk),
  .bank_id(6'b010111),
  .wren(wren_a_0_23),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_23),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_23)
);

omem_bank mem_bank_0_23_b (
  .clk(clk),
  .bank_id(6'b010111),
  .wren(wren_b_0_23),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_23),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_23)
);

omem_bank mem_bank_0_24_a (
  .clk(clk),
  .bank_id(6'b011000),
  .wren(wren_a_0_24),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_24),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_24)
);

omem_bank mem_bank_0_24_b (
  .clk(clk),
  .bank_id(6'b011000),
  .wren(wren_b_0_24),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_24),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_24)
);

omem_bank mem_bank_0_25_a (
  .clk(clk),
  .bank_id(6'b011001),
  .wren(wren_a_0_25),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_25),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_25)
);

omem_bank mem_bank_0_25_b (
  .clk(clk),
  .bank_id(6'b011001),
  .wren(wren_b_0_25),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_25),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_25)
);

omem_bank mem_bank_0_26_a (
  .clk(clk),
  .bank_id(6'b011010),
  .wren(wren_a_0_26),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_26),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_26)
);

omem_bank mem_bank_0_26_b (
  .clk(clk),
  .bank_id(6'b011010),
  .wren(wren_b_0_26),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_26),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_26)
);

omem_bank mem_bank_0_27_a (
  .clk(clk),
  .bank_id(6'b011011),
  .wren(wren_a_0_27),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_27),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_27)
);

omem_bank mem_bank_0_27_b (
  .clk(clk),
  .bank_id(6'b011011),
  .wren(wren_b_0_27),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_27),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_27)
);

omem_bank mem_bank_0_28_a (
  .clk(clk),
  .bank_id(6'b011100),
  .wren(wren_a_0_28),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_28),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_28)
);

omem_bank mem_bank_0_28_b (
  .clk(clk),
  .bank_id(6'b011100),
  .wren(wren_b_0_28),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_28),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_28)
);

omem_bank mem_bank_0_29_a (
  .clk(clk),
  .bank_id(6'b011101),
  .wren(wren_a_0_29),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_29),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_29)
);

omem_bank mem_bank_0_29_b (
  .clk(clk),
  .bank_id(6'b011101),
  .wren(wren_b_0_29),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_29),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_29)
);

omem_bank mem_bank_0_30_a (
  .clk(clk),
  .bank_id(6'b011110),
  .wren(wren_a_0_30),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_30),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_30)
);

omem_bank mem_bank_0_30_b (
  .clk(clk),
  .bank_id(6'b011110),
  .wren(wren_b_0_30),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_30),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_30)
);

omem_bank mem_bank_0_31_a (
  .clk(clk),
  .bank_id(6'b011111),
  .wren(wren_a_0_31),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_31),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_31)
);

omem_bank mem_bank_0_31_b (
  .clk(clk),
  .bank_id(6'b011111),
  .wren(wren_b_0_31),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_31),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_31)
);

omem_bank mem_bank_0_32_a (
  .clk(clk),
  .bank_id(6'b100000),
  .wren(wren_a_0_32),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_32),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_32)
);

omem_bank mem_bank_0_32_b (
  .clk(clk),
  .bank_id(6'b100000),
  .wren(wren_b_0_32),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_32),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_32)
);

omem_bank mem_bank_0_33_a (
  .clk(clk),
  .bank_id(6'b100001),
  .wren(wren_a_0_33),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_33),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_33)
);

omem_bank mem_bank_0_33_b (
  .clk(clk),
  .bank_id(6'b100001),
  .wren(wren_b_0_33),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_33),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_33)
);

omem_bank mem_bank_0_34_a (
  .clk(clk),
  .bank_id(6'b100010),
  .wren(wren_a_0_34),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_34),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_34)
);

omem_bank mem_bank_0_34_b (
  .clk(clk),
  .bank_id(6'b100010),
  .wren(wren_b_0_34),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_34),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_34)
);

omem_bank mem_bank_0_35_a (
  .clk(clk),
  .bank_id(6'b100011),
  .wren(wren_a_0_35),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_35),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_35)
);

omem_bank mem_bank_0_35_b (
  .clk(clk),
  .bank_id(6'b100011),
  .wren(wren_b_0_35),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_35),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_35)
);

omem_bank mem_bank_0_36_a (
  .clk(clk),
  .bank_id(6'b100100),
  .wren(wren_a_0_36),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_36),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_36)
);

omem_bank mem_bank_0_36_b (
  .clk(clk),
  .bank_id(6'b100100),
  .wren(wren_b_0_36),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_36),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_36)
);

omem_bank mem_bank_0_37_a (
  .clk(clk),
  .bank_id(6'b100101),
  .wren(wren_a_0_37),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_37),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_37)
);

omem_bank mem_bank_0_37_b (
  .clk(clk),
  .bank_id(6'b100101),
  .wren(wren_b_0_37),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_37),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_37)
);

omem_bank mem_bank_0_38_a (
  .clk(clk),
  .bank_id(6'b100110),
  .wren(wren_a_0_38),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_38),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_38)
);

omem_bank mem_bank_0_38_b (
  .clk(clk),
  .bank_id(6'b100110),
  .wren(wren_b_0_38),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_38),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_38)
);

omem_bank mem_bank_0_39_a (
  .clk(clk),
  .bank_id(6'b100111),
  .wren(wren_a_0_39),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_39),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_39)
);

omem_bank mem_bank_0_39_b (
  .clk(clk),
  .bank_id(6'b100111),
  .wren(wren_b_0_39),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_39),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_39)
);

omem_bank mem_bank_0_40_a (
  .clk(clk),
  .bank_id(6'b101000),
  .wren(wren_a_0_40),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_40),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_40)
);

omem_bank mem_bank_0_40_b (
  .clk(clk),
  .bank_id(6'b101000),
  .wren(wren_b_0_40),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_40),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_40)
);

omem_bank mem_bank_0_41_a (
  .clk(clk),
  .bank_id(6'b101001),
  .wren(wren_a_0_41),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_41),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_41)
);

omem_bank mem_bank_0_41_b (
  .clk(clk),
  .bank_id(6'b101001),
  .wren(wren_b_0_41),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_41),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_41)
);

omem_bank mem_bank_0_42_a (
  .clk(clk),
  .bank_id(6'b101010),
  .wren(wren_a_0_42),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_42),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_42)
);

omem_bank mem_bank_0_42_b (
  .clk(clk),
  .bank_id(6'b101010),
  .wren(wren_b_0_42),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_42),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_42)
);

omem_bank mem_bank_0_43_a (
  .clk(clk),
  .bank_id(6'b101011),
  .wren(wren_a_0_43),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_43),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_43)
);

omem_bank mem_bank_0_43_b (
  .clk(clk),
  .bank_id(6'b101011),
  .wren(wren_b_0_43),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_43),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_43)
);

omem_bank mem_bank_0_44_a (
  .clk(clk),
  .bank_id(6'b101100),
  .wren(wren_a_0_44),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_44),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_44)
);

omem_bank mem_bank_0_44_b (
  .clk(clk),
  .bank_id(6'b101100),
  .wren(wren_b_0_44),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_44),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_44)
);

omem_bank mem_bank_0_45_a (
  .clk(clk),
  .bank_id(6'b101101),
  .wren(wren_a_0_45),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_45),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_45)
);

omem_bank mem_bank_0_45_b (
  .clk(clk),
  .bank_id(6'b101101),
  .wren(wren_b_0_45),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_45),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_45)
);

omem_bank mem_bank_0_46_a (
  .clk(clk),
  .bank_id(6'b101110),
  .wren(wren_a_0_46),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_46),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_46)
);

omem_bank mem_bank_0_46_b (
  .clk(clk),
  .bank_id(6'b101110),
  .wren(wren_b_0_46),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_46),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_46)
);

omem_bank mem_bank_0_47_a (
  .clk(clk),
  .bank_id(6'b101111),
  .wren(wren_a_0_47),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_47),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_47)
);

omem_bank mem_bank_0_47_b (
  .clk(clk),
  .bank_id(6'b101111),
  .wren(wren_b_0_47),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_47),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_47)
);

endmodule
module top (
input clk,
input rst,
input	[31:0] element_length,
input	[31:0] sample_size,
input wren_q,
input	[7:0] wraddr_q,
input	[5:0] wrid_q,
input	[63:0] wrdin_q,
input vld_in,
input	[10:0] rdaddr_a,
input	[10:0] rdaddr_b,
input	[63:0] val_i_y_0,
input rden_q,
input	[7:0] rdaddr_q,
input	[5:0] rdid_q,
output	[31:0] rddout_q_a_0,
output	[31:0] rddout_q_b_0
);

reg [31:0] element_length_t;
reg [31:0] cur_element_length;
reg vld_in_t, rst_l;
reg [7:0] wraddr;
wire [63:0] d1_val_i_y_0;
du_s #(64) du_99_0 (.clk(clk), .stall(1'b0), .dout(d1_val_i_y_0),.din(val_i_y_0));

	
	
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
	


wire [63:0] rddout_a_0_0;
wire [63:0] rddout_a_0_1;
wire [63:0] rddout_a_0_2;
wire [63:0] rddout_a_0_3;
wire [63:0] rddout_a_0_4;
wire [63:0] rddout_a_0_5;
wire [63:0] rddout_a_0_6;
wire [63:0] rddout_a_0_7;
wire [63:0] rddout_a_0_8;
wire [63:0] rddout_a_0_9;
wire [63:0] rddout_a_0_10;
wire [63:0] rddout_a_0_11;
wire [63:0] rddout_a_0_12;
wire [63:0] rddout_a_0_13;
wire [63:0] rddout_a_0_14;
wire [63:0] rddout_a_0_15;
wire [63:0] rddout_a_0_16;
wire [63:0] rddout_a_0_17;
wire [63:0] rddout_a_0_18;
wire [63:0] rddout_a_0_19;
wire [63:0] rddout_a_0_20;
wire [63:0] rddout_a_0_21;
wire [63:0] rddout_a_0_22;
wire [63:0] rddout_a_0_23;
wire [63:0] rddout_a_0_24;
wire [63:0] rddout_a_0_25;
wire [63:0] rddout_a_0_26;
wire [63:0] rddout_a_0_27;
wire [63:0] rddout_a_0_28;
wire [63:0] rddout_a_0_29;
wire [63:0] rddout_a_0_30;
wire [63:0] rddout_a_0_31;
wire [63:0] rddout_a_0_32;
wire [63:0] rddout_a_0_33;
wire [63:0] rddout_a_0_34;
wire [63:0] rddout_a_0_35;
wire [63:0] rddout_a_0_36;
wire [63:0] rddout_a_0_37;
wire [63:0] rddout_a_0_38;
wire [63:0] rddout_a_0_39;
wire [63:0] rddout_a_0_40;
wire [63:0] rddout_a_0_41;
wire [63:0] rddout_a_0_42;
wire [63:0] rddout_a_0_43;
wire [63:0] rddout_a_0_44;
wire [63:0] rddout_a_0_45;
wire [63:0] rddout_a_0_46;
wire [63:0] rddout_a_0_47;


wire [63:0] rddout_b_0_0;
wire [63:0] rddout_b_0_1;
wire [63:0] rddout_b_0_2;
wire [63:0] rddout_b_0_3;
wire [63:0] rddout_b_0_4;
wire [63:0] rddout_b_0_5;
wire [63:0] rddout_b_0_6;
wire [63:0] rddout_b_0_7;
wire [63:0] rddout_b_0_8;
wire [63:0] rddout_b_0_9;
wire [63:0] rddout_b_0_10;
wire [63:0] rddout_b_0_11;
wire [63:0] rddout_b_0_12;
wire [63:0] rddout_b_0_13;
wire [63:0] rddout_b_0_14;
wire [63:0] rddout_b_0_15;
wire [63:0] rddout_b_0_16;
wire [63:0] rddout_b_0_17;
wire [63:0] rddout_b_0_18;
wire [63:0] rddout_b_0_19;
wire [63:0] rddout_b_0_20;
wire [63:0] rddout_b_0_21;
wire [63:0] rddout_b_0_22;
wire [63:0] rddout_b_0_23;
wire [63:0] rddout_b_0_24;
wire [63:0] rddout_b_0_25;
wire [63:0] rddout_b_0_26;
wire [63:0] rddout_b_0_27;
wire [63:0] rddout_b_0_28;
wire [63:0] rddout_b_0_29;
wire [63:0] rddout_b_0_30;
wire [63:0] rddout_b_0_31;
wire [63:0] rddout_b_0_32;
wire [63:0] rddout_b_0_33;
wire [63:0] rddout_b_0_34;
wire [63:0] rddout_b_0_35;
wire [63:0] rddout_b_0_36;
wire [63:0] rddout_b_0_37;
wire [63:0] rddout_b_0_38;
wire [63:0] rddout_b_0_39;
wire [63:0] rddout_b_0_40;
wire [63:0] rddout_b_0_41;
wire [63:0] rddout_b_0_42;
wire [63:0] rddout_b_0_43;
wire [63:0] rddout_b_0_44;
wire [63:0] rddout_b_0_45;
wire [63:0] rddout_b_0_46;
wire [63:0] rddout_b_0_47;




wire [63:0] val_i_x_0_a;
wire [63:0] val_i_x_1_a;
wire [63:0] val_i_x_2_a;
wire [63:0] val_i_x_3_a;
wire [63:0] val_i_x_4_a;
wire [63:0] val_i_x_5_a;
wire [63:0] val_i_x_6_a;
wire [63:0] val_i_x_7_a;
wire [63:0] val_i_x_8_a;
wire [63:0] val_i_x_9_a;
wire [63:0] val_i_x_10_a;
wire [63:0] val_i_x_11_a;
wire [63:0] val_i_x_12_a;
wire [63:0] val_i_x_13_a;
wire [63:0] val_i_x_14_a;
wire [63:0] val_i_x_15_a;
wire [63:0] val_i_x_16_a;
wire [63:0] val_i_x_17_a;
wire [63:0] val_i_x_18_a;
wire [63:0] val_i_x_19_a;
wire [63:0] val_i_x_20_a;
wire [63:0] val_i_x_21_a;
wire [63:0] val_i_x_22_a;
wire [63:0] val_i_x_23_a;
wire [63:0] val_i_x_24_a;
wire [63:0] val_i_x_25_a;
wire [63:0] val_i_x_26_a;
wire [63:0] val_i_x_27_a;
wire [63:0] val_i_x_28_a;
wire [63:0] val_i_x_29_a;
wire [63:0] val_i_x_30_a;
wire [63:0] val_i_x_31_a;
wire [63:0] val_i_x_32_a;
wire [63:0] val_i_x_33_a;
wire [63:0] val_i_x_34_a;
wire [63:0] val_i_x_35_a;
wire [63:0] val_i_x_36_a;
wire [63:0] val_i_x_37_a;
wire [63:0] val_i_x_38_a;
wire [63:0] val_i_x_39_a;
wire [63:0] val_i_x_40_a;
wire [63:0] val_i_x_41_a;
wire [63:0] val_i_x_42_a;
wire [63:0] val_i_x_43_a;
wire [63:0] val_i_x_44_a;
wire [63:0] val_i_x_45_a;
wire [63:0] val_i_x_46_a;
wire [63:0] val_i_x_47_a;
wire [31:0] linkdiseq_0_0_a;
wire [31:0] linkdiseq_0_1_a;
wire [31:0] linkdiseq_0_2_a;
wire [31:0] linkdiseq_0_3_a;
wire [31:0] linkdiseq_0_4_a;
wire [31:0] linkdiseq_0_5_a;
wire [31:0] linkdiseq_0_6_a;
wire [31:0] linkdiseq_0_7_a;
wire [31:0] linkdiseq_0_8_a;
wire [31:0] linkdiseq_0_9_a;
wire [31:0] linkdiseq_0_10_a;
wire [31:0] linkdiseq_0_11_a;
wire [31:0] linkdiseq_0_12_a;
wire [31:0] linkdiseq_0_13_a;
wire [31:0] linkdiseq_0_14_a;
wire [31:0] linkdiseq_0_15_a;
wire [31:0] linkdiseq_0_16_a;
wire [31:0] linkdiseq_0_17_a;
wire [31:0] linkdiseq_0_18_a;
wire [31:0] linkdiseq_0_19_a;
wire [31:0] linkdiseq_0_20_a;
wire [31:0] linkdiseq_0_21_a;
wire [31:0] linkdiseq_0_22_a;
wire [31:0] linkdiseq_0_23_a;
wire [31:0] linkdiseq_0_24_a;
wire [31:0] linkdiseq_0_25_a;
wire [31:0] linkdiseq_0_26_a;
wire [31:0] linkdiseq_0_27_a;
wire [31:0] linkdiseq_0_28_a;
wire [31:0] linkdiseq_0_29_a;
wire [31:0] linkdiseq_0_30_a;
wire [31:0] linkdiseq_0_31_a;
wire [31:0] linkdiseq_0_32_a;
wire [31:0] linkdiseq_0_33_a;
wire [31:0] linkdiseq_0_34_a;
wire [31:0] linkdiseq_0_35_a;
wire [31:0] linkdiseq_0_36_a;
wire [31:0] linkdiseq_0_37_a;
wire [31:0] linkdiseq_0_38_a;
wire [31:0] linkdiseq_0_39_a;
wire [31:0] linkdiseq_0_40_a;
wire [31:0] linkdiseq_0_41_a;
wire [31:0] linkdiseq_0_42_a;
wire [31:0] linkdiseq_0_43_a;
wire [31:0] linkdiseq_0_44_a;
wire [31:0] linkdiseq_0_45_a;
wire [31:0] linkdiseq_0_46_a;
wire [31:0] linkdiseq_0_47_a;
wire [0:0] linkdiseq_wren_0_0_a;
wire [0:0] linkdiseq_wren_0_1_a;
wire [0:0] linkdiseq_wren_0_2_a;
wire [0:0] linkdiseq_wren_0_3_a;
wire [0:0] linkdiseq_wren_0_4_a;
wire [0:0] linkdiseq_wren_0_5_a;
wire [0:0] linkdiseq_wren_0_6_a;
wire [0:0] linkdiseq_wren_0_7_a;
wire [0:0] linkdiseq_wren_0_8_a;
wire [0:0] linkdiseq_wren_0_9_a;
wire [0:0] linkdiseq_wren_0_10_a;
wire [0:0] linkdiseq_wren_0_11_a;
wire [0:0] linkdiseq_wren_0_12_a;
wire [0:0] linkdiseq_wren_0_13_a;
wire [0:0] linkdiseq_wren_0_14_a;
wire [0:0] linkdiseq_wren_0_15_a;
wire [0:0] linkdiseq_wren_0_16_a;
wire [0:0] linkdiseq_wren_0_17_a;
wire [0:0] linkdiseq_wren_0_18_a;
wire [0:0] linkdiseq_wren_0_19_a;
wire [0:0] linkdiseq_wren_0_20_a;
wire [0:0] linkdiseq_wren_0_21_a;
wire [0:0] linkdiseq_wren_0_22_a;
wire [0:0] linkdiseq_wren_0_23_a;
wire [0:0] linkdiseq_wren_0_24_a;
wire [0:0] linkdiseq_wren_0_25_a;
wire [0:0] linkdiseq_wren_0_26_a;
wire [0:0] linkdiseq_wren_0_27_a;
wire [0:0] linkdiseq_wren_0_28_a;
wire [0:0] linkdiseq_wren_0_29_a;
wire [0:0] linkdiseq_wren_0_30_a;
wire [0:0] linkdiseq_wren_0_31_a;
wire [0:0] linkdiseq_wren_0_32_a;
wire [0:0] linkdiseq_wren_0_33_a;
wire [0:0] linkdiseq_wren_0_34_a;
wire [0:0] linkdiseq_wren_0_35_a;
wire [0:0] linkdiseq_wren_0_36_a;
wire [0:0] linkdiseq_wren_0_37_a;
wire [0:0] linkdiseq_wren_0_38_a;
wire [0:0] linkdiseq_wren_0_39_a;
wire [0:0] linkdiseq_wren_0_40_a;
wire [0:0] linkdiseq_wren_0_41_a;
wire [0:0] linkdiseq_wren_0_42_a;
wire [0:0] linkdiseq_wren_0_43_a;
wire [0:0] linkdiseq_wren_0_44_a;
wire [0:0] linkdiseq_wren_0_45_a;
wire [0:0] linkdiseq_wren_0_46_a;
wire [0:0] linkdiseq_wren_0_47_a;


wire [63:0] val_i_x_0_b;
wire [63:0] val_i_x_1_b;
wire [63:0] val_i_x_2_b;
wire [63:0] val_i_x_3_b;
wire [63:0] val_i_x_4_b;
wire [63:0] val_i_x_5_b;
wire [63:0] val_i_x_6_b;
wire [63:0] val_i_x_7_b;
wire [63:0] val_i_x_8_b;
wire [63:0] val_i_x_9_b;
wire [63:0] val_i_x_10_b;
wire [63:0] val_i_x_11_b;
wire [63:0] val_i_x_12_b;
wire [63:0] val_i_x_13_b;
wire [63:0] val_i_x_14_b;
wire [63:0] val_i_x_15_b;
wire [63:0] val_i_x_16_b;
wire [63:0] val_i_x_17_b;
wire [63:0] val_i_x_18_b;
wire [63:0] val_i_x_19_b;
wire [63:0] val_i_x_20_b;
wire [63:0] val_i_x_21_b;
wire [63:0] val_i_x_22_b;
wire [63:0] val_i_x_23_b;
wire [63:0] val_i_x_24_b;
wire [63:0] val_i_x_25_b;
wire [63:0] val_i_x_26_b;
wire [63:0] val_i_x_27_b;
wire [63:0] val_i_x_28_b;
wire [63:0] val_i_x_29_b;
wire [63:0] val_i_x_30_b;
wire [63:0] val_i_x_31_b;
wire [63:0] val_i_x_32_b;
wire [63:0] val_i_x_33_b;
wire [63:0] val_i_x_34_b;
wire [63:0] val_i_x_35_b;
wire [63:0] val_i_x_36_b;
wire [63:0] val_i_x_37_b;
wire [63:0] val_i_x_38_b;
wire [63:0] val_i_x_39_b;
wire [63:0] val_i_x_40_b;
wire [63:0] val_i_x_41_b;
wire [63:0] val_i_x_42_b;
wire [63:0] val_i_x_43_b;
wire [63:0] val_i_x_44_b;
wire [63:0] val_i_x_45_b;
wire [63:0] val_i_x_46_b;
wire [63:0] val_i_x_47_b;
wire [31:0] linkdiseq_0_0_b;
wire [31:0] linkdiseq_0_1_b;
wire [31:0] linkdiseq_0_2_b;
wire [31:0] linkdiseq_0_3_b;
wire [31:0] linkdiseq_0_4_b;
wire [31:0] linkdiseq_0_5_b;
wire [31:0] linkdiseq_0_6_b;
wire [31:0] linkdiseq_0_7_b;
wire [31:0] linkdiseq_0_8_b;
wire [31:0] linkdiseq_0_9_b;
wire [31:0] linkdiseq_0_10_b;
wire [31:0] linkdiseq_0_11_b;
wire [31:0] linkdiseq_0_12_b;
wire [31:0] linkdiseq_0_13_b;
wire [31:0] linkdiseq_0_14_b;
wire [31:0] linkdiseq_0_15_b;
wire [31:0] linkdiseq_0_16_b;
wire [31:0] linkdiseq_0_17_b;
wire [31:0] linkdiseq_0_18_b;
wire [31:0] linkdiseq_0_19_b;
wire [31:0] linkdiseq_0_20_b;
wire [31:0] linkdiseq_0_21_b;
wire [31:0] linkdiseq_0_22_b;
wire [31:0] linkdiseq_0_23_b;
wire [31:0] linkdiseq_0_24_b;
wire [31:0] linkdiseq_0_25_b;
wire [31:0] linkdiseq_0_26_b;
wire [31:0] linkdiseq_0_27_b;
wire [31:0] linkdiseq_0_28_b;
wire [31:0] linkdiseq_0_29_b;
wire [31:0] linkdiseq_0_30_b;
wire [31:0] linkdiseq_0_31_b;
wire [31:0] linkdiseq_0_32_b;
wire [31:0] linkdiseq_0_33_b;
wire [31:0] linkdiseq_0_34_b;
wire [31:0] linkdiseq_0_35_b;
wire [31:0] linkdiseq_0_36_b;
wire [31:0] linkdiseq_0_37_b;
wire [31:0] linkdiseq_0_38_b;
wire [31:0] linkdiseq_0_39_b;
wire [31:0] linkdiseq_0_40_b;
wire [31:0] linkdiseq_0_41_b;
wire [31:0] linkdiseq_0_42_b;
wire [31:0] linkdiseq_0_43_b;
wire [31:0] linkdiseq_0_44_b;
wire [31:0] linkdiseq_0_45_b;
wire [31:0] linkdiseq_0_46_b;
wire [31:0] linkdiseq_0_47_b;
wire [0:0] linkdiseq_wren_0_0_b;
wire [0:0] linkdiseq_wren_0_1_b;
wire [0:0] linkdiseq_wren_0_2_b;
wire [0:0] linkdiseq_wren_0_3_b;
wire [0:0] linkdiseq_wren_0_4_b;
wire [0:0] linkdiseq_wren_0_5_b;
wire [0:0] linkdiseq_wren_0_6_b;
wire [0:0] linkdiseq_wren_0_7_b;
wire [0:0] linkdiseq_wren_0_8_b;
wire [0:0] linkdiseq_wren_0_9_b;
wire [0:0] linkdiseq_wren_0_10_b;
wire [0:0] linkdiseq_wren_0_11_b;
wire [0:0] linkdiseq_wren_0_12_b;
wire [0:0] linkdiseq_wren_0_13_b;
wire [0:0] linkdiseq_wren_0_14_b;
wire [0:0] linkdiseq_wren_0_15_b;
wire [0:0] linkdiseq_wren_0_16_b;
wire [0:0] linkdiseq_wren_0_17_b;
wire [0:0] linkdiseq_wren_0_18_b;
wire [0:0] linkdiseq_wren_0_19_b;
wire [0:0] linkdiseq_wren_0_20_b;
wire [0:0] linkdiseq_wren_0_21_b;
wire [0:0] linkdiseq_wren_0_22_b;
wire [0:0] linkdiseq_wren_0_23_b;
wire [0:0] linkdiseq_wren_0_24_b;
wire [0:0] linkdiseq_wren_0_25_b;
wire [0:0] linkdiseq_wren_0_26_b;
wire [0:0] linkdiseq_wren_0_27_b;
wire [0:0] linkdiseq_wren_0_28_b;
wire [0:0] linkdiseq_wren_0_29_b;
wire [0:0] linkdiseq_wren_0_30_b;
wire [0:0] linkdiseq_wren_0_31_b;
wire [0:0] linkdiseq_wren_0_32_b;
wire [0:0] linkdiseq_wren_0_33_b;
wire [0:0] linkdiseq_wren_0_34_b;
wire [0:0] linkdiseq_wren_0_35_b;
wire [0:0] linkdiseq_wren_0_36_b;
wire [0:0] linkdiseq_wren_0_37_b;
wire [0:0] linkdiseq_wren_0_38_b;
wire [0:0] linkdiseq_wren_0_39_b;
wire [0:0] linkdiseq_wren_0_40_b;
wire [0:0] linkdiseq_wren_0_41_b;
wire [0:0] linkdiseq_wren_0_42_b;
wire [0:0] linkdiseq_wren_0_43_b;
wire [0:0] linkdiseq_wren_0_44_b;
wire [0:0] linkdiseq_wren_0_45_b;
wire [0:0] linkdiseq_wren_0_46_b;
wire [0:0] linkdiseq_wren_0_47_b;


wire [0:0] wren_a_0_0;
wire [0:0] wren_a_0_1;
wire [0:0] wren_a_0_2;
wire [0:0] wren_a_0_3;
wire [0:0] wren_a_0_4;
wire [0:0] wren_a_0_5;
wire [0:0] wren_a_0_6;
wire [0:0] wren_a_0_7;
wire [0:0] wren_a_0_8;
wire [0:0] wren_a_0_9;
wire [0:0] wren_a_0_10;
wire [0:0] wren_a_0_11;
wire [0:0] wren_a_0_12;
wire [0:0] wren_a_0_13;
wire [0:0] wren_a_0_14;
wire [0:0] wren_a_0_15;
wire [0:0] wren_a_0_16;
wire [0:0] wren_a_0_17;
wire [0:0] wren_a_0_18;
wire [0:0] wren_a_0_19;
wire [0:0] wren_a_0_20;
wire [0:0] wren_a_0_21;
wire [0:0] wren_a_0_22;
wire [0:0] wren_a_0_23;
wire [0:0] wren_a_0_24;
wire [0:0] wren_a_0_25;
wire [0:0] wren_a_0_26;
wire [0:0] wren_a_0_27;
wire [0:0] wren_a_0_28;
wire [0:0] wren_a_0_29;
wire [0:0] wren_a_0_30;
wire [0:0] wren_a_0_31;
wire [0:0] wren_a_0_32;
wire [0:0] wren_a_0_33;
wire [0:0] wren_a_0_34;
wire [0:0] wren_a_0_35;
wire [0:0] wren_a_0_36;
wire [0:0] wren_a_0_37;
wire [0:0] wren_a_0_38;
wire [0:0] wren_a_0_39;
wire [0:0] wren_a_0_40;
wire [0:0] wren_a_0_41;
wire [0:0] wren_a_0_42;
wire [0:0] wren_a_0_43;
wire [0:0] wren_a_0_44;
wire [0:0] wren_a_0_45;
wire [0:0] wren_a_0_46;
wire [0:0] wren_a_0_47;
wire [31:0] wrdin_a_0_0;
wire [31:0] wrdin_a_0_1;
wire [31:0] wrdin_a_0_2;
wire [31:0] wrdin_a_0_3;
wire [31:0] wrdin_a_0_4;
wire [31:0] wrdin_a_0_5;
wire [31:0] wrdin_a_0_6;
wire [31:0] wrdin_a_0_7;
wire [31:0] wrdin_a_0_8;
wire [31:0] wrdin_a_0_9;
wire [31:0] wrdin_a_0_10;
wire [31:0] wrdin_a_0_11;
wire [31:0] wrdin_a_0_12;
wire [31:0] wrdin_a_0_13;
wire [31:0] wrdin_a_0_14;
wire [31:0] wrdin_a_0_15;
wire [31:0] wrdin_a_0_16;
wire [31:0] wrdin_a_0_17;
wire [31:0] wrdin_a_0_18;
wire [31:0] wrdin_a_0_19;
wire [31:0] wrdin_a_0_20;
wire [31:0] wrdin_a_0_21;
wire [31:0] wrdin_a_0_22;
wire [31:0] wrdin_a_0_23;
wire [31:0] wrdin_a_0_24;
wire [31:0] wrdin_a_0_25;
wire [31:0] wrdin_a_0_26;
wire [31:0] wrdin_a_0_27;
wire [31:0] wrdin_a_0_28;
wire [31:0] wrdin_a_0_29;
wire [31:0] wrdin_a_0_30;
wire [31:0] wrdin_a_0_31;
wire [31:0] wrdin_a_0_32;
wire [31:0] wrdin_a_0_33;
wire [31:0] wrdin_a_0_34;
wire [31:0] wrdin_a_0_35;
wire [31:0] wrdin_a_0_36;
wire [31:0] wrdin_a_0_37;
wire [31:0] wrdin_a_0_38;
wire [31:0] wrdin_a_0_39;
wire [31:0] wrdin_a_0_40;
wire [31:0] wrdin_a_0_41;
wire [31:0] wrdin_a_0_42;
wire [31:0] wrdin_a_0_43;
wire [31:0] wrdin_a_0_44;
wire [31:0] wrdin_a_0_45;
wire [31:0] wrdin_a_0_46;
wire [31:0] wrdin_a_0_47;


wire [0:0] wren_b_0_0;
wire [0:0] wren_b_0_1;
wire [0:0] wren_b_0_2;
wire [0:0] wren_b_0_3;
wire [0:0] wren_b_0_4;
wire [0:0] wren_b_0_5;
wire [0:0] wren_b_0_6;
wire [0:0] wren_b_0_7;
wire [0:0] wren_b_0_8;
wire [0:0] wren_b_0_9;
wire [0:0] wren_b_0_10;
wire [0:0] wren_b_0_11;
wire [0:0] wren_b_0_12;
wire [0:0] wren_b_0_13;
wire [0:0] wren_b_0_14;
wire [0:0] wren_b_0_15;
wire [0:0] wren_b_0_16;
wire [0:0] wren_b_0_17;
wire [0:0] wren_b_0_18;
wire [0:0] wren_b_0_19;
wire [0:0] wren_b_0_20;
wire [0:0] wren_b_0_21;
wire [0:0] wren_b_0_22;
wire [0:0] wren_b_0_23;
wire [0:0] wren_b_0_24;
wire [0:0] wren_b_0_25;
wire [0:0] wren_b_0_26;
wire [0:0] wren_b_0_27;
wire [0:0] wren_b_0_28;
wire [0:0] wren_b_0_29;
wire [0:0] wren_b_0_30;
wire [0:0] wren_b_0_31;
wire [0:0] wren_b_0_32;
wire [0:0] wren_b_0_33;
wire [0:0] wren_b_0_34;
wire [0:0] wren_b_0_35;
wire [0:0] wren_b_0_36;
wire [0:0] wren_b_0_37;
wire [0:0] wren_b_0_38;
wire [0:0] wren_b_0_39;
wire [0:0] wren_b_0_40;
wire [0:0] wren_b_0_41;
wire [0:0] wren_b_0_42;
wire [0:0] wren_b_0_43;
wire [0:0] wren_b_0_44;
wire [0:0] wren_b_0_45;
wire [0:0] wren_b_0_46;
wire [0:0] wren_b_0_47;
wire [31:0] wrdin_b_0_0;
wire [31:0] wrdin_b_0_1;
wire [31:0] wrdin_b_0_2;
wire [31:0] wrdin_b_0_3;
wire [31:0] wrdin_b_0_4;
wire [31:0] wrdin_b_0_5;
wire [31:0] wrdin_b_0_6;
wire [31:0] wrdin_b_0_7;
wire [31:0] wrdin_b_0_8;
wire [31:0] wrdin_b_0_9;
wire [31:0] wrdin_b_0_10;
wire [31:0] wrdin_b_0_11;
wire [31:0] wrdin_b_0_12;
wire [31:0] wrdin_b_0_13;
wire [31:0] wrdin_b_0_14;
wire [31:0] wrdin_b_0_15;
wire [31:0] wrdin_b_0_16;
wire [31:0] wrdin_b_0_17;
wire [31:0] wrdin_b_0_18;
wire [31:0] wrdin_b_0_19;
wire [31:0] wrdin_b_0_20;
wire [31:0] wrdin_b_0_21;
wire [31:0] wrdin_b_0_22;
wire [31:0] wrdin_b_0_23;
wire [31:0] wrdin_b_0_24;
wire [31:0] wrdin_b_0_25;
wire [31:0] wrdin_b_0_26;
wire [31:0] wrdin_b_0_27;
wire [31:0] wrdin_b_0_28;
wire [31:0] wrdin_b_0_29;
wire [31:0] wrdin_b_0_30;
wire [31:0] wrdin_b_0_31;
wire [31:0] wrdin_b_0_32;
wire [31:0] wrdin_b_0_33;
wire [31:0] wrdin_b_0_34;
wire [31:0] wrdin_b_0_35;
wire [31:0] wrdin_b_0_36;
wire [31:0] wrdin_b_0_37;
wire [31:0] wrdin_b_0_38;
wire [31:0] wrdin_b_0_39;
wire [31:0] wrdin_b_0_40;
wire [31:0] wrdin_b_0_41;
wire [31:0] wrdin_b_0_42;
wire [31:0] wrdin_b_0_43;
wire [31:0] wrdin_b_0_44;
wire [31:0] wrdin_b_0_45;
wire [31:0] wrdin_b_0_46;
wire [31:0] wrdin_b_0_47;



	
	
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
	


assign val_i_x_0_a[63:0] = rddout_a_0_0[63:0];
assign val_i_x_1_a[63:0] = rddout_a_0_1[63:0];
assign val_i_x_2_a[63:0] = rddout_a_0_2[63:0];
assign val_i_x_3_a[63:0] = rddout_a_0_3[63:0];
assign val_i_x_4_a[63:0] = rddout_a_0_4[63:0];
assign val_i_x_5_a[63:0] = rddout_a_0_5[63:0];
assign val_i_x_6_a[63:0] = rddout_a_0_6[63:0];
assign val_i_x_7_a[63:0] = rddout_a_0_7[63:0];
assign val_i_x_8_a[63:0] = rddout_a_0_8[63:0];
assign val_i_x_9_a[63:0] = rddout_a_0_9[63:0];
assign val_i_x_10_a[63:0] = rddout_a_0_10[63:0];
assign val_i_x_11_a[63:0] = rddout_a_0_11[63:0];
assign val_i_x_12_a[63:0] = rddout_a_0_12[63:0];
assign val_i_x_13_a[63:0] = rddout_a_0_13[63:0];
assign val_i_x_14_a[63:0] = rddout_a_0_14[63:0];
assign val_i_x_15_a[63:0] = rddout_a_0_15[63:0];
assign val_i_x_16_a[63:0] = rddout_a_0_16[63:0];
assign val_i_x_17_a[63:0] = rddout_a_0_17[63:0];
assign val_i_x_18_a[63:0] = rddout_a_0_18[63:0];
assign val_i_x_19_a[63:0] = rddout_a_0_19[63:0];
assign val_i_x_20_a[63:0] = rddout_a_0_20[63:0];
assign val_i_x_21_a[63:0] = rddout_a_0_21[63:0];
assign val_i_x_22_a[63:0] = rddout_a_0_22[63:0];
assign val_i_x_23_a[63:0] = rddout_a_0_23[63:0];
assign val_i_x_24_a[63:0] = rddout_a_0_24[63:0];
assign val_i_x_25_a[63:0] = rddout_a_0_25[63:0];
assign val_i_x_26_a[63:0] = rddout_a_0_26[63:0];
assign val_i_x_27_a[63:0] = rddout_a_0_27[63:0];
assign val_i_x_28_a[63:0] = rddout_a_0_28[63:0];
assign val_i_x_29_a[63:0] = rddout_a_0_29[63:0];
assign val_i_x_30_a[63:0] = rddout_a_0_30[63:0];
assign val_i_x_31_a[63:0] = rddout_a_0_31[63:0];
assign val_i_x_32_a[63:0] = rddout_a_0_32[63:0];
assign val_i_x_33_a[63:0] = rddout_a_0_33[63:0];
assign val_i_x_34_a[63:0] = rddout_a_0_34[63:0];
assign val_i_x_35_a[63:0] = rddout_a_0_35[63:0];
assign val_i_x_36_a[63:0] = rddout_a_0_36[63:0];
assign val_i_x_37_a[63:0] = rddout_a_0_37[63:0];
assign val_i_x_38_a[63:0] = rddout_a_0_38[63:0];
assign val_i_x_39_a[63:0] = rddout_a_0_39[63:0];
assign val_i_x_40_a[63:0] = rddout_a_0_40[63:0];
assign val_i_x_41_a[63:0] = rddout_a_0_41[63:0];
assign val_i_x_42_a[63:0] = rddout_a_0_42[63:0];
assign val_i_x_43_a[63:0] = rddout_a_0_43[63:0];
assign val_i_x_44_a[63:0] = rddout_a_0_44[63:0];
assign val_i_x_45_a[63:0] = rddout_a_0_45[63:0];
assign val_i_x_46_a[63:0] = rddout_a_0_46[63:0];
assign val_i_x_47_a[63:0] = rddout_a_0_47[63:0];


assign val_i_x_0_b[63:0] = rddout_b_0_0[63:0];
assign val_i_x_1_b[63:0] = rddout_b_0_1[63:0];
assign val_i_x_2_b[63:0] = rddout_b_0_2[63:0];
assign val_i_x_3_b[63:0] = rddout_b_0_3[63:0];
assign val_i_x_4_b[63:0] = rddout_b_0_4[63:0];
assign val_i_x_5_b[63:0] = rddout_b_0_5[63:0];
assign val_i_x_6_b[63:0] = rddout_b_0_6[63:0];
assign val_i_x_7_b[63:0] = rddout_b_0_7[63:0];
assign val_i_x_8_b[63:0] = rddout_b_0_8[63:0];
assign val_i_x_9_b[63:0] = rddout_b_0_9[63:0];
assign val_i_x_10_b[63:0] = rddout_b_0_10[63:0];
assign val_i_x_11_b[63:0] = rddout_b_0_11[63:0];
assign val_i_x_12_b[63:0] = rddout_b_0_12[63:0];
assign val_i_x_13_b[63:0] = rddout_b_0_13[63:0];
assign val_i_x_14_b[63:0] = rddout_b_0_14[63:0];
assign val_i_x_15_b[63:0] = rddout_b_0_15[63:0];
assign val_i_x_16_b[63:0] = rddout_b_0_16[63:0];
assign val_i_x_17_b[63:0] = rddout_b_0_17[63:0];
assign val_i_x_18_b[63:0] = rddout_b_0_18[63:0];
assign val_i_x_19_b[63:0] = rddout_b_0_19[63:0];
assign val_i_x_20_b[63:0] = rddout_b_0_20[63:0];
assign val_i_x_21_b[63:0] = rddout_b_0_21[63:0];
assign val_i_x_22_b[63:0] = rddout_b_0_22[63:0];
assign val_i_x_23_b[63:0] = rddout_b_0_23[63:0];
assign val_i_x_24_b[63:0] = rddout_b_0_24[63:0];
assign val_i_x_25_b[63:0] = rddout_b_0_25[63:0];
assign val_i_x_26_b[63:0] = rddout_b_0_26[63:0];
assign val_i_x_27_b[63:0] = rddout_b_0_27[63:0];
assign val_i_x_28_b[63:0] = rddout_b_0_28[63:0];
assign val_i_x_29_b[63:0] = rddout_b_0_29[63:0];
assign val_i_x_30_b[63:0] = rddout_b_0_30[63:0];
assign val_i_x_31_b[63:0] = rddout_b_0_31[63:0];
assign val_i_x_32_b[63:0] = rddout_b_0_32[63:0];
assign val_i_x_33_b[63:0] = rddout_b_0_33[63:0];
assign val_i_x_34_b[63:0] = rddout_b_0_34[63:0];
assign val_i_x_35_b[63:0] = rddout_b_0_35[63:0];
assign val_i_x_36_b[63:0] = rddout_b_0_36[63:0];
assign val_i_x_37_b[63:0] = rddout_b_0_37[63:0];
assign val_i_x_38_b[63:0] = rddout_b_0_38[63:0];
assign val_i_x_39_b[63:0] = rddout_b_0_39[63:0];
assign val_i_x_40_b[63:0] = rddout_b_0_40[63:0];
assign val_i_x_41_b[63:0] = rddout_b_0_41[63:0];
assign val_i_x_42_b[63:0] = rddout_b_0_42[63:0];
assign val_i_x_43_b[63:0] = rddout_b_0_43[63:0];
assign val_i_x_44_b[63:0] = rddout_b_0_44[63:0];
assign val_i_x_45_b[63:0] = rddout_b_0_45[63:0];
assign val_i_x_46_b[63:0] = rddout_b_0_46[63:0];
assign val_i_x_47_b[63:0] = rddout_b_0_47[63:0];


du_s #(1) du_2_0 (.clk(clk), .stall(1'b0), .dout(wren_a_0_0),.din(linkdiseq_wren_0_0_a));
du_s #(1) du_2_1 (.clk(clk), .stall(1'b0), .dout(wren_a_0_1),.din(linkdiseq_wren_0_1_a));
du_s #(1) du_2_2 (.clk(clk), .stall(1'b0), .dout(wren_a_0_2),.din(linkdiseq_wren_0_2_a));
du_s #(1) du_2_3 (.clk(clk), .stall(1'b0), .dout(wren_a_0_3),.din(linkdiseq_wren_0_3_a));
du_s #(1) du_2_4 (.clk(clk), .stall(1'b0), .dout(wren_a_0_4),.din(linkdiseq_wren_0_4_a));
du_s #(1) du_2_5 (.clk(clk), .stall(1'b0), .dout(wren_a_0_5),.din(linkdiseq_wren_0_5_a));
du_s #(1) du_2_6 (.clk(clk), .stall(1'b0), .dout(wren_a_0_6),.din(linkdiseq_wren_0_6_a));
du_s #(1) du_2_7 (.clk(clk), .stall(1'b0), .dout(wren_a_0_7),.din(linkdiseq_wren_0_7_a));
du_s #(1) du_2_8 (.clk(clk), .stall(1'b0), .dout(wren_a_0_8),.din(linkdiseq_wren_0_8_a));
du_s #(1) du_2_9 (.clk(clk), .stall(1'b0), .dout(wren_a_0_9),.din(linkdiseq_wren_0_9_a));
du_s #(1) du_2_10 (.clk(clk), .stall(1'b0), .dout(wren_a_0_10),.din(linkdiseq_wren_0_10_a));
du_s #(1) du_2_11 (.clk(clk), .stall(1'b0), .dout(wren_a_0_11),.din(linkdiseq_wren_0_11_a));
du_s #(1) du_2_12 (.clk(clk), .stall(1'b0), .dout(wren_a_0_12),.din(linkdiseq_wren_0_12_a));
du_s #(1) du_2_13 (.clk(clk), .stall(1'b0), .dout(wren_a_0_13),.din(linkdiseq_wren_0_13_a));
du_s #(1) du_2_14 (.clk(clk), .stall(1'b0), .dout(wren_a_0_14),.din(linkdiseq_wren_0_14_a));
du_s #(1) du_2_15 (.clk(clk), .stall(1'b0), .dout(wren_a_0_15),.din(linkdiseq_wren_0_15_a));
du_s #(1) du_2_16 (.clk(clk), .stall(1'b0), .dout(wren_a_0_16),.din(linkdiseq_wren_0_16_a));
du_s #(1) du_2_17 (.clk(clk), .stall(1'b0), .dout(wren_a_0_17),.din(linkdiseq_wren_0_17_a));
du_s #(1) du_2_18 (.clk(clk), .stall(1'b0), .dout(wren_a_0_18),.din(linkdiseq_wren_0_18_a));
du_s #(1) du_2_19 (.clk(clk), .stall(1'b0), .dout(wren_a_0_19),.din(linkdiseq_wren_0_19_a));
du_s #(1) du_2_20 (.clk(clk), .stall(1'b0), .dout(wren_a_0_20),.din(linkdiseq_wren_0_20_a));
du_s #(1) du_2_21 (.clk(clk), .stall(1'b0), .dout(wren_a_0_21),.din(linkdiseq_wren_0_21_a));
du_s #(1) du_2_22 (.clk(clk), .stall(1'b0), .dout(wren_a_0_22),.din(linkdiseq_wren_0_22_a));
du_s #(1) du_2_23 (.clk(clk), .stall(1'b0), .dout(wren_a_0_23),.din(linkdiseq_wren_0_23_a));
du_s #(1) du_2_24 (.clk(clk), .stall(1'b0), .dout(wren_a_0_24),.din(linkdiseq_wren_0_24_a));
du_s #(1) du_2_25 (.clk(clk), .stall(1'b0), .dout(wren_a_0_25),.din(linkdiseq_wren_0_25_a));
du_s #(1) du_2_26 (.clk(clk), .stall(1'b0), .dout(wren_a_0_26),.din(linkdiseq_wren_0_26_a));
du_s #(1) du_2_27 (.clk(clk), .stall(1'b0), .dout(wren_a_0_27),.din(linkdiseq_wren_0_27_a));
du_s #(1) du_2_28 (.clk(clk), .stall(1'b0), .dout(wren_a_0_28),.din(linkdiseq_wren_0_28_a));
du_s #(1) du_2_29 (.clk(clk), .stall(1'b0), .dout(wren_a_0_29),.din(linkdiseq_wren_0_29_a));
du_s #(1) du_2_30 (.clk(clk), .stall(1'b0), .dout(wren_a_0_30),.din(linkdiseq_wren_0_30_a));
du_s #(1) du_2_31 (.clk(clk), .stall(1'b0), .dout(wren_a_0_31),.din(linkdiseq_wren_0_31_a));
du_s #(1) du_2_32 (.clk(clk), .stall(1'b0), .dout(wren_a_0_32),.din(linkdiseq_wren_0_32_a));
du_s #(1) du_2_33 (.clk(clk), .stall(1'b0), .dout(wren_a_0_33),.din(linkdiseq_wren_0_33_a));
du_s #(1) du_2_34 (.clk(clk), .stall(1'b0), .dout(wren_a_0_34),.din(linkdiseq_wren_0_34_a));
du_s #(1) du_2_35 (.clk(clk), .stall(1'b0), .dout(wren_a_0_35),.din(linkdiseq_wren_0_35_a));
du_s #(1) du_2_36 (.clk(clk), .stall(1'b0), .dout(wren_a_0_36),.din(linkdiseq_wren_0_36_a));
du_s #(1) du_2_37 (.clk(clk), .stall(1'b0), .dout(wren_a_0_37),.din(linkdiseq_wren_0_37_a));
du_s #(1) du_2_38 (.clk(clk), .stall(1'b0), .dout(wren_a_0_38),.din(linkdiseq_wren_0_38_a));
du_s #(1) du_2_39 (.clk(clk), .stall(1'b0), .dout(wren_a_0_39),.din(linkdiseq_wren_0_39_a));
du_s #(1) du_2_40 (.clk(clk), .stall(1'b0), .dout(wren_a_0_40),.din(linkdiseq_wren_0_40_a));
du_s #(1) du_2_41 (.clk(clk), .stall(1'b0), .dout(wren_a_0_41),.din(linkdiseq_wren_0_41_a));
du_s #(1) du_2_42 (.clk(clk), .stall(1'b0), .dout(wren_a_0_42),.din(linkdiseq_wren_0_42_a));
du_s #(1) du_2_43 (.clk(clk), .stall(1'b0), .dout(wren_a_0_43),.din(linkdiseq_wren_0_43_a));
du_s #(1) du_2_44 (.clk(clk), .stall(1'b0), .dout(wren_a_0_44),.din(linkdiseq_wren_0_44_a));
du_s #(1) du_2_45 (.clk(clk), .stall(1'b0), .dout(wren_a_0_45),.din(linkdiseq_wren_0_45_a));
du_s #(1) du_2_46 (.clk(clk), .stall(1'b0), .dout(wren_a_0_46),.din(linkdiseq_wren_0_46_a));
du_s #(1) du_2_47 (.clk(clk), .stall(1'b0), .dout(wren_a_0_47),.din(linkdiseq_wren_0_47_a));
du_s #(1) du_3_0 (.clk(clk), .stall(1'b0), .dout(wren_b_0_0),.din(linkdiseq_wren_0_0_b));
du_s #(1) du_3_1 (.clk(clk), .stall(1'b0), .dout(wren_b_0_1),.din(linkdiseq_wren_0_1_b));
du_s #(1) du_3_2 (.clk(clk), .stall(1'b0), .dout(wren_b_0_2),.din(linkdiseq_wren_0_2_b));
du_s #(1) du_3_3 (.clk(clk), .stall(1'b0), .dout(wren_b_0_3),.din(linkdiseq_wren_0_3_b));
du_s #(1) du_3_4 (.clk(clk), .stall(1'b0), .dout(wren_b_0_4),.din(linkdiseq_wren_0_4_b));
du_s #(1) du_3_5 (.clk(clk), .stall(1'b0), .dout(wren_b_0_5),.din(linkdiseq_wren_0_5_b));
du_s #(1) du_3_6 (.clk(clk), .stall(1'b0), .dout(wren_b_0_6),.din(linkdiseq_wren_0_6_b));
du_s #(1) du_3_7 (.clk(clk), .stall(1'b0), .dout(wren_b_0_7),.din(linkdiseq_wren_0_7_b));
du_s #(1) du_3_8 (.clk(clk), .stall(1'b0), .dout(wren_b_0_8),.din(linkdiseq_wren_0_8_b));
du_s #(1) du_3_9 (.clk(clk), .stall(1'b0), .dout(wren_b_0_9),.din(linkdiseq_wren_0_9_b));
du_s #(1) du_3_10 (.clk(clk), .stall(1'b0), .dout(wren_b_0_10),.din(linkdiseq_wren_0_10_b));
du_s #(1) du_3_11 (.clk(clk), .stall(1'b0), .dout(wren_b_0_11),.din(linkdiseq_wren_0_11_b));
du_s #(1) du_3_12 (.clk(clk), .stall(1'b0), .dout(wren_b_0_12),.din(linkdiseq_wren_0_12_b));
du_s #(1) du_3_13 (.clk(clk), .stall(1'b0), .dout(wren_b_0_13),.din(linkdiseq_wren_0_13_b));
du_s #(1) du_3_14 (.clk(clk), .stall(1'b0), .dout(wren_b_0_14),.din(linkdiseq_wren_0_14_b));
du_s #(1) du_3_15 (.clk(clk), .stall(1'b0), .dout(wren_b_0_15),.din(linkdiseq_wren_0_15_b));
du_s #(1) du_3_16 (.clk(clk), .stall(1'b0), .dout(wren_b_0_16),.din(linkdiseq_wren_0_16_b));
du_s #(1) du_3_17 (.clk(clk), .stall(1'b0), .dout(wren_b_0_17),.din(linkdiseq_wren_0_17_b));
du_s #(1) du_3_18 (.clk(clk), .stall(1'b0), .dout(wren_b_0_18),.din(linkdiseq_wren_0_18_b));
du_s #(1) du_3_19 (.clk(clk), .stall(1'b0), .dout(wren_b_0_19),.din(linkdiseq_wren_0_19_b));
du_s #(1) du_3_20 (.clk(clk), .stall(1'b0), .dout(wren_b_0_20),.din(linkdiseq_wren_0_20_b));
du_s #(1) du_3_21 (.clk(clk), .stall(1'b0), .dout(wren_b_0_21),.din(linkdiseq_wren_0_21_b));
du_s #(1) du_3_22 (.clk(clk), .stall(1'b0), .dout(wren_b_0_22),.din(linkdiseq_wren_0_22_b));
du_s #(1) du_3_23 (.clk(clk), .stall(1'b0), .dout(wren_b_0_23),.din(linkdiseq_wren_0_23_b));
du_s #(1) du_3_24 (.clk(clk), .stall(1'b0), .dout(wren_b_0_24),.din(linkdiseq_wren_0_24_b));
du_s #(1) du_3_25 (.clk(clk), .stall(1'b0), .dout(wren_b_0_25),.din(linkdiseq_wren_0_25_b));
du_s #(1) du_3_26 (.clk(clk), .stall(1'b0), .dout(wren_b_0_26),.din(linkdiseq_wren_0_26_b));
du_s #(1) du_3_27 (.clk(clk), .stall(1'b0), .dout(wren_b_0_27),.din(linkdiseq_wren_0_27_b));
du_s #(1) du_3_28 (.clk(clk), .stall(1'b0), .dout(wren_b_0_28),.din(linkdiseq_wren_0_28_b));
du_s #(1) du_3_29 (.clk(clk), .stall(1'b0), .dout(wren_b_0_29),.din(linkdiseq_wren_0_29_b));
du_s #(1) du_3_30 (.clk(clk), .stall(1'b0), .dout(wren_b_0_30),.din(linkdiseq_wren_0_30_b));
du_s #(1) du_3_31 (.clk(clk), .stall(1'b0), .dout(wren_b_0_31),.din(linkdiseq_wren_0_31_b));
du_s #(1) du_3_32 (.clk(clk), .stall(1'b0), .dout(wren_b_0_32),.din(linkdiseq_wren_0_32_b));
du_s #(1) du_3_33 (.clk(clk), .stall(1'b0), .dout(wren_b_0_33),.din(linkdiseq_wren_0_33_b));
du_s #(1) du_3_34 (.clk(clk), .stall(1'b0), .dout(wren_b_0_34),.din(linkdiseq_wren_0_34_b));
du_s #(1) du_3_35 (.clk(clk), .stall(1'b0), .dout(wren_b_0_35),.din(linkdiseq_wren_0_35_b));
du_s #(1) du_3_36 (.clk(clk), .stall(1'b0), .dout(wren_b_0_36),.din(linkdiseq_wren_0_36_b));
du_s #(1) du_3_37 (.clk(clk), .stall(1'b0), .dout(wren_b_0_37),.din(linkdiseq_wren_0_37_b));
du_s #(1) du_3_38 (.clk(clk), .stall(1'b0), .dout(wren_b_0_38),.din(linkdiseq_wren_0_38_b));
du_s #(1) du_3_39 (.clk(clk), .stall(1'b0), .dout(wren_b_0_39),.din(linkdiseq_wren_0_39_b));
du_s #(1) du_3_40 (.clk(clk), .stall(1'b0), .dout(wren_b_0_40),.din(linkdiseq_wren_0_40_b));
du_s #(1) du_3_41 (.clk(clk), .stall(1'b0), .dout(wren_b_0_41),.din(linkdiseq_wren_0_41_b));
du_s #(1) du_3_42 (.clk(clk), .stall(1'b0), .dout(wren_b_0_42),.din(linkdiseq_wren_0_42_b));
du_s #(1) du_3_43 (.clk(clk), .stall(1'b0), .dout(wren_b_0_43),.din(linkdiseq_wren_0_43_b));
du_s #(1) du_3_44 (.clk(clk), .stall(1'b0), .dout(wren_b_0_44),.din(linkdiseq_wren_0_44_b));
du_s #(1) du_3_45 (.clk(clk), .stall(1'b0), .dout(wren_b_0_45),.din(linkdiseq_wren_0_45_b));
du_s #(1) du_3_46 (.clk(clk), .stall(1'b0), .dout(wren_b_0_46),.din(linkdiseq_wren_0_46_b));
du_s #(1) du_3_47 (.clk(clk), .stall(1'b0), .dout(wren_b_0_47),.din(linkdiseq_wren_0_47_b));
du_s #(32) du_4_0 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_0),.din(linkdiseq_0_0_a));
du_s #(32) du_4_1 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_1),.din(linkdiseq_0_1_a));
du_s #(32) du_4_2 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_2),.din(linkdiseq_0_2_a));
du_s #(32) du_4_3 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_3),.din(linkdiseq_0_3_a));
du_s #(32) du_4_4 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_4),.din(linkdiseq_0_4_a));
du_s #(32) du_4_5 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_5),.din(linkdiseq_0_5_a));
du_s #(32) du_4_6 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_6),.din(linkdiseq_0_6_a));
du_s #(32) du_4_7 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_7),.din(linkdiseq_0_7_a));
du_s #(32) du_4_8 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_8),.din(linkdiseq_0_8_a));
du_s #(32) du_4_9 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_9),.din(linkdiseq_0_9_a));
du_s #(32) du_4_10 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_10),.din(linkdiseq_0_10_a));
du_s #(32) du_4_11 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_11),.din(linkdiseq_0_11_a));
du_s #(32) du_4_12 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_12),.din(linkdiseq_0_12_a));
du_s #(32) du_4_13 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_13),.din(linkdiseq_0_13_a));
du_s #(32) du_4_14 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_14),.din(linkdiseq_0_14_a));
du_s #(32) du_4_15 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_15),.din(linkdiseq_0_15_a));
du_s #(32) du_4_16 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_16),.din(linkdiseq_0_16_a));
du_s #(32) du_4_17 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_17),.din(linkdiseq_0_17_a));
du_s #(32) du_4_18 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_18),.din(linkdiseq_0_18_a));
du_s #(32) du_4_19 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_19),.din(linkdiseq_0_19_a));
du_s #(32) du_4_20 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_20),.din(linkdiseq_0_20_a));
du_s #(32) du_4_21 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_21),.din(linkdiseq_0_21_a));
du_s #(32) du_4_22 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_22),.din(linkdiseq_0_22_a));
du_s #(32) du_4_23 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_23),.din(linkdiseq_0_23_a));
du_s #(32) du_4_24 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_24),.din(linkdiseq_0_24_a));
du_s #(32) du_4_25 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_25),.din(linkdiseq_0_25_a));
du_s #(32) du_4_26 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_26),.din(linkdiseq_0_26_a));
du_s #(32) du_4_27 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_27),.din(linkdiseq_0_27_a));
du_s #(32) du_4_28 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_28),.din(linkdiseq_0_28_a));
du_s #(32) du_4_29 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_29),.din(linkdiseq_0_29_a));
du_s #(32) du_4_30 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_30),.din(linkdiseq_0_30_a));
du_s #(32) du_4_31 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_31),.din(linkdiseq_0_31_a));
du_s #(32) du_4_32 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_32),.din(linkdiseq_0_32_a));
du_s #(32) du_4_33 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_33),.din(linkdiseq_0_33_a));
du_s #(32) du_4_34 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_34),.din(linkdiseq_0_34_a));
du_s #(32) du_4_35 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_35),.din(linkdiseq_0_35_a));
du_s #(32) du_4_36 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_36),.din(linkdiseq_0_36_a));
du_s #(32) du_4_37 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_37),.din(linkdiseq_0_37_a));
du_s #(32) du_4_38 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_38),.din(linkdiseq_0_38_a));
du_s #(32) du_4_39 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_39),.din(linkdiseq_0_39_a));
du_s #(32) du_4_40 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_40),.din(linkdiseq_0_40_a));
du_s #(32) du_4_41 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_41),.din(linkdiseq_0_41_a));
du_s #(32) du_4_42 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_42),.din(linkdiseq_0_42_a));
du_s #(32) du_4_43 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_43),.din(linkdiseq_0_43_a));
du_s #(32) du_4_44 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_44),.din(linkdiseq_0_44_a));
du_s #(32) du_4_45 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_45),.din(linkdiseq_0_45_a));
du_s #(32) du_4_46 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_46),.din(linkdiseq_0_46_a));
du_s #(32) du_4_47 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_47),.din(linkdiseq_0_47_a));
du_s #(32) du_5_0 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_0),.din(linkdiseq_0_0_b));
du_s #(32) du_5_1 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_1),.din(linkdiseq_0_1_b));
du_s #(32) du_5_2 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_2),.din(linkdiseq_0_2_b));
du_s #(32) du_5_3 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_3),.din(linkdiseq_0_3_b));
du_s #(32) du_5_4 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_4),.din(linkdiseq_0_4_b));
du_s #(32) du_5_5 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_5),.din(linkdiseq_0_5_b));
du_s #(32) du_5_6 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_6),.din(linkdiseq_0_6_b));
du_s #(32) du_5_7 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_7),.din(linkdiseq_0_7_b));
du_s #(32) du_5_8 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_8),.din(linkdiseq_0_8_b));
du_s #(32) du_5_9 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_9),.din(linkdiseq_0_9_b));
du_s #(32) du_5_10 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_10),.din(linkdiseq_0_10_b));
du_s #(32) du_5_11 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_11),.din(linkdiseq_0_11_b));
du_s #(32) du_5_12 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_12),.din(linkdiseq_0_12_b));
du_s #(32) du_5_13 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_13),.din(linkdiseq_0_13_b));
du_s #(32) du_5_14 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_14),.din(linkdiseq_0_14_b));
du_s #(32) du_5_15 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_15),.din(linkdiseq_0_15_b));
du_s #(32) du_5_16 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_16),.din(linkdiseq_0_16_b));
du_s #(32) du_5_17 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_17),.din(linkdiseq_0_17_b));
du_s #(32) du_5_18 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_18),.din(linkdiseq_0_18_b));
du_s #(32) du_5_19 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_19),.din(linkdiseq_0_19_b));
du_s #(32) du_5_20 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_20),.din(linkdiseq_0_20_b));
du_s #(32) du_5_21 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_21),.din(linkdiseq_0_21_b));
du_s #(32) du_5_22 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_22),.din(linkdiseq_0_22_b));
du_s #(32) du_5_23 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_23),.din(linkdiseq_0_23_b));
du_s #(32) du_5_24 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_24),.din(linkdiseq_0_24_b));
du_s #(32) du_5_25 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_25),.din(linkdiseq_0_25_b));
du_s #(32) du_5_26 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_26),.din(linkdiseq_0_26_b));
du_s #(32) du_5_27 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_27),.din(linkdiseq_0_27_b));
du_s #(32) du_5_28 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_28),.din(linkdiseq_0_28_b));
du_s #(32) du_5_29 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_29),.din(linkdiseq_0_29_b));
du_s #(32) du_5_30 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_30),.din(linkdiseq_0_30_b));
du_s #(32) du_5_31 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_31),.din(linkdiseq_0_31_b));
du_s #(32) du_5_32 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_32),.din(linkdiseq_0_32_b));
du_s #(32) du_5_33 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_33),.din(linkdiseq_0_33_b));
du_s #(32) du_5_34 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_34),.din(linkdiseq_0_34_b));
du_s #(32) du_5_35 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_35),.din(linkdiseq_0_35_b));
du_s #(32) du_5_36 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_36),.din(linkdiseq_0_36_b));
du_s #(32) du_5_37 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_37),.din(linkdiseq_0_37_b));
du_s #(32) du_5_38 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_38),.din(linkdiseq_0_38_b));
du_s #(32) du_5_39 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_39),.din(linkdiseq_0_39_b));
du_s #(32) du_5_40 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_40),.din(linkdiseq_0_40_b));
du_s #(32) du_5_41 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_41),.din(linkdiseq_0_41_b));
du_s #(32) du_5_42 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_42),.din(linkdiseq_0_42_b));
du_s #(32) du_5_43 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_43),.din(linkdiseq_0_43_b));
du_s #(32) du_5_44 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_44),.din(linkdiseq_0_44_b));
du_s #(32) du_5_45 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_45),.din(linkdiseq_0_45_b));
du_s #(32) du_5_46 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_46),.din(linkdiseq_0_46_b));
du_s #(32) du_5_47 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_47),.din(linkdiseq_0_47_b));


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
 .rddout_b_0_7(rddout_b_0_7),
 .rddout_a_0_8(rddout_a_0_8),
 .rddout_b_0_8(rddout_b_0_8),
 .rddout_a_0_9(rddout_a_0_9),
 .rddout_b_0_9(rddout_b_0_9),
 .rddout_a_0_10(rddout_a_0_10),
 .rddout_b_0_10(rddout_b_0_10),
 .rddout_a_0_11(rddout_a_0_11),
 .rddout_b_0_11(rddout_b_0_11),
 .rddout_a_0_12(rddout_a_0_12),
 .rddout_b_0_12(rddout_b_0_12),
 .rddout_a_0_13(rddout_a_0_13),
 .rddout_b_0_13(rddout_b_0_13),
 .rddout_a_0_14(rddout_a_0_14),
 .rddout_b_0_14(rddout_b_0_14),
 .rddout_a_0_15(rddout_a_0_15),
 .rddout_b_0_15(rddout_b_0_15),
 .rddout_a_0_16(rddout_a_0_16),
 .rddout_b_0_16(rddout_b_0_16),
 .rddout_a_0_17(rddout_a_0_17),
 .rddout_b_0_17(rddout_b_0_17),
 .rddout_a_0_18(rddout_a_0_18),
 .rddout_b_0_18(rddout_b_0_18),
 .rddout_a_0_19(rddout_a_0_19),
 .rddout_b_0_19(rddout_b_0_19),
 .rddout_a_0_20(rddout_a_0_20),
 .rddout_b_0_20(rddout_b_0_20),
 .rddout_a_0_21(rddout_a_0_21),
 .rddout_b_0_21(rddout_b_0_21),
 .rddout_a_0_22(rddout_a_0_22),
 .rddout_b_0_22(rddout_b_0_22),
 .rddout_a_0_23(rddout_a_0_23),
 .rddout_b_0_23(rddout_b_0_23),
 .rddout_a_0_24(rddout_a_0_24),
 .rddout_b_0_24(rddout_b_0_24),
 .rddout_a_0_25(rddout_a_0_25),
 .rddout_b_0_25(rddout_b_0_25),
 .rddout_a_0_26(rddout_a_0_26),
 .rddout_b_0_26(rddout_b_0_26),
 .rddout_a_0_27(rddout_a_0_27),
 .rddout_b_0_27(rddout_b_0_27),
 .rddout_a_0_28(rddout_a_0_28),
 .rddout_b_0_28(rddout_b_0_28),
 .rddout_a_0_29(rddout_a_0_29),
 .rddout_b_0_29(rddout_b_0_29),
 .rddout_a_0_30(rddout_a_0_30),
 .rddout_b_0_30(rddout_b_0_30),
 .rddout_a_0_31(rddout_a_0_31),
 .rddout_b_0_31(rddout_b_0_31),
 .rddout_a_0_32(rddout_a_0_32),
 .rddout_b_0_32(rddout_b_0_32),
 .rddout_a_0_33(rddout_a_0_33),
 .rddout_b_0_33(rddout_b_0_33),
 .rddout_a_0_34(rddout_a_0_34),
 .rddout_b_0_34(rddout_b_0_34),
 .rddout_a_0_35(rddout_a_0_35),
 .rddout_b_0_35(rddout_b_0_35),
 .rddout_a_0_36(rddout_a_0_36),
 .rddout_b_0_36(rddout_b_0_36),
 .rddout_a_0_37(rddout_a_0_37),
 .rddout_b_0_37(rddout_b_0_37),
 .rddout_a_0_38(rddout_a_0_38),
 .rddout_b_0_38(rddout_b_0_38),
 .rddout_a_0_39(rddout_a_0_39),
 .rddout_b_0_39(rddout_b_0_39),
 .rddout_a_0_40(rddout_a_0_40),
 .rddout_b_0_40(rddout_b_0_40),
 .rddout_a_0_41(rddout_a_0_41),
 .rddout_b_0_41(rddout_b_0_41),
 .rddout_a_0_42(rddout_a_0_42),
 .rddout_b_0_42(rddout_b_0_42),
 .rddout_a_0_43(rddout_a_0_43),
 .rddout_b_0_43(rddout_b_0_43),
 .rddout_a_0_44(rddout_a_0_44),
 .rddout_b_0_44(rddout_b_0_44),
 .rddout_a_0_45(rddout_a_0_45),
 .rddout_b_0_45(rddout_b_0_45),
 .rddout_a_0_46(rddout_a_0_46),
 .rddout_b_0_46(rddout_b_0_46),
 .rddout_a_0_47(rddout_a_0_47),
 .rddout_b_0_47(rddout_b_0_47)
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
 .val_i_x_8(val_i_x_8_a),
 .val_i_x_9(val_i_x_9_a),
 .val_i_x_10(val_i_x_10_a),
 .val_i_x_11(val_i_x_11_a),
 .val_i_x_12(val_i_x_12_a),
 .val_i_x_13(val_i_x_13_a),
 .val_i_x_14(val_i_x_14_a),
 .val_i_x_15(val_i_x_15_a),
 .val_i_x_16(val_i_x_16_a),
 .val_i_x_17(val_i_x_17_a),
 .val_i_x_18(val_i_x_18_a),
 .val_i_x_19(val_i_x_19_a),
 .val_i_x_20(val_i_x_20_a),
 .val_i_x_21(val_i_x_21_a),
 .val_i_x_22(val_i_x_22_a),
 .val_i_x_23(val_i_x_23_a),
 .val_i_x_24(val_i_x_24_a),
 .val_i_x_25(val_i_x_25_a),
 .val_i_x_26(val_i_x_26_a),
 .val_i_x_27(val_i_x_27_a),
 .val_i_x_28(val_i_x_28_a),
 .val_i_x_29(val_i_x_29_a),
 .val_i_x_30(val_i_x_30_a),
 .val_i_x_31(val_i_x_31_a),
 .val_i_x_32(val_i_x_32_a),
 .val_i_x_33(val_i_x_33_a),
 .val_i_x_34(val_i_x_34_a),
 .val_i_x_35(val_i_x_35_a),
 .val_i_x_36(val_i_x_36_a),
 .val_i_x_37(val_i_x_37_a),
 .val_i_x_38(val_i_x_38_a),
 .val_i_x_39(val_i_x_39_a),
 .val_i_x_40(val_i_x_40_a),
 .val_i_x_41(val_i_x_41_a),
 .val_i_x_42(val_i_x_42_a),
 .val_i_x_43(val_i_x_43_a),
 .val_i_x_44(val_i_x_44_a),
 .val_i_x_45(val_i_x_45_a),
 .val_i_x_46(val_i_x_46_a),
 .val_i_x_47(val_i_x_47_a),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0_a),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1_a),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2_a),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3_a),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4_a),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5_a),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6_a),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7_a),
 .linkdiseq_wren_0_8(linkdiseq_wren_0_8_a),
 .linkdiseq_wren_0_9(linkdiseq_wren_0_9_a),
 .linkdiseq_wren_0_10(linkdiseq_wren_0_10_a),
 .linkdiseq_wren_0_11(linkdiseq_wren_0_11_a),
 .linkdiseq_wren_0_12(linkdiseq_wren_0_12_a),
 .linkdiseq_wren_0_13(linkdiseq_wren_0_13_a),
 .linkdiseq_wren_0_14(linkdiseq_wren_0_14_a),
 .linkdiseq_wren_0_15(linkdiseq_wren_0_15_a),
 .linkdiseq_wren_0_16(linkdiseq_wren_0_16_a),
 .linkdiseq_wren_0_17(linkdiseq_wren_0_17_a),
 .linkdiseq_wren_0_18(linkdiseq_wren_0_18_a),
 .linkdiseq_wren_0_19(linkdiseq_wren_0_19_a),
 .linkdiseq_wren_0_20(linkdiseq_wren_0_20_a),
 .linkdiseq_wren_0_21(linkdiseq_wren_0_21_a),
 .linkdiseq_wren_0_22(linkdiseq_wren_0_22_a),
 .linkdiseq_wren_0_23(linkdiseq_wren_0_23_a),
 .linkdiseq_wren_0_24(linkdiseq_wren_0_24_a),
 .linkdiseq_wren_0_25(linkdiseq_wren_0_25_a),
 .linkdiseq_wren_0_26(linkdiseq_wren_0_26_a),
 .linkdiseq_wren_0_27(linkdiseq_wren_0_27_a),
 .linkdiseq_wren_0_28(linkdiseq_wren_0_28_a),
 .linkdiseq_wren_0_29(linkdiseq_wren_0_29_a),
 .linkdiseq_wren_0_30(linkdiseq_wren_0_30_a),
 .linkdiseq_wren_0_31(linkdiseq_wren_0_31_a),
 .linkdiseq_wren_0_32(linkdiseq_wren_0_32_a),
 .linkdiseq_wren_0_33(linkdiseq_wren_0_33_a),
 .linkdiseq_wren_0_34(linkdiseq_wren_0_34_a),
 .linkdiseq_wren_0_35(linkdiseq_wren_0_35_a),
 .linkdiseq_wren_0_36(linkdiseq_wren_0_36_a),
 .linkdiseq_wren_0_37(linkdiseq_wren_0_37_a),
 .linkdiseq_wren_0_38(linkdiseq_wren_0_38_a),
 .linkdiseq_wren_0_39(linkdiseq_wren_0_39_a),
 .linkdiseq_wren_0_40(linkdiseq_wren_0_40_a),
 .linkdiseq_wren_0_41(linkdiseq_wren_0_41_a),
 .linkdiseq_wren_0_42(linkdiseq_wren_0_42_a),
 .linkdiseq_wren_0_43(linkdiseq_wren_0_43_a),
 .linkdiseq_wren_0_44(linkdiseq_wren_0_44_a),
 .linkdiseq_wren_0_45(linkdiseq_wren_0_45_a),
 .linkdiseq_wren_0_46(linkdiseq_wren_0_46_a),
 .linkdiseq_wren_0_47(linkdiseq_wren_0_47_a),
 .linkdiseq_0_0(linkdiseq_0_0_a),
 .linkdiseq_0_1(linkdiseq_0_1_a),
 .linkdiseq_0_2(linkdiseq_0_2_a),
 .linkdiseq_0_3(linkdiseq_0_3_a),
 .linkdiseq_0_4(linkdiseq_0_4_a),
 .linkdiseq_0_5(linkdiseq_0_5_a),
 .linkdiseq_0_6(linkdiseq_0_6_a),
 .linkdiseq_0_7(linkdiseq_0_7_a),
 .linkdiseq_0_8(linkdiseq_0_8_a),
 .linkdiseq_0_9(linkdiseq_0_9_a),
 .linkdiseq_0_10(linkdiseq_0_10_a),
 .linkdiseq_0_11(linkdiseq_0_11_a),
 .linkdiseq_0_12(linkdiseq_0_12_a),
 .linkdiseq_0_13(linkdiseq_0_13_a),
 .linkdiseq_0_14(linkdiseq_0_14_a),
 .linkdiseq_0_15(linkdiseq_0_15_a),
 .linkdiseq_0_16(linkdiseq_0_16_a),
 .linkdiseq_0_17(linkdiseq_0_17_a),
 .linkdiseq_0_18(linkdiseq_0_18_a),
 .linkdiseq_0_19(linkdiseq_0_19_a),
 .linkdiseq_0_20(linkdiseq_0_20_a),
 .linkdiseq_0_21(linkdiseq_0_21_a),
 .linkdiseq_0_22(linkdiseq_0_22_a),
 .linkdiseq_0_23(linkdiseq_0_23_a),
 .linkdiseq_0_24(linkdiseq_0_24_a),
 .linkdiseq_0_25(linkdiseq_0_25_a),
 .linkdiseq_0_26(linkdiseq_0_26_a),
 .linkdiseq_0_27(linkdiseq_0_27_a),
 .linkdiseq_0_28(linkdiseq_0_28_a),
 .linkdiseq_0_29(linkdiseq_0_29_a),
 .linkdiseq_0_30(linkdiseq_0_30_a),
 .linkdiseq_0_31(linkdiseq_0_31_a),
 .linkdiseq_0_32(linkdiseq_0_32_a),
 .linkdiseq_0_33(linkdiseq_0_33_a),
 .linkdiseq_0_34(linkdiseq_0_34_a),
 .linkdiseq_0_35(linkdiseq_0_35_a),
 .linkdiseq_0_36(linkdiseq_0_36_a),
 .linkdiseq_0_37(linkdiseq_0_37_a),
 .linkdiseq_0_38(linkdiseq_0_38_a),
 .linkdiseq_0_39(linkdiseq_0_39_a),
 .linkdiseq_0_40(linkdiseq_0_40_a),
 .linkdiseq_0_41(linkdiseq_0_41_a),
 .linkdiseq_0_42(linkdiseq_0_42_a),
 .linkdiseq_0_43(linkdiseq_0_43_a),
 .linkdiseq_0_44(linkdiseq_0_44_a),
 .linkdiseq_0_45(linkdiseq_0_45_a),
 .linkdiseq_0_46(linkdiseq_0_46_a),
 .linkdiseq_0_47(linkdiseq_0_47_a)
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
 .val_i_x_8(val_i_x_8_b),
 .val_i_x_9(val_i_x_9_b),
 .val_i_x_10(val_i_x_10_b),
 .val_i_x_11(val_i_x_11_b),
 .val_i_x_12(val_i_x_12_b),
 .val_i_x_13(val_i_x_13_b),
 .val_i_x_14(val_i_x_14_b),
 .val_i_x_15(val_i_x_15_b),
 .val_i_x_16(val_i_x_16_b),
 .val_i_x_17(val_i_x_17_b),
 .val_i_x_18(val_i_x_18_b),
 .val_i_x_19(val_i_x_19_b),
 .val_i_x_20(val_i_x_20_b),
 .val_i_x_21(val_i_x_21_b),
 .val_i_x_22(val_i_x_22_b),
 .val_i_x_23(val_i_x_23_b),
 .val_i_x_24(val_i_x_24_b),
 .val_i_x_25(val_i_x_25_b),
 .val_i_x_26(val_i_x_26_b),
 .val_i_x_27(val_i_x_27_b),
 .val_i_x_28(val_i_x_28_b),
 .val_i_x_29(val_i_x_29_b),
 .val_i_x_30(val_i_x_30_b),
 .val_i_x_31(val_i_x_31_b),
 .val_i_x_32(val_i_x_32_b),
 .val_i_x_33(val_i_x_33_b),
 .val_i_x_34(val_i_x_34_b),
 .val_i_x_35(val_i_x_35_b),
 .val_i_x_36(val_i_x_36_b),
 .val_i_x_37(val_i_x_37_b),
 .val_i_x_38(val_i_x_38_b),
 .val_i_x_39(val_i_x_39_b),
 .val_i_x_40(val_i_x_40_b),
 .val_i_x_41(val_i_x_41_b),
 .val_i_x_42(val_i_x_42_b),
 .val_i_x_43(val_i_x_43_b),
 .val_i_x_44(val_i_x_44_b),
 .val_i_x_45(val_i_x_45_b),
 .val_i_x_46(val_i_x_46_b),
 .val_i_x_47(val_i_x_47_b),
 .linkdiseq_wren_0_0(linkdiseq_wren_0_0_b),
 .linkdiseq_wren_0_1(linkdiseq_wren_0_1_b),
 .linkdiseq_wren_0_2(linkdiseq_wren_0_2_b),
 .linkdiseq_wren_0_3(linkdiseq_wren_0_3_b),
 .linkdiseq_wren_0_4(linkdiseq_wren_0_4_b),
 .linkdiseq_wren_0_5(linkdiseq_wren_0_5_b),
 .linkdiseq_wren_0_6(linkdiseq_wren_0_6_b),
 .linkdiseq_wren_0_7(linkdiseq_wren_0_7_b),
 .linkdiseq_wren_0_8(linkdiseq_wren_0_8_b),
 .linkdiseq_wren_0_9(linkdiseq_wren_0_9_b),
 .linkdiseq_wren_0_10(linkdiseq_wren_0_10_b),
 .linkdiseq_wren_0_11(linkdiseq_wren_0_11_b),
 .linkdiseq_wren_0_12(linkdiseq_wren_0_12_b),
 .linkdiseq_wren_0_13(linkdiseq_wren_0_13_b),
 .linkdiseq_wren_0_14(linkdiseq_wren_0_14_b),
 .linkdiseq_wren_0_15(linkdiseq_wren_0_15_b),
 .linkdiseq_wren_0_16(linkdiseq_wren_0_16_b),
 .linkdiseq_wren_0_17(linkdiseq_wren_0_17_b),
 .linkdiseq_wren_0_18(linkdiseq_wren_0_18_b),
 .linkdiseq_wren_0_19(linkdiseq_wren_0_19_b),
 .linkdiseq_wren_0_20(linkdiseq_wren_0_20_b),
 .linkdiseq_wren_0_21(linkdiseq_wren_0_21_b),
 .linkdiseq_wren_0_22(linkdiseq_wren_0_22_b),
 .linkdiseq_wren_0_23(linkdiseq_wren_0_23_b),
 .linkdiseq_wren_0_24(linkdiseq_wren_0_24_b),
 .linkdiseq_wren_0_25(linkdiseq_wren_0_25_b),
 .linkdiseq_wren_0_26(linkdiseq_wren_0_26_b),
 .linkdiseq_wren_0_27(linkdiseq_wren_0_27_b),
 .linkdiseq_wren_0_28(linkdiseq_wren_0_28_b),
 .linkdiseq_wren_0_29(linkdiseq_wren_0_29_b),
 .linkdiseq_wren_0_30(linkdiseq_wren_0_30_b),
 .linkdiseq_wren_0_31(linkdiseq_wren_0_31_b),
 .linkdiseq_wren_0_32(linkdiseq_wren_0_32_b),
 .linkdiseq_wren_0_33(linkdiseq_wren_0_33_b),
 .linkdiseq_wren_0_34(linkdiseq_wren_0_34_b),
 .linkdiseq_wren_0_35(linkdiseq_wren_0_35_b),
 .linkdiseq_wren_0_36(linkdiseq_wren_0_36_b),
 .linkdiseq_wren_0_37(linkdiseq_wren_0_37_b),
 .linkdiseq_wren_0_38(linkdiseq_wren_0_38_b),
 .linkdiseq_wren_0_39(linkdiseq_wren_0_39_b),
 .linkdiseq_wren_0_40(linkdiseq_wren_0_40_b),
 .linkdiseq_wren_0_41(linkdiseq_wren_0_41_b),
 .linkdiseq_wren_0_42(linkdiseq_wren_0_42_b),
 .linkdiseq_wren_0_43(linkdiseq_wren_0_43_b),
 .linkdiseq_wren_0_44(linkdiseq_wren_0_44_b),
 .linkdiseq_wren_0_45(linkdiseq_wren_0_45_b),
 .linkdiseq_wren_0_46(linkdiseq_wren_0_46_b),
 .linkdiseq_wren_0_47(linkdiseq_wren_0_47_b),
 .linkdiseq_0_0(linkdiseq_0_0_b),
 .linkdiseq_0_1(linkdiseq_0_1_b),
 .linkdiseq_0_2(linkdiseq_0_2_b),
 .linkdiseq_0_3(linkdiseq_0_3_b),
 .linkdiseq_0_4(linkdiseq_0_4_b),
 .linkdiseq_0_5(linkdiseq_0_5_b),
 .linkdiseq_0_6(linkdiseq_0_6_b),
 .linkdiseq_0_7(linkdiseq_0_7_b),
 .linkdiseq_0_8(linkdiseq_0_8_b),
 .linkdiseq_0_9(linkdiseq_0_9_b),
 .linkdiseq_0_10(linkdiseq_0_10_b),
 .linkdiseq_0_11(linkdiseq_0_11_b),
 .linkdiseq_0_12(linkdiseq_0_12_b),
 .linkdiseq_0_13(linkdiseq_0_13_b),
 .linkdiseq_0_14(linkdiseq_0_14_b),
 .linkdiseq_0_15(linkdiseq_0_15_b),
 .linkdiseq_0_16(linkdiseq_0_16_b),
 .linkdiseq_0_17(linkdiseq_0_17_b),
 .linkdiseq_0_18(linkdiseq_0_18_b),
 .linkdiseq_0_19(linkdiseq_0_19_b),
 .linkdiseq_0_20(linkdiseq_0_20_b),
 .linkdiseq_0_21(linkdiseq_0_21_b),
 .linkdiseq_0_22(linkdiseq_0_22_b),
 .linkdiseq_0_23(linkdiseq_0_23_b),
 .linkdiseq_0_24(linkdiseq_0_24_b),
 .linkdiseq_0_25(linkdiseq_0_25_b),
 .linkdiseq_0_26(linkdiseq_0_26_b),
 .linkdiseq_0_27(linkdiseq_0_27_b),
 .linkdiseq_0_28(linkdiseq_0_28_b),
 .linkdiseq_0_29(linkdiseq_0_29_b),
 .linkdiseq_0_30(linkdiseq_0_30_b),
 .linkdiseq_0_31(linkdiseq_0_31_b),
 .linkdiseq_0_32(linkdiseq_0_32_b),
 .linkdiseq_0_33(linkdiseq_0_33_b),
 .linkdiseq_0_34(linkdiseq_0_34_b),
 .linkdiseq_0_35(linkdiseq_0_35_b),
 .linkdiseq_0_36(linkdiseq_0_36_b),
 .linkdiseq_0_37(linkdiseq_0_37_b),
 .linkdiseq_0_38(linkdiseq_0_38_b),
 .linkdiseq_0_39(linkdiseq_0_39_b),
 .linkdiseq_0_40(linkdiseq_0_40_b),
 .linkdiseq_0_41(linkdiseq_0_41_b),
 .linkdiseq_0_42(linkdiseq_0_42_b),
 .linkdiseq_0_43(linkdiseq_0_43_b),
 .linkdiseq_0_44(linkdiseq_0_44_b),
 .linkdiseq_0_45(linkdiseq_0_45_b),
 .linkdiseq_0_46(linkdiseq_0_46_b),
 .linkdiseq_0_47(linkdiseq_0_47_b)
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
 .wren_a_0_8(wren_a_0_8),
 .wren_b_0_8(wren_b_0_8),
 .wren_a_0_9(wren_a_0_9),
 .wren_b_0_9(wren_b_0_9),
 .wren_a_0_10(wren_a_0_10),
 .wren_b_0_10(wren_b_0_10),
 .wren_a_0_11(wren_a_0_11),
 .wren_b_0_11(wren_b_0_11),
 .wren_a_0_12(wren_a_0_12),
 .wren_b_0_12(wren_b_0_12),
 .wren_a_0_13(wren_a_0_13),
 .wren_b_0_13(wren_b_0_13),
 .wren_a_0_14(wren_a_0_14),
 .wren_b_0_14(wren_b_0_14),
 .wren_a_0_15(wren_a_0_15),
 .wren_b_0_15(wren_b_0_15),
 .wren_a_0_16(wren_a_0_16),
 .wren_b_0_16(wren_b_0_16),
 .wren_a_0_17(wren_a_0_17),
 .wren_b_0_17(wren_b_0_17),
 .wren_a_0_18(wren_a_0_18),
 .wren_b_0_18(wren_b_0_18),
 .wren_a_0_19(wren_a_0_19),
 .wren_b_0_19(wren_b_0_19),
 .wren_a_0_20(wren_a_0_20),
 .wren_b_0_20(wren_b_0_20),
 .wren_a_0_21(wren_a_0_21),
 .wren_b_0_21(wren_b_0_21),
 .wren_a_0_22(wren_a_0_22),
 .wren_b_0_22(wren_b_0_22),
 .wren_a_0_23(wren_a_0_23),
 .wren_b_0_23(wren_b_0_23),
 .wren_a_0_24(wren_a_0_24),
 .wren_b_0_24(wren_b_0_24),
 .wren_a_0_25(wren_a_0_25),
 .wren_b_0_25(wren_b_0_25),
 .wren_a_0_26(wren_a_0_26),
 .wren_b_0_26(wren_b_0_26),
 .wren_a_0_27(wren_a_0_27),
 .wren_b_0_27(wren_b_0_27),
 .wren_a_0_28(wren_a_0_28),
 .wren_b_0_28(wren_b_0_28),
 .wren_a_0_29(wren_a_0_29),
 .wren_b_0_29(wren_b_0_29),
 .wren_a_0_30(wren_a_0_30),
 .wren_b_0_30(wren_b_0_30),
 .wren_a_0_31(wren_a_0_31),
 .wren_b_0_31(wren_b_0_31),
 .wren_a_0_32(wren_a_0_32),
 .wren_b_0_32(wren_b_0_32),
 .wren_a_0_33(wren_a_0_33),
 .wren_b_0_33(wren_b_0_33),
 .wren_a_0_34(wren_a_0_34),
 .wren_b_0_34(wren_b_0_34),
 .wren_a_0_35(wren_a_0_35),
 .wren_b_0_35(wren_b_0_35),
 .wren_a_0_36(wren_a_0_36),
 .wren_b_0_36(wren_b_0_36),
 .wren_a_0_37(wren_a_0_37),
 .wren_b_0_37(wren_b_0_37),
 .wren_a_0_38(wren_a_0_38),
 .wren_b_0_38(wren_b_0_38),
 .wren_a_0_39(wren_a_0_39),
 .wren_b_0_39(wren_b_0_39),
 .wren_a_0_40(wren_a_0_40),
 .wren_b_0_40(wren_b_0_40),
 .wren_a_0_41(wren_a_0_41),
 .wren_b_0_41(wren_b_0_41),
 .wren_a_0_42(wren_a_0_42),
 .wren_b_0_42(wren_b_0_42),
 .wren_a_0_43(wren_a_0_43),
 .wren_b_0_43(wren_b_0_43),
 .wren_a_0_44(wren_a_0_44),
 .wren_b_0_44(wren_b_0_44),
 .wren_a_0_45(wren_a_0_45),
 .wren_b_0_45(wren_b_0_45),
 .wren_a_0_46(wren_a_0_46),
 .wren_b_0_46(wren_b_0_46),
 .wren_a_0_47(wren_a_0_47),
 .wren_b_0_47(wren_b_0_47),
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
 .wrdin_a_0_8(wrdin_a_0_8),
 .wrdin_b_0_8(wrdin_b_0_8),
 .wrdin_a_0_9(wrdin_a_0_9),
 .wrdin_b_0_9(wrdin_b_0_9),
 .wrdin_a_0_10(wrdin_a_0_10),
 .wrdin_b_0_10(wrdin_b_0_10),
 .wrdin_a_0_11(wrdin_a_0_11),
 .wrdin_b_0_11(wrdin_b_0_11),
 .wrdin_a_0_12(wrdin_a_0_12),
 .wrdin_b_0_12(wrdin_b_0_12),
 .wrdin_a_0_13(wrdin_a_0_13),
 .wrdin_b_0_13(wrdin_b_0_13),
 .wrdin_a_0_14(wrdin_a_0_14),
 .wrdin_b_0_14(wrdin_b_0_14),
 .wrdin_a_0_15(wrdin_a_0_15),
 .wrdin_b_0_15(wrdin_b_0_15),
 .wrdin_a_0_16(wrdin_a_0_16),
 .wrdin_b_0_16(wrdin_b_0_16),
 .wrdin_a_0_17(wrdin_a_0_17),
 .wrdin_b_0_17(wrdin_b_0_17),
 .wrdin_a_0_18(wrdin_a_0_18),
 .wrdin_b_0_18(wrdin_b_0_18),
 .wrdin_a_0_19(wrdin_a_0_19),
 .wrdin_b_0_19(wrdin_b_0_19),
 .wrdin_a_0_20(wrdin_a_0_20),
 .wrdin_b_0_20(wrdin_b_0_20),
 .wrdin_a_0_21(wrdin_a_0_21),
 .wrdin_b_0_21(wrdin_b_0_21),
 .wrdin_a_0_22(wrdin_a_0_22),
 .wrdin_b_0_22(wrdin_b_0_22),
 .wrdin_a_0_23(wrdin_a_0_23),
 .wrdin_b_0_23(wrdin_b_0_23),
 .wrdin_a_0_24(wrdin_a_0_24),
 .wrdin_b_0_24(wrdin_b_0_24),
 .wrdin_a_0_25(wrdin_a_0_25),
 .wrdin_b_0_25(wrdin_b_0_25),
 .wrdin_a_0_26(wrdin_a_0_26),
 .wrdin_b_0_26(wrdin_b_0_26),
 .wrdin_a_0_27(wrdin_a_0_27),
 .wrdin_b_0_27(wrdin_b_0_27),
 .wrdin_a_0_28(wrdin_a_0_28),
 .wrdin_b_0_28(wrdin_b_0_28),
 .wrdin_a_0_29(wrdin_a_0_29),
 .wrdin_b_0_29(wrdin_b_0_29),
 .wrdin_a_0_30(wrdin_a_0_30),
 .wrdin_b_0_30(wrdin_b_0_30),
 .wrdin_a_0_31(wrdin_a_0_31),
 .wrdin_b_0_31(wrdin_b_0_31),
 .wrdin_a_0_32(wrdin_a_0_32),
 .wrdin_b_0_32(wrdin_b_0_32),
 .wrdin_a_0_33(wrdin_a_0_33),
 .wrdin_b_0_33(wrdin_b_0_33),
 .wrdin_a_0_34(wrdin_a_0_34),
 .wrdin_b_0_34(wrdin_b_0_34),
 .wrdin_a_0_35(wrdin_a_0_35),
 .wrdin_b_0_35(wrdin_b_0_35),
 .wrdin_a_0_36(wrdin_a_0_36),
 .wrdin_b_0_36(wrdin_b_0_36),
 .wrdin_a_0_37(wrdin_a_0_37),
 .wrdin_b_0_37(wrdin_b_0_37),
 .wrdin_a_0_38(wrdin_a_0_38),
 .wrdin_b_0_38(wrdin_b_0_38),
 .wrdin_a_0_39(wrdin_a_0_39),
 .wrdin_b_0_39(wrdin_b_0_39),
 .wrdin_a_0_40(wrdin_a_0_40),
 .wrdin_b_0_40(wrdin_b_0_40),
 .wrdin_a_0_41(wrdin_a_0_41),
 .wrdin_b_0_41(wrdin_b_0_41),
 .wrdin_a_0_42(wrdin_a_0_42),
 .wrdin_b_0_42(wrdin_b_0_42),
 .wrdin_a_0_43(wrdin_a_0_43),
 .wrdin_b_0_43(wrdin_b_0_43),
 .wrdin_a_0_44(wrdin_a_0_44),
 .wrdin_b_0_44(wrdin_b_0_44),
 .wrdin_a_0_45(wrdin_a_0_45),
 .wrdin_b_0_45(wrdin_b_0_45),
 .wrdin_a_0_46(wrdin_a_0_46),
 .wrdin_b_0_46(wrdin_b_0_46),
 .wrdin_a_0_47(wrdin_a_0_47),
 .wrdin_b_0_47(wrdin_b_0_47),
 .rden_q(rden_q),
 .rdaddr_q(rdaddr_q),
 .rdid_q(rdid_q),
 .rddout_q_a_0(rddout_q_a_0),
 .rddout_q_b_0(rddout_q_b_0)
);


endmodule
