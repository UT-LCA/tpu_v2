
///////////////////////////////////////////////////////
//Rounding logic based on convergent rounding described 
//here: https://zipcpu.com/dsp/2017/07/22/rounding.html
///////////////////////////////////////////////////////
module rounding_11.0_8( i_data, o_data );
parameter 11.0 = 32;
parameter 8 = 16;
input  [11.0-1:0] i_data;
output [8-1:0] o_data;

wire [11.0-1:0] w_convergent;

assign	w_convergent = i_data[(11.0-1):0]
			+ { {(8){1'b0}},
				i_data[(11.0-8)],
                                {(11.0-8-1){!i_data[(11.0-8)]}}};

assign o_data = w_convergent[(11.0-1):(11.0-8)];

endmodule


endmodule
        