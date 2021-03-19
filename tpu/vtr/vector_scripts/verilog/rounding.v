
///////////////////////////////////////////////////////
//Rounding logic based on convergent rounding described 
//here: https://zipcpu.com/dsp/2017/07/22/rounding.html
///////////////////////////////////////////////////////
module rounding_13_10( i_data, o_data );
input  [13-1:0] i_data;
output [10-1:0] o_data;

wire [13-1:0] w_convergent;

assign	w_convergent = i_data[(13-1):0]
			+ { {(10){1'b0}},
				i_data[(13-10)],
                                {(13-10-1){!i_data[(13-10)]}}};

assign o_data = w_convergent[(13-1):(13-10)];

endmodule
        