///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element_8_11.0 (
  A, B, OUT, MODE
);

input [8-1:0] A;
input [8-1:0] B;
output [11.0-1:0] OUT;
input [1:0] MODE;

wire [11.0-1:0] greater;
wire [11.0-1:0] smaller;
wire [11.0-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule
///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element_11.0_11.0 (
  A, B, OUT, MODE
);

input [11.0-1:0] A;
input [11.0-1:0] B;
output [11.0-1:0] OUT;
input [1:0] MODE;

wire [11.0-1:0] greater;
wire [11.0-1:0] smaller;
wire [11.0-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule
