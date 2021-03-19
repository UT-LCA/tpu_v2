///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element_10_13 (
  A, B, OUT, MODE
);

input [10-1:0] A;
input [10-1:0] B;
output [13-1:0] OUT;
input [1:0] MODE;

wire [13-1:0] greater;
wire [13-1:0] smaller;
wire [13-1:0] sum;

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
module reduction_processing_element_13_13 (
  A, B, OUT, MODE
);

input [13-1:0] A;
input [13-1:0] B;
output [13-1:0] OUT;
input [1:0] MODE;

wire [13-1:0] greater;
wire [13-1:0] smaller;
wire [13-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule
