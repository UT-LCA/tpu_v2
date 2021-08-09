///////////////////////////////////////////////////////////////////
//Module to reduce multiple values (add, max, min) into one final result.
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Numerics. We use fixed point format:
//  Most significant 8 bits represent integer part and Least significant 8 bits
//  represent fraction part
//  i.e. IIIIIIIIFFFFFFFF = IIIIIIII.FFFFFFFF
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// There are 32 inputs to the reduction unit. We use a tree structure to reduce the 32 values.
// It is assumed that the number of addressses supplied (end_addr - start_addr + 1) is a multiple
// of 32. If the real application needs to reduce a number of values that are not a multiple of
// 32, then the application must pad the values in the input BRAM appropriately
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// A user is expected to use the resetn signal everytime before starting a new reduction operation.
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Accumulation is done in 20 bits (16 + log(16))
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Each entry of the RAM contains `NUM_INPUTS (which is 32) values. So,
// 1 read of the RAM provides all the inputs required for going through
// the reduction unit once. 
//////////////////////////////////////////////////////////////////


////////////////////////////////////////////
// Top module
////////////////////////////////////////////
module reduction_layer#(
 parameter DWIDTH = 16,
 parameter LOGDWIDTH = $clog2(DWIDTH)
)(
  input clk,
  input resetn, //resets the processing elements
  input en, //indicates valid reduction operation
  input  [1:0] reduction_type, //can have 3 values: 0 (Add), 1 (Max), 2 (Min)
  //input read,
  input  [8 * DWIDTH -1:0] a, // input data to reduction logic
  output [8 * DWIDTH -1:0] reduced_out, //output
  output reg done, //output is valid when done is 1
  output reg busy,
  output reg valid
);

wire [DWIDTH + LOGDWIDTH-1:0] reduced_out_unrounded;

reduction_unit ucu(
  .clk(clk),
  .resetn(resetn),
  .en(en),
  .inp0(a[1*DWIDTH-1:0*DWIDTH]), 
  .inp1(a[2*DWIDTH-1:1*DWIDTH]), 
  .inp2(a[3*DWIDTH-1:2*DWIDTH]), 
  .inp3(a[4*DWIDTH-1:3*DWIDTH]), 
  .inp4(a[5*DWIDTH-1:4*DWIDTH]), 
  .inp5(a[6*DWIDTH-1:5*DWIDTH]), 
  .inp6(a[7*DWIDTH-1:6*DWIDTH]), 
  .inp7(a[8*DWIDTH-1:7*DWIDTH]), 
  .mode(reduction_type),
  .outp(reduced_out_unrounded)
);
defparam
    ucu.DWIDTH = DWIDTH,
    ucu.LOGDWIDTH = LOGDWIDTH;

////////////////////////////////////////////////////////////////
// Rounding of the output of reduction unit (from 20 bits to 16 bits).
// This is required only when reduction type is "sum"
////////////////////////////////////////////////////////////////
rounding #(DWIDTH+LOGDWIDTH, DWIDTH) u_round(.i_data(reduced_out_unrounded), .o_data(reduced_out_add));

assign reduced_out_1 = (reduction_type==2'b0) ? reduced_out_add : reduced_out_unrounded[DWIDTH-1:0];
//assign reduced_out = {8{reduced_out_1}};
assign reduced_out = {8{reduced_out_unrounded[DWIDTH-1:0]}};

reg[2:0] count;

always@(posedge clk)begin
  if(!resetn)begin
    count <= 3'h0;
    valid <= 1'b0;
  end
  else begin
    if(en)
        count <= 3'h4;
    if( ~en & (count != 3'h0))
        count <= count - 1;
    if(~en & (count == 3'h2))
        valid <= 1'b1;
    else
        valid <= 1'b0;
  end
end

always@(*)begin
  if(count != 0)begin
    busy = 1'b1;
  end
  else begin
    busy = 1'b0;
  end
end

always@(posedge clk)begin
  if(!resetn)begin
    done <= 1'b0;
  end
  else begin
    if(count == 8)
      done <= 1'b1;
    else if(count ==0)
      done <= 1'b0;
  end
end

endmodule


///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element #(
  parameter IN_DWIDTH = 16,
  parameter OUT_DWIDTH = 4
)(
  A, B, OUT, MODE
);

input [IN_DWIDTH-1:0] A;
input [IN_DWIDTH-1:0] B;
output [OUT_DWIDTH-1:0] OUT;
input [1:0] MODE;

wire [OUT_DWIDTH-1:0] greater;
wire [OUT_DWIDTH-1:0] smaller;
wire [OUT_DWIDTH-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule


///////////////////////////////////////////////////////
// Reduction unit. It's a tree of processing elements.
// There are 32 inputs and one output and 6 stages. 
//
// The output is
// wider (more bits) than the inputs. It has logN more
// bits (if N is the number of bits in the inputs). This
// is based on https://zipcpu.com/dsp/2017/07/22/rounding.html.
// 
// The last stage is special. It adds the previous 
// result. This is useful when we have more than 32 inputs
// to reduce. We send the next set of 32 inputs in the next
// clock after the first set. 
// 
// Each stage of the tree is pipelined.
///////////////////////////////////////////////////////
module reduction_unit(
  clk,
  resetn,
  en,
  inp0, 
  inp1, 
  inp2, 
  inp3, 
  inp4, 
  inp5, 
  inp6, 
  inp7, 

  mode,
  outp
);
  parameter DWIDTH = 16;
  parameter LOGDWIDTH = 4;

  input clk;
  input resetn;
  input en;
  input  [DWIDTH-1 : 0] inp0; 
  input  [DWIDTH-1 : 0] inp1; 
  input  [DWIDTH-1 : 0] inp2; 
  input  [DWIDTH-1 : 0] inp3; 
  input  [DWIDTH-1 : 0] inp4; 
  input  [DWIDTH-1 : 0] inp5; 
  input  [DWIDTH-1 : 0] inp6; 
  input  [DWIDTH-1 : 0] inp7; 

  input [1:0] mode;
  output [DWIDTH+LOGDWIDTH-1 : 0] outp;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage3_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage3_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute2_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute2_out_stage3_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute3_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute3_out_stage3_reg;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage2;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage2_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage2;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage2_reg;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage1;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage1_reg;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage0;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] outp;

  always @(posedge clk) begin
    if (~resetn) begin
      outp <= 0;
      compute0_out_stage3_reg <= 0;
      compute1_out_stage3_reg <= 0;
      compute2_out_stage3_reg <= 0;
      compute3_out_stage3_reg <= 0;
      compute0_out_stage2_reg <= 0;
      compute1_out_stage2_reg <= 0;
      compute0_out_stage1_reg <= 0;
    end

    else begin
      if(en)begin
        compute0_out_stage3_reg <= compute0_out_stage3;
        compute1_out_stage3_reg <= compute1_out_stage3;
        compute2_out_stage3_reg <= compute2_out_stage3;
        compute3_out_stage3_reg <= compute3_out_stage3;
      end
      else begin
        compute0_out_stage3_reg <= 0;
        compute1_out_stage3_reg <= 0;
        compute2_out_stage3_reg <= 0;
        compute3_out_stage3_reg <= 0;
      end

      compute0_out_stage2_reg <= compute0_out_stage2;
      compute1_out_stage2_reg <= compute1_out_stage2;

      compute0_out_stage1_reg <= compute0_out_stage1;

      outp <= compute0_out_stage0;
    end
  end
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage3(.A(inp0),.B(inp1),.OUT(compute0_out_stage3), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute1_stage3(.A(inp2),.B(inp3),.OUT(compute1_out_stage3), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute2_stage3(.A(inp4),.B(inp5),.OUT(compute2_out_stage3), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute3_stage3(.A(inp6),.B(inp7),.OUT(compute3_out_stage3), .MODE(mode));

  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage2(.A(compute0_out_stage3_reg),.B(compute1_out_stage3_reg),.OUT(compute0_out_stage2), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute1_stage2(.A(compute2_out_stage3_reg),.B(compute3_out_stage3_reg),.OUT(compute1_out_stage2), .MODE(mode));

  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage1(.A(compute0_out_stage2_reg),.B(compute1_out_stage2_reg),.OUT(compute0_out_stage1), .MODE(mode));

  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage0(.A(outp),       .B(compute0_out_stage1_reg),     .OUT(compute0_out_stage0), .MODE(mode));

endmodule
///////////////////////////////////////////////////////
//Rounding logic based on convergent rounding described 
//here: https://zipcpu.com/dsp/2017/07/22/rounding.html
///////////////////////////////////////////////////////
module rounding( i_data, o_data );
parameter IWID = 32;
parameter OWID = 16;
input  [IWID-1:0] i_data;
output [OWID-1:0] o_data;

wire [IWID-1:0] w_convergent;

assign	w_convergent = i_data[(IWID-1):0]
			+ { {(OWID){1'b0}},
				i_data[(IWID-OWID)],
				{(IWID-OWID-1){!i_data[(IWID-OWID)]}}};

assign o_data = w_convergent[(IWID-1):(IWID-OWID)];

endmodule
