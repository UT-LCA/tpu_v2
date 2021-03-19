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
module reduction_unit_10_3(
  clk,
  reset,

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

  input clk;
  input reset;
  input  [10-1 : 0] inp0; 
  input  [10-1 : 0] inp1; 
  input  [10-1 : 0] inp2; 
  input  [10-1 : 0] inp3; 
  input  [10-1 : 0] inp4; 
  input  [10-1 : 0] inp5; 
  input  [10-1 : 0] inp6; 
  input  [10-1 : 0] inp7; 

  input [1:0] mode;
  output [10+3-1 : 0] outp;

  wire   [10+3-1 : 0] compute0_out_stage4;
  reg    [10+3-1 : 0] compute0_out_stage4_reg;
  wire   [10+3-1 : 0] compute1_out_stage4;
  reg    [10+3-1 : 0] compute1_out_stage4_reg;
  wire   [10+3-1 : 0] compute2_out_stage4;
  reg    [10+3-1 : 0] compute2_out_stage4_reg;
  wire   [10+3-1 : 0] compute3_out_stage4;
  wire   [10+3-1 : 0] compute3_out_stage4_reg;
  wire   [10+3-1 : 0] compute4_out_stage4;
  reg    [10+3-1 : 0] compute4_out_stage4_reg;
  wire   [10+3-1 : 0] compute5_out_stage4;
  reg    [10+3-1 : 0] compute5_out_stage4_reg;
  wire   [10+3-1 : 0] compute6_out_stage4;
  reg    [10+3-1 : 0] compute6_out_stage4_reg;
  wire   [10+3-1 : 0] compute7_out_stage4;
  wire   [10+3-1 : 0] compute7_out_stage4_reg;

  wire   [10+3-1 : 0] compute0_out_stage3;
  reg    [10+3-1 : 0] compute0_out_stage3_reg;
  wire   [10+3-1 : 0] compute1_out_stage3;
  reg    [10+3-1 : 0] compute1_out_stage3_reg;
  wire   [10+3-1 : 0] compute2_out_stage3;
  reg    [10+3-1 : 0] compute2_out_stage3_reg;
  wire   [10+3-1 : 0] compute3_out_stage3;
  wire   [10+3-1 : 0] compute3_out_stage3_reg;

  wire   [10+3-1 : 0] compute0_out_stage2;
  reg    [10+3-1 : 0] compute0_out_stage2_reg;
  wire   [10+3-1 : 0] compute1_out_stage2;
  reg    [10+3-1 : 0] compute1_out_stage2_reg;

  wire   [10+3-1 : 0] compute0_out_stage1;
  reg    [10+3-1 : 0] compute0_out_stage1_reg;

  wire   [10+3-1 : 0] compute0_out_stage0;
  reg    [10+3-1 : 0] outp;

  always @(posedge clk) begin
    if (reset) begin
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
      compute0_out_stage3_reg <= compute0_out_stage3;
      compute1_out_stage3_reg <= compute1_out_stage3;
      compute2_out_stage3_reg <= compute2_out_stage3;
      compute3_out_stage3_reg <= compute3_out_stage3;

      compute0_out_stage2_reg <= compute0_out_stage2;
      compute1_out_stage2_reg <= compute1_out_stage2;

      compute0_out_stage1_reg <= compute0_out_stage1;

      outp <= compute0_out_stage0;
    end
  end
  reduction_processing_element_10_13 compute0_stage3(.A(compute0_out_stage4_reg),       .B(compute1_out_stage4_reg),    .OUT(compute0_out_stage3), .MODE(mode));
  reduction_processing_element_10_13 compute1_stage3(.A(compute2_out_stage4_reg),       .B(compute3_out_stage4_reg),    .OUT(compute1_out_stage3), .MODE(mode));
  reduction_processing_element_10_13 compute2_stage3(.A(compute4_out_stage4_reg),       .B(compute5_out_stage4_reg),    .OUT(compute2_out_stage3), .MODE(mode));
  reduction_processing_element_10_13 compute3_stage3(.A(compute6_out_stage4_reg),       .B(compute7_out_stage4_reg),    .OUT(compute3_out_stage3), .MODE(mode));

  reduction_processing_element_13_13 compute0_stage2(.A(compute0_out_stage3_reg),       .B(compute1_out_stage3_reg),    .OUT(compute0_out_stage2), .MODE(mode));
  reduction_processing_element_13_13 compute1_stage2(.A(compute2_out_stage3_reg),       .B(compute3_out_stage3_reg),    .OUT(compute1_out_stage2), .MODE(mode));

  reduction_processing_element_13_13 compute0_stage1(.A(compute0_out_stage2_reg),       .B(compute1_out_stage2_reg),    .OUT(compute0_out_stage1), .MODE(mode));

  reduction_processing_element_13_13 compute0_stage0(.A(outp),       .B(compute0_out_stage1_reg),     .OUT(compute0_out_stage0), .MODE(mode));

endmodule
