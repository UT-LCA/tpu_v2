
module trp_unit_8 (
 input clk,
 input resetn,
 input en,
 input [8-1:0] a,
 input [1:0] mode,
 input read,
 output busy,
 output reg valid,
 output reg[8-1:0] out
);

reg en_reduction;
reg en_transpose;
reg read_transpose;
reg read_reduction;
wire transpose_busy;
wire reduction_busy;
wire reduction_done;
wire [8-1:0] reduction_out;
wire [8-1:0] transpose_out;
 
 
assign busy = transpose_busy || reduction_busy;

always@(*)begin
  if(mode == 2'b11)begin
    en_transpose = en;
    en_reduction = 1'b0;
    read_transpose = read;
    read_reduction = 1'b0;
  end
  else begin
    en_transpose = 1'b0;
    en_reduction = en;
    read_transpose = 1'b0;
    read_reduction = read;
  end
end

always@(*)begin
  if(transpose_busy)begin
    out = transpose_out;
    valid = 1'b1;
  end
  else if(reduction_done)begin
    out = reduction_out;
    valid = 1'b1;
  end
  else begin
    out = 'h0;
    valid = 1'b0;
  end
end

reduction_layer u_reduction_layer(
  .clk(clk),
  .resetn(resetn),
  .en(en_reduction),
  .read(read_reduction),
  .reduction_type(mode),
  .a(a),
  .reduced_out(reduction_out),
  .done(reduction_done),
  .busy(reduction_busy)
);

transpose u_transpose(
  .clk(clk),
  .resetn(resetn),
  .read(),
  .en(en_transpose), 
  .a(a),
  .out(transpose_out),
  .busy(transpose_busy)
);

endmodule


/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_17(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [17-1:0] d;
output [17-1:0] q;
reg [17-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_1(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_8(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [8-1:0] d;
output [8-1:0] q;
reg [8-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_66(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [66-1:0] d;
output [66-1:0] q;
reg [66-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_32(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_5.0(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [5.0-1:0] d;
output [5.0-1:0] q;
reg [5.0-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_5(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [5-1:0] d;
output [5-1:0] q;
reg [5-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_32(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_6(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [6-1:0] d;
output [6-1:0] q;
reg [6-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_2(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [2-1:0] d;
output [2-1:0] q;
reg [2-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule

module bfloat_adder_8 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [8-1:0] a,
 input [8-1:0] b,
 output reg[8-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a + b;
end

endmodule
        
module local_mult_33_33_66(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [33-1:0] dataa;
input [33-1:0] datab;
input clock;
input clken;
input aclr;
output reg [66-1:0] result;

wire [33-1:0] unsignedinputA;
wire [33-1:0] unsignedinputB;
wire [66-1:0] unsignedoutputP;

wire gated_clock;

assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else
       result <= unsignedoutputP; 
end
endmodulemodule local_mult_33_33_66(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [33-1:0] dataa;
input [33-1:0] datab;
input clock;
input clken;
input aclr;
output reg [66-1:0] result;

wire [33-1:0] unsignedinputA;
wire [33-1:0] unsignedinputB;
wire [66-1:0] unsignedoutputP;

wire gated_clock;

assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else
       result <= unsignedoutputP; 
end
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
module reduction_unit_8_3.0(
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
//  parameter 8 = 16;
//  parameter 3.0 = 4;

  input clk;
  input reset;
  input  [8-1 : 0] inp0; 
  input  [8-1 : 0] inp1; 
  input  [8-1 : 0] inp2; 
  input  [8-1 : 0] inp3; 
  input  [8-1 : 0] inp4; 
  input  [8-1 : 0] inp5; 
  input  [8-1 : 0] inp6; 
  input  [8-1 : 0] inp7; 

  input [1:0] mode;
  output [8+3.0-1 : 0] outp;

  wire   [8+3.0-1 : 0] compute0_out_stage3;
  reg    [8+3.0-1 : 0] compute0_out_stage3_reg;
  wire   [8+3.0-1 : 0] compute1_out_stage3;
  reg    [8+3.0-1 : 0] compute1_out_stage3_reg;
  wire   [8+3.0-1 : 0] compute2_out_stage3;
  reg    [8+3.0-1 : 0] compute2_out_stage3_reg;
  wire   [8+3.0-1 : 0] compute3_out_stage3;
  reg    [8+3.0-1 : 0] compute3_out_stage3_reg;

  wire   [8+3.0-1 : 0] compute0_out_stage2;
  reg    [8+3.0-1 : 0] compute0_out_stage2_reg;
  wire   [8+3.0-1 : 0] compute1_out_stage2;
  reg    [8+3.0-1 : 0] compute1_out_stage2_reg;

  wire   [8+3.0-1 : 0] compute0_out_stage1;
  reg    [8+3.0-1 : 0] compute0_out_stage1_reg;

  wire   [8+3.0-1 : 0] compute0_out_stage0;
  reg    [8+3.0-1 : 0] outp;

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
  reduction_processing_element_8_11.0 compute0_stage3(.A(compute0_out_stage4_reg),       .B(compute1_out_stage4_reg),    .OUT(compute0_out_stage3), .MODE(mode));
  reduction_processing_element_8_11.0 compute1_stage3(.A(compute2_out_stage4_reg),       .B(compute3_out_stage4_reg),    .OUT(compute1_out_stage3), .MODE(mode));
  reduction_processing_element_8_11.0 compute2_stage3(.A(compute4_out_stage4_reg),       .B(compute5_out_stage4_reg),    .OUT(compute2_out_stage3), .MODE(mode));
  reduction_processing_element_8_11.0 compute3_stage3(.A(compute6_out_stage4_reg),       .B(compute7_out_stage4_reg),    .OUT(compute3_out_stage3), .MODE(mode));

  reduction_processing_element_11.0_11.0 compute0_stage2(.A(compute0_out_stage3_reg),       .B(compute1_out_stage3_reg),    .OUT(compute0_out_stage2), .MODE(mode));
  reduction_processing_element_11.0_11.0 compute1_stage2(.A(compute2_out_stage3_reg),       .B(compute3_out_stage3_reg),    .OUT(compute1_out_stage2), .MODE(mode));

  reduction_processing_element_11.0_11.0 compute0_stage1(.A(compute0_out_stage2_reg),       .B(compute1_out_stage2_reg),    .OUT(compute0_out_stage1), .MODE(mode));

  reduction_processing_element_11.0_11.0 compute0_stage0(.A(outp),       .B(compute0_out_stage1_reg),     .OUT(compute0_out_stage0), .MODE(mode));

endmodule


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
        
/****************************************************************************
          Saturate unit

  Takes a signed input and saturates it

  sat sign
   0    x   NOP (pass through)
   1    1   VS[ADD/SUB]
   1    0   VS[ADD/SUB]_U

parameter
  SATSUMOP_NOP=2'b00,
  SATSUMOP_VS =2'b11,
  SATSUMOP_VSU=2'b10;

****************************************************************************/


module vlane_saturatesum_32(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH+2-1:0] in;
input [1:0] op;
output [WIDTH-1:0] out;

reg [WIDTH-1:0] out;

wire op_saturate;
wire op_signed;

assign op_saturate=op[1];
assign op_signed=op[0];

wire [WIDTH-1:0] maxunsigned;
wire [WIDTH-1:0] minunsigned;
wire [WIDTH-1:0] maxsigned;
wire [WIDTH-1:0] minsigned;

assign maxunsigned = {WIDTH{1'b1}};
assign minunsigned = 0;
assign maxsigned = {1'b0,{WIDTH-1{1'b1}}};
assign minsigned = {1'b1,{WIDTH-1{1'b0}}};

wire [WIDTH-1:0] through;

assign through=in[WIDTH-1:0];

wire [2:0] top3bits=(op_saturate) ? in[WIDTH+2-1:WIDTH-1] : 3'b0 ;

  always@*
    case(top3bits)
      3'b010: out=maxunsigned;
      3'b011: out=maxunsigned;
      3'b001: out=(op_signed) ? maxsigned : through;
      3'b111: out=(op_signed) ? through : minunsigned;
      3'b110: out=(op_signed) ? minsigned : minunsigned;
      default: out=through;
    endcase

endmodule
/****************************************************************************
          Control Register File

   - Has one read port (a) and one write port (c)
   - vl promoted as first-class entities
****************************************************************************/
module vregfile_control_32_32_5.0 (
    clk,
    resetn, 

    a_reg, 
    a_en,
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we,

    vl,
    matmul_masks
    );

input clk;
input resetn;

input a_en;
input [5.0-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

output [32-1:0] vl;
output [3*8-1:0] matmul_masks;

reg [32-1:0] vl;
reg [32-1:0] matmul_masks;

ram_wrapper_5.0_32_32 reg_file1(
        .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en),
        .address_a(c_reg[5.0-1:0]),
        .address_b(a_reg[5.0-1:0]),
        .wren_a(c_we),
        .wren_b(1'b0),
        .data_a(c_writedatain),
        .data_b(0),
        .out_a(),
        .out_b(a_readdataout)
  );

`ifdef TEST_BENCH
   initial begin
       $readmemh("vregfile_control.dat",reg_file1.dpram1.ram,'h0);
   end
`endif 

always@(posedge clk) begin
  if (!resetn) begin
    vl<=0;
    matmul_masks<=32'hffffffff;
  end
  else begin
    if (c_we) begin
      if (c_reg==0) begin
        vl<=c_writedatain;
      end 
      else if (c_reg==31) begin //a_rows
        matmul_masks[1*8-1:0*8] <= c_writedatain[8-1:0];
      end
      else if (c_reg==30) begin //a_cols, b_rows
        matmul_masks[2*8-1:1*8] <= c_writedatain[8-1:0];
      end
      else if (c_reg==29) begin //b_cols
        matmul_masks[3*8-1:2*8] <= c_writedatain[8-1:0];
      end
    end  
  end
end

endmodule

\ 
`include "vlane_mulshift.v"
`include "vlane_barrelshifter.v"

/****************************************************************************
          MUL unit

opA/B ---------------------------|         |                       | |----| Multiplier |--+------------------
         ------|shiftregister|---|/                     |--|shiftregister|-

 |  Pipe stage 1  |      Pipe stage 2    |    Pipe stage 3
                         done/stall
   operands----Multiply------------barrelshifter ----------

Notes:

  Stalls no matter the vector length - if the mask bits are all off we could
  theoretically skip this computation, but then we would need multiplexing
  logic to order the different results correctly so they line up with their
  lane.  Since we use a shift register to do this, all NUMLANES multiplies are
  performed.

****************************************************************************/

module vmul_unit_5_8_3_8 (clk, resetn,
            op,
            activate,
            en,
            squash,
            stall,
            opA, opB,
            vshamt,
            vmask,
            in_dst,
            in_dst_we,
            out_dst,
            out_dst_we,
            out_dst_mask,
            result
            );

parameter NUMLANES=2**3;
parameter WIDTH=2**5;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
input [((5==0) ? 1 : 5)-1:0] vshamt;  // Fixed point rounding
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [8-1:0] in_dst;
input                     in_dst_we;
output [3*8-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [8*WIDTH-1:0] mul_opA;
  wire [8*WIDTH-1:0] mul_opB;
  wire [8*WIDTH-1:0] mul_result;
  wire [8*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
  wire [4:0] ctrl_op[3:1];                  //3 pipe stages
  wire [3:1] ctrl_activate;                 //3 pipe stages
  //amana: Making modification for VTR
  //wire [((5==0) ? 1 : 5)-1:0] ctrl_vshamt[3:1]; //3 pipe stages
  wire [((5==0) ? 1*3 : 5*3)-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*8-1:0] opA_elmshifter_shiftin_right_NC;
 
 //TODO: update the parameters
  velmshifter_1_256  opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*8{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*8-1:0] opB_elmshifter_shiftin_right_NC;

  //TODO: update the parameters
  velmshifter_1_256 opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*8{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [8-1:0] mask_elmshifter_shiftin_right_NC;

  //TODO: Update the parameters 
  velmshifter_1_8  mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done & ctrl_activate[1]),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({8{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || 8==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>8)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*8) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> 8*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> 8*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  pipe_5_2 oppipe (
    .d(op),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(oppipe_squash_NC),
    .q({ctrl_op[3],ctrl_op[2],ctrl_op[1]}));

  wire [3:1] activatepipe_squash_NC; 
  pipe_1_2  activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(activatepipe_squash_NC),
    .q({ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]}));

  wire [3:1] vshamtpipe_squash_NC; 

//TODO: remove parameters

  pipe_5_2 vshamtpipe (
    .d(vshamt),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(vshamtpipe_squash_NC),
    //.q({ctrl_vshamt[3],ctrl_vshamt[2],ctrl_vshamt[1]}));
    .q(ctrl_vshamt));


//============== Instantiate across lanes =============                  
    vlane_mulshift_32_5  vmul0(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*0 +: WIDTH]),
      .opB(mul_opB[WIDTH*0 +: WIDTH]),
      .sa( mul_opB[WIDTH*0+((5>0)?5-1:0) : WIDTH*0] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*0 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift0(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(0+1)-1:WIDTH*0]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(0+1)-1:WIDTH*0])
      );

        

    vlane_mulshift_32_5  vmul1(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*1 +: WIDTH]),
      .opB(mul_opB[WIDTH*1 +: WIDTH]),
      .sa( mul_opB[WIDTH*1+((5>0)?5-1:0) : WIDTH*1] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*1 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift1(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(1+1)-1:WIDTH*1]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(1+1)-1:WIDTH*1])
      );

        

    vlane_mulshift_32_5  vmul2(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*2 +: WIDTH]),
      .opB(mul_opB[WIDTH*2 +: WIDTH]),
      .sa( mul_opB[WIDTH*2+((5>0)?5-1:0) : WIDTH*2] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*2 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift2(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(2+1)-1:WIDTH*2]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(2+1)-1:WIDTH*2])
      );

        

    vlane_mulshift_32_5  vmul3(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*3 +: WIDTH]),
      .opB(mul_opB[WIDTH*3 +: WIDTH]),
      .sa( mul_opB[WIDTH*3+((5>0)?5-1:0) : WIDTH*3] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*3 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift3(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(3+1)-1:WIDTH*3]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(3+1)-1:WIDTH*3])
      );

        

    vlane_mulshift_32_5  vmul4(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*4 +: WIDTH]),
      .opB(mul_opB[WIDTH*4 +: WIDTH]),
      .sa( mul_opB[WIDTH*4+((5>0)?5-1:0) : WIDTH*4] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*4 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift4(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(4+1)-1:WIDTH*4]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(4+1)-1:WIDTH*4])
      );

        

    vlane_mulshift_32_5  vmul5(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*5 +: WIDTH]),
      .opB(mul_opB[WIDTH*5 +: WIDTH]),
      .sa( mul_opB[WIDTH*5+((5>0)?5-1:0) : WIDTH*5] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*5 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift5(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(5+1)-1:WIDTH*5]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(5+1)-1:WIDTH*5])
      );

        

    vlane_mulshift_32_5  vmul6(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*6 +: WIDTH]),
      .opB(mul_opB[WIDTH*6 +: WIDTH]),
      .sa( mul_opB[WIDTH*6+((5>0)?5-1:0) : WIDTH*6] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*6 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift6(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(6+1)-1:WIDTH*6]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(6+1)-1:WIDTH*6])
      );

        

    vlane_mulshift_32_5  vmul7(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*7 +: WIDTH]),
      .opB(mul_opB[WIDTH*7 +: WIDTH]),
      .sa( mul_opB[WIDTH*7+((5>0)?5-1:0) : WIDTH*7] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*7 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift7(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(7+1)-1:WIDTH*7]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(7+1)-1:WIDTH*7])
      );

        

//TO DO: parameters

  //Shift Register for all multiplier results
  wire [8*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  velmshifter_1_256  result_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(1'b0),
    .shift(~done),
    .dir_left(1'b0),
    .squash(1'b0),
    //Enable/Disable rshifter for fixed-point multiply support
    //.shiftin_left(rshift_result),
    .shiftin_left(mul_result),
    .shiftin_right(shiftin_right_result_elmshifter_NC),
    .inpipe(inpipe_result_elmshifter_NC),
    .outpipe(result_buffered)
  );

  //Enable/Disable rshifter for fixed-point multiply support
  //assign result_tmp=rshift_result;
  assign result_tmp=mul_result;

  assign result=(result_tmp<<((NUMLANES/8-1)*8*WIDTH)) |
                (result_buffered>>8*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  pipe_8_2 dstpipe (
    .d( in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  pipe_1_2 dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2:1] dstmaskpipe_squash_NC;
  pipe_8_2 dstmaskpipe (
    .d( vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule


/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_32_8_8 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );

parameter 8=4;      
parameter 8=4;     // We can either shift by 1, or jump by this amount 
parameter 32=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*32-1:0 ]  inpipe;
output [ 8*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_32 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(32*8) :
                      outpipe >>(32*8)),
      .outpipe(outpipe));

endmodule
        
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_32_8_8 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );

parameter 8=4;      
parameter 8=4;     // We can either shift by 1, or jump by this amount 
parameter 32=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*32-1:0 ]  inpipe;
output [ 8*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_32 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(32*8) :
                      outpipe >>(32*8)),
      .outpipe(outpipe));

endmodule
        
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_1_8_8 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );

parameter 8=4;      
parameter 8=4;     // We can either shift by 1, or jump by this amount 
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*1-1:0 ]  inpipe;
output [ 8*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_1 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(1*8) :
                      outpipe >>(1*8)),
      .outpipe(outpipe));

endmodule
        
module velmshifter_laneunit_151 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter 151=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 151-1:0 ]  inpipe;
input [ 151-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 151-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 151-1:0 ] outpipe;

reg [ 151-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_1 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 1-1:0 ]  inpipe;
input [ 1-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 1-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 1-1:0 ] outpipe;

reg [ 1-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_1 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 1-1:0 ]  inpipe;
input [ 1-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 1-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 1-1:0 ] outpipe;

reg [ 1-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_256 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 256-1:0 ]  inpipe;
input [ 256-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 256-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 256-1:0 ] outpipe;

reg [ 256-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_8 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 8-1:0 ]  inpipe;
input [ 8-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 8-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 8-1:0 ] outpipe;

reg [ 8-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_256 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 256-1:0 ]  inpipe;
input [ 256-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 256-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 256-1:0 ] outpipe;

reg [ 256-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_8 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 8-1:0 ]  inpipe;
input [ 8-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 8-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 8-1:0 ] outpipe;

reg [ 8-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        
/****************************************************************************
          Base Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_base_32_16_4.0 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input [4.0-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input a_en, c_we;

        ram_wrapper_4.0_16_32 reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[4.0-1:0]),
	    .address_b(a_reg[4.0-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );

endmodule

        
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

/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag_2_1_8_256_8
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

parameter 2=1;
parameter 1=0;
parameter 8=32;
parameter 256=32;
parameter 8=5;

parameter 128=256/2;
parameter 7=8-1;

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2-1:0] c_we;

input [2*7-1:0] a_reg,b_reg,c_reg;
output [2*8-1:0] a_readdataout, b_readdataout;
input [2*8-1:0] c_writedatain;

genvar k;

generate
for (k=0; k<2; k=k+1)
begin : bank_gen
      ram_wrapper_7_128_8 reg_file1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[k]),
          .address_a(c_reg[k* 7 +: 7]),
          .address_b(a_reg[k* 7 +: 7]),
          .wren_a(c_we[k]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ k*8 +: 8]),
          .data_b(0),
          .out_a(),
          .out_b(a_readdataout[k*8 +: 8])
      );

      ram_wrapper_7_128_8 reg_file2(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[k]),
          .address_a(c_reg[k* 7 +: 7]),
          .address_b(b_reg[k* 7 +: 7]),
          .wren_a(c_we[k]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ k*8 +: 8]),
          .data_b(0),
          .out_a(),
          .out_b(b_readdataout[ k* 8 +: 8])
      );
end
endgenerate

endmodule

/****************************************************************************
          MUL unit

Operation table

     half sat  op unsign dir
2      0   0   0   0    0    |  Zero (This value is used to reset multiplier)
14     1   0   1   1    0    |  LMULU Lower MULtiply Unsigned
15     1   0   1   1    1    |  UMULU Upper MULtiply Unsigned
16     1   0   1   0    0    |  LMUL Lower MULtiply signed
17     1   0   1   0    1    |  UMUL Upper MULtiply signed
4      0   0   1   1    0    |  MULLOU
4      0   0   1   1    1    |  MULHIU
6      0   0   1   0    0    |  MULLO
6      0   0   1   0    1    |  MULHI
0      0   0   0   1    0    |  ShiftLeft
8      0   1   0   1    0    |  ShiftLeftSatU
10     0   1   0   0    0    |  ShiftLeftSat
1      0   0   0   1    1    |  ShiftRightLogic
3      0   0   0   0    1    |  ShiftRightArith
****************************************************************************/
module vlane_mulshift_32_5(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH=32;
parameter LOG2WIDTH=5;

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

/********* Control Signals *********/
wire is_signed, dir, is_mul, saturate, half;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [WIDTH-1:0] opA_mux_out;
wire [WIDTH-1:0] opB_mul;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;
wire [WIDTH-1:0] hi;
wire [WIDTH-1:0] lo;

assign opA_mux_out = (~half) ? ( opA ) : (WIDTH<2) ? 0 : (dir) ?
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH-1]}},opA[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH/2-1]}},opA[WIDTH/2-1:0]};

assign opB_mul = (~half) ? opB : (WIDTH<2) ? 0 : (dir) ? 
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH-1]}},opB[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH/2-1]}},opB[WIDTH/2-1:0]};

assign opB_mux_out=(is_mul) ? {is_signed&opB_mul[WIDTH-1],opB_mul} : decoded_sa;

reg zeroout;

`ifndef USE_INHOUSE_LOGIC
  `define USE_INHOUSE_LOGIC
`endif

always@(posedge clk)
  if (en[1])
    zeroout<=(op[3:0]==0);

`ifdef USE_INHOUSE_LOGIC
wire [(WIDTH+1)-1:0] dataa;
wire aclr;

assign dataa = {is_signed&opA_mux_out[WIDTH-1],opA_mux_out};
assign aclr = ~resetn;

local_mult_33_33_66 local_mult_component (
.dataa(dataa),
.datab(opB_mux_out),
.clock(clk),
.clken(en[1]),
.aclr(aclr),
.result({dum2,dum,hi,lo})
);
`else 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(en[1]),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif
// if A is positive/negative make it maximum/minimum positive/negative
wire [WIDTH-1:0] signedsaturate=
                    (opA_mux_out[WIDTH-1]) ? {1'b1,{WIDTH-1{1'b0}}} : 
                                             {1'b0,{WIDTH-1{1'b1}}};

reg [WIDTH-1:0] saturatedval_s2;
reg saturate_s2;

//Capture saturated value and saturate signal for next stage
always@(posedge clk)
  if (!resetn)
  begin
    saturatedval_s2<=0;
    saturate_s2<=0;
  end
  else if (en[1])
  begin
    saturatedval_s2<=((~is_signed) ? {WIDTH{1'b1}} : signedsaturate);
    saturate_s2<=saturate;
  end

reg sel_hi;

always@(posedge clk)
  if (!resetn)
    sel_hi<=0;
  else if (en[1])
    sel_hi<=(is_mul && dir || ~is_mul && dir && |sa);

assign result =(zeroout) ? 0 : 
                (saturate_s2 && |hi ) ? saturatedval_s2 : 
                (sel_hi) ? hi : lo;


assign {dum3, left_sa}= (dir) ? WIDTH-sa : {1'b0,sa};

//Decoder - computes 2^left_sa
always@*
begin
    decoded_sa=1<<left_sa;
end
endmodule/****************************************************************************
          MUL unit

Operation table

     half sat  op unsign dir
2      0   0   0   0    0    |  Zero (This value is used to reset multiplier)
14     1   0   1   1    0    |  LMULU Lower MULtiply Unsigned
15     1   0   1   1    1    |  UMULU Upper MULtiply Unsigned
16     1   0   1   0    0    |  LMUL Lower MULtiply signed
17     1   0   1   0    1    |  UMUL Upper MULtiply signed
4      0   0   1   1    0    |  MULLOU
4      0   0   1   1    1    |  MULHIU
6      0   0   1   0    0    |  MULLO
6      0   0   1   0    1    |  MULHI
0      0   0   0   1    0    |  ShiftLeft
8      0   1   0   1    0    |  ShiftLeftSatU
10     0   1   0   0    0    |  ShiftLeftSat
1      0   0   0   1    1    |  ShiftRightLogic
3      0   0   0   0    1    |  ShiftRightArith
****************************************************************************/
module vlane_mulshift_32_5(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH=32;
parameter LOG2WIDTH=5;

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

/********* Control Signals *********/
wire is_signed, dir, is_mul, saturate, half;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [WIDTH-1:0] opA_mux_out;
wire [WIDTH-1:0] opB_mul;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;
wire [WIDTH-1:0] hi;
wire [WIDTH-1:0] lo;

assign opA_mux_out = (~half) ? ( opA ) : (WIDTH<2) ? 0 : (dir) ?
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH-1]}},opA[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH/2-1]}},opA[WIDTH/2-1:0]};

assign opB_mul = (~half) ? opB : (WIDTH<2) ? 0 : (dir) ? 
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH-1]}},opB[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH/2-1]}},opB[WIDTH/2-1:0]};

assign opB_mux_out=(is_mul) ? {is_signed&opB_mul[WIDTH-1],opB_mul} : decoded_sa;

reg zeroout;

`ifndef USE_INHOUSE_LOGIC
  `define USE_INHOUSE_LOGIC
`endif

always@(posedge clk)
  if (en[1])
    zeroout<=(op[3:0]==0);

`ifdef USE_INHOUSE_LOGIC
wire [(WIDTH+1)-1:0] dataa;
wire aclr;

assign dataa = {is_signed&opA_mux_out[WIDTH-1],opA_mux_out};
assign aclr = ~resetn;

local_mult_33_33_66 local_mult_component (
.dataa(dataa),
.datab(opB_mux_out),
.clock(clk),
.clken(en[1]),
.aclr(aclr),
.result({dum2,dum,hi,lo})
);
`else 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(en[1]),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif
// if A is positive/negative make it maximum/minimum positive/negative
wire [WIDTH-1:0] signedsaturate=
                    (opA_mux_out[WIDTH-1]) ? {1'b1,{WIDTH-1{1'b0}}} : 
                                             {1'b0,{WIDTH-1{1'b1}}};

reg [WIDTH-1:0] saturatedval_s2;
reg saturate_s2;

//Capture saturated value and saturate signal for next stage
always@(posedge clk)
  if (!resetn)
  begin
    saturatedval_s2<=0;
    saturate_s2<=0;
  end
  else if (en[1])
  begin
    saturatedval_s2<=((~is_signed) ? {WIDTH{1'b1}} : signedsaturate);
    saturate_s2<=saturate;
  end

reg sel_hi;

always@(posedge clk)
  if (!resetn)
    sel_hi<=0;
  else if (en[1])
    sel_hi<=(is_mul && dir || ~is_mul && dir && |sa);

assign result =(zeroout) ? 0 : 
                (saturate_s2 && |hi ) ? saturatedval_s2 : 
                (sel_hi) ? hi : lo;


assign {dum3, left_sa}= (dir) ? WIDTH-sa : {1'b0,sa};

//Decoder - computes 2^left_sa
always@*
begin
    decoded_sa=1<<left_sa;
end
endmodule
/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter(clk, resetn,
            opB, sa, 
            op, 
            result);
parameter 1=32;
parameter 8=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter (8-2)=8-2;

input clk;
input resetn;

input [1-1:0] opB;
input [8-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [1-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [1-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst1(
  .data({sign_ext&opB[1-1],opB}),
  .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
  .direction(shift_direction),
  .result(dum,partial_result)
 );
 defparam
    local_shifter_inst1.LPM_WIDTH = 1+1,
    local_shifter_inst1.LPM_WIDTHDIST = 8,
    local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[1-1],opB}),
    .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = 1+1,
    shifter_inst1.lpm_widthdist = 8,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

register_2 partial_reg
  ({dum,partial_result},clk,resetn,1'b1,{dum_,partial_result_});

wire [5-1:0] sa_2;
wire shift_direction_2;

register_5 secondstage (sa, clk,resetn,1'b1,sa_2); 

register_1 secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst2(
  .data({dum_,partial_result_}),
  .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
  .direction(shift_direction_2),
  .result({dum2,result})
 );
 defparam
    local_shifter_inst2.LPM_WIDTH = 1+1,
    local_shifter_inst2.LPM_WIDTHDIST = ((8-2)>0) ? (8-2) : 1,
    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_2_2_ARITHMATIC shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,resulti}));
 defparam 
    shifter_inst2.lpm_width = 1+1,
    shifter_inst2.lpm_widthdist = ((8-2)>0) ? (8-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        /****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter(clk, resetn,
            opB, sa, 
            op, 
            result);
parameter 1=32;
parameter 8=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter (8-2)=8-2;

input clk;
input resetn;

input [1-1:0] opB;
input [8-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [1-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [1-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst1(
  .data({sign_ext&opB[1-1],opB}),
  .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
  .direction(shift_direction),
  .result(dum,partial_result)
 );
 defparam
    local_shifter_inst1.LPM_WIDTH = 1+1,
    local_shifter_inst1.LPM_WIDTHDIST = 8,
    local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[1-1],opB}),
    .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = 1+1,
    shifter_inst1.lpm_widthdist = 8,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

register_2 partial_reg
  ({dum,partial_result},clk,resetn,1'b1,{dum_,partial_result_});

wire [5-1:0] sa_2;
wire shift_direction_2;

register_5 secondstage (sa, clk,resetn,1'b1,sa_2); 

register_1 secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst2(
  .data({dum_,partial_result_}),
  .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
  .direction(shift_direction_2),
  .result({dum2,result})
 );
 defparam
    local_shifter_inst2.LPM_WIDTH = 1+1,
    local_shifter_inst2.LPM_WIDTHDIST = ((8-2)>0) ? (8-2) : 1,
    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_2_2_ARITHMATIC shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,resulti}));
 defparam 
    shifter_inst2.lpm_width = 1+1,
    shifter_inst2.lpm_widthdist = ((8-2)>0) ? (8-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        
`define DWIDTH 32
`define AWIDTH 10
`define MEM_SIZE 1024

///////////////////////////////////////////////////////////
//MAT_MUL_SIZE refers to the dimension of the matrix
//multiplication performed by the matmul unit. The value 
//8 means it will multiply an 8x8 matrix with an 8x8 matrix.
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////
//MAT_MUL_SIZE should be equal to number of lanes in the vector processor
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////

`define MASK_WIDTH 8
`define LOG2_MAT_MUL_SIZE 3

`define BB_MAT_MUL_SIZE `MAT_MUL_SIZE
`define NUM_CYCLES_IN_MAC 3
`define MEM_ACCESS_LATENCY 1
`define REG_DATAWIDTH 32
`define REG_ADDRWIDTH 8
`define ADDR_STRIDE_WIDTH 16
`define MAX_BITS_POOL 3

`define PIPE_STAGES_MATMUL 29

`include "matmul_8x8.v"

module matmul_unit_8_33_8(
 clk,
 resetn,
 activate,
 en,
 squash,
 stall,
 a_data,
 b_data,
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols,
 c_data, 
 vmask,
 in_dst,
 in_dst_we,
 out_dst,
 out_dst_we,
 out_dst_mask
);

 input clk;
 input resetn;
 input activate;
 input [33:1] en;  //Enable for each pipestage
 input [33:1] squash;  //Squash for each pipestage
 output stall;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data;
 input [8-1:0] vmask;

 input    [8-1:0] in_dst;
 input                     in_dst_we;
 output [33*8-1:0] out_dst;
 output            [33-1:0] out_dst_we;
 output   [33*8-1:0] out_dst_mask;

wire [33:1] ctrl_activate;
wire [33:1] squash_activatepipe_NC;
pipe #(1,33-1) activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(squash_activatepipe_NC),
    .q(ctrl_activate));

wire in_progress;
assign stall=in_progress;

//The actual matrix multiplier block. This is an abridged
//version of the matmul block used in the TPU v1 design.
matmul_8x8 mat(
    .clk(clk),
    .reset(~resetn),
    .pe_reset(~resetn),
    .start(activate),
    .in_progress(in_progress),
    .done(done_NC),
    .a_data(a_data),
    .b_data(b_data),
    .c_data_out(c_data), 
    .c_data_available(c_data_available),
    .validity_mask_a_rows(validity_mask_a_rows),
    .validity_mask_a_cols(validity_mask_a_cols),
    .validity_mask_b_rows(validity_mask_b_rows),
    .validity_mask_b_cols(validity_mask_b_cols)
);

wire [33:1] squash_dstpipe_NC;
pipe #(8,33-1) dstpipe (
    .d(in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[33-1:1] & {1'b1,{(33-2){~stall}}} ),
    .squash(squash_dstpipe_NC),
    .q(out_dst));

pipe #(1,33-1) dstwepipe (
    .d(in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[33-1:1] & {1'b1,{(33-2){~stall}}} ),
    .squash(squash[33-1:1]),
    .q(out_dst_we));

wire [33:1] squash_dstmaskpipe_NC;
pipe #(8,33-1) dstmaskpipe (
    .d(vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[33-1:1] & {1'b1,{(33-2){~stall}}} ),
    .squash(squash_dstmaskpipe_NC),
    .q(out_dst_mask));
endmodule
/****************************************************************************
          inc Register File

   - Has one read port (a) and one write port (c)
   - inc0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_inc_32_8_3.0 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input a_en;
input [3.0-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

     ram_wrapper_3.0_8_32 reg_file1(
         .clk(clk),
         .resetn(resetn),
         .rden_a(1'b0),
         .rden_b(a_en),
         .address_a(c_reg[3.0-1:0]),
         .address_b(a_reg[3.0-1:0]),
         .wren_a(c_we&(|c_reg)),
         .wren_b(1'b0),
         .data_a(c_writedatain),
         .data_b(0),
         .out_a(),
         .out_b(a_readdataout)
     );

endmodule

 

/******************************************************************************
 * Vector Pipeline
 *
 *                vl (arrives from vector control pipeline in Stage2)
 *                vs
 *                vc
 *|   Stage1  |  Stage2   |  Stage3  |  Stage4  |  Stage5  | Stage6  |  Stage7 |
 *| instr     |           |          | dispatch |          |         |
 *| decode    | control   | bank     | regfile  | execute  | WB
 *
 *****************************************************************************/

`include "vdispatcher.v"
`include "vlane_alu.v"
`include "vmul_unit.v"
`include "vmem_unit.v"
`include "vlane_flagalu.v"
`include "matmul_unit.v"

`define LO(x,b) ((x)&~({1024{1'b1}}<<b))

module vlanes_8_3_8_3_8_64_6_2_1_32_5_2_1_0_128_7_128_7_32_32_32_5 (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive

    stall_in,
    is_stalled,
    has_memop,

    // Control register values 
    vc_in,
    vl_in,
    vbase_in,
    vinc_in,
    vstride_in,
    vs_in,
    matmul_masks_in,

    // vs Writeback
    vs_writedata,
    vs_we,       // Actually issues write command for vs (after done stalling)
    vs_wetrack,  // says there exist a scalar write operation in pipe
    vs_dst,

    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,    //Prefetch hint
    dbus_wait

    );

parameter NUMMEMPARALLELLANES=2;
parameter LOG2NUMMEMPARALLELLANES=1;
parameter LOG2NUMMULLANES=8;

parameter VRIDWIDTH=5;
parameter VELMIDWIDTH=6-3;
parameter REGIDWIDTH=VRIDWIDTH+6-3;
parameter BANKREGIDWIDTH=VRIDWIDTH+6-3-1;

// Register identifier = { vr[0-31], element }

`define VRID_RANGE REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH
`define VRELM_RANGE REGIDWIDTH-VRIDWIDTH-1:0

// NUMFUS = ALU*2 + MUL + MEM + FALU*MEMBANKS + MATMUL(1) + BFLOAT
// UNITS(2) + ACTIVATION + TRP
parameter NUMFUS=4+2*(2-1)*0+1+4; //Adding 1 for matmul FU
parameter FU_ALU=0;
parameter FU_MUL=FU_ALU+(2-1)*0+1;
parameter FU_MEM=FU_MUL+1;
parameter FU_FALU=FU_MEM+1;
parameter FU_MATMUL=FU_FALU+1;
parameter FU_BFADDER = FU_MATMUL + 1;
parameter FU_BFMULT = FU_BFADDER + 1;
parameter FU_ACT = FU_BFMULT + 1;
parameter FU_TRP = FU_ACT + 1;

input clk;
input resetn;

//Instruction Format:
//9 - op[21] mask 1/0
//8 - op[23] scalar/vec src2
//7 - op[22] scalar/vec src1
//6:0 - {op[24],op[5:0]} op

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

input [3:0] stall_in;  
output [`MAX_PIPE_STAGES-1:0] is_stalled;
output has_memop;

// Control register values
input  [ 32-1 : 0 ]   vc_in;
input  [ 32-1 : 0 ]   vl_in;
input  [ 32-1 : 0 ]   vbase_in;
input  [ 32-1 : 0 ]   vinc_in;
input  [ 32-1 : 0 ]   vstride_in;
input  [ 32-1 : 0 ]   vs_in;
input  [3*`MAT_MUL_SIZE-1 : 0]  matmul_masks_in;

// vs Writeback
output       [ 32-1 : 0 ]   vs_writedata;
output                           vs_we;
output               [ 5 : 2 ]   vs_wetrack;
output [ 5-1 : 0 ]   vs_dst;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (128/8)-1 : 0 ]   dbus_byteen;
output  [ 128-1 : 0 ]  dbus_writedata;
input   [ 128-1 : 0 ]  dbus_readdata;
input               dbus_cachematch;
input               dbus_cachemiss;
input               dbus_wait;
output  [ 31 : 0 ]  dbus_prefetch;


`include "visa.v"

parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

parameter ALUOP_ZERO    =11'b00100000101,
          ALUOP_ADD     =11'b00010000000,
          ALUOP_ADDU    =11'b00000000000,
          ALUOP_SUB     =11'b00011000000,
          ALUOP_SUBU    =11'b00001000000,
          ALUOP_CMP_EQ  =11'b00011000000,
          ALUOP_CMP_NEQ =11'b00011000001,
          ALUOP_CMP_LT  =11'b00011000010,
          ALUOP_CMP_LE  =11'b00011000011,
          ALUOP_CMP_LT_U=11'b00001000010,
          ALUOP_CMP_LE_U=11'b00001000011,
          ALUOP_AND     =11'b00000000101,
          ALUOP_OR      =11'b00000000001,
          ALUOP_XOR     =11'b00000001001,
          ALUOP_NOR     =11'b00000001101,
          ALUOP_MIN     =11'b00111010001,
          ALUOP_MIN_U   =11'b00101010001,
          ALUOP_MAX     =11'b00111100001,
          ALUOP_MAX_U   =11'b00101100001,
          ALUOP_ABS     =11'b11111000010,
          ALUOP_MERGE   =11'b10100000011;

parameter SATSUMOP_NOP=2'b00,
          SATSUMOP_VS =2'b11,
          SATSUMOP_VSU=2'b10;

parameter SATSIZEOP_VSATUW=4'b0000,
          SATSIZEOP_VSATUB=4'b0001,
          SATSIZEOP_VSATUH=4'b0010,
          SATSIZEOP_VSATW  =4'b1100,
          SATSIZEOP_VSATB  =4'b1101,
          SATSIZEOP_VSATH  =4'b1110,
          SATSIZEOP_VSATSUW=4'b0100,
          SATSIZEOP_VSATSUB=4'b0101,
          SATSIZEOP_VSATSUH=4'b0110;

parameter MULOP_ZERO  =5'b00000,
          MULOP_LMULU =5'b10110,
          MULOP_UMULU =5'b10111,
          MULOP_LMUL  =5'b10100,
          MULOP_UMUL  =5'b10101,
          MULOP_MULUHI=5'b00111,
          MULOP_MULULO=5'b00110,
          MULOP_MULHI =5'b00101,
          MULOP_MULLO =5'b00100,
          MULOP_SLL   =5'b00010,
          MULOP_SLS   =5'b01010,
          MULOP_SLSU  =5'b01000,
          MULOP_SRL   =5'b00011,
          MULOP_SRA   =5'b00001;

parameter MEMOP_SHIFT=7'b0000000, 
          MEMOP_LDUB=7'b1000000,
          MEMOP_LDUH=7'b1000100,
          MEMOP_LDUW=7'b1001000,
          MEMOP_LDB=7'b1000010,
          MEMOP_LDH=7'b1000110,
          MEMOP_LDW=7'b1001010,
          MEMOP_LDSUB=7'b1010000,
          MEMOP_LDSUH=7'b1010100,
          MEMOP_LDSUW=7'b1011000,
          MEMOP_LDSB=7'b1010010,
          MEMOP_LDSH=7'b1010110,
          MEMOP_LDSW=7'b1011010,
          MEMOP_LDXUB=7'b1100000,
          MEMOP_LDXUH=7'b1100100,
          MEMOP_LDXUW=7'b1101000,
          MEMOP_LDXB=7'b1100010,
          MEMOP_LDXH=7'b1100110,
          MEMOP_LDXW=7'b1101010,
          MEMOP_STB=7'b1000001,
          MEMOP_STH=7'b1000101,
          MEMOP_STW=7'b1001001,
          MEMOP_STSB=7'b1010001,
          MEMOP_STSH=7'b1010101,
          MEMOP_STSW=7'b1011001,
          MEMOP_STXB=7'b1100001,
          MEMOP_STXH=7'b1100101,
          MEMOP_STXW=7'b1101001;

parameter FLAGOP_AND=3'b000,
          FLAGOP_OR= 3'b001,
          FLAGOP_XOR=3'b010,
          FLAGOP_NOR=3'b011,
          FLAGOP_CLR=3'b100,
          FLAGOP_SET=3'b101;

wire                        [ 9 : 0 ]   ir_op;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 4 : 0 ]   ir_dst;
wire                                    ir_mask;

// Control register saved values
reg                       pipe_advance_s2_r;
reg   [ 32-1 : 0 ]   vc_in_saved;
reg   [ 32-1 : 0 ]   vl_in_saved;
reg   [ 32-1 : 0 ]   vbase_in_saved;
reg   [ 32-1 : 0 ]   vinc_in_saved;
reg   [ 32-1 : 0 ]   vstride_in_saved;
reg   [ 32-1 : 0 ]   vs_in_saved;

// Control register values
//TODO: Rethink whether the following vars (vc, vl, vbase, vinc, vstride)
//need to be change to use `MAX_PIPE_STAGES. I think these variables are 
//only used for memory operations and for that, we only need 6 stages.
//Not a big deal because most of these are not used much. Synthesis tool
//will optimize out the unnecessary bits anyway.
wire  [ 32-1 : 0 ]   vc[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vl[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vbase[`MAX_PIPE_STAGES-1:2];
reg   [ 32-1 : 0 ]   vbase_s4;
wire  [ 32-1 : 0 ]   vinc[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vstride[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vs[`MAX_PIPE_STAGES-1:2];
reg   [ 32-1 : 0 ]   vs_s3[2-1:0];
reg   [ 32-1 : 0 ]   vs_s4[NUMFUS-1:0];
reg   [ 32-1 : 0 ]   vc_s3[2-1:0];
reg   [ 32-1 : 0 ]   vc_s4[NUMFUS-1:0];

// Vector register file signals
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   vr_a_reg;
wire     [ 2*8*2*8-1 : 0 ]   _vr_a_readdataout;
reg                         [2-1:0]   vr_a_en;
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   vr_b_reg;
wire     [ 2*8*2*8-1 : 0 ]   _vr_b_readdataout;
reg                         [2-1:0]   vr_b_en;
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   _vr_c_reg;
reg      [ 2*8*2*8-1 : 0 ]   _vr_c_writedatain;
reg        [ 2*2*8-1 : 0 ]   vr_c_byteen;
reg                         [2-1:0]   vr_c_we;

reg          [ 32*8-1 : 0 ]   vr_a_readdataout[2-1:0];
reg          [ 32*8-1 : 0 ]   vr_b_readdataout[2-1:0];
reg          [ 32*8-1 : 0 ]   vr_c_writedatain[2-1:0];
reg           [ REGIDWIDTH-1 : 0 ]   vr_c_reg[2-1:0]; //for testbench and debugging

// Flag register file signals
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
wire               [ 2*8-1 : 0 ]   vf_a_readdataout;
reg                            [ 2-1:0]   vf_a_en;
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_b_reg;
wire               [ 2*8-1 : 0 ]   vf_b_readdataout;
reg                            [ 2-1:0]   vf_b_en;
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_c_reg;
reg                [ 2*8-1 : 0 ]   vf_c_writedatain;
reg                            [ 2-1:0]   vf_c_we;

wire             [ REGIDWIDTH-1 : 0 ]   wb_dst[NUMFUS-1:0];
wire                     [NUMFUS-1:0]   wb_dst_we;
wire               [ 8-1 : 0 ]   wb_dst_mask[NUMFUS-1:0];

wire                        [ `MAX_PIPE_STAGES-1 : 4 ]   dst_we[NUMFUS-1:0];
wire             [ REGIDWIDTH-1 : 0 ]   dst[NUMFUS-1:0][`MAX_PIPE_STAGES-1:4];
wire               [ 8-1 : 0 ]   dst_mask[NUMFUS-1:0][`MAX_PIPE_STAGES-1:4];
wire             [ REGIDWIDTH-1 : 0 ]   dst_s2;
reg              [ REGIDWIDTH-1 : 0 ]   _dst_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   dst_s3[2-1:0];
reg     [ 2*REGIDWIDTH-1 : 0 ]   t_dst_s3;
reg              [ REGIDWIDTH-1 : 0 ]   dst_s4[NUMFUS-1:0];
reg                  [ NUMFUS-1 : 0 ]   dst_we_s4;
reg  [(1+(2-1)*0)*REGIDWIDTH-1:0] alu_dst[4:4];
reg  [(1+(2-1)*0)-1:0]            alu_dst_we[4:4];
reg  [(1+(2-1)*0)*REGIDWIDTH-1:0] falu_dst[4:4];
reg  [(1+(2-1)*0)-1:0]            falu_dst_we[4:4];

wire                                    imask_s2;
reg                [ 2-1 : 0 ]   imask_s3;

wire             [ REGIDWIDTH-1 : 0 ]   src1_s2;
wire             [ REGIDWIDTH-1 : 0 ]   src2_s2;
reg              [ REGIDWIDTH-1 : 0 ]   src1_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   src2_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src1_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src2_s3[2-1:0];
wire                                    src1scalar_s2;
wire                                    src2scalar_s2;
reg                    [2-1:0]   src1scalar_s3;
reg                    [2-1:0]   src2scalar_s3;
reg                      [NUMFUS-1:0]   src1scalar_s4;
reg                      [NUMFUS-1:0]   src2scalar_s4;

reg                    [8-1:0]   lane_en[2-1:0];
reg                    [8-1:0]   vlane_en[NUMFUS-1:0];
reg                                     mem_last_subvector_s4;

reg                     [6-1:0]   src_start_delayed;
wire                    [6-1:0]   src_elm;
wire                    [6-1:0]   src_limit;
reg                     [6-1:0]   src_limit_s3[2-1:0];
wire                    [6-1:0]   src_start;

wire                    [32-1:0]   total_shamt;
wire                    [6-1:0]   dst_start;

reg          [ 32*8-1 : 0 ]   vr_src1[NUMFUS-1:0];
reg          [ 32*8-1 : 0 ]   vr_src2[NUMFUS-1:0];
wire         [ 32*8-1 : 0 ]   matmul_out;
wire         [ 32*8-1 : 0 ]   bfadder_result_s5;
wire         [ 32*8-1 : 0 ]   bfmult_result_s5;
wire         [ 32*8-1 : 0 ]   act_result_s5;
wire         [ 32*8-1 : 0 ]   trp_out;
reg                [ 8-1 : 0 ]   vf_src1[NUMFUS-1:0];
reg                [ 8-1 : 0 ]   vf_src2[NUMFUS-1:0];
reg                [ 8-1 : 0 ]   vmask[NUMFUS-1:0];
reg                [ 8-1 : 0 ]   vmask_final[2-1:0];

reg        [ 32*8-1 : 0 ]   vstrideoffset_s4;
wire     [ 32*8-1 : 0 ]   load_result_s5;
wire               [ 8-1 : 0 ]   load_result_mask_s5;

wire [ 32*8-1 : 0 ] alu_result_s5[(2-1)*0:0];
wire           [ 8-1 : 0 ] alu_cmpresult_s4[(2-1)*0:0];
wire           [ 8-1 : 0 ] flagalu_result_s4[(2-1)*0:0];
wire           [ 8-1 : 0 ] flagalu_result_s5[(2-1)*0:0];
wire [ 32*8-1 : 0 ] mulshift_result_s5;

//Support 1 Lane processor
wire [((3>0) ? 3 : 1)-1:0] elmshamt[`MAX_PIPE_STAGES-1:2];

reg ctrl1_vr_a_en; // SRC1
reg ctrl1_vr_b_en; // SRC2
reg ctrl1_vr_c_en; // SRC3
reg ctrl1_vr_d_we; // DEST
reg ctrl1_vf_a_en;
reg ctrl1_vf_b_en;
reg ctrl1_vf_c_we;
reg ctrl1_vs_we;
reg ctrl1_usesvssel;    // 1-if instruction can have .sv/.vs variants
reg [1:0] ctrl1_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg [1:0] ctrl1_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_srclimit_sel;  //0-vl, 1 vl+vindex
reg [1:0] ctrl1_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_setvlto1;
reg ctrl1_mem_dir_left;  //1-left, 0-right
reg ctrl1_rshiftnonzero;
reg [10:0] ctrl1_alu_op;
reg [1:0] ctrl1_satsum_op;
reg [3:0] ctrl1_satsize_op;
reg [4:0] ctrl1_mulshift_op;
reg ctrl1_matmul_en;
reg ctrl1_bfadder_en;
reg ctrl1_bfmult_en;
reg ctrl1_act_en;
reg ctrl1_trp_en;
reg ctrl1_memunit_en;
reg ctrl1_mem_en;
reg [6:0] ctrl1_memunit_op;
reg ctrl1_ismasked;
reg [2:0] ctrl1_flagalu_op;
reg ctrl1_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl1_volatiledest;
reg ctrl1_vrdest_sel;  //0-dest, 1-src2
reg ctrl1_vf_a_sel; //0-0/1 from instr, 1-src1

wire [5:4] ctrl_vs_we;
wire [6:0] ctrl_memunit_op[`MAX_PIPE_STAGES-1:1] ;

wire ctrl2_vr_a_en; // SRC1
wire ctrl2_vr_b_en; // SRC2
wire ctrl2_vr_c_we; // DEST
wire ctrl2_vf_a_en;
wire ctrl2_vf_b_en;
wire ctrl2_vf_c_we;
wire ctrl2_vs_we;
wire ctrl2_useslanes;
wire [1:0] ctrl2_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire [1:0] ctrl2_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_srclimit_sel;  //0-vl, 1 vl+vindex
wire [1:0] ctrl2_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_setvlto1;
wire ctrl2_mem_dir_left;  //1-left, 0-right
wire ctrl2_rshiftnonzero;
wire [10:0] ctrl2_alu_op;
wire [1:0] ctrl2_satsum_op;
wire [3:0] ctrl2_satsize_op;
wire [4:0] ctrl2_mulshift_op;
wire ctrl2_memunit_en;
wire ctrl2_mem_en;
wire [6:0] ctrl2_memunit_op;
wire ctrl2_ismasked;
wire [2:0] ctrl2_flagalu_op;
wire ctrl2_vf_wbsel;   //0-flag ALU, 1-normal ALU
wire ctrl2_volatiledest;
wire ctrl2_vf_a_sel; //0-0/1 from instr, 1-src1
wire ctrl2_mulshift_en;
wire ctrl2_matmul_en;
wire ctrl2_bfadder_en;
wire ctrl2_bfmult_en;
wire ctrl2_act_en;
wire ctrl2_trp_en;
wire ctrl2_alufalu_en;

reg [2-1:0] ctrl3_vr_a_en; // SRC1
reg [2-1:0] ctrl3_vr_b_en; // SRC2
reg [2-1:0] ctrl3_vr_c_we; // DEST
reg [2-1:0] ctrl3_vf_a_en;
reg [2-1:0] ctrl3_vf_b_en;
reg [2-1:0] ctrl3_vf_c_we;
reg [2-1:0] ctrl3_vs_we;
reg [2-1:0] ctrl3_useslanes;
reg [2-1:0] ctrl3_mem_dir_left;
reg [2-1:0] ctrl3_rshiftnonzero;
reg [10:0] ctrl3_alu_op[2-1:0];
reg [1:0] ctrl3_satsum_op[2-1:0];
reg [3:0] ctrl3_satsize_op[2-1:0];
reg [4:0] ctrl3_mulshift_op[2-1:0];
reg [2-1:0] ctrl3_memunit_en;
reg [2-1:0] ctrl3_mem_en;
reg [6:0] ctrl3_memunit_op[2-1:0];
reg [2-1:0] ctrl3_ismasked;
reg [2:0] ctrl3_flagalu_op[2-1:0];
reg [2-1:0] ctrl3_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg [2-1:0] ctrl3_volatiledest;
reg [2-1:0] ctrl3_vf_a_sel; //0-0/1 from instr, 1-src1
reg [2-1:0] ctrl3_mulshift_en;
reg [2-1:0] ctrl3_matmul_en;
reg [2-1:0] ctrl3_trp_en;
reg [2-1:0] ctrl3_bfadder_en;
reg [2-1:0] ctrl3_bfmult_en;
reg [2-1:0] ctrl3_act_en;
reg [2-1:0] ctrl3_alufalu_en;

reg ctrl4_mem_dir_left;
reg ctrl4_rshiftnonzero;
reg [10:0] ctrl4_alu_op[(2-1)*0:0];
reg [1:0] ctrl4_satsum_op[(2-1)*0:0];
reg [3:0] ctrl4_satsize_op[(2-1)*0:0];
reg [4:0] ctrl4_mulshift_op;
reg ctrl4_memunit_en;
reg ctrl4_mem_en;
reg [6:0] ctrl4_memunit_op;
reg [NUMFUS-1:0] ctrl4_ismasked;
reg [2:0] ctrl4_flagalu_op[(2-1)*0:0];
//reg ctrl4_vf_wbsel[(2-1)*0:0];   //0-flag ALU, 1-normal ALU
reg [(2-1)*0:0] ctrl4_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl4_volatiledest;
//reg ctrl4_vf_a_sel[(2-1)*0:0]; //0-0/1 from instr, 1-src1
reg [(2-1)*0:0] ctrl4_vf_a_sel; //0-0/1 from instr, 1-src1
reg ctrl4_mulshift_en;
reg ctrl4_matmul_en;
reg ctrl4_trp_en;
reg ctrl4_bfadder_en;
reg ctrl4_bfmult_en;
reg ctrl4_act_en;

wire ctrl5_mem_en;

wire [`VRELM_RANGE] regid_pad;

integer bd;
genvar  ba;
integer bi;
integer i;
integer j;
genvar  bk;
genvar  k;
integer m;
integer n;
integer b;
integer b3;
integer f3;
integer bn;
integer fn;
integer fn2;
integer bw;

wire [2*6-1:0] rdelm;
wire [2*6-1:0] wrelm;
// MSb of count entries indicates if instruction is dead or not.
wire [2*(6-3+1)-1:0] count;
reg  [2-1:0]   last_subvector;
wire [2-1:0]   first_subvector;
reg  [2-1:0] wrongbank_s3;
reg  [2-1:0] alive_s3;
reg  [2-1:0] banksel_s4[NUMFUS-1:0];

wire dispatcher_shift;
wire dispatcher_rotate;

wire [`MAX_PIPE_STAGES-1:0] internal_pipe_advance;
wire [`MAX_PIPE_STAGES-1:0] pipe_advance;
wire [`MAX_PIPE_STAGES-1:0] pipe_squash;
wire stall_srcstart;
wire stall_dispatcher;
wire stall_hazsrc1;
wire stall_hazsrc2;
wire stall_hazfsrc1;
wire stall_hazfsrc2;
wire stall_memunit;
wire _stall_memunit;
wire stall_mulcooldown;
wire stall_mulunit;
wire stall_matmul;

// DEBUG signals for Modelsim
wire  [7:0] D_instr[2:1];
reg   [7:0] D_instr_s3[2-1:0];
reg   [7:0] D_instr_s4[NUMFUS-1:0];
wire  [7:0] D_instr_s5[NUMFUS-1:0];
wire  [7:0] D_instr_s6[NUMFUS-1:0];
reg   [NUMFUS-1:0] D_last_subvector_s4;
wire  [NUMFUS-1:0] D_last_subvector_s5;
wire  [NUMFUS-1:0] D_last_subvector_s6;
wire  [NUMFUS-1:0] D_last_subvector_s31;
wire  [NUMFUS-1:0] D_wb_last_subvector;
reg   [2-1:0] D_last_subvector_done;
reg   [2-1:0] D_wb_instrdone;

// Module instance
  pipe_8_1  debuginstrpipe (
      .d( {instr[25:24],instr[5:0]} ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[2:1] ),
      .squash( pipe_squash[2:1] ),
      .q( {D_instr[2],D_instr[1]}));



// Module instance
    pipereg_8 debugintrfupipereg1_0 (
      .d( D_instr_s4[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[0]));

// Module instance
    pipereg_8 debugintrfupipereg2_0 (
      .d( D_instr_s5[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[0]));

// Module instance
    pipereg_1 debuglastpipereg1_0 (
      .d( D_last_subvector_s4[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[0]));

// Module instance
    pipereg_1 debuglastpipereg2_0 (
      .d( D_last_subvector_s5[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[0]));



// Module instance
    pipereg_8 debugintrfupipereg1_1 (
      .d( D_instr_s4[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[1]));

// Module instance
    pipereg_8 debugintrfupipereg2_1 (
      .d( D_instr_s5[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[1]));

// Module instance
    pipereg_1 debuglastpipereg1_1 (
      .d( D_last_subvector_s4[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[1]));

// Module instance
    pipereg_1 debuglastpipereg2_1 (
      .d( D_last_subvector_s5[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[1]));



// Module instance
    pipereg_8 debugintrfupipereg1_2 (
      .d( D_instr_s4[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[2]));

// Module instance
    pipereg_8 debugintrfupipereg2_2 (
      .d( D_instr_s5[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[2]));

// Module instance
    pipereg_1 debuglastpipereg1_2 (
      .d( D_last_subvector_s4[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[2]));

// Module instance
    pipereg_1 debuglastpipereg2_2 (
      .d( D_last_subvector_s5[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[2]));



// Module instance
    pipereg_8 debugintrfupipereg1_3 (
      .d( D_instr_s4[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[3]));

// Module instance
    pipereg_8 debugintrfupipereg2_3 (
      .d( D_instr_s5[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[3]));

// Module instance
    pipereg_1 debuglastpipereg1_3 (
      .d( D_last_subvector_s4[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[3]));

// Module instance
    pipereg_1 debuglastpipereg2_3 (
      .d( D_last_subvector_s5[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[3]));



// Module instance
    pipereg_8 debugintrfupipereg1_4 (
      .d( D_instr_s4[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[4]));

// Module instance
    pipereg_8 debugintrfupipereg2_4 (
      .d( D_instr_s5[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[4]));

// Module instance
    pipereg_1 debuglastpipereg1_4 (
      .d( D_last_subvector_s4[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[4]));

// Module instance
    pipereg_1 debuglastpipereg2_4 (
      .d( D_last_subvector_s5[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[4]));



// Module instance
    pipereg_8 debugintrfupipereg1_5 (
      .d( D_instr_s4[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[5]));

// Module instance
    pipereg_8 debugintrfupipereg2_5 (
      .d( D_instr_s5[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[5]));

// Module instance
    pipereg_1 debuglastpipereg1_5 (
      .d( D_last_subvector_s4[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[5]));

// Module instance
    pipereg_1 debuglastpipereg2_5 (
      .d( D_last_subvector_s5[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[5]));



// Module instance
    pipereg_8 debugintrfupipereg1_6 (
      .d( D_instr_s4[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[6]));

// Module instance
    pipereg_8 debugintrfupipereg2_6 (
      .d( D_instr_s5[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[6]));

// Module instance
    pipereg_1 debuglastpipereg1_6 (
      .d( D_last_subvector_s4[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[6]));

// Module instance
    pipereg_1 debuglastpipereg2_6 (
      .d( D_last_subvector_s5[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[6]));



// Module instance
    pipereg_8 debugintrfupipereg1_7 (
      .d( D_instr_s4[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[7]));

// Module instance
    pipereg_8 debugintrfupipereg2_7 (
      .d( D_instr_s5[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[7]));

// Module instance
    pipereg_1 debuglastpipereg1_7 (
      .d( D_last_subvector_s4[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[7]));

// Module instance
    pipereg_1 debuglastpipereg2_7 (
      .d( D_last_subvector_s5[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[7]));



// Module instance
    pipereg_8 debugintrfupipereg1_8 (
      .d( D_instr_s4[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[8]));

// Module instance
    pipereg_8 debugintrfupipereg2_8 (
      .d( D_instr_s5[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[8]));

// Module instance
    pipereg_1 debuglastpipereg1_8 (
      .d( D_last_subvector_s4[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[8]));

// Module instance
    pipereg_1 debuglastpipereg2_8 (
      .d( D_last_subvector_s5[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[8]));



//tell vpu to hold vc,vbase,etc values
assign is_stalled=~internal_pipe_advance | {stall_srcstart,2'b0};

assign has_memop=ctrl1_mem_en|ctrl2_mem_en|(|ctrl3_mem_en)|ctrl4_mem_en|ctrl5_mem_en;

/******************************************************************************/
/************************** Inter-Pipe Signals ********************************/
/******************************************************************************/

/* Memunit is in stage 5 but we stall in stage 4 so we don't squash the
* destination register contents*/

assign pipe_advance=internal_pipe_advance & ~{3'b0,stall_in};

assign internal_pipe_advance[0]=internal_pipe_advance[1];
assign internal_pipe_advance[1]=internal_pipe_advance[2];
assign internal_pipe_advance[2]=internal_pipe_advance[3] && 
                          ~stall_srcstart &&
                          ~stall_dispatcher &&
                          ~stall_hazsrc1 && ~stall_hazsrc2 && 
                          ~stall_hazfsrc1 && ~stall_hazfsrc2 && 
                          ~stall_mulcooldown;
assign internal_pipe_advance[3]=internal_pipe_advance[4];
assign internal_pipe_advance[4]=internal_pipe_advance[5];
assign internal_pipe_advance[5]=internal_pipe_advance[6] && ~stall_mulunit && ~stall_memunit;
//assign internal_pipe_advance[6]=1'b1;

assign pipe_squash[0]=pipe_advance[1]&~pipe_advance[0];
assign pipe_squash[1]=pipe_advance[2]&~pipe_advance[1];
assign pipe_squash[2]=pipe_advance[3]&~pipe_advance[2];
assign pipe_squash[3]=pipe_advance[4]&~pipe_advance[3];
assign pipe_squash[4]=pipe_advance[5]&~pipe_advance[4];
assign pipe_squash[5]=pipe_advance[6]&~pipe_advance[5];
//assign pipe_squash[6]=1'b0;

//The code below basically replicates the statements above for
//pipe stages from 6 to MAX_PIPE_STAGES. Using generate statement
//to reduce typing.



  assign pipe_squash[0] = pipe_advance[0+1] & ~pipe_advance[0];
  if (0==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[0] = internal_pipe_advance[0+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[0] = internal_pipe_advance[0+1];
  end



  assign pipe_squash[1] = pipe_advance[1+1] & ~pipe_advance[1];
  if (1==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[1] = internal_pipe_advance[1+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[1] = internal_pipe_advance[1+1];
  end



  assign pipe_squash[2] = pipe_advance[2+1] & ~pipe_advance[2];
  if (2==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[2] = internal_pipe_advance[2+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[2] = internal_pipe_advance[2+1];
  end



  assign pipe_squash[3] = pipe_advance[3+1] & ~pipe_advance[3];
  if (3==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[3] = internal_pipe_advance[3+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[3] = internal_pipe_advance[3+1];
  end



  assign pipe_squash[4] = pipe_advance[4+1] & ~pipe_advance[4];
  if (4==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[4] = internal_pipe_advance[4+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[4] = internal_pipe_advance[4+1];
  end



  assign pipe_squash[5] = pipe_advance[5+1] & ~pipe_advance[5];
  if (5==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[5] = internal_pipe_advance[5+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[5] = internal_pipe_advance[5+1];
  end



  assign pipe_squash[6] = pipe_advance[6+1] & ~pipe_advance[6];
  if (6==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[6] = internal_pipe_advance[6+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[6] = internal_pipe_advance[6+1];
  end



  assign pipe_squash[7] = pipe_advance[7+1] & ~pipe_advance[7];
  if (7==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[7] = internal_pipe_advance[7+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[7] = internal_pipe_advance[7+1];
  end



  assign pipe_squash[8] = pipe_advance[8+1] & ~pipe_advance[8];
  if (8==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[8] = internal_pipe_advance[8+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[8] = internal_pipe_advance[8+1];
  end



  assign pipe_squash[9] = pipe_advance[9+1] & ~pipe_advance[9];
  if (9==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[9] = internal_pipe_advance[9+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[9] = internal_pipe_advance[9+1];
  end



  assign pipe_squash[10] = pipe_advance[10+1] & ~pipe_advance[10];
  if (10==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[10] = internal_pipe_advance[10+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[10] = internal_pipe_advance[10+1];
  end



  assign pipe_squash[11] = pipe_advance[11+1] & ~pipe_advance[11];
  if (11==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[11] = internal_pipe_advance[11+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[11] = internal_pipe_advance[11+1];
  end



  assign pipe_squash[12] = pipe_advance[12+1] & ~pipe_advance[12];
  if (12==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[12] = internal_pipe_advance[12+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[12] = internal_pipe_advance[12+1];
  end



  assign pipe_squash[13] = pipe_advance[13+1] & ~pipe_advance[13];
  if (13==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[13] = internal_pipe_advance[13+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[13] = internal_pipe_advance[13+1];
  end



  assign pipe_squash[14] = pipe_advance[14+1] & ~pipe_advance[14];
  if (14==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[14] = internal_pipe_advance[14+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[14] = internal_pipe_advance[14+1];
  end



  assign pipe_squash[15] = pipe_advance[15+1] & ~pipe_advance[15];
  if (15==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[15] = internal_pipe_advance[15+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[15] = internal_pipe_advance[15+1];
  end



  assign pipe_squash[16] = pipe_advance[16+1] & ~pipe_advance[16];
  if (16==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[16] = internal_pipe_advance[16+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[16] = internal_pipe_advance[16+1];
  end



  assign pipe_squash[17] = pipe_advance[17+1] & ~pipe_advance[17];
  if (17==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[17] = internal_pipe_advance[17+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[17] = internal_pipe_advance[17+1];
  end



  assign pipe_squash[18] = pipe_advance[18+1] & ~pipe_advance[18];
  if (18==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[18] = internal_pipe_advance[18+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[18] = internal_pipe_advance[18+1];
  end



  assign pipe_squash[19] = pipe_advance[19+1] & ~pipe_advance[19];
  if (19==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[19] = internal_pipe_advance[19+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[19] = internal_pipe_advance[19+1];
  end



  assign pipe_squash[20] = pipe_advance[20+1] & ~pipe_advance[20];
  if (20==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[20] = internal_pipe_advance[20+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[20] = internal_pipe_advance[20+1];
  end



  assign pipe_squash[21] = pipe_advance[21+1] & ~pipe_advance[21];
  if (21==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[21] = internal_pipe_advance[21+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[21] = internal_pipe_advance[21+1];
  end



  assign pipe_squash[22] = pipe_advance[22+1] & ~pipe_advance[22];
  if (22==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[22] = internal_pipe_advance[22+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[22] = internal_pipe_advance[22+1];
  end



  assign pipe_squash[23] = pipe_advance[23+1] & ~pipe_advance[23];
  if (23==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[23] = internal_pipe_advance[23+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[23] = internal_pipe_advance[23+1];
  end



  assign pipe_squash[24] = pipe_advance[24+1] & ~pipe_advance[24];
  if (24==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[24] = internal_pipe_advance[24+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[24] = internal_pipe_advance[24+1];
  end



  assign pipe_squash[25] = pipe_advance[25+1] & ~pipe_advance[25];
  if (25==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[25] = internal_pipe_advance[25+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[25] = internal_pipe_advance[25+1];
  end



  assign pipe_squash[26] = pipe_advance[26+1] & ~pipe_advance[26];
  if (26==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[26] = internal_pipe_advance[26+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[26] = internal_pipe_advance[26+1];
  end



  assign pipe_squash[27] = pipe_advance[27+1] & ~pipe_advance[27];
  if (27==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[27] = internal_pipe_advance[27+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[27] = internal_pipe_advance[27+1];
  end



  assign pipe_squash[28] = pipe_advance[28+1] & ~pipe_advance[28];
  if (28==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[28] = internal_pipe_advance[28+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[28] = internal_pipe_advance[28+1];
  end



  assign pipe_squash[29] = pipe_advance[29+1] & ~pipe_advance[29];
  if (29==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[29] = internal_pipe_advance[29+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[29] = internal_pipe_advance[29+1];
  end



  assign pipe_squash[30] = pipe_advance[30+1] & ~pipe_advance[30];
  if (30==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[30] = internal_pipe_advance[30+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[30] = internal_pipe_advance[30+1];
  end



  assign pipe_squash[31] = pipe_advance[31+1] & ~pipe_advance[31];
  if (31==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[31] = internal_pipe_advance[31+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[31] = internal_pipe_advance[31+1];
  end



  assign pipe_squash[32] = pipe_advance[32+1] & ~pipe_advance[32];
  if (32==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[32] = internal_pipe_advance[32+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[32] = internal_pipe_advance[32+1];
  end



assign pipe_squash[`MAX_PIPE_STAGES-1]=1'b0;
assign internal_pipe_advance[`MAX_PIPE_STAGES-1]=1'b1;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

  assign ir_op={instr[25:22],instr[5:0]}; //10 bits
  assign ir_dst=instr[10:6];
  assign ir_src1=instr[15:11];
  assign ir_src2=instr[20:16];
  assign ir_mask=instr[21];

  // Determine which instruction read from which ports
  always@*
  begin
    ctrl1_vr_a_en=0;
    ctrl1_vr_b_en=0;
    ctrl1_vr_c_en=0;
    ctrl1_vf_a_en=0;
    ctrl1_vf_b_en=0;
    ctrl1_vf_a_sel=0;
    ctrl1_usesvssel=0;
    case(ir_op)
    COP2_VADD:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VADD_U:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSUB:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSUB_U:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VMULHI:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMULHI_U:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMOD:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VMOD_U:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_EQ:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_NE:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_LT:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_U_LT:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_LE:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_U_LE:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VDIV:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VDIV_U:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMIN:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMIN_U:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMAX:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMAX_U:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMULLO:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VABS:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=1;        end
    COP2_VAND:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VOR:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXOR:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VNOR:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSLL:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSRL:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSRA:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSAT_B:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_H:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_W:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_B:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_H:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_W:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_L:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_U_B:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_U_H:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_U_W:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSADD:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSADD_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSSUB:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSSUB_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSRR:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSRR_U:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSLS:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSLS_U:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VXUMUL:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMUL_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMUL:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMUL_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMADD:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMADD_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMSUB:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMSUB_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMADD:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMADD_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMSUB:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMSUB_U:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VINS_VV:

        begin          ctrl1_vr_a_en=1;          ctrl1_vr_b_en=1;        end
      //COP2_VINS_SV: doesn't read any vectors or flags
    COP2_VEXT_VV:

        begin          ctrl1_vr_a_en=1;        end
    COP2_VEXT_SV:

        begin          ctrl1_vr_a_en=1;        end
    COP2_VEXT_U_SV:

        begin          ctrl1_vr_a_en=1;        end
    COP2_VCOMPRESS:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VEXPAND:

        begin          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VMERGE:

        begin          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
      //COP2_VFINS:
    COP2_VEXTHALF:

        begin          ctrl1_vr_a_en=1;        end
    COP2_VHALF:

        begin          ctrl1_vr_a_en=1;        end
    COP2_VHALFUP:

        begin          ctrl1_vr_a_en=1;        end
    COP2_VHALFDN:

        begin          ctrl1_vr_a_en=1;        end
      //COP2_VSATVL:
    COP2_VFAND:

        begin          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
    COP2_VFOR:

        begin          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
    COP2_VFXOR:

        begin          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
    COP2_VFNOR:

        begin          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
      //COP2_VFCLR:
      //COP2_VFSET:
    COP2_VIOTA:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VCIOTA:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFPOP:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFFF1:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFFL1:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFSETBF:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFSETIF:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFSETOF:

        begin          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
      //COP2_VFMT8:
      //COP2_VFMF8:
      //COP2_VFCLR8:
      //COP2_VFOR8:
      //COP2_VFLD:
    COP2_VLD_B:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLD_H:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLD_W:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLD_L:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLD_U_B:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLD_U_H:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLD_U_W:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDS_B:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDS_H:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDS_W:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDS_L:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDS_U_B:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDS_U_H:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDS_U_W:

        begin          ctrl1_vf_a_en=1;        end
    COP2_VLDX_B:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_H:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_W:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_L:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_U_B:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_U_H:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_U_W:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
      //COP2_VFST:
    COP2_VST_B:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VST_H:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VST_W:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VST_L:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_B:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_H:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_W:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_L:

        begin          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTX_B:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTX_H:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTX_W:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTX_L:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_B:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_H:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_W:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_L:

        begin          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode enables 
  always@*
  begin
    ctrl1_vr_d_we=0;
    ctrl1_vf_c_we=0;
    ctrl1_vs_we=0;
    ctrl1_vrdest_sel=0;
    ctrl1_elmshamt_sel=0;
    ctrl1_srcshamt_sel=0;
    ctrl1_srclimit_sel=0;
    ctrl1_dstshamt_sel=0;
    ctrl1_mem_dir_left=0;
    ctrl1_rshiftnonzero=0;
    ctrl1_memunit_en=0;
    ctrl1_mem_en=0;
    ctrl1_ismasked=0;
    ctrl1_setvlto1=0;
    ctrl1_vf_wbsel=0;
    ctrl1_volatiledest=0;
    case(ir_op)
    COP2_VADD:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VADD_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSUB:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSUB_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMULHI:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMULHI_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VDIV:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VDIV_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMOD:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMOD_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_EQ:

        begin          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_NE:

        begin          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_LT:

        begin          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_U_LT:

        begin          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_LE:

        begin          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_U_LE:

        begin          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VMIN:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMIN_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMAX:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMAX_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMULLO:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VABS:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VAND:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VOR:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VXOR:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VNOR:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSLL:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRL:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRA:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_L:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_U_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_U_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_U_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSADD:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSADD_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSSUB:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSSUB_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRR:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRR_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSLS:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSLS_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VXUMUL:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMUL_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMUL:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMUL_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMADD:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMADD_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMSUB:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMSUB_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMADD:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMADD_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMSUB:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMSUB_U:

        begin          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VINS_VV:

        begin          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=0;          ctrl1_dstshamt_sel=3;          ctrl1_mem_dir_left=1;          ctrl1_volatiledest=1;        end
    COP2_VINS_SV:

        begin          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_dstshamt_sel=3;          ctrl1_mem_dir_left=1;          ctrl1_setvlto1=1;        end
    COP2_VEXT_VV:

        begin          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=3;          ctrl1_srclimit_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_volatiledest=1;        end
    COP2_VEXT_SV:

        begin          ctrl1_vs_we=1;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=3;          ctrl1_srclimit_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_setvlto1=1;        end
    COP2_VEXT_U_SV:

        begin          ctrl1_vs_we=1;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=3;          ctrl1_srclimit_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_setvlto1=1;        end
    COP2_VCOMPRESS:

        begin          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_mem_dir_left=0;          ctrl1_ismasked=1;          ctrl1_volatiledest=1;        end
    COP2_VEXPAND:

        begin          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_mem_dir_left=1;          ctrl1_ismasked=1;          ctrl1_volatiledest=1;        end
    COP2_VMERGE:

        begin          ctrl1_vr_d_we=1;        end
    COP2_VFINS:

        begin          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=0;          ctrl1_mem_dir_left=1;        end
    COP2_VEXTHALF:

        begin          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=1;          ctrl1_srcshamt_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
    COP2_VHALF:

        begin          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=1;          ctrl1_srcshamt_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
    COP2_VHALFUP:

        begin          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=2;          ctrl1_srcshamt_sel=2;          ctrl1_mem_dir_left=0;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
    COP2_VHALFDN:

        begin          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=2;          ctrl1_srcshamt_sel=0;          ctrl1_dstshamt_sel=2;          ctrl1_mem_dir_left=1;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
      //COP2_VSATVL:
    COP2_VFAND:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFOR:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFXOR:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFNOR:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFCLR:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFSET:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VIOTA:

        begin          ctrl1_vr_d_we=1;        end
    COP2_VCIOTA:

        begin          ctrl1_vr_d_we=1;        end
    COP2_VFPOP:

        begin          ctrl1_vs_we=1;        end
    COP2_VFFF1:

        begin          ctrl1_vs_we=1;        end
    COP2_VFFL1:

        begin          ctrl1_vs_we=1;        end
    COP2_VFSETBF:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFSETIF:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFSETOF:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFMT8:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFMF8:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFCLR8:

        begin          ctrl1_vf_c_we=1;        end
    COP2_VFOR8:

        begin          ctrl1_vf_c_we=1;        end
      //COP2_VFLD,
    COP2_VLD_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_L:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_U_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_U_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_U_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_L:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_U_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_U_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_U_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_L:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_U_B:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_U_H:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_U_W:

        begin          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
      //COP2_VFST:
    COP2_VST_B:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VST_H:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VST_W:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VST_L:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_B:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_H:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_W:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_L:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_B:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_H:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_W:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_L:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_B:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_H:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_W:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_L:

        begin          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode instructions FU op codes
  initial
  begin
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;  //Aid subsetting by setting to zero
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_matmul_en=0;
    ctrl1_trp_en = 1'b0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_memunit_op=0;
    ctrl1_flagalu_op=FLAGOP_CLR;
  end
  always@*
  begin
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_memunit_op=0;
    ctrl1_matmul_en=0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_trp_en = 1'b0;
    ctrl1_flagalu_op=FLAGOP_CLR;
    case(ir_op)
      COP2_VADD:      ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO;
      COP2_VADD_U:    ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO;
      COP2_VSUB:      ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO;
      COP2_VSUB_U:    ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
      COP2_VMULHI:  ctrl1_mulshift_op=MULOP_MULHI;
      COP2_VMULHI_U:ctrl1_mulshift_op=MULOP_MULUHI;
      COP2_VDIV:    ctrl1_matmul_en=1'b1;  //Note: This is a hack. We're using VDIV opcode for matmul operation
      COP2_VBFADD:    ctrl1_bfadder_en = 1'b1;
      COP2_VBFMULT:   ctrl1_bfmult_en = 1'b1;
      COP2_VACT:      ctrl1_act_en = 1'b1;
      COP2_VTRP:      ctrl1_trp_en = 1'b1;
      //COP2_VDIV_U,
      //COP2_VMOD,
      //COP2_VMOD_U,
      COP2_VCMP_EQ:   ctrl1_alu_op=ALUOP_CMP_EQ^ALUOP_ZERO;
      COP2_VCMP_NE:   ctrl1_alu_op=ALUOP_CMP_NEQ^ALUOP_ZERO;
      COP2_VCMP_LT:   ctrl1_alu_op=ALUOP_CMP_LT^ALUOP_ZERO;
      COP2_VCMP_U_LT: ctrl1_alu_op=ALUOP_CMP_LT_U^ALUOP_ZERO;
      COP2_VCMP_LE:   ctrl1_alu_op=ALUOP_CMP_LE^ALUOP_ZERO;
      COP2_VCMP_U_LE: ctrl1_alu_op=ALUOP_CMP_LE_U^ALUOP_ZERO;
      COP2_VMIN:      ctrl1_alu_op=ALUOP_MIN^ALUOP_ZERO;
      COP2_VMIN_U:    ctrl1_alu_op=ALUOP_MIN_U^ALUOP_ZERO;
      COP2_VMAX:      ctrl1_alu_op=ALUOP_MAX^ALUOP_ZERO;
      COP2_VMAX_U:    ctrl1_alu_op=ALUOP_MAX_U^ALUOP_ZERO;
      COP2_VMULLO:    ctrl1_mulshift_op=MULOP_MULLO;
      COP2_VABS:      ctrl1_alu_op=ALUOP_ABS^ALUOP_ZERO;
      COP2_VAND:      ctrl1_alu_op=ALUOP_AND^ALUOP_ZERO;
      COP2_VOR:       ctrl1_alu_op=ALUOP_OR^ALUOP_ZERO;
      COP2_VXOR:      ctrl1_alu_op=ALUOP_XOR^ALUOP_ZERO;
      COP2_VNOR:      ctrl1_alu_op=ALUOP_NOR^ALUOP_ZERO;
      COP2_VSLL: ctrl1_mulshift_op=MULOP_SLL;
      COP2_VSRL: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSRA: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSAT_B: ctrl1_satsize_op=SATSIZEOP_VSATB;
      COP2_VSAT_H: ctrl1_satsize_op=SATSIZEOP_VSATH;
      COP2_VSAT_W: ctrl1_satsize_op=SATSIZEOP_VSATW;
      COP2_VSAT_SU_B: ctrl1_satsize_op=SATSIZEOP_VSATSUB;
      COP2_VSAT_SU_H: ctrl1_satsize_op=SATSIZEOP_VSATSUH;
      COP2_VSAT_SU_W: ctrl1_satsize_op=SATSIZEOP_VSATSUW;
      //COP2_VSAT_SU_L:
      COP2_VSAT_U_B: ctrl1_satsize_op=SATSIZEOP_VSATUB;
      COP2_VSAT_U_H: ctrl1_satsize_op=SATSIZEOP_VSATUH;
      COP2_VSAT_U_W: ctrl1_satsize_op=SATSIZEOP_VSATUW;
      COP2_VSADD: begin ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSADD_U: begin 
          ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO; 
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSSUB: begin ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSSUB_U: begin 
          ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSRR: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSRR_U: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSLS: ctrl1_mulshift_op=MULOP_SLS;
      COP2_VSLS_U: ctrl1_mulshift_op=MULOP_SLSU;
      COP2_VXUMUL: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMUL_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMUL: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMUL_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXUMADD: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMADD_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXUMSUB: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMSUB_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMADD: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMADD_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXLMSUB: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMSUB_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VINS_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VINS_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_U_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VCOMPRESS:
      //COP2_VEXPAND:
      COP2_VMERGE:      ctrl1_alu_op=ALUOP_MERGE^ALUOP_ZERO;
      //COP2_VFINS:
      COP2_VEXTHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFUP: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFDN: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VSATVL:
      COP2_VFAND: ctrl1_flagalu_op=FLAGOP_AND;
      COP2_VFOR: ctrl1_flagalu_op=FLAGOP_OR;
      COP2_VFXOR: ctrl1_flagalu_op=FLAGOP_XOR;
      COP2_VFNOR: ctrl1_flagalu_op=FLAGOP_NOR;
      COP2_VFCLR: ctrl1_flagalu_op=FLAGOP_CLR;
      COP2_VFSET: ctrl1_flagalu_op=FLAGOP_SET;
      //COP2_VIOTA,
      //COP2_VCIOTA:
      //COP2_VFPOP,
      //COP2_VFFF1,
      //COP2_VFFL1:
      //COP2_VFSETBF,
      //COP2_VFSETIF,
      //COP2_VFSETOF,
      //COP2_VFMT8,
      //COP2_VFMF8,
      //COP2_VFCLR8,
      //COP2_VFOR8:
      //COP2_VFLD:
      COP2_VLD_B: ctrl1_memunit_op=MEMOP_LDB;
      COP2_VLD_H: ctrl1_memunit_op=MEMOP_LDH;
      COP2_VLD_W: ctrl1_memunit_op=MEMOP_LDW;
      //COP2_VLD_L,
      COP2_VLD_U_B: ctrl1_memunit_op=MEMOP_LDUB;
      COP2_VLD_U_H: ctrl1_memunit_op=MEMOP_LDUH;
      COP2_VLD_U_W: ctrl1_memunit_op=MEMOP_LDUW;
      COP2_VLDS_B: ctrl1_memunit_op=MEMOP_LDSB;
      COP2_VLDS_H: ctrl1_memunit_op=MEMOP_LDSH;
      COP2_VLDS_W: ctrl1_memunit_op=MEMOP_LDSW;
      //COP2_VLDS_L:
      COP2_VLDS_U_B: ctrl1_memunit_op=MEMOP_LDSUB;
      COP2_VLDS_U_H: ctrl1_memunit_op=MEMOP_LDSUH;
      COP2_VLDS_U_W: ctrl1_memunit_op=MEMOP_LDSUW;
      COP2_VLDX_B: ctrl1_memunit_op=MEMOP_LDXB;
      COP2_VLDX_H: ctrl1_memunit_op=MEMOP_LDXH;
      COP2_VLDX_W: ctrl1_memunit_op=MEMOP_LDXW;
      //COP2_VLDX_L:
      COP2_VLDX_U_B: ctrl1_memunit_op=MEMOP_LDXUB;
      COP2_VLDX_U_H: ctrl1_memunit_op=MEMOP_LDXUH;
      COP2_VLDX_U_W: ctrl1_memunit_op=MEMOP_LDXUW;
      //COP2_VFST:
      COP2_VST_B: ctrl1_memunit_op=MEMOP_STB;
      COP2_VST_H: ctrl1_memunit_op=MEMOP_STH;
      COP2_VST_W: ctrl1_memunit_op=MEMOP_STW;
      //COP2_VST_L:
      COP2_VSTS_B: ctrl1_memunit_op=MEMOP_STSB;
      COP2_VSTS_H: ctrl1_memunit_op=MEMOP_STSH;
      COP2_VSTS_W: ctrl1_memunit_op=MEMOP_STSW;
      //COP2_VSTS_L:
      COP2_VSTX_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTX_H: ctrl1_memunit_op=MEMOP_STXH;
      COP2_VSTX_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTX_L:
      COP2_VSTXO_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTXO_H: ctrl1_memunit_op=MEMOP_STXH;
      COP2_VSTXO_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTXO_L:
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  assign regid_pad=0;

// Module instance
  pipereg_8  pipe1reg_secondstageonly (
      .d( {
        ctrl1_elmshamt_sel,
        ctrl1_srcshamt_sel,
        ctrl1_srclimit_sel,
        ctrl1_dstshamt_sel,
        ctrl1_setvlto1
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( 1'b1 ),
      .q({
        ctrl2_elmshamt_sel,
        ctrl2_srcshamt_sel,
        ctrl2_srclimit_sel, //used to get squashed, pretend doesn't need to
        ctrl2_dstshamt_sel,
        ctrl2_setvlto1
      }));

  // *********** Pipeline signals that need to be squashed *********

//module instance
  pipereg_17  pipe1regwsquash (
      .d( {
        ctrl1_vr_d_we,
        ctrl1_vf_c_we,
        ctrl1_vs_we,
        ctrl1_vr_a_en|ctrl1_vr_c_en,
        ctrl1_vr_b_en,
        ctrl1_vf_a_en,
        ctrl1_vf_b_en,
        ctrl1_vr_a_en|ctrl1_vr_b_en|ctrl1_vr_c_en|ctrl1_vr_d_we|ctrl1_vf_a_en|ctrl1_vf_b_en|ctrl1_vf_c_we,
        ctrl1_memunit_en,
        ctrl1_mem_en,
        |ctrl1_mulshift_op,
        ctrl1_matmul_en,
        ctrl1_bfadder_en,
        ctrl1_bfmult_en,
        ctrl1_act_en,
        ctrl1_trp_en,
        (ctrl1_alu_op!=(ALUOP_ZERO^ALUOP_ZERO)) || ctrl1_vf_c_we
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( ~pipe_squash[1] ),
      .q({
        ctrl2_vr_c_we,
        ctrl2_vf_c_we,
        ctrl2_vs_we,
        ctrl2_vr_a_en,
        ctrl2_vr_b_en,
        ctrl2_vf_a_en,
        ctrl2_vf_b_en,
        ctrl2_useslanes,
        ctrl2_memunit_en,
        ctrl2_mem_en,
        ctrl2_mulshift_en,
        ctrl2_matmul_en,
        ctrl2_bfadder_en,
        ctrl2_bfmult_en,
        ctrl2_act_en,
        ctrl2_trp_en,
        ctrl2_alufalu_en
        })
      );

  // *********** Pipeline signals that don't need to be squashed *********
  pipereg_66  pipe1reg (
      .d( {
        {(ctrl1_vrdest_sel) ? ir_src2 : ir_dst, regid_pad},
        {(ctrl1_vr_c_en ) ? ir_dst : ir_src1, regid_pad},
        {ir_src2,regid_pad},
        ir_op[7] && ctrl1_usesvssel,
        ir_op[6] && ctrl1_usesvssel,
        ir_mask,
        ctrl1_vf_a_sel,
        ctrl1_rshiftnonzero,
        ctrl1_mem_dir_left,
        ctrl1_alu_op,
        ctrl1_satsum_op,
        ctrl1_satsize_op,
        ctrl1_mulshift_op,
        ctrl1_memunit_op,
        ctrl1_ismasked,
        ctrl1_flagalu_op,
        ctrl1_vf_wbsel,
        ctrl1_volatiledest
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( 1'b1 ),
      .q({
        dst_s2,
        src1_s2,
        src2_s2,
        src1scalar_s2,
        src2scalar_s2,
        imask_s2,
        ctrl2_vf_a_sel,
        ctrl2_rshiftnonzero,
        ctrl2_mem_dir_left,
        ctrl2_alu_op,
        ctrl2_satsum_op,
        ctrl2_satsize_op,
        ctrl2_mulshift_op,
        ctrl2_memunit_op,
        ctrl2_ismasked,
        ctrl2_flagalu_op,
        ctrl2_vf_wbsel,
        ctrl2_volatiledest
      }));
  
  wire [6:1] squash_ctrlmemoppipe_NC;

//module instance
  pipe_7_5  ctrlmemoppipe (
      .d( ctrl1_memunit_op ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:1] & {4'b1,ctrl2_memunit_en,1'b1} ),
      .squash(squash_ctrlmemoppipe_NC),
      //.squash( pipe_squash[6:1] ),
      .q( {ctrl_memunit_op[6],ctrl_memunit_op[5],ctrl_memunit_op[4],ctrl_memunit_op[3],ctrl_memunit_op[2],ctrl_memunit_op[1]} ));

/******************************************************************************/
/******************************* 2nd Pipeline Stage ***************************/
/******************************************************************************/

  // if src_start!=0 stall pipeline to calculate it and then do haz check

  onecyclestall shamtstall(ctrl2_srcshamt_sel!=0,clk,resetn,stall_srcstart);

  always@(posedge clk)
    if (!resetn || pipe_advance[1] )
      src_start_delayed<=0;
    else 
      src_start_delayed<=src_start;

  assign src_start= ( ctrl2_srcshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_srcshamt_sel==1 ) ? vl[2][32-1:1] :   // vl>>2
               ( ctrl2_srcshamt_sel==2 ) ? 1 << vc[2][6-1:0]://2^vcindex
               0;

  assign src_elm=src_start_delayed & (-1<<3);

  assign src_limit= ((ctrl2_setvlto1) ? 0 : vl[2][6-1:0] - 1'b1) +
                    ((ctrl2_srclimit_sel) ? vc[2][6-1:0] : 0);


  /******************* Adjust dest to account for shift  ****************/

  // Compute real destination register - accounting for shifting instructions

  assign dst_start= ( ctrl2_dstshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_dstshamt_sel==2 ) ? 1 << vc[2][6-1:0]://2^vcindex
               0;

  //assign dst_elm = {dst_start[6-1:0]>>3,{3{1'b0}}};

  assign total_shamt= ( ctrl2_elmshamt_sel==3 ) ? vc[2] :          // vcindex
               ( ctrl2_elmshamt_sel==1 ) ? vl[2][32-1:1] :    // vl>>2
               ( ctrl2_elmshamt_sel==2 ) ? 1 << vc[2][6-1:0]://2^vcindex
               0;

  /************ Save vc_in values to not stall control pipeline ************/
  always@(posedge clk)
    if (!resetn)
      pipe_advance_s2_r<=0;
    else
      pipe_advance_s2_r<=pipe_advance[1];

  always@(posedge clk)
    if (!resetn)
    begin
      vc_in_saved<=0;
      vl_in_saved<=0;
      vbase_in_saved<=0;
      vinc_in_saved<=0;
      vstride_in_saved<=0;
      vs_in_saved<=0;
    end
    else if (pipe_advance_s2_r)
    begin
      vc_in_saved<=vc_in;
      vl_in_saved<=vl_in;
      vbase_in_saved<=vbase_in;
      vinc_in_saved<=vinc_in;
      vstride_in_saved<=vstride_in;
      vs_in_saved<=vs_in;
    end

  wire [6:2] squash_vcpipe_NC;
//module instance 
 pipe_32_4 vcpipe (
      .d( (!pipe_advance_s2_r) ? vc_in_saved : vc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vcpipe_NC),
      .q( {vc[6],vc[5],vc[4],vc[3],vc[2]} ));

  wire [6:2] squash_vlpipe_NC;

//module instance 
  pipe_32_4 vlpipe (
      .d( (!pipe_advance_s2_r) ? vl_in_saved : vl_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vlpipe_NC),
      .q( {vl[6],vl[5],vl[4],vl[3],vl[2]} ));

  wire [6:2] squash_vbasepipe_NC;

//module instance 
  pipe_32_4 vbasepipe (
      .d( (!pipe_advance_s2_r) ? vbase_in_saved : vbase_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vbasepipe_NC),
      .q( {vbase[6],vbase[5],vbase[4],vbase[3],vbase[2]} ));

  wire [6:2] squash_vincpipe_NC;

//module instance 
  pipe_32_4 vincpipe (
      .d( (!pipe_advance_s2_r) ? vinc_in_saved : vinc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vincpipe_NC),
      .q( {vinc[6],vinc[5],vinc[4],vinc[3],vinc[2]} ));

  //stride register also used for elmt shamt for vext/vhalf/etc in memunit
  wire [6:2] squash_vstridepipe_NC;

//module instance 
  pipe_32_4 vstridepipe (
      .d( (ctrl2_memunit_en&~ctrl2_mem_en) ? 
            ((3>0) ?
                  total_shamt[((3>0) ? 3 : 1)-1:0] : 0) :
          (!pipe_advance_s2_r) ? vstride_in_saved : vstride_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vstridepipe_NC),
      .q( {vstride[6],vstride[5],vstride[4],vstride[3],vstride[2]} ));


  /*****************************    ISSUER    *****************************/

  assign stall_dispatcher=
    //Structural hazard
    (|(ctrl3_memunit_en&~last_subvector) && ctrl2_memunit_en) ||
    (|(ctrl3_mulshift_en&~last_subvector) && ctrl2_mulshift_en) ||
    (|(ctrl3_matmul_en&~last_subvector) && ctrl2_matmul_en) ||
    (|(ctrl3_bfadder_en&~last_subvector) && ctrl2_bfadder_en) ||
    (|(ctrl3_bfmult_en&~last_subvector) && ctrl2_bfmult_en) ||
    (|(ctrl3_act_en&~last_subvector) && ctrl2_act_en) ||
    (|(ctrl3_trp_en&~last_subvector) && ctrl2_trp_en) ||
    (|(ctrl3_alufalu_en&~last_subvector) && ctrl2_alufalu_en && 0==0)||
    //Entry 0 is taken
    (dispatcher_rotate && ctrl2_useslanes);

  //assign stall_mulcooldown=|(ctrl3_mulshift_en&last_subvector);
  assign stall_mulcooldown=1'b0;

  assign dispatcher_shift=pipe_advance[3];
  // Rotate if last entry is alive (count not -1) and greater than zero
  assign dispatcher_rotate=(~count[2*(6-3+1)-1]) && 
                          ((count>>((2-1)*(6-3+1)))!=0);

`define DISPATCHWIDTH 6+     1+     1+     1+     1+     1+     1+     1+     1+     1+     1+     1+     1+     REGIDWIDTH+     REGIDWIDTH+     REGIDWIDTH+     1+     1+     1+     1+     1+     1+     11+     2+     4+     5+     7+     1+     3+     1+     1+     1+     1+     32+     32+     8
wire [2*(`DISPATCHWIDTH)-1:0] dispatcher_instr;

//module instance
  vdispatcher_2_151_6_6__4 vdispatcher(
      .clk(clk),
      .resetn(resetn),
      .shift(dispatcher_shift),
      .rotate(dispatcher_rotate), //rotate and shift must be high to rotate
      .inshift_instr( {
          src_limit,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vs_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_useslanes,
          (pipe_squash[2]) ? 1'b0 : ctrl2_memunit_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mem_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mulshift_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_matmul_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfadder_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfmult_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_act_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_trp_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_alufalu_en,
          dst_s2,
          src1_s2,
          src2_s2,
          src1scalar_s2,
          src2scalar_s2,
          imask_s2,
          ctrl2_vf_a_sel,
          ctrl2_rshiftnonzero,
          ctrl2_mem_dir_left,
          ctrl2_alu_op,
          ctrl2_satsum_op,
          ctrl2_satsize_op,
          ctrl2_mulshift_op,
          ctrl2_memunit_op,
          ctrl2_ismasked,
          ctrl2_flagalu_op,
          ctrl2_vf_wbsel,
          ctrl2_volatiledest,
          (!pipe_advance_s2_r) ? vc_in_saved : vc_in,
          (!pipe_advance_s2_r) ? vs_in_saved : vs_in,
          (pipe_squash[2]) ? 8'b0 : D_instr[2]
          }),
      .inshift_first(ctrl2_useslanes),
      .inshift_rdelm(src_start),
      .inshift_wrelm(dst_start),
      .inshift_count((pipe_squash[2] || !ctrl2_useslanes) ? -1 : (src_limit[6-1:0]>>3) - (src_elm[6-1:0]>>3) ),
      .increment((~wrongbank_s3)&{2{pipe_advance[3]}}),
      .rdelm_add_sub(0),
      .wrelm_add_sub(0),
      .count_add_sub(1),
      .rdelm_valuetoadd(8),
      .wrelm_valuetoadd(8),
      .count_valuetoadd(1),
      .instr(dispatcher_instr),
      .first(first_subvector),
      .rdelm(rdelm),
      .wrelm(wrelm),
      .count(count)
    );

  

  /******************* Stall on RAW HAZARDS ************************/

  always@*
    for (bd=0; bd<1+(2-1)*0; bd=bd+1)
    begin
      alu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH]=dst[FU_ALU+bd][4];
      alu_dst_we[4][bd]=dst_we[FU_ALU+bd][4];
      falu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH]=dst[FU_FALU+bd][4];
      falu_dst_we[4][bd]=dst_we[FU_FALU+bd][4];
    end

//module instance 
  hazardchecker_8_3_4_2 src1hazchecker(
      .src( src1_s2 |(src_start_delayed[6-1:0]>>3) ),
      .src_valid(ctrl2_vr_a_en),
      .dst({
        alu_dst[4],
        dst[FU_MUL][4],
        dst[FU_MEM][4],
        dst[FU_MATMUL][4]
        }),
      .dst_valid({
        alu_dst_we[4],
        dst_we[FU_MUL][4],
        dst_we[FU_MEM][4],
        dst_we[FU_MATMUL][4]
        }),
      .dst_mode({{3+(2-1)*0{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc1));
//module instance 

  hazardchecker_8_3_4_2 src2hazchecker(
      .src( src2_s2 |(src_start_delayed[6-1:0]>>3) ),
      .src_valid(ctrl2_vr_b_en),
      .dst({
        alu_dst[4],
        dst[FU_MUL][4],
        dst[FU_MEM][4],
        dst[FU_MATMUL][4]
        }),
      .dst_valid({
        alu_dst_we[4],
        dst_we[FU_MUL][4],
        dst_we[FU_MEM][4],
        dst_we[FU_MATMUL][4]
        }),
      .dst_mode({{3+(2-1)*0{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc2));

  //Check flag hazards - flags always start at 0th element
//module instance 
  hazardchecker_8_3_1_2 fsrc1hazchecker(
      // Check for mask flag and src1 flag depending on instruction
      .src( (ctrl2_vf_a_sel) ? src1_s2 : (imask_s2<<VELMIDWIDTH)),
      .src_valid(ctrl2_vf_a_en),
      .dst(falu_dst[4]),
      .dst_valid(falu_dst_we[4]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc1));

//module instance 
  hazardchecker_8_3_1_2 fsrc2hazchecker(
      .src( src2_s2 ),
      .src_valid(ctrl2_vf_b_en),
      .dst(falu_dst[4]),
      .dst_valid(falu_dst_we[4]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc2));


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  always@*
    for (bi=0; bi<2; bi=bi+1)
    begin
      {
        src_limit_s3[bi],
        ctrl3_vr_c_we[bi],
        ctrl3_vf_c_we[bi],
        ctrl3_vs_we[bi],
        ctrl3_vr_a_en[bi],
        ctrl3_vr_b_en[bi],
        ctrl3_vf_a_en[bi],
        ctrl3_vf_b_en[bi],
        ctrl3_useslanes[bi],
        ctrl3_memunit_en[bi],
        ctrl3_mem_en[bi],
        ctrl3_mulshift_en[bi],
        ctrl3_matmul_en[bi],
        ctrl3_bfadder_en[bi],
        ctrl3_bfmult_en[bi],
        ctrl3_act_en[bi],
        ctrl3_trp_en[bi],
        ctrl3_alufalu_en[bi],
        _dst_s3[bi],
        _src1_s3[bi],
        _src2_s3[bi],
        src1scalar_s3[bi],
        src2scalar_s3[bi],
        imask_s3[bi],
        ctrl3_vf_a_sel[bi],
        ctrl3_rshiftnonzero[bi],
        ctrl3_mem_dir_left[bi],
        ctrl3_alu_op[bi],
        ctrl3_satsum_op[bi],
        ctrl3_satsize_op[bi],
        ctrl3_mulshift_op[bi],
        ctrl3_memunit_op[bi],
        ctrl3_ismasked[bi],
        ctrl3_flagalu_op[bi],
        ctrl3_vf_wbsel[bi],
        ctrl3_volatiledest[bi],
        vc_s3[bi],
        vs_s3[bi],
        D_instr_s3[bi]
      } = dispatcher_instr>>(bi*(`DISPATCHWIDTH));

      last_subvector[bi]=~|count[bi*(6-3+1)+:6-3];
      alive_s3[bi]=~count[(bi+1)*(6-3+1)-1];

      for (i=0; i<8; i=i+1)
        lane_en[bi][i]= (3==0);
        //amana: Removed support for 1 lane. This is not a usecase for us. 
        //The code was just causing errors with the parser to convert code 
        //to VTR compatible code.
        // || //Support 1 lane
        //  ~((first_subvector[bi]) && 
        //      i<`LO(rdelm[bi*6 +: 6],3) ||
        //    (last_subvector[bi]) && 
        //      i>src_limit_s3[bi][((3>0) ? 3 : 1)-1:0] );
    end


  // ************* Map from issuer to register file banks *************
  always@*
    for (b=0; b<2; b=b+1)
    begin 
      dst_s3[b]=_dst_s3[b] | (wrelm[b*6+:6]>>3);
      t_dst_s3[b*REGIDWIDTH +: REGIDWIDTH]=dst_s3[b];
      src1_s3[b]=_src1_s3[b] | (rdelm[b*6+:6]>>3);
      src2_s3[b]=_src2_s3[b] | (rdelm[b*6+:6]>>3);

      wrongbank_s3[b]=(`LO(rdelm[b*6+:6]>>3,1)!=b);

      vr_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src1_s3[b]>>1;
      vr_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[b]>>1;
      vr_a_en[b]=ctrl3_vr_a_en[b] && pipe_advance[3];
      vr_b_en[b]=ctrl3_vr_b_en[b] && pipe_advance[3];

      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]= 
        {(ctrl3_vf_a_sel[b]) ? _src1_s3[b][`VRID_RANGE] : imask_s3[b],
        src1_s3[b][`VRELM_RANGE]}>>1;
      vf_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[b]>>1;
      vf_a_en[b]=ctrl3_vf_a_en[b] && pipe_advance[3];
      vf_b_en[b]=ctrl3_vf_b_en[b] && pipe_advance[3];
    end

  // ************* Map from issuer/banks to Functional Units *************
  always@(posedge clk)
    if (!resetn)
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=0;
      ctrl4_volatiledest=0;
    end
    else if (pipe_advance[3])
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=|ctrl3_mem_en;
      ctrl4_volatiledest=0;

      for (f3=0; f3<NUMFUS; f3=f3+1)
      begin
        D_instr_s4[f3]=0;
        D_last_subvector_s4[f3]=0;
        dst_s4[f3]=0;
      end

      for (b3=0; b3<2; b3=b3+1)
      begin
        //if instruction is alive && is in correct bank
        if ( alive_s3[b3]  &&     //alive
            ~wrongbank_s3[b3] &&  //correct bank
            ~pipe_squash[3])
          if (ctrl3_mulshift_en[b3])    //is multiply
          begin
            D_instr_s4[FU_MUL]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MUL]=last_subvector[b3];
            ctrl4_mulshift_en=ctrl3_mulshift_en[b3];
            banksel_s4[FU_MUL]=b3;
            vs_s4[FU_MUL]=vs_s3[b3];
            vc_s4[FU_MUL]=vc_s3[b3];
            dst_s4[FU_MUL]=dst_s3[b3];
            dst_we_s4[FU_MUL]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MUL]=lane_en[b3];
            src1scalar_s4[FU_MUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MUL]=ctrl3_ismasked[b3];
            ctrl4_mulshift_op=ctrl3_mulshift_op[b3];
            ctrl4_rshiftnonzero=ctrl3_rshiftnonzero[b3];
          end
          else if (ctrl3_matmul_en[b3])    //is matmul
          begin
            D_instr_s4[FU_MATMUL]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MATMUL]=last_subvector[b3];
            ctrl4_matmul_en=ctrl3_matmul_en[b3];
            banksel_s4[FU_MATMUL]=b3;
            vs_s4[FU_MATMUL]=vs_s3[b3];
            vc_s4[FU_MATMUL]=vc_s3[b3];
            dst_s4[FU_MATMUL]=dst_s3[b3];
            dst_we_s4[FU_MATMUL]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MATMUL]=lane_en[b3];
            src1scalar_s4[FU_MATMUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MATMUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MATMUL]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfadder_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_BFADDER]=D_instr_s3[b3];
            D_last_subvector_s4[FU_BFADDER]=last_subvector[b3];
            ctrl4_bfadder_en=ctrl3_bfadder_en[b3];
            banksel_s4[FU_BFADDER]=b3;
            vs_s4[FU_BFADDER]=vs_s3[b3];
            vc_s4[FU_BFADDER]=vc_s3[b3];
            dst_s4[FU_BFADDER]=dst_s3[b3];
            dst_we_s4[FU_BFADDER]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFADDER]=lane_en[b3];
            src1scalar_s4[FU_BFADDER]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFADDER]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFADDER]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfmult_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_BFMULT]=D_instr_s3[b3];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_bfmult_en = ctrl3_bfmult_en[b3];
            banksel_s4[FU_BFMULT]=b3;
            vs_s4[FU_BFMULT]=vs_s3[b3];
            vc_s4[FU_BFMULT]=vc_s3[b3];
            dst_s4[FU_BFMULT]=dst_s3[b3];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFMULT]=lane_en[b3];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_trp_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_BFMULT]=D_instr_s3[b3];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[FU_BFMULT]=b3;
            vs_s4[FU_BFMULT]=vs_s3[b3];
            vc_s4[FU_BFMULT]=vc_s3[b3];
            dst_s4[FU_BFMULT]=dst_s3[b3];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFMULT]=lane_en[b3];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_act_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_TRP]=D_instr_s3[b3];
            D_last_subvector_s4[FU_TRP]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[FU_TRP]=b3;
            vs_s4[FU_TRP]=vs_s3[b3];
            vc_s4[FU_TRP]=vc_s3[b3];
            dst_s4[FU_TRP]=dst_s3[b3];
            dst_we_s4[FU_TRP]=ctrl3_vr_c_we[b3];
            vlane_en[FU_TRP]=lane_en[b3];
            src1scalar_s4[FU_TRP]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRP]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRP]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_memunit_en[b3] && !ctrl3_mem_en[b3]) //is memunit shift
          begin
            D_instr_s4[FU_MEM]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[FU_MEM]=b3;
            vs_s4[FU_MEM]=vs_s3[b3];
            dst_s4[FU_MEM]=dst_s3[b3];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MEM]=lane_en[b3];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[b3];
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_memunit_en[b3] &&  ctrl3_mem_en[b3]) //is mem operation
          begin
            D_instr_s4[FU_MEM]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[FU_MEM]=b3;
            // Use vs to store current length for prefetching
            vs_s4[FU_MEM]=count[b3*(6-3+1)+:(6-3)]<<3;
            dst_s4[FU_MEM]=dst_s3[b3];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MEM]=lane_en[b3];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[b3];
            //Load base on first subvector or if INDEXED memory operation
            vbase_s4=(|(first_subvector&ctrl3_mem_en) || ctrl_memunit_op[3][5])?
              vbase[3] : vbase_s4 + ((((ctrl_memunit_op[3][4])? vstride[3]: 1)
                      <<ctrl_memunit_op[3][3:2])<<3);
            // Partial Address Gen for each lane - just do multiplication part
            for (m=0; m<8; m=m+1)
              vstrideoffset_s4[m*32 +: 32] = 
                                ((ctrl_memunit_op[3][4]) ? vstride[3] : 1)*m;
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_alu_op[b3]!=(ALUOP_ZERO^ALUOP_ZERO)) //is ALU
          begin
            D_instr_s4[FU_ALU+b3*0]=D_instr_s3[b3];
            D_last_subvector_s4[FU_ALU+b3*0]=last_subvector[b3];
            banksel_s4[FU_ALU+b3*0]=b3;
            vs_s4[FU_ALU+b3*0]=vs_s3[b3];
            dst_we_s4[FU_ALU+b3*0]=ctrl3_vr_c_we[b3];
            dst_s4[FU_ALU+b3*0]=dst_s3[b3];
            dst_we_s4[FU_FALU+b3*0]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*0]=dst_s3[b3];
            vlane_en[FU_ALU+b3*0]=lane_en[b3];
            src1scalar_s4[FU_ALU+b3*0]=src1scalar_s3[b3];
            src2scalar_s4[FU_ALU+b3*0]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ALU+b3*0]=ctrl3_ismasked[b3];
            ctrl4_alu_op[b3*0]=ctrl3_alu_op[b3];
            ctrl4_satsum_op[b3*0]=ctrl3_satsum_op[b3];
            ctrl4_satsize_op[b3*0]=ctrl3_satsize_op[b3];
            ctrl4_vf_wbsel[b3*0]=ctrl3_vf_wbsel[b3];
          end
          else if (ctrl3_vf_c_we[b3])
          begin                                    //is FALU
            D_instr_s4[FU_FALU+b3*0]=D_instr_s3[b3];
            D_last_subvector_s4[FU_FALU+b3*0]=last_subvector[b3];
            banksel_s4[FU_FALU+b3*0]=b3;
            vs_s4[FU_FALU+b3*0]=|vs_s3[b3];
            dst_we_s4[FU_FALU+b3*0]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*0]=dst_s3[b3];
            vlane_en[FU_FALU+b3*0]=lane_en[b3];
            src1scalar_s4[FU_FALU+b3*0]=src1scalar_s3[b3];
            src2scalar_s4[FU_FALU+b3*0]=src2scalar_s3[b3];
            ctrl4_flagalu_op[b3*0]=ctrl3_flagalu_op[b3];
            ctrl4_vf_wbsel[b3*0]=ctrl3_vf_wbsel[b3];
            ctrl4_vf_a_sel[b3*0]=ctrl3_vf_a_sel[b3];
          end
        end
      end

// module instance
  vregfile_vector_2_1_128_256_8 vregfile_vector (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vr_a_reg),
      .a_readdataout(_vr_a_readdataout),
      .a_en(vr_a_en),
      .b_reg(vr_b_reg),
      .b_readdataout(_vr_b_readdataout),
      .b_en(vr_b_en),
      .c_reg(_vr_c_reg),
      .c_writedatain(_vr_c_writedatain),
      .c_byteen(vr_c_byteen),
      .c_we(vr_c_we));

// module instance
  vregfile_flag_2_1_8_256_8 
    vregfile_flag (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vf_a_reg),
      .a_readdataout(vf_a_readdataout),
      .a_en(vf_a_en),
      .b_reg(vf_b_reg),
      .b_readdataout(vf_b_readdataout),
      .b_en(vf_b_en),
      .c_reg(vf_c_reg),
      .c_writedatain(vf_c_writedatain),
      .c_we(vf_c_we));


// module instance
  pipereg_1 sdstwepipereg (
      .d( |ctrl3_vs_we ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[3] ),
      .squashn( ~pipe_squash[3] ),
      .q(ctrl_vs_we[4]));

/******************************************************************************/
/************************** 4th Pipeline Stage ********************************/
/******************************************************************************/

  //Convert register file width (8*2) to datpath width (32)
  always@*
    for (bn=0; bn<2; bn=bn+1)
      for (n=0; n<8; n=n+1)
      begin
        vr_a_readdataout[bn][32*n +: 32] = 
          _vr_a_readdataout[bn*8*2*8 + 8*2*n +: 8*2];
        vr_b_readdataout[bn][32*n +: 32] = 
          _vr_b_readdataout[bn*8*2*8 + 8*2*n +: 8*2];
        _vr_c_writedatain[bn*8*2*8 + 8*2*n +: 8*2] = 
          vr_c_writedatain[bn][32*n +: 32];
      end

  //Bank Multiplex for each functional unit
  always@*
  begin
    for (fn=0; fn<=FU_MUL; fn=fn+1)
    begin
      vr_src1[fn] =(src1scalar_s4[fn]) ? {8{vs_s4[fn][32-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[fn]];
      vr_src2[fn] =(src2scalar_s4[fn]) ? {8{vs_s4[fn][32-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[fn]];
      vf_src1[fn] = vf_a_readdataout[banksel_s4[fn]*8 +: 8];

      vmask[fn] =vlane_en[fn] &
            ((ctrl4_ismasked[fn]) ?
              vf_a_readdataout[banksel_s4[fn]*8 +: 8] :
              {8{1'b1}}) ;
    end

    vr_src1[FU_MEM] = vr_a_readdataout[banksel_s4[FU_MEM]];
    vr_src2[FU_MEM] = vr_b_readdataout[banksel_s4[FU_MEM]];
    vmask[FU_MEM] =  vlane_en[FU_MEM] &
           ((ctrl4_ismasked[FU_MEM]) ?  
             vf_a_readdataout[banksel_s4[FU_MEM]*8 +: 8] :
             {8{1'b1}}) ;

    vr_src1[FU_MATMUL] =(src1scalar_s4[FU_MATMUL]) ? {8{vs_s4[FU_MATMUL][32-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[FU_MATMUL]];
    vr_src2[FU_MATMUL] =(src2scalar_s4[FU_MATMUL]) ? {8{vs_s4[FU_MATMUL][32-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[FU_MATMUL]];
    vmask[FU_MATMUL] =  vlane_en[FU_MATMUL] &
           ((ctrl4_ismasked[FU_MATMUL]) ?  
             vf_a_readdataout[banksel_s4[FU_MATMUL]*8 +: 8] :
             {8{1'b1}}) ;

    for (fn2=FU_FALU; fn2<=FU_FALU+(2-1)*0; fn2=fn2+1)
    begin
      vf_src1[fn2] =(src1scalar_s4[fn2]&ctrl4_vf_a_sel[fn2-FU_FALU]) ? {8{vs_s4[fn2][0]}} :
        vf_a_readdataout[banksel_s4[fn2]*8 +: 8];

      //Only FALU uses this
      vf_src2[fn2] = (src2scalar_s4[fn2]) ? {8{vs_s4[fn2][0]}} : 
        vf_b_readdataout[banksel_s4[fn2]*8 +: 8];
    end

  end

/******************************************************************************/
/************************** 5th Pipeline Stage ********************************/
/******************************************************************************/

  // *********************** FUNCTIONAL UNITS *********************
  //
  // Note that functional units (recently) carry the register they're
  // writing to and a write signal so that once the result is computed
  // it can issue the full register write request.  As a result, it 
  // needs to know the status of the pipeline (pipe_advance and pipe_squash)

  // If mem_unit is stalled, pipeline can still go on if both
  //    i) it's a store hence requiring no writeback bank, AND
  //    ii) another memory instruction isn't waiting on the memunit
  assign stall_memunit=_stall_memunit && (dst_we[FU_MEM][5] ||
    (ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul));

//module instance 

  pipereg_1 memen5pipereg (
      .d( ctrl4_mem_en ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] && ~_stall_memunit ),
      .squashn( ~pipe_squash[4] | _stall_memunit ),
      .q(ctrl5_mem_en ));

 //============== Memory Unit =============

//module instance 

  vmem_unit_32_8_3_8_3_32_128_7_128_7_-3_8 vmem_unit(
    .clk(clk),
    .resetn(resetn),
    .enable(ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul),  //unit on and pipe active
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .op(ctrl4_memunit_op),
    .stall(_stall_memunit),
    .last_subvector(mem_last_subvector_s4),
    // Parameters ports
    .cbase(vbase_s4),
    .cstride(vstride[4]),
    .cprefetch(vs_s4[FU_MEM]),
    // Vector ports
    .vmask(vmask[FU_MEM]),
    .vstrideoffset(vstrideoffset_s4),
    .vindex(vr_src1[FU_MEM]),
    .vwritedata(vr_src2[FU_MEM]),
    .voutput(load_result_s5),
    .voutput_we(load_result_mask_s5),
    // Writeback ports
    .in_dst(dst_s4[FU_MEM]),
    .in_dst_we(dst_we_s4[FU_MEM]),
    .out_dst(dst[FU_MEM][5]),
    .out_dst_we(dst_we[FU_MEM][5]),
    .in_vs_dst_we(ctrl_vs_we[4]),
    .out_vs_dst_we(ctrl_vs_we[5]),
    // Vector operations ports
    .sa(vstride[4]),
    .dir_left(ctrl4_mem_dir_left),
    // Data memory interface
    .dmem_en(dbus_en),
    .dmem_address(dbus_address),
    .dmem_we(dbus_we),
    .dmem_byteen(dbus_byteen),
    .dmem_writedata(dbus_writedata),
    .dmem_readdata(dbus_readdata),
    .dmem_cachematch(dbus_cachematch),
    .dmem_cachemiss(dbus_cachemiss),
    .dmem_prefetch(dbus_prefetch),
    .dmem_wait(dbus_wait)
    );


  assign dst[FU_MEM][4]=dst_s4[FU_MEM];
  assign dst_we[FU_MEM][4]=dst_we_s4[FU_MEM];

  //============== Multiplier Unit (spans stages 4-6) =============

//module instance 

  vmul_unit_5_8_3_8 vmul_unit(
    .clk(clk),
    .resetn(resetn),
    .op(ctrl4_mulshift_op),
    .activate(ctrl4_mulshift_en),
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .stall(stall_mulunit),
    .opA(vr_src1[FU_MUL]),
    .opB(vr_src2[FU_MUL]),
    .vshamt( (ctrl4_rshiftnonzero) ? vc_s4[FU_MUL] : 0 ),
    .vmask(vmask[FU_MUL]),
    .in_dst(dst_s4[FU_MUL]),
    .in_dst_we(dst_we_s4[FU_MUL]),
    .out_dst({dst[FU_MUL][6],dst[FU_MUL][5],dst[FU_MUL][4]}),
    .out_dst_we(dst_we[FU_MUL][6:4]),
    .out_dst_mask({dst_mask[FU_MUL][6],dst_mask[FU_MUL][5],dst_mask[FU_MUL][4]}),
    .result(mulshift_result_s5)
  );

  //============== ALU Unit =============

  //If APB value is true, create one ALU per bank (per lane)

  wire squash_aludstpipe_NC;
wire squash_faludstpipe_NC;
wire squash_aludstmaskpipe_NC;


 //This code is just for assigning from signals connected
 //to the matmul, which are linear multi bit signals, to the
 //multi-dimensional signals.
 wire [`MATMUL_STAGES*REGIDWIDTH-1:0] dst_matmul;
 wire [`MATMUL_STAGES*8-1:0] dst_mask_matmul;



      assi0n dst[FU_MATMUL][0+4] = dst_matmul[REGIDWIDTH*(0+1)-1:REGIDWIDTH*0];
      assi0n dst_mask[FU_MATMUL][0+4] = dst_mask_matmul[8*(0+1)-1:8*0];



      assi1n dst[FU_MATMUL][1+4] = dst_matmul[REGIDWIDTH*(1+1)-1:REGIDWIDTH*1];
      assi1n dst_mask[FU_MATMUL][1+4] = dst_mask_matmul[8*(1+1)-1:8*1];



      assi2n dst[FU_MATMUL][2+4] = dst_matmul[REGIDWIDTH*(2+1)-1:REGIDWIDTH*2];
      assi2n dst_mask[FU_MATMUL][2+4] = dst_mask_matmul[8*(2+1)-1:8*2];



      assi3n dst[FU_MATMUL][3+4] = dst_matmul[REGIDWIDTH*(3+1)-1:REGIDWIDTH*3];
      assi3n dst_mask[FU_MATMUL][3+4] = dst_mask_matmul[8*(3+1)-1:8*3];



      assi4n dst[FU_MATMUL][4+4] = dst_matmul[REGIDWIDTH*(4+1)-1:REGIDWIDTH*4];
      assi4n dst_mask[FU_MATMUL][4+4] = dst_mask_matmul[8*(4+1)-1:8*4];



      assi5n dst[FU_MATMUL][5+4] = dst_matmul[REGIDWIDTH*(5+1)-1:REGIDWIDTH*5];
      assi5n dst_mask[FU_MATMUL][5+4] = dst_mask_matmul[8*(5+1)-1:8*5];



      assi6n dst[FU_MATMUL][6+4] = dst_matmul[REGIDWIDTH*(6+1)-1:REGIDWIDTH*6];
      assi6n dst_mask[FU_MATMUL][6+4] = dst_mask_matmul[8*(6+1)-1:8*6];



      assi7n dst[FU_MATMUL][7+4] = dst_matmul[REGIDWIDTH*(7+1)-1:REGIDWIDTH*7];
      assi7n dst_mask[FU_MATMUL][7+4] = dst_mask_matmul[8*(7+1)-1:8*7];



      assi8n dst[FU_MATMUL][8+4] = dst_matmul[REGIDWIDTH*(8+1)-1:REGIDWIDTH*8];
      assi8n dst_mask[FU_MATMUL][8+4] = dst_mask_matmul[8*(8+1)-1:8*8];



      assi9n dst[FU_MATMUL][9+4] = dst_matmul[REGIDWIDTH*(9+1)-1:REGIDWIDTH*9];
      assi9n dst_mask[FU_MATMUL][9+4] = dst_mask_matmul[8*(9+1)-1:8*9];



      assi10n dst[FU_MATMUL][10+4] = dst_matmul[REGIDWIDTH*(10+1)-1:REGIDWIDTH*10];
      assi10n dst_mask[FU_MATMUL][10+4] = dst_mask_matmul[8*(10+1)-1:8*10];



      assi11n dst[FU_MATMUL][11+4] = dst_matmul[REGIDWIDTH*(11+1)-1:REGIDWIDTH*11];
      assi11n dst_mask[FU_MATMUL][11+4] = dst_mask_matmul[8*(11+1)-1:8*11];



      assi12n dst[FU_MATMUL][12+4] = dst_matmul[REGIDWIDTH*(12+1)-1:REGIDWIDTH*12];
      assi12n dst_mask[FU_MATMUL][12+4] = dst_mask_matmul[8*(12+1)-1:8*12];



      assi13n dst[FU_MATMUL][13+4] = dst_matmul[REGIDWIDTH*(13+1)-1:REGIDWIDTH*13];
      assi13n dst_mask[FU_MATMUL][13+4] = dst_mask_matmul[8*(13+1)-1:8*13];



      assi14n dst[FU_MATMUL][14+4] = dst_matmul[REGIDWIDTH*(14+1)-1:REGIDWIDTH*14];
      assi14n dst_mask[FU_MATMUL][14+4] = dst_mask_matmul[8*(14+1)-1:8*14];



      assi15n dst[FU_MATMUL][15+4] = dst_matmul[REGIDWIDTH*(15+1)-1:REGIDWIDTH*15];
      assi15n dst_mask[FU_MATMUL][15+4] = dst_mask_matmul[8*(15+1)-1:8*15];



      assi16n dst[FU_MATMUL][16+4] = dst_matmul[REGIDWIDTH*(16+1)-1:REGIDWIDTH*16];
      assi16n dst_mask[FU_MATMUL][16+4] = dst_mask_matmul[8*(16+1)-1:8*16];



      assi17n dst[FU_MATMUL][17+4] = dst_matmul[REGIDWIDTH*(17+1)-1:REGIDWIDTH*17];
      assi17n dst_mask[FU_MATMUL][17+4] = dst_mask_matmul[8*(17+1)-1:8*17];



      assi18n dst[FU_MATMUL][18+4] = dst_matmul[REGIDWIDTH*(18+1)-1:REGIDWIDTH*18];
      assi18n dst_mask[FU_MATMUL][18+4] = dst_mask_matmul[8*(18+1)-1:8*18];



      assi19n dst[FU_MATMUL][19+4] = dst_matmul[REGIDWIDTH*(19+1)-1:REGIDWIDTH*19];
      assi19n dst_mask[FU_MATMUL][19+4] = dst_mask_matmul[8*(19+1)-1:8*19];



      assi20n dst[FU_MATMUL][20+4] = dst_matmul[REGIDWIDTH*(20+1)-1:REGIDWIDTH*20];
      assi20n dst_mask[FU_MATMUL][20+4] = dst_mask_matmul[8*(20+1)-1:8*20];



      assi21n dst[FU_MATMUL][21+4] = dst_matmul[REGIDWIDTH*(21+1)-1:REGIDWIDTH*21];
      assi21n dst_mask[FU_MATMUL][21+4] = dst_mask_matmul[8*(21+1)-1:8*21];



      assi22n dst[FU_MATMUL][22+4] = dst_matmul[REGIDWIDTH*(22+1)-1:REGIDWIDTH*22];
      assi22n dst_mask[FU_MATMUL][22+4] = dst_mask_matmul[8*(22+1)-1:8*22];



      assi23n dst[FU_MATMUL][23+4] = dst_matmul[REGIDWIDTH*(23+1)-1:REGIDWIDTH*23];
      assi23n dst_mask[FU_MATMUL][23+4] = dst_mask_matmul[8*(23+1)-1:8*23];



      assi24n dst[FU_MATMUL][24+4] = dst_matmul[REGIDWIDTH*(24+1)-1:REGIDWIDTH*24];
      assi24n dst_mask[FU_MATMUL][24+4] = dst_mask_matmul[8*(24+1)-1:8*24];



      assi25n dst[FU_MATMUL][25+4] = dst_matmul[REGIDWIDTH*(25+1)-1:REGIDWIDTH*25];
      assi25n dst_mask[FU_MATMUL][25+4] = dst_mask_matmul[8*(25+1)-1:8*25];



      assi26n dst[FU_MATMUL][26+4] = dst_matmul[REGIDWIDTH*(26+1)-1:REGIDWIDTH*26];
      assi26n dst_mask[FU_MATMUL][26+4] = dst_mask_matmul[8*(26+1)-1:8*26];



      assi27n dst[FU_MATMUL][27+4] = dst_matmul[REGIDWIDTH*(27+1)-1:REGIDWIDTH*27];
      assi27n dst_mask[FU_MATMUL][27+4] = dst_mask_matmul[8*(27+1)-1:8*27];



      assi28n dst[FU_MATMUL][28+4] = dst_matmul[REGIDWIDTH*(28+1)-1:REGIDWIDTH*28];
      assi28n dst_mask[FU_MATMUL][28+4] = dst_mask_matmul[8*(28+1)-1:8*28];



///////////////////////////
// Matmul unit
///////////////////////////

//module instance 

matmul_unit_8_33_8 u_matmul(
.clk(clk),
.resetn(resetn),
.activate(ctrl4_matmul_en),
.en(pipe_advance[`MAX_PIPE_STAGES-1:4]),
.squash(pipe_squash[`MAX_PIPE_STAGES-1:4]),
.stall(stall_matmul),
.a_data(vr_src1[FU_MATMUL][8*32-1:0]),
.b_data(vr_src2[FU_MATMUL][8*32-1:0]),
.validity_mask_a_rows(matmul_masks_in[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE]),
.validity_mask_a_cols(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_rows(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_cols(matmul_masks_in[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE]),
.c_data(matmul_out), 
.vmask(vmask[FU_MATMUL]),
.in_dst(dst_s4[FU_MATMUL]),
.in_dst_we(dst_we_s4[FU_MATMUL]),
.out_dst(dst_matmul),
.out_dst_we(dst_we[FU_MATMUL][`MAX_PIPE_STAGES-1:4]),
.out_dst_mask(dst_mask_matmul)
);

//module instance 

trp_unit_8 u_trp (
.clk(clk),
.resetn(resetn),
.en(ctrl4_trp_en),
.a(vr_src1[FU_TRP][8*32-1:0]),
.mode(),
.read(),
.busy(),
.valid(),
.out(trp_out)
);


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][0 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][0 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[0*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][0 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][0 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[0*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][0 * LANEWIDTHE +: 32]),
    .out(act_result_s5[0*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][1 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][1 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[1*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][1 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][1 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[1*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][1 * LANEWIDTHE +: 32]),
    .out(act_result_s5[1*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][2 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][2 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[2*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][2 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][2 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[2*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][2 * LANEWIDTHE +: 32]),
    .out(act_result_s5[2*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][3 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][3 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[3*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][3 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][3 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[3*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][3 * LANEWIDTHE +: 32]),
    .out(act_result_s5[3*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_4(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][4 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][4 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[4*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_4(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][4 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][4 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[4*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_4(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][4 * LANEWIDTHE +: 32]),
    .out(act_result_s5[4*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_5(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][5 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][5 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[5*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_5(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][5 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][5 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[5*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_5(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][5 * LANEWIDTHE +: 32]),
    .out(act_result_s5[5*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_6(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][6 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][6 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[6*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_6(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][6 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][6 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[6*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_6(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][6 * LANEWIDTHE +: 32]),
    .out(act_result_s5[6*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_8 bf_add_7(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][7 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFADDER][7 * LANEWIDTHE +: 32]),
    .out(bfadder_result_s5[7*32 +: 32])
    );
    
//module instance 
    bfloat_mult_8 bf_mult_7(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][7 * LANEWIDTHE +: 32]),
    .b(vr_src2[FU_BFMULT][7 * LANEWIDTHE +: 32]),
    .out(bfmult_result_s5[7*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_8 inst_activation_7(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][7 * LANEWIDTHE +: 32]),
    .out(act_result_s5[7*32 +: 32])
    );



/******************************************************************************/
/************************** WB Pipeline Stage ********************************/
/******************************************************************************/

    assign wb_dst[FU_ALU+0]=dst[FU_ALU+0][5];
    assign wb_dst_we[FU_ALU+0]=dst_we[FU_ALU+0][5] && ~pipe_squash[5];
    assign wb_dst_mask[FU_ALU+0]=dst_mask[FU_ALU+0][5];
    assign D_wb_last_subvector[FU_ALU+0]=D_last_subvector_s5[FU_ALU+0];

    assign D_wb_last_subvector[FU_FALU+0]=D_last_subvector_s5[FU_FALU+0];



  assign wb_dst[FU_MEM]=dst[FU_MEM][5];
  assign wb_dst_we[FU_MEM]=dst_we[FU_MEM][5] && (~pipe_advance[5]|~pipe_squash[5]);
  assign wb_dst_mask[FU_MEM]=load_result_mask_s5;
  assign D_wb_last_subvector[FU_MEM]=D_last_subvector_s5[FU_MEM];

  assign wb_dst[FU_MUL]=dst[FU_MUL][5];
  assign wb_dst_we[FU_MUL]=dst_we[FU_MUL][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MUL]=dst_mask[FU_MUL][5];
  assign D_wb_last_subvector[FU_MUL]=D_last_subvector_s5[FU_MUL];


  assign wb_dst[FU_BFADDER] = dst[FU_BFADDER][5];
  assign wb_dst_we[FU_BFADDER] = dst_we[FU_BFADDER][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_BFADDER] = dst_mask[FU_BFADDER][5];
  assign D_wb_last_subvector[FU_BFADDER] = D_last_subvector_s5[FU_BFADDER];

  assign wb_dst[FU_BFMULT] = dst[FU_BFMULT][5];
  assign wb_dst_we[FU_BFMULT] = dst_we[FU_BFMULT][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_BFMULT] = dst_mask[FU_BFMULT][5];
  assign D_wb_last_subvector[FU_BFMULT] = D_last_subvector_s5[FU_BFMULT];

  assign wb_dst[FU_ACT] = dst[FU_ACT][5];
  assign wb_dst_we[FU_ACT] = dst_we[FU_ACT][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_ACT] = dst_mask[FU_ACT][5];
  assign D_wb_last_subvector[FU_ACT] = D_last_subvector_s5[FU_ACT];

  assign wb_dst[FU_MATMUL]=dst[FU_MATMUL][5];
  assign wb_dst_we[FU_MATMUL]=dst_we[FU_MATMUL][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MATMUL]=dst_mask[FU_MATMUL][5];
  //TODO: There is no code that assigns to the s31 var used below. Need to add that code
  //This is only a debug var, so it doesn't affect functionality
  assign D_wb_last_subvector[FU_MATMUL]=D_last_subvector_s31[FU_MATMUL];

  // ******************  Map functional units to banks ******************
  always@*
    for (bw=0; bw<2; bw=bw+1)
    begin
      vr_c_we[bw]=(wb_dst_we[FU_MUL] && `LO(wb_dst[FU_MUL],1)==bw) ||
                  (wb_dst_we[FU_MATMUL] && `LO(wb_dst[FU_MATMUL],1)==bw) ||
                  (wb_dst_we[FU_BFADDER] && `LO(wb_dst[FU_BFADDER],1)==bw) ||
                  (wb_dst_we[FU_BFMULT] && `LO(wb_dst[FU_BFMULT],1)==bw) ||
                  (wb_dst_we[FU_ACT] && `LO(wb_dst[FU_ACT],1)==bw) ||
                  (wb_dst_we[FU_MEM] && `LO(wb_dst[FU_MEM],1)==bw) ||
                  (wb_dst_we[FU_ALU] && `LO(wb_dst[FU_ALU],1)==bw && 0==0) ||
                  (wb_dst_we[FU_ALU+bw] && 0!=0);

      //TODO: Update this code for matmul. This is for debug only, so skipping it for now.
      //Tells test_bench when to record register file contents.
      //Record if instruction writes to VRF, on last subvector, and not stalled
      D_wb_instrdone[bw] = pipe_advance[5] && (
        ((0==0) ?
          (dst_we[FU_ALU][5] && D_wb_last_subvector[FU_ALU] && `LO(wb_dst[FU_ALU],1)==bw) :
          (dst_we[FU_ALU+bw][5] && D_wb_last_subvector[FU_ALU+bw])) || 
        (dst_we[FU_MUL][5] && D_wb_last_subvector[FU_MUL] && `LO(wb_dst[FU_MUL],1)==bw) || 
        (dst_we[FU_MEM][5] && D_wb_last_subvector[FU_MEM] && `LO(wb_dst[FU_MEM],1)==bw));

      //Take matmul output
      if (wb_dst_we[FU_MATMUL] && `LO(wb_dst[FU_MATMUL],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MATMUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MATMUL]>>1;
        vr_c_reg[bw]= wb_dst[FU_MATMUL];
        vr_c_writedatain[bw]= matmul_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MATMUL];
      end      
      else if (wb_dst_we[FU_TRP] && `LO(wb_dst[FU_TRP],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_TRP];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_TRP]>>1;
        vr_c_reg[bw]= wb_dst[FU_TRP];
        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRP];
      end
      //Take bfadder output
      else if (wb_dst_we[FU_BFADDER] && `LO(wb_dst[FU_BFADDER],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFADDER];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFADDER]>>1;
        vr_c_reg[bw]= wb_dst[FU_BFADDER];
        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFADDER];
      end
      else if (wb_dst_we[FU_BFMULT] && `LO(wb_dst[FU_BFMULT],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFMULT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFMULT]>>1;
        vr_c_reg[bw]= wb_dst[FU_BFMULT];
        vr_c_writedatain[bw]= bfmult_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFMULT];
      end
      else if (wb_dst_we[FU_ACT] && `LO(wb_dst[FU_ACT],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_ACT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ACT]>>1;
        vr_c_reg[bw]= wb_dst[FU_ACT];
        vr_c_writedatain[bw]= act_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_ACT];
      end
      //Take multiplier output
      else if (wb_dst_we[FU_MUL] && `LO(wb_dst[FU_MUL],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MUL]>>1;
        vr_c_reg[bw]= wb_dst[FU_MUL];
        vr_c_writedatain[bw]= mulshift_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MUL];
      end
      //Take Memory unit output
      else if (wb_dst_we[FU_MEM] && `LO(wb_dst[FU_MEM],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MEM];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MEM]>>1;
        vr_c_reg[bw]= wb_dst[FU_MEM];
        vr_c_writedatain[bw]= load_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MEM];
      end
      else
      //Take ALU output
      begin
        vmask_final[bw]=wb_dst_mask[FU_ALU+bw*0];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ALU+bw*0]>>1;
        vr_c_reg[bw]= wb_dst[FU_ALU+bw*0];
        vr_c_writedatain[bw]= alu_result_s5[bw*0];
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=
          (D_wb_last_subvector[FU_ALU+bw*0] && `LO(wb_dst[FU_ALU+bw*0],1)==bw) |
          (D_wb_last_subvector[FU_FALU+bw*0] && `LO(dst[FU_FALU+bw*0][5],1)==bw);
      end


      //Generate byte enable from mask
      for (j=0; j<8; j=j+1)
        vr_c_byteen[bw*2*8 + j*2 +: 2 ]={2{vmask_final[bw][j]}};

      //*********** Flag writeback ***********
      vf_c_reg[bw*BANKREGIDWIDTH+:BANKREGIDWIDTH]=dst[FU_FALU+bw*0][5]>>1;
      vf_c_writedatain[bw*8+:8]=flagalu_result_s5[bw*0];
      vf_c_we[bw]=dst_we[FU_FALU+bw*0][5] && `LO(dst[FU_FALU+bw*0][5],1)==bw;
    end

  //********** Scalar writeback ***********
  assign vs_wetrack={ctrl_vs_we[5:4],|ctrl3_vs_we,ctrl2_vs_we};
  assign vs_we=ctrl_vs_we[5] & load_result_mask_s5[0];
  assign vs_dst=wb_dst[FU_MEM][`VRID_RANGE];
  assign vs_writedata=load_result_s5[32-1:0];

endmodule


/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_vector_2_1_128_256_8
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_byteen, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2*7-1:0] a_reg,b_reg,c_reg;
output [2*128-1:0] a_readdataout, b_readdataout;
input [2*128-1:0] c_writedatain;
input [((128>=8) ? 2*128/8-1 : 2-1):0] c_byteen;
input [2-1:0] c_we;

  genvar k;

  generate
  for (k=0; k<2; k=k+1)
  begin : bank_gen
          ram_wrapper_7_128_128 reg_file1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[k*7 +: 7]),
  	    .address_b(a_reg[k*7 +: 7]),
  	    .wren_a(c_we[k] & ((128>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*128 +: 128]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(a_readdataout[k*128 +: 128])
          );
  
          ram_wrapper_7_128_128 reg_file2(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[k*7 +: 7]),
  	    .address_b(b_reg[k*7 +: 7]),
  	    .wren_a(c_we[k] & ((128>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*128 +: 128]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(b_readdataout[k*128 +: 128])
          );
  end
  endgenerate

endmodule

module local_add_sub_34_0_SIGNED(
dataa,
datab,
cin,
add_sub,
result
);

input[34-1:0] dataa;
input[34-1:0] datab;
input cin;
input add_sub;
output reg [34-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule
/************************
 *
 *************************/

module vmem_crossbar_128_7_8_32_5 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-5;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDfTH*8)-1 : 0] sel;
input  [128-1 : 0]            in;
output [(32*8)-1 : 0] out;


     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(0+1)*SELWIDTH - 1 : 0*SELWIDTH],
        0n,
        out[(0+1)*32 - 1 : 0*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(1+1)*SELWIDTH - 1 : 1*SELWIDTH],
        1n,
        out[(1+1)*32 - 1 : 1*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(2+1)*SELWIDTH - 1 : 2*SELWIDTH],
        2n,
        out[(2+1)*32 - 1 : 2*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(3+1)*SELWIDTH - 1 : 3*SELWIDTH],
        3n,
        out[(3+1)*32 - 1 : 3*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(4+1)*SELWIDTH - 1 : 4*SELWIDTH],
        4n,
        out[(4+1)*32 - 1 : 4*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(5+1)*SELWIDTH - 1 : 5*SELWIDTH],
        5n,
        out[(5+1)*32 - 1 : 5*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(6+1)*SELWIDTH - 1 : 6*SELWIDTH],
        6n,
        out[(6+1)*32 - 1 : 6*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(7+1)*SELWIDTH - 1 : 7*SELWIDTH],
        7n,
        out[(7+1)*32 - 1 : 7*32]);

 endmodule 



module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 6=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 6-1:0 ] valuetoadd;

input [ 6-1:0 ] inshift_data;

input [ 2*6-1:0 ]  inparallel_data;
output [ 2*6-1:0 ] outparallel_data;

wire [ 6-1:0 ]  shiftin_right;

reg [ 2*6-1:0 ] outparallel_data;
reg [ 2*6-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*6-1:(2-1)*6];

  reg [ 6-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*6 +: 6] != {6{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] + v;
      else
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<6) | shiftin_right;
      


endmodule



module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 6=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 6-1:0 ] valuetoadd;

input [ 6-1:0 ] inshift_data;

input [ 2*6-1:0 ]  inparallel_data;
output [ 2*6-1:0 ] outparallel_data;

wire [ 6-1:0 ]  shiftin_right;

reg [ 2*6-1:0 ] outparallel_data;
reg [ 2*6-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*6-1:(2-1)*6];

  reg [ 6-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*6 +: 6] != {6{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] + v;
      else
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<6) | shiftin_right;
      


endmodule



module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 4=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 4-1:0 ] valuetoadd;

input [ 4-1:0 ] inshift_data;

input [ 2*4-1:0 ]  inparallel_data;
output [ 2*4-1:0 ] outparallel_data;

wire [ 4-1:0 ]  shiftin_right;

reg [ 2*4-1:0 ] outparallel_data;
reg [ 2*4-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*4-1:(2-1)*4];

  reg [ 4-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*4 +: 4] != {4{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*4 +: 4]=outparallel_data[i*4+:4] + v;
      else
        outparallel_added[i*4 +: 4]=outparallel_data[i*4+:4] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<4) | shiftin_right;
      


endmodule


/****************************************************************************
          Scalar Register File

   - Has one read port (a) and one write port (c)
   - vs0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_scalar_32_32_5 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input a_en;
input [5-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

    ram_wrapper_5_32_32 reg_file1(
        .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en),
        .address_a(c_reg[5-1:0]),
        .address_b(a_reg[5-1:0]),
        .wren_a(c_we & (|c_reg)),
        .wren_b(1'b0),
        .data_a(c_writedatain),
        .data_b(0),
        .out_a(),
        .out_b(a_readdataout)
    );

endmodule
        

/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_2_151 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter 2=4;      
parameter 151=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 2-1:0 ]  squash;

input [ 2*151-1:0 ]  inpipe;
output [ 2*151-1:0 ] outpipe;

input [ 151-1:0 ]  shiftin_left;
input [ 151-1:0 ]  shiftin_right;

wire [ (2+1)*151-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_151 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[151-1:0],
      _outpipe[((2>1) ? 151 : 0)+:151], //Support 1 lane
      shiftin_right,
      _outpipe[151-1:0]);
 // defparam velmshifter_laneunit0.151=151;

  //Generate everything in between 


      velmsh0fter_laneun0t_151 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*151-1:0*151],
          _outp0pe[(0+2)*151-1:(0+1)*151],
          _outp0pe[(0)*151-1:(0-1)*151],
          _outp0pe[(0+1)*151-1:0*151]);
     // defparam velmsh0fter_laneun0t.151=151;


      velmsh1fter_laneun1t_151 velmsh1fter_laneun1t(clk,resetn,load,sh1ft,d1r_left,
          squash[1],
          1np1pe[(1+1)*151-1:1*151],
          _outp1pe[(1+2)*151-1:(1+1)*151],
          _outp1pe[(1)*151-1:(1-1)*151],
          _outp1pe[(1+1)*151-1:1*151]);
     // defparam velmsh1fter_laneun1t.151=151;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_151 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[2-1],
      inpipe[2*151-1:(2-1)*151],
      shiftin_left,
      _outpipe[(((2>1) ? 2:2)-2)*151 +: 151], //L=1
      _outpipe[((2>1) ? (2-1)*151 : 151) +: 151]); //L=1
   // defparam velmshifter_laneunitlast.151=151;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_2_1 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter 2=4;      
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 2-1:0 ]  squash;

input [ 2*1-1:0 ]  inpipe;
output [ 2*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (2+1)*1-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_1 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[1-1:0],
      _outpipe[((2>1) ? 1 : 0)+:1], //Support 1 lane
      shiftin_right,
      _outpipe[1-1:0]);
 // defparam velmshifter_laneunit0.1=1;

  //Generate everything in between 


      velmsh0fter_laneun0t_1 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*1-1:0*1],
          _outp0pe[(0+2)*1-1:(0+1)*1],
          _outp0pe[(0)*1-1:(0-1)*1],
          _outp0pe[(0+1)*1-1:0*1]);
     // defparam velmsh0fter_laneun0t.1=1;


      velmsh1fter_laneun1t_1 velmsh1fter_laneun1t(clk,resetn,load,sh1ft,d1r_left,
          squash[1],
          1np1pe[(1+1)*1-1:1*1],
          _outp1pe[(1+2)*1-1:(1+1)*1],
          _outp1pe[(1)*1-1:(1-1)*1],
          _outp1pe[(1+1)*1-1:1*1]);
     // defparam velmsh1fter_laneun1t.1=1;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[2-1],
      inpipe[2*1-1:(2-1)*1],
      shiftin_left,
      _outpipe[(((2>1) ? 2:2)-2)*1 +: 1], //L=1
      _outpipe[((2>1) ? (2-1)*1 : 1) +: 1]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_8_1 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter 8=4;      
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 8-1:0 ]  squash;

input [ 8*1-1:0 ]  inpipe;
output [ 8*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (8+1)*1-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_1 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[1-1:0],
      _outpipe[((8>1) ? 1 : 0)+:1], //Support 1 lane
      shiftin_right,
      _outpipe[1-1:0]);
 // defparam velmshifter_laneunit0.1=1;

  //Generate everything in between 


      velmsh0fter_laneun0t_1 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*1-1:0*1],
          _outp0pe[(0+2)*1-1:(0+1)*1],
          _outp0pe[(0)*1-1:(0-1)*1],
          _outp0pe[(0+1)*1-1:0*1]);
     // defparam velmsh0fter_laneun0t.1=1;


      velmsh1fter_laneun1t_1 velmsh1fter_laneun1t(clk,resetn,load,sh1ft,d1r_left,
          squash[1],
          1np1pe[(1+1)*1-1:1*1],
          _outp1pe[(1+2)*1-1:(1+1)*1],
          _outp1pe[(1)*1-1:(1-1)*1],
          _outp1pe[(1+1)*1-1:1*1]);
     // defparam velmsh1fter_laneun1t.1=1;


      velmsh2fter_laneun2t_1 velmsh2fter_laneun2t(clk,resetn,load,sh2ft,d2r_left,
          squash[2],
          2np2pe[(2+1)*1-1:2*1],
          _outp2pe[(2+2)*1-1:(2+1)*1],
          _outp2pe[(2)*1-1:(2-1)*1],
          _outp2pe[(2+1)*1-1:2*1]);
     // defparam velmsh2fter_laneun2t.1=1;


      velmsh3fter_laneun3t_1 velmsh3fter_laneun3t(clk,resetn,load,sh3ft,d3r_left,
          squash[3],
          3np3pe[(3+1)*1-1:3*1],
          _outp3pe[(3+2)*1-1:(3+1)*1],
          _outp3pe[(3)*1-1:(3-1)*1],
          _outp3pe[(3+1)*1-1:3*1]);
     // defparam velmsh3fter_laneun3t.1=1;


      velmsh4fter_laneun4t_1 velmsh4fter_laneun4t(clk,resetn,load,sh4ft,d4r_left,
          squash[4],
          4np4pe[(4+1)*1-1:4*1],
          _outp4pe[(4+2)*1-1:(4+1)*1],
          _outp4pe[(4)*1-1:(4-1)*1],
          _outp4pe[(4+1)*1-1:4*1]);
     // defparam velmsh4fter_laneun4t.1=1;


      velmsh5fter_laneun5t_1 velmsh5fter_laneun5t(clk,resetn,load,sh5ft,d5r_left,
          squash[5],
          5np5pe[(5+1)*1-1:5*1],
          _outp5pe[(5+2)*1-1:(5+1)*1],
          _outp5pe[(5)*1-1:(5-1)*1],
          _outp5pe[(5+1)*1-1:5*1]);
     // defparam velmsh5fter_laneun5t.1=1;


      velmsh6fter_laneun6t_1 velmsh6fter_laneun6t(clk,resetn,load,sh6ft,d6r_left,
          squash[6],
          6np6pe[(6+1)*1-1:6*1],
          _outp6pe[(6+2)*1-1:(6+1)*1],
          _outp6pe[(6)*1-1:(6-1)*1],
          _outp6pe[(6+1)*1-1:6*1]);
     // defparam velmsh6fter_laneun6t.1=1;


      velmsh7fter_laneun7t_1 velmsh7fter_laneun7t(clk,resetn,load,sh7ft,d7r_left,
          squash[7],
          7np7pe[(7+1)*1-1:7*1],
          _outp7pe[(7+2)*1-1:(7+1)*1],
          _outp7pe[(7)*1-1:(7-1)*1],
          _outp7pe[(7+1)*1-1:7*1]);
     // defparam velmsh7fter_laneun7t.1=1;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[8-1],
      inpipe[8*1-1:(8-1)*1],
      shiftin_left,
      _outpipe[(((8>1) ? 8:2)-2)*1 +: 1], //L=1
      _outpipe[((8>1) ? (8-1)*1 : 1) +: 1]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_256 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter 1=4;      
parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*256-1:0 ]  inpipe;
output [ 1*256-1:0 ] outpipe;

input [ 256-1:0 ]  shiftin_left;
input [ 256-1:0 ]  shiftin_right;

wire [ (1+1)*256-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_256 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[256-1:0],
      _outpipe[((1>1) ? 256 : 0)+:256], //Support 1 lane
      shiftin_right,
      _outpipe[256-1:0]);
 // defparam velmshifter_laneunit0.256=256;

  //Generate everything in between 


      velmsh0fter_laneun0t_256 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*256-1:0*256],
          _outp0pe[(0+2)*256-1:(0+1)*256],
          _outp0pe[(0)*256-1:(0-1)*256],
          _outp0pe[(0+1)*256-1:0*256]);
     // defparam velmsh0fter_laneun0t.256=256;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_256 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*256-1:(1-1)*256],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*256 +: 256], //L=1
      _outpipe[((1>1) ? (1-1)*256 : 256) +: 256]); //L=1
   // defparam velmshifter_laneunitlast.256=256;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_8 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter 1=4;      
parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*8-1:0 ]  inpipe;
output [ 1*8-1:0 ] outpipe;

input [ 8-1:0 ]  shiftin_left;
input [ 8-1:0 ]  shiftin_right;

wire [ (1+1)*8-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_8 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[8-1:0],
      _outpipe[((1>1) ? 8 : 0)+:8], //Support 1 lane
      shiftin_right,
      _outpipe[8-1:0]);
 // defparam velmshifter_laneunit0.8=8;

  //Generate everything in between 


      velmsh0fter_laneun0t_8 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*8-1:0*8],
          _outp0pe[(0+2)*8-1:(0+1)*8],
          _outp0pe[(0)*8-1:(0-1)*8],
          _outp0pe[(0+1)*8-1:0*8]);
     // defparam velmsh0fter_laneun0t.8=8;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_8 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*8-1:(1-1)*8],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*8 +: 8], //L=1
      _outpipe[((1>1) ? (1-1)*8 : 8) +: 8]); //L=1
   // defparam velmshifter_laneunitlast.8=8;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_256 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter 1=4;      
parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*256-1:0 ]  inpipe;
output [ 1*256-1:0 ] outpipe;

input [ 256-1:0 ]  shiftin_left;
input [ 256-1:0 ]  shiftin_right;

wire [ (1+1)*256-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_256 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[256-1:0],
      _outpipe[((1>1) ? 256 : 0)+:256], //Support 1 lane
      shiftin_right,
      _outpipe[256-1:0]);
 // defparam velmshifter_laneunit0.256=256;

  //Generate everything in between 


      velmsh0fter_laneun0t_256 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*256-1:0*256],
          _outp0pe[(0+2)*256-1:(0+1)*256],
          _outp0pe[(0)*256-1:(0-1)*256],
          _outp0pe[(0+1)*256-1:0*256]);
     // defparam velmsh0fter_laneun0t.256=256;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_256 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*256-1:(1-1)*256],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*256 +: 256], //L=1
      _outpipe[((1>1) ? (1-1)*256 : 256) +: 256]); //L=1
   // defparam velmshifter_laneunitlast.256=256;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_8 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter 1=4;      
parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*8-1:0 ]  inpipe;
output [ 1*8-1:0 ] outpipe;

input [ 8-1:0 ]  shiftin_left;
input [ 8-1:0 ]  shiftin_right;

wire [ (1+1)*8-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_8 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[8-1:0],
      _outpipe[((1>1) ? 8 : 0)+:8], //Support 1 lane
      shiftin_right,
      _outpipe[8-1:0]);
 // defparam velmshifter_laneunit0.8=8;

  //Generate everything in between 


      velmsh0fter_laneun0t_8 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*8-1:0*8],
          _outp0pe[(0+2)*8-1:(0+1)*8],
          _outp0pe[(0)*8-1:(0-1)*8],
          _outp0pe[(0+1)*8-1:0*8]);
     // defparam velmsh0fter_laneun0t.8=8;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_8 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*8-1:(1-1)*8],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*8 +: 8], //L=1
      _outpipe[((1>1) ? (1-1)*8 : 8) +: 8]); //L=1
   // defparam velmshifter_laneunitlast.8=8;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        

module vdispatcher_2_151_6_6_4(
    clk,
    resetn,

    shift,
    rotate, //shift must also be high to rotate

    inshift_instr,
    inshift_first,
    inshift_rdelm,
    inshift_wrelm,
    inshift_count,

    increment,
    rdelm_add_sub,
    wrelm_add_sub,
    count_add_sub,
    rdelm_valuetoadd,
    wrelm_valuetoadd,
    count_valuetoadd,

    instr,
    first,
    rdelm,
    wrelm,
    count

    );

input clk;
input resetn;

input  shift;
input  rotate;

input [ 151-1:0 ] inshift_instr;
input                    inshift_first;
input [ 6-1:0 ] inshift_rdelm;
input [ 6-1:0 ] inshift_wrelm;
input [ 4-1:0 ] inshift_count;

input [ 2-1:0 ] increment;

input rdelm_add_sub;
input wrelm_add_sub;
input count_add_sub;
input [ 6-1:0 ] rdelm_valuetoadd;
input [ 6-1:0 ] wrelm_valuetoadd;
input [ 4-1:0 ] count_valuetoadd;

output [ 2*151-1:0 ] instr;
output            [ 2-1:0 ] first;
output [ 2*6-1:0 ] rdelm;
output [ 2*6-1:0 ] wrelm;
output [ 2*4-1:0 ] count;

wire [2*151-1:0 ] inparallel_data_inst_NC;
vdispatcher_shift_2_151 vdispatcher_instr (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash({2{1'b0}}),
      .inshift_data(inshift_instr),
      .inparallel_data(inparallel_data_inst_NC),
      .outparallel_data(instr));

wire [ 2-1:0 ] inparallel_data_first_NC;
vdispatcher_shift_2_1 vdispatcher_first (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash(increment&~{32'b0,shift&~rotate}),
      .inshift_data(inshift_first),
      .inparallel_data(inparallel_data_first_NC),
      .outparallel_data(first));

wire [ 2*6-1:0 ] inparallel_data_rdelm_NC;
wire [ 2-1:0 ] squash_rdelm_NC;
vdispatcher_add_2_6 vdispatcher_rdelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_rdelm_NC),
      .add_sub(rdelm_add_sub),
      .valuetoadd(rdelm_valuetoadd),
      .inshift_data(inshift_rdelm),
      .inparallel_data(inparallel_data_rdelm_NC),
      .outparallel_data(rdelm));

wire [ 2*6-1:0 ] inparallel_data_wrelm_NC;
wire [ 2-1:0 ] squash_wrelm_NC;
vdispatcher_add_2_6 vdispatcher_wrelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_wrelm_NC),
      .add_sub(wrelm_add_sub),
      .valuetoadd(wrelm_valuetoadd),
      .inshift_data(inshift_wrelm),
      .inparallel_data(inparallel_data_wrelm_NC),
      .outparallel_data(wrelm));

wire [ 2*4-1:0 ] inparallel_data_count_NC;
wire [ 2-1:0 ] squash_count_NC;
vdispatcher_add_2_4 vdispatcher_count(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_count_NC),
      .add_sub(count_add_sub),
      .valuetoadd(count_valuetoadd),
      .inshift_data(inshift_count),
      .inparallel_data(inparallel_data_count_NC),
      .outparallel_data(count));


endmodule


module bfloat_mult_8 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [8-1:0] a,
 input [8-1:0] b,
 output reg[8-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a * b;
end

endmodule
        
/******************************************************************************

0  VFAND
1  VFOR
2  VFXOR
3  VFNOR
4  VFCLR
5  VFSET

Not handled here:
  VIOTA
  VCIOTA
  VFPOP
  VFFF1
  VFFL1
  VFSETBF
  VFSETIF
  VFSETOF

******************************************************************************/

module vlane_flagalu (
    clk,
    resetn,

    src1,
    src2,

    op,

    result

    );

input clk;
input resetn;

input src1;
input src2;
input [2:0] op;

output result;

reg result;

  always@*
    case(op)
      0: result=src1&src2;
      1: result=src1|src2;
      2: result=src1^src2;
      3: result=~(src1|src2);
      4: result=1'b0;
      5: result=1'b1;
      default: result=1'b0;
    endcase

endmodule
/****************************************************************************
          Stride Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_stride_32_8_3.0 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input a_en;
input [3.0-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

        ram_wrapper_3.0_8_32 reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[3.0-1:0]),
	    .address_b(a_reg[3.0-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
endmodule
        


/****************************************************************************
 *           Load data translator
 *- moves read data to appropriate byte/halfword and zero/sign extends
 *****************************************************************************/
module vstore_data_translator(
    write_data,             // data in least significant position
    d_address,
    store_size,             // 0-byte, 1-16bits, 2-32bits, 3-64bits
    d_byteena,
    d_writedataout);        // shifted data to coincide with address
parameter WIDTH=32;

input [WIDTH-1:0] write_data;
input [1:0] d_address;
input [1:0] store_size;
output [3:0] d_byteena;
output [WIDTH-1:0] d_writedataout;

reg [3:0] d_byteena;
reg [WIDTH-1:0] d_writedataout;

always @*
begin
    case (store_size)
        2'b00:
        begin
            case(d_address[1:0])
                2'b00: begin d_byteena=4'b1000; 
                        d_writedataout={write_data[7:0],24'b0}; end
                2'b01: begin d_byteena=4'b0100;
                        d_writedataout={8'b0,write_data[7:0],16'b0}; end
                2'b10: begin d_byteena=4'b0010;
                        d_writedataout={16'b0,write_data[7:0],8'b0}; end
                default: begin d_byteena=4'b0001;
                        d_writedataout={24'b0,write_data[7:0]}; end
            endcase
        end
        2'b01:
        begin
            d_writedataout=(d_address[1]) ? {16'b0,write_data[15:0]} : 
                                            {write_data[15:0],16'b0};
            d_byteena=(d_address[1]) ? 4'b0011 : 4'b1100 ;
        end
        default:
        begin
            d_byteena=4'b1111;
            d_writedataout=write_data;
        end
    endcase
end

endmodule

                  
module local_shifter_2_2_ARITHMATIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMATIC";

input [2-1:0] data;
input [2-1:0] distance;
input direction;

output reg [2-1:0] result;
reg [2-1:0] arith_reg; 
always @* begin
  arith_reg = {2{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[2-1] == 1'b1)
          result =  ((arith_reg <<(2 - distance))|| (data >> distance));
      else
          result = data >> distance;
    end
  end
  else begin
    if(direction == 1'b0)begin
        result = data << distance;
    end
    else begin
        result = data >> distance;
    end
  end 
end
endmodulemodule local_shifter_2_2_ARITHMATIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMATIC";

input [2-1:0] data;
input [2-1:0] distance;
input direction;

output reg [2-1:0] result;
reg [2-1:0] arith_reg; 
always @* begin
  arith_reg = {2{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[2-1] == 1'b1)
          result =  ((arith_reg <<(2 - distance))|| (data >> distance));
      else
          result = data >> distance;
    end
  end
  else begin
    if(direction == 1'b0)begin
        result = data << distance;
    end
    else begin
        result = data >> distance;
    end
  end 
end
endmodule
/****************************************************************************

  ALU Circuit:

         |    _     flags
 src1 ---0---| \    |
             >+|----+----|Sat|------|M src1 --|M\--|-/         |sum|      |U|---
        |U|                       +-|X/
 src2 --|X/                       |    1
                                  |
      src2 ------------|M\        |
 src1 ----------|&|    |U|--|Sat|-+
                |||----|X/  |siz|
 src2 ---0------|^|      0
         |


              ADD SUB   CMP    CMP.U & | ^ ~| MIN MAX ABS MRG
              S U S U = ! < <= < <=           S U S U

adderopA_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   1
adderopB_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   0
logicopB_sel  x x x x x x x x  x x   0 0 0 0  1 1 1 1  1   1
signed        1 0 1 0 1 1 1 1  0 0   x x x x  1 0 1 0  1   x
addsub        0 0 1 1 1 1 1 1  1 1   x x x x  1 1 1 1  1   0
max           x x x x x x x x  x x   0 0 0 0  0 0 1 1  0   0
min           x x x x x x x x  x x   0 0 0 0  1 1 0 0  0   0
logic[1:0]    x x x x x x x x  x x   1 0 2 3  0 0 0 0  0   0
mux1_sel[1:0] 0 0 0 0 x x x x  x x   1 1 1 1  1 1 1 1  2   3 -- Combine
flag_sel[1:0] x x x x 0 1 2 3  2 3   x x x x  x x x x  x   x /

ALUOP_ZERO    =11'bzz1zzzz0101
ALUOP_ADD     =11'b00z10zzzz00
ALUOP_ADDU    =11'b00z00zzzz00
ALUOP_SUB     =11'b00z11zzzz00
ALUOP_SUBU    =11'b00z01zzzz00
ALUOP_CMP_EQ  =11'b00z11zzzz00
ALUOP_CMP_NEQ =11'b00z11zzzz01
ALUOP_CMP_LT  =11'b00z11zzzz10
ALUOP_CMP_LE  =11'b00z11zzzz11
ALUOP_CMP_LT_U=11'b00z01zzzz10
ALUOP_CMP_LE_U=11'b00z01zzzz11
ALUOP_AND     =11'bzz0zz000101
ALUOP_OR      =11'bzz0zz000001
ALUOP_XOR     =11'bzz0zz001001
ALUOP_NOR     =11'bzz0zz001101
ALUOP_MIN     =11'b00111010001
ALUOP_MIN_U   =11'b00101010001
ALUOP_MAX     =11'b00111100001
ALUOP_MAX_U   =11'b00101100001
ALUOP_ABS     =11'b11111000010
ALUOP_MERGE   =11'b101z0000011

****************************************************************************/

//`include "vlane_saturate.v"

module vlane_alu_32(
    clk,
    resetn,

    pipe_en,
    pipe_squashn,

    src1,
    src2,
    mask,

    op,
    satsum_op,
    satsize_op,

    cmp_result,
    result

    );

parameter WIDTH=32;

input clk;
input resetn;

input pipe_en;
input pipe_squashn;

input  [WIDTH-1:0] src1;
input  [WIDTH-1:0] src2;

input  mask;

input  [10:0] op;
input  [1:0] satsum_op;
input  [3:0] satsize_op;

output cmp_result;
output [WIDTH-1:0] result;

wire [WIDTH-1:0] adder_opA;
wire [WIDTH-1:0] adder_opB;
wire [WIDTH+2-1:0] adder_result;

wire [WIDTH-1:0] logic_opA;
wire [WIDTH-1:0] logic_opB;
reg  [WIDTH-1:0] logic_result;

wire [WIDTH-1:0] mux0_result;
wire [WIDTH-1:0] mux1_result;

wire [WIDTH-1:0] satsum_result;
wire [WIDTH-1:0] satsize_result;

wire lt;
wire neq;
wire eq;
wire le;

wire ctrl_adderopA_sel;
wire ctrl_adderopB_sel;
wire ctrl_logicopB_sel;
wire ctrl_signed;
wire ctrl_addsub;
wire ctrl_max;
wire ctrl_min;
wire [1:0] ctrl_logic;
wire [1:0] ctrl_mux1_sel;
wire [1:0] ctrl_flag_sel;

  assign ctrl_adderopA_sel=op[10];
  assign ctrl_adderopB_sel=op[9];
  assign ctrl_logicopB_sel=op[8];
  assign ctrl_signed=op[7];
  assign ctrl_addsub=~op[6];  //Make add 0, sub 1
  assign ctrl_max=op[5];
  assign ctrl_min=op[4];
  assign ctrl_logic=op[3:2];
  assign ctrl_mux1_sel=op[1:0];
  assign ctrl_flag_sel=op[1:0];

  assign adder_opA=(ctrl_adderopA_sel) ? 0 : src1;
  assign adder_opB=(ctrl_adderopB_sel) ? src1 : src2;

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

  `ifdef USE_INHOUSE_LOGIC
  wire [(WIDTH+2)-1:0] dataa;
  wire [(WIDTH+2)-1:0] datab;
  wire cin;

  assign dataa = {{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA};
  assign datab = {{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB};
  assign cin = ~ctrl_addsub;

  local_add_sub_34_0_SIGNED local_adder_inst(
      .dataa(dataa),
      .datab(datab),
      .cin(cin),
      .add_sub(ctrl_addsub),
      .result(adder_result)
  );
  `else
  lpm_add_sub adder_inst(
      .dataa({{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA}),
      .datab({{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB}),
      .cin(~ctrl_addsub),
      .add_sub(ctrl_addsub),
      .result(adder_result)
          // synopsys translate_off
          , .cout (), .overflow (), .clken (), .clock (), .aclr ()
          // synopsys translate_on
      );
  defparam
      adder_inst.lpm_width=WIDTH+2,
      adder_inst.lpm_pipeline=0,
      adder_inst.lpm_representation="SIGNED";
  `endif

  assign lt=adder_result[WIDTH];
  assign neq=|adder_result;
  assign eq=~neq;
  assign le=lt||eq;

  assign cmp_result= (ctrl_flag_sel==0) ? eq :
                     (ctrl_flag_sel==1) ? neq :
                     (ctrl_flag_sel==2) ? lt : le;

  assign logic_opA=src1;
  assign logic_opB=(ctrl_logicopB_sel) ? 0 : src2;

  always@*
    case(ctrl_logic)
        2'b00:
            logic_result=logic_opA|logic_opB;
        2'b01:
            logic_result=logic_opA&logic_opB;
        2'b10:
            logic_result=logic_opA^logic_opB;
        2'b11:
            logic_result=~(logic_opA|logic_opB);
    endcase

  assign mux0_result=((lt && ctrl_max) || (~lt && ctrl_min)) ?  src2 : 
                                                                logic_result;

  /************* PIPELINE at this point ***************/
  reg [WIDTH-1:0] mux0_result_s2;
  reg [WIDTH+2-1:0] adder_result_s2;
  reg  [3:0] satsize_op_s2;
  reg  [3:0] satsum_op_s2;
  reg ctrl_mux1_sel_s2;

  always@(posedge clk)
  begin
    if (!resetn || !pipe_squashn)
    begin
      mux0_result_s2<=0;
      adder_result_s2<=0;
      satsize_op_s2<=0;
      satsum_op_s2<=0;
      ctrl_mux1_sel_s2<=0;
    end
    else if (pipe_en)
    begin
      mux0_result_s2<=mux0_result;
      adder_result_s2<=adder_result;
      satsize_op_s2<=satsize_op;
      satsum_op_s2<=satsum_op;
      ctrl_mux1_sel_s2<=((ctrl_mux1_sel==0) ||
                         (ctrl_mux1_sel==2 && src1[WIDTH-1]) ||
                         (ctrl_mux1_sel==3 && mask) );
    end
  end

  vlane_saturatesize satsize(   //Unit only works for 32-bit inputs
      .in(mux0_result_s2),
      .op(satsize_op_s2),
      .out(satsize_result)
      );

  vlane_saturatesum_32 satsum(
      .in(adder_result_s2),
      .op(satsum_op_s2),
      .out(satsum_result)
      );

  assign mux1_result= (ctrl_mux1_sel_s2) ? 
                              satsum_result : satsize_result;

  assign result=mux1_result;

endmodule
/****************************************************************************
          Saturate unit for different widths given 32-bit input

  Only works for 32-bit inputs

  Interprets input as signed/unsigned, saturates to signed/unsigned 
  byte/half/word

  NOP (pass through) when size=word and signed==outsigned

   out
   sign sign size
    0    0    00    VSAT_U_W - NOP (pass through)
    0    0    01    VSAT_U_B
    0    0    10    VSAT_U_H
    1    1    00    VSAT_W - NOP (pass through)
    1    1    01    VSAT_B
    1    1    10    VSAT_H
    0    1    00    VSAT_SU_W
    0    1    01    VSAT_SU_B
    0    1    10    VSAT_SU_H

parameter
  SATSIZEOP_VSATUW=4'b0000,
  SATSIZEOP_VSATUB=4'b0001,
  SATSIZEOP_VSATUH=4'b0010,
  SATSIZEOP_VSATW  =4'b1100,
  SATSIZEOP_VSATB  =4'b1101,
  SATSIZEOP_VSATH  =4'b1110,
  SATSIZEOP_VSATSUW=4'b0100,
  SATSIZEOP_VSATSUB=4'b0101,
  SATSIZEOP_VSATSUH=4'b0110;

****************************************************************************/

module vlane_saturatesize(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH-1:0] in;
input [3:0] op;
output [WIDTH-1:0] out;

wire op_outsigned;
wire op_signed;
wire op_size;

reg [WIDTH-1:0] out;

  assign op_outsigned=op[3];
  assign op_signed=op[2];
  assign op_size=op[1:0]; //0 - word, 1 - byte, 2 - half

always@*
case(op_size)
    2'b01:  //byte
    case({op_signed,op_outsigned})
        2'b11:    // signed
        out = ((in[WIDTH-1])&(!(&in[WIDTH-2:7]))) ? {{WIDTH-8{1'b1}},8'h80} :
                ((~in[WIDTH-1]) && (|in[WIDTH-2:7])) ? 127 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:8])) ? 255 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:8]) ? 255 : in;
        endcase
    2'b10:  //half-word 16-bits
    case({op_signed,op_outsigned})
        2'b11:    // signed
        out=((in[WIDTH-1])&(!(&in[WIDTH-2:15])))? {{WIDTH-16{1'b1}},16'h8000}:
                ((~in[WIDTH-1]) && (|in[WIDTH-2:15])) ? 32767 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:16])) ? 65535 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:16]) ? 65535 : in;
        endcase
    default: 
    case({op_signed,op_outsigned})
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 : in;
        default:  //2'b00: unsigned
        out=in;
        endcase
endcase
  
endmodule

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
module reduction_layer_8_3.0_32_5_10 
(
  input clk,
  input resetn, //resets the processing elements
  input en, //indicates valid reduction operation
  input  [1:0] reduction_type, //can have 3 values: 0 (Add), 1 (Max), 2 (Min)
  input read,
  input  [8 * 8 -1:0] a, // input data to reduction logic
  output [8 * 8 -1:0] reduced_out, //output
  output reg done, //output is valid when done is 1
  output reg busy
);

wire [8 + 3.0-1:0] reduced_out_unrounded;

reduction_unit_8_3.0 ucu(
  .clk(clk),
  .reset(reset_reduction_unit),
  .inp0(a[1*8-1:0*8]), 
  .inp1(a[2*8-1:1*8]), 
  .inp2(a[3*8-1:2*8]), 
  .inp3(a[4*8-1:3*8]), 
  .inp4(a[5*8-1:4*8]), 
  .inp5(a[6*8-1:5*8]), 
  .inp6(a[7*8-1:6*8]), 
  .inp7(a[8*8-1:7*8]), 
  .mode(reduction_type),
  .outp(reduced_out_unrounded1)
);

////////////////////////////////////////////////////////////////
// Rounding of the output of reduction unit (from 20 bits to 16 bits).
// This is required only when reduction type is "sum"
////////////////////////////////////////////////////////////////
rounding_11.0_8 u_round(.i_data(reduced_out_unrounded), .o_data(reduced_out_add));

assign reduced_out_1 = (reduction_type==2'b0) ? reduced_out_add : reduced_out_unrounded[8-1:0];
assign reduced_out = {8}reduced_out_1}};

reg[2:0] count;

always@(posedge clk)begin
  if(!resetn)begin
    count <= 3'b0;
  end
  else begin
    if(en)
        count <= count + 1;
    if(read)
        count <= count - 1;
  end
end

always@(*)begin
  if(count == 8)begin
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

 

/****************************************************************************
 *           Load data translator
 *- moves read data to appropriate byte/halfword and zero/sign extends
 *****************************************************************************/
module vload_data_translator(
    d_readdatain,
    d_address,
    load_size,
    load_sign_ext,
    d_loadresult);
parameter WIDTH=32;

input [WIDTH-1:0] d_readdatain;
input [1:0] d_address;
input [1:0] load_size;
input load_sign_ext;
output [WIDTH-1:0] d_loadresult;

reg [WIDTH-1:0] d_loadresult;

always @(d_readdatain or d_address or load_size or load_sign_ext)
begin
    case (load_size)
        2'b00:
        begin
            case (d_address[1:0])
                2'd0: d_loadresult[7:0]=d_readdatain[31:24];
                2'd1: d_loadresult[7:0]=d_readdatain[23:16];
                2'd2: d_loadresult[7:0]=d_readdatain[15:8];
                default: d_loadresult[7:0]=d_readdatain[7:0];
            endcase
            d_loadresult[31:8]={24{load_sign_ext&d_loadresult[7]}};
        end
        2'b01:
        begin
            case (d_address[1])
                2'd0: d_loadresult[15:0]=d_readdatain[31:16];
                default: d_loadresult[15:0]=d_readdatain[15:0];
            endcase
            d_loadresult[31:16]={16{load_sign_ext&d_loadresult[15]}};
        end
        default:
            d_loadresult=d_readdatain;
    endcase
end
endmodule


/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_7_5(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [7-1:0]  d;
input              clk;
input              resetn;
input  [5-1:0] en;
input  [5-1:0] squash;
output [7*(5+1)-1:0] q;

reg [7*5-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 7-1:0 ]<= 0;
    else if (en[0])
      tq[ 7-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<5; i=i+1)
      if (!resetn || squash[i] )
        tq[i*7 +: 7 ]<= 0;
      else if (en[i])
        tq[i*7 +: 7 ]<=tq[(i-1)*7 +: 7 ];
  end

  assign q[7-1:0]=d;
  assign q[7*(5+1)-1:7]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_32_4(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [32-1:0]  d;
input              clk;
input              resetn;
input  [4-1:0] en;
input  [4-1:0] squash;
output [32*(4+1)-1:0] q;

reg [32*4-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 32-1:0 ]<= 0;
    else if (en[0])
      tq[ 32-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<4; i=i+1)
      if (!resetn || squash[i] )
        tq[i*32 +: 32 ]<= 0;
      else if (en[i])
        tq[i*32 +: 32 ]<=tq[(i-1)*32 +: 32 ];
  end

  assign q[32-1:0]=d;
  assign q[32*(4+1)-1:32]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_8_1(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [8-1:0]  d;
input              clk;
input              resetn;
input  [1-1:0] en;
input  [1-1:0] squash;
output [8*(1+1)-1:0] q;

reg [8*1-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<1; i=i+1)
      if (!resetn || squash[i] )
        tq[i*8 +: 8 ]<= 0;
      else if (en[i])
        tq[i*8 +: 8 ]<=tq[(i-1)*8 +: 8 ];
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_3(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [1-1:0]  d;
input              clk;
input              resetn;
input  [3-1:0] en;
input  [3-1:0] squash;
output [1*(3+1)-1:0] q;

reg [1*3-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 1-1:0 ]<= 0;
    else if (en[0])
      tq[ 1-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<3; i=i+1)
      if (!resetn || squash[i] )
        tq[i*1 +: 1 ]<= 0;
      else if (en[i])
        tq[i*1 +: 1 ]<=tq[(i-1)*1 +: 1 ];
  end

  assign q[1-1:0]=d;
  assign q[1*(3+1)-1:1]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_8_1(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [8-1:0]  d;
input              clk;
input              resetn;
input  [1-1:0] en;
input  [1-1:0] squash;
output [8*(1+1)-1:0] q;

reg [8*1-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<1; i=i+1)
      if (!resetn || squash[i] )
        tq[i*8 +: 8 ]<= 0;
      else if (en[i])
        tq[i*8 +: 8 ]<=tq[(i-1)*8 +: 8 ];
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        
module transpose_32_2_1 (
  input clk,
  input resetn,
  input read,
  input en, 
  input [2 * 32 -1:0] a,
  output reg [2 * 32 -1:0] out,
  output reg busy
);

reg [2 * 32 - 1:0] data0;
reg [2 * 32 - 1:0] data1;
reg [2 * 32 - 1:0] data2;
reg [2 * 32 - 1:0] data3;
reg [2 * 32 - 1:0] data4;
reg [2 * 32 - 1:0] data5;
reg [2 * 32 - 1:0] data6;
reg [2 * 32 - 1:0] data7;
reg [1:0] count;
integer i;

always@(posedge clk)begin
  if(!resetn)
    count <= 'h0;
  else begin
    if(en)
      count <= count + 1;
    else if(read)
      count <= count - 1 ;
  end
end

always@(posedge clk)begin
  if(!resetn)
     busy <= 1'b0;
  else
    if((count == 2-1) && en)
       busy <= 1'b1;
    else if((count == 'h1) && read)
       busy <= 1'b0; 
end

always @(posedge clk)begin
  if(!resetn)begin
    data0 <= 'h0;
    data1 <= 'h0;
    data2 <= 'h0;
    data3 <= 'h0;
    data4 <= 'h0;
    data5 <= 'h0;
    data6 <= 'h0;
    data7 <= 'h0;
    out <= 'h0;
  end
  else begin
    out <= data0;
    if(en)begin
      data0[count*32 +: 32] = a[0*32 +: 32];
      data1[count*32 +: 32] = a[1*32 +: 32];
      data2[count*32 +: 32] = a[2*32 +: 32];
      data3[count*32 +: 32] = a[3*32 +: 32];
      data4[count*32 +: 32] = a[4*32 +: 32];
      data5[count*32 +: 32] = a[5*32 +: 32];
      data6[count*32 +: 32] = a[6*32 +: 32];
      data7[count*32 +: 32] = a[7*32 +: 32];
    end
    else if(busy & read)begin
        data0 <= data1;
        data1 <= data2;
        data2 <= data3;
        data3 <= data4;
        data4 <= data5;
        data5 <= data6;
        data6 <= data7;
    end
    
  end  
end

endmodule

 

/******************************************************************************
  Vector Control Pipeline

          Stage 1       Stage 2         Stage 3
  ----|--------------|-----------------|-----------------|
      | Decode       | RF/EX           |WB & Send to cpu

 * Note, vs register file is written back to in Stage 3 also, but we check 
 * the vector lanes first to see if they're writing to the vs RF.  If so then
 * we stall.

******************************************************************************/

`include "options.v"

`include "vregfile_base.v"
`include "vregfile_control.v"
`include "vregfile_flag.v"
`include "vregfile_inc.v"
`include "vregfile_scalar.v"
`include "vregfile_stride.v"
`include "vregfile_vector.v"
`include "vcomponents.v"
`include "vlanes.v"

module vpu_7_7 (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive
    has_memop,    // indicates vector pipeline has a memory operation

    // For mtc2/ctc2 instructions
    scalar_in,    
    scalar_in_en,
    scalar_in_wait,

    // For cfc2 instructions
    scalar_out,
    scalar_out_en,
    scalar_out_wait,

    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,
    dbus_wait

    );

parameter LOG2NUMLANES=`LOG2NUMLANES;
parameter LOG2MVL=`LOG2MVL;
parameter LOG2VPW=`LOG2VPW;
parameter LOG2LANEWIDTH=`LOG2LANEWIDTHBITS;
parameter LOG2NUMMEMLANES=`LOG2NUMMEMLANES;
parameter NUMMULLANES=2**`LOG2NUMMULLANES;
parameter LOG2NUMBANKS=`LOG2NUMBANKS;
parameter ALUPERBANK=`ALUPERBANK;

parameter NUMLANES=2**LOG2NUMLANES;
parameter MVL=2**LOG2MVL;
parameter VPW=2**LOG2VPW;
parameter LANEWIDTH=2**LOG2LANEWIDTH;
parameter DMEM_WRITEWIDTH=2**7; //Width of write bus to memory
parameter DMEM_READWIDTH=2**7; // Width of read bus from memory
parameter NUMMEMLANES=2**LOG2NUMMEMLANES;
parameter NUMBANKS=2**LOG2NUMBANKS;

parameter VCWIDTH=32;
parameter NUMVCREGS=64;
parameter LOG2NUMVCREGS=6;
parameter NUMNONMEMVCREGS=32;
parameter LOG2NUMNONMEMVCREGS=5;
parameter NUMVBASEREGS=16;
parameter LOG2NUMVBASEREGS=4;
parameter NUMVINCREGS=8;
parameter LOG2NUMVINCREGS=3;
parameter NUMVSTRIDEREGS=8;
parameter LOG2NUMVSTRIDEREGS=3;

parameter VSWIDTH=32;
parameter NUMVSREGS=32;
parameter LOG2NUMVSREGS=5;

parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

`include "visa.v"

input clk;
input resetn;

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

output has_memop;

// For mtc2/ctc2 instructions
input [31:0] scalar_in;    
input scalar_in_en;
output scalar_in_wait;

// For cfc2 instructions
output [31:0] scalar_out;
output scalar_out_en;
input scalar_out_wait;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (DMEM_WRITEWIDTH/8)-1 : 0 ]   dbus_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]       dbus_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]        dbus_readdata;
input         dbus_cachematch;
input         dbus_cachemiss;
output  [ 31 : 0 ]  dbus_prefetch;
input               dbus_wait;


reg                        [ 31 : 0 ]   ir;
wire                       [ 31 : 0 ]   ir2;
wire                       [ 31 : 0 ]   ir3;

wire                                    stall1;
wire                                    stall2;
wire                                    stall3;
wire                        [ 3 : 1 ]   internal_stall;
wire                                    squash1;
wire                                    squash2;
wire        [`MAX_PIPE_STAGES-1 : 0 ]   vlanes_stalled;

wire                        [ 5 : 0 ]   ir_cop2;
wire                        [ 9 : 0 ]   ir_op;
wire                        [ 5 : 0 ]   ir_vcr;
wire                        [ 4 : 0 ]   ir_vsr;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 5 : 0 ]   ir_vcr_r;

wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_a_readdataout;
wire                                    vc_a_en;
wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_c_writedatain;
wire                                    vc_c_we;
wire                [ VCWIDTH-1 : 0 ]   vl;
//Masks for the matmul (3 masks. each is 8-bit.)
//1-bit for each row/column element in the matmul. we have an 8x8 matmul)
wire         [3*`MAT_MUL_SIZE-1 : 0 ]   matmul_masks;

wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_a_readdataout;
wire                                    vbase_a_en;
wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_c_writedatain;
wire                                    vbase_c_we;

wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_a_reg;
wire                                    vinc_a_en;
wire                [ VCWIDTH-1 : 0 ]   vinc_a_readdataout;
wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vinc_c_writedatain;
wire                                    vinc_c_we;

wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_a_reg;
wire                                    vstride_a_en;
wire                [ VCWIDTH-1 : 0 ]   vstride_a_readdataout;
wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vstride_c_writedatain;
wire                                    vstride_c_we;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_a_readdataout;
wire                                    vs_a_en;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_c_writedatain;
wire                                    vs_c_we;

wire                [ VCWIDTH-1 : 0 ]   vc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vl_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vbase_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vinc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vstride_readdataout;
wire                [ VSWIDTH-1 : 0 ]   vs_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out_s3;
wire                [ VCWIDTH-1 : 0 ]   _vc_c_writedatain;
wire                [ VCWIDTH-1 : 0 ]   fwddata;
wire                                    fwd_vc;
wire                                    fwd_vl;
wire                                    fwd_vbase;
wire                                    fwd_vinc;
wire                                    fwd_vstride;

reg                                     ctrl_vc_a_en;
reg                                     ctrl_vl_a_en;
reg                                     ctrl_vbase_a_en;
reg                                     ctrl_vinc_a_en;
reg                                     ctrl_vstride_a_en;
reg                                     ctrl_vs_a_en;

wire          [ LOG2NUMVCREGS-1 : 0 ]   vc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vbase_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vinc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vstride_rd_reg;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_rd_reg;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vlanes_vs_dst;
wire                                    vlanes_vs_we;
wire                        [ 5 : 2 ]   vlanes_vs_wetrack;
wire                [ VSWIDTH-1 : 0 ]   vlanes_vs_writedata;

wire                [ VCWIDTH-1 : 0 ]   vbase_plus_vinc;

wire [LOG2NUMVCREGS-1:0] ir_base;
wire [LOG2NUMVCREGS-1:0] ir_inc;
wire [LOG2NUMVCREGS-1:0] ir_stride;
wire [LOG2NUMVCREGS-1:0] vcdest_s1;
wire [LOG2NUMVCREGS-1:0] vcdest_s2;
wire [LOG2NUMVCREGS-1:0] vsdest_s1;
wire [LOG2NUMVCREGS-1:0] vsdest_s2;
wire       wevalid;
wire       wevalid_s2;
wire       haz_vc;
wire       haz_vl;
wire       haz_vbase;
wire       haz_vinc;
wire       haz_vstride;
wire       haz_vs_RAW;
wire       haz_vs_WAW;

reg cfc_satisfied;

reg ctrl__vc_writedatain_sel;          //1 - vmstc, 0 - mtc;
reg [1:0] ctrl_vc_writedatain_sel;  //0 - vmstc/mtc, 1-vhalf, 2/3-vsatvl
reg ctrl_vc_we;
reg ctrl_vbase_writedatain_sel;    //1 - ctc/mstc, 0 - vld/vst
reg ctrl_scalarin_en;   //1 when ctc2/mtc2
reg ctrl_scalar_out_en;  //1 when cfc2
reg [1:0] ctrl_vcdest_sel;  //0 - ctc/mstc, 1- ld/st, 2 - satvl/half
reg ctrl_vsdest_sel;  //1 - mtc2, 0 - vmcts
reg ctrl_vs_we;
reg ctrl_rdctl_sel;        // 1 - cfc2/vmcts  0 - ld/st,
reg [1:0] ctrl_rdvc_sel;   // 2-vshamt, 3-vindex, other-ir_vcr


wire ctrl_scalarin_en_s2;
wire ctrl_scalar_out_en_s2;
wire      scalar_out_en_s3;
wire ctrl_vc_we_s2;
wire ctrl_vbase_writedatain_sel_s2;
wire ctrl__vc_writedatain_sel_s2;
wire [1:0] ctrl_vc_writedatain_sel_s2;
wire ctrl_vsdest_sel_s2;
wire ctrl_vs_we_s2;

/************************ Instruction Register ******************************/
wire is_cop2;
reg is_cop2_s1;

  assign is_cop2=instr[31:26]==6'b010010;

  always @(posedge clk)
    if (!resetn || (~is_cop2&~stall1) || (~instr_en&~stall1))
      ir<=32'h0;    // NOP  (Used to use VMSTC $vc48,$vs0)
    else if (instr_en&~stall1)
      ir<=instr;

  always @(posedge clk)
    if (!resetn)
      is_cop2_s1<=1'b0;
    else if (instr_en)
      is_cop2_s1<=is_cop2;

  assign instr_wait = stall1 & is_cop2;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

//Flag instructions which don't use lanes so they don't stall when lanes stalled
  reg ctrl_doesnt_use_lanes;
  always@*
  begin
    ctrl_doesnt_use_lanes=0;
    casez(ir_op)
      0,
      COP2_VSATVL,
      //COP2_VMCTS:  //Omit since vlanes modifies scalar
      COP2_VMSTC,
      COP2_CFC2,
      COP2_CTC2:
        ctrl_doesnt_use_lanes=1;
    endcase
  end

  assign internal_stall[1]=internal_stall[2] | haz_vs_RAW;
  assign stall1=internal_stall[1] | (vlanes_stalled[1]&&~ctrl_doesnt_use_lanes);
  assign squash1 = (stall1&~stall2)|~resetn;

  pipereg IR_reg2(ir,clk,resetn,~stall1,~squash1,ir2);

  assign ir_cop2=ir[31:26];
  assign ir_op={ir[25:22],ir[5:0]}; //10 bits
  assign ir_vcr=ir[15:10];
  assign ir_vsr=ir[15:11];
  assign ir_src1=ir[15:11];
  assign ir_src2=ir[20:16];
  assign ir_base={2'b10,ir[15:12]};
  assign ir_inc={3'b110,ir[11:9]};
  assign ir_stride={3'b111,ir[8:6]};

  assign wevalid = (ir_cop2==6'b010010) & ctrl_vc_we & (vcdest_s1!=48);

  pipereg_1 wevalid_reg1(wevalid,clk,~squash1, ~stall1,1'b1, wevalid_s2);

  assign vcdest_s1 = (ctrl_vcdest_sel==0) ? ir_vcr :
                    (ctrl_vcdest_sel==1) ? ir_base : 0;

  pipereg_6 vcdest_reg1(vcdest_s1,clk,~squash1, ~stall1,1'b1,
                      vcdest_s2);

  assign vsdest_s1 = (ctrl_vsdest_sel) ? ir_vsr : ir_src2;

  pipereg_5 vsdest_reg1(vsdest_s1,clk,~squash1, ~stall1,1'b1,
                      vsdest_s2);

  pipereg_5 vsdestsel_reg1
        (ctrl_vsdest_sel,clk,~squash1, ~stall1,1'b1, ctrl_vsdest_sel_s2);

  // Before reading all source operands, we need to:
  //  1. Determine what all the sources are
  always@*
  begin
    ctrl_vc_a_en=0;
    ctrl_vl_a_en=0;
    ctrl_vbase_a_en=0;
    ctrl_vinc_a_en=0;
    ctrl_vstride_a_en=0;
    ctrl_vs_a_en=0;
    casez(ir_op)
      COP2_VADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMULHI:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMULHI_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VDIV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VDIV_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMOD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMOD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_EQ:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_NE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LT:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LT:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMIN:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMIN_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMAX:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMAX_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMULLO:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VABS:
          ctrl_vl_a_en=1;
      COP2_VAND:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VNOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSLL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRA:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSAT_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_H:
          ctrl_vl_a_en=1;
      COP2_VSAT_W:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_H:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_W:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_L:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_H:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_W:
          ctrl_vl_a_en=1;
      COP2_VSADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSRR_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSLS:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSLS_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VXUMUL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMUL_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VINS_VV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VINS_SV:
      begin
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VEXT_VV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VEXT_SV:
          ctrl_vc_a_en=1;
      COP2_VEXT_U_SV:
          ctrl_vc_a_en=1;
      COP2_VCOMPRESS:
          ctrl_vl_a_en=1;
      COP2_VEXPAND:
          ctrl_vl_a_en=1;
      COP2_VMERGE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VFINS:
        begin
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=1;
        end
      COP2_VEXTHALF:
          ctrl_vl_a_en=1;
      COP2_VHALF:
          ctrl_vl_a_en=1;
      COP2_VHALFUP:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VHALFDN:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSATVL:
          ctrl_vl_a_en=1;
      COP2_VFAND:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFXOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFNOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFCLR:
          ctrl_vl_a_en=1;
      COP2_VFSET:
          ctrl_vl_a_en=1;
      COP2_VIOTA:
          ctrl_vl_a_en=1;
      COP2_VCIOTA:
          ctrl_vl_a_en=1;
      COP2_VFPOP:
          ctrl_vl_a_en=1;
      COP2_VFFF1:
          ctrl_vl_a_en=1;
      COP2_VFFL1:
          ctrl_vl_a_en=1;
      COP2_VFSETBF:
          ctrl_vl_a_en=1;
      COP2_VFSETIF:
          ctrl_vl_a_en=1;
      COP2_VFSETOF:
          ctrl_vl_a_en=1;
      COP2_VFMT8:
          ctrl_vl_a_en=1;
      COP2_VFMF8:
          ctrl_vl_a_en=1;
      COP2_VFCLR8:
          ctrl_vl_a_en=1;
      COP2_VFOR8:
          ctrl_vl_a_en=1;
      COP2_VFLD:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_U_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_U_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_U_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLDS_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_U_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_U_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_U_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDX_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_H:
          ctrl_vbase_a_en=1;
      COP2_VLDX_W:
          ctrl_vbase_a_en=1;
      COP2_VLDX_L:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_H:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_W:
          ctrl_vbase_a_en=1;
      COP2_VFST:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VSTS_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTS_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTS_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTS_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTX_B:
          ctrl_vbase_a_en=1;
      COP2_VSTX_H:
          ctrl_vbase_a_en=1;
      COP2_VSTX_W:
          ctrl_vbase_a_en=1;
      COP2_VSTX_L:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_B:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_H:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_W:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_L:
          ctrl_vbase_a_en=1;
      COP2_VMCTS:
        begin
          ctrl_vc_a_en=1;
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VMSTC:
          ctrl_vs_a_en=1;
      COP2_CFC2:
        begin
          ctrl_vc_a_en=1;
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
    endcase
  end

  //  2. Determine if any of the sources have a RAW hazard
  assign haz_vc=(ctrl_vc_a_en&wevalid_s2)&&(vc_rd_reg==vcdest_s2);
  assign haz_vl=(ctrl_vl_a_en&wevalid_s2)&&(vcdest_s2==0);
  assign haz_vbase=(ctrl_vbase_a_en&wevalid_s2)&&(vbase_rd_reg==vcdest_s2);
  assign haz_vinc=(ctrl_vinc_a_en&wevalid_s2)&&(vinc_rd_reg==vcdest_s2)&&(vcdest_s2!=48);
  assign haz_vstride=(ctrl_vstride_a_en&wevalid_s2)&(vstride_rd_reg==vcdest_s2);

  // Coarse-grained checks for VS - don't check registers
  assign haz_vs_RAW=ctrl_vs_a_en && (ctrl_vs_we_s2 || (|vlanes_vs_wetrack));
  assign haz_vs_WAW=ctrl_vs_we_s2 && (|vlanes_vs_wetrack[5:3]);

  pipereg_32 fwddata_reg (
       (ctrl_vbase_writedatain_sel_s2) ?  vbase_plus_vinc : vc_c_writedatain,
       clk,~squash1, ~stall1,1'b1,fwddata);

  pipereg_1  fwdvc_reg ( haz_vc,clk,~squash1, ~stall1,1'b1,fwd_vc);
  pipereg_1  fwdvl_reg ( haz_vl,clk,~squash1, ~stall1,1'b1,fwd_vl);
  pipereg_1  fwdvbase_reg ( haz_vbase,clk,~squash1, ~stall1,1'b1,fwd_vbase);
  pipereg_1  fwdvinc_reg ( haz_vinc,clk,~squash1, ~stall1,1'b1,fwd_vinc);
  pipereg_1  fwdvstride_reg(haz_vstride,clk,~squash1,~stall1,1,fwd_vstride);

  /************************ REGISTER FILES ******************************/

  assign vc_rd_reg= (ctrl_rdvc_sel==2) ? 2 :  //vshamt
                    (ctrl_rdvc_sel==3) ? 3 :  //vindex
                    ir_vcr;
  assign vbase_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_base;
  assign vinc_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_inc;
  assign vstride_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_stride;
  assign vs_rd_reg= (ctrl_vs_a_en & ir_op[BIT_VSSRC1]) ? 
                        ir_src1[LOG2NUMVSREGS-1:0] : 
                        ir_src2[LOG2NUMVSREGS-1:0];

  assign vc_a_reg= vc_rd_reg[LOG2NUMNONMEMVCREGS-1:0];
  assign vbase_a_reg= vbase_rd_reg[LOG2NUMVBASEREGS-1:0];
  assign vinc_a_reg=vinc_rd_reg[LOG2NUMVINCREGS-1:0];
  assign vstride_a_reg=vstride_rd_reg[LOG2NUMVSTRIDEREGS-1:0];

  assign vs_a_reg=vs_rd_reg[LOG2NUMVSREGS-1:0];

  assign vc_a_en=ctrl_vc_a_en&~stall1;
  assign vbase_a_en=ctrl_vbase_a_en&~stall1;
  assign vinc_a_en=ctrl_vinc_a_en&~stall1;
  assign vstride_a_en=ctrl_vstride_a_en&~stall1;
  assign vs_a_en=ctrl_vs_a_en&~stall1;

  vregfile_control_32_32_5.0 vregfile_control (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vc_a_reg), 
      .a_en(vc_a_en),
      .a_readdataout(vc_a_readdataout),
      .c_reg(vc_c_reg), 
      .c_writedatain(vc_c_writedatain), 
      .c_we(vc_c_we),
      .vl(vl),
      //The reserved registers vc31, vc30, vc29 are used
      //for the matmul's masks.
      .matmul_masks(matmul_masks)
      );

  vregfile_base_32_16_4.0 vregfile_base (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vbase_a_reg), 
      .a_en(vbase_a_en), 
      .a_readdataout(vbase_a_readdataout),
      .c_reg(vbase_c_reg), 
      .c_writedatain(vbase_c_writedatain), 
      .c_we(vbase_c_we));

  vregfile_inc_32_8_3.0 vregfile_inc (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vinc_a_reg), 
      .a_en(vinc_a_en), 
      .a_readdataout(vinc_a_readdataout),
      .c_reg(vinc_c_reg), 
      .c_writedatain(vinc_c_writedatain), 
      .c_we(vinc_c_we));

  vregfile_stride_32_8_3.0 vregfile_stride (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vstride_a_reg), 
      .a_en(vstride_a_en), 
      .a_readdataout(vstride_a_readdataout),
      .c_reg(vstride_c_reg), 
      .c_writedatain(vstride_c_writedatain), 
      .c_we(vstride_c_we));

  vregfile_scalar_32_32_5 vregfile_scalar (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vs_a_reg), 
      .a_en(vs_a_en), 
      .a_readdataout(vs_a_readdataout),
      .c_reg(vs_c_reg), 
      .c_writedatain(vs_c_writedatain), 
      .c_we(vs_c_we));

////////////////////////////////////////////////////////////      
//    parameters sequence for regfiles
//
//    defparam WIDTH=VSWIDTH;
//    defparam NUMREGS=NUMVSREGS;
//    defparam LOG2NUMREGS=LOG2NUMVSREGS;
////////////////////////////////////////////////////////////
  pipereg_6 vc_reg(ir_vcr,clk,resetn,vc_a_en,1'b1,ir_vcr_r);

  pipereg_1 r1(ctrl_scalarin_en,clk,~squash1,~stall1,1'b1,ctrl_scalarin_en_s2);
  pipereg_1 r2(ctrl_scalar_out_en,clk,~squash1,~stall1,1'b1,ctrl_scalar_out_en_s2);
  pipereg_1 r4(ctrl_vc_we,clk,~squash1,~stall1,1'b1,ctrl_vc_we_s2);
  pipereg_1 r5(ctrl_vbase_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vbase_writedatain_sel_s2);
  pipereg_1 r6(ctrl__vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl__vc_writedatain_sel_s2);
  pipereg_2 r7(ctrl_vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vc_writedatain_sel_s2);
  pipereg_1 r8(ctrl_vs_we,clk,~squash1,~stall1,1'b1,ctrl_vs_we_s2);


  /*********************** Control control signals ****************************/
  always@*
  begin
    ctrl_scalarin_en=0;
    ctrl_scalar_out_en=0;
    ctrl__vc_writedatain_sel=0;
    ctrl_vc_writedatain_sel=0;
    ctrl_vbase_writedatain_sel=0;
    ctrl_vc_we=0;
    ctrl_rdctl_sel=0;
    ctrl_rdvc_sel=0;
    ctrl_vcdest_sel=0;
    ctrl_vsdest_sel=0;
    ctrl_vs_we=0;
    casez(ir_op)
      COP2_VSRR:  ctrl_rdvc_sel=2;
      COP2_VSRR_U:  ctrl_rdvc_sel=2;
      COP2_VSLS:  ctrl_rdvc_sel=2;
      COP2_VSLS_U:  ctrl_rdvc_sel=2;
      COP2_VXUMUL:  ctrl_rdvc_sel=2;
      COP2_VXUMUL_U:  ctrl_rdvc_sel=2;
      COP2_VXLMUL:  ctrl_rdvc_sel=2;
      COP2_VXLMUL_U:  ctrl_rdvc_sel=2;
      COP2_VXUMADD:  ctrl_rdvc_sel=2;
      COP2_VXUMADD_U:  ctrl_rdvc_sel=2;
      COP2_VXUMSUB:  ctrl_rdvc_sel=2;
      COP2_VXUMSUB_U:  ctrl_rdvc_sel=2;
      COP2_VXLMADD:  ctrl_rdvc_sel=2;
      COP2_VXLMADD_U:  ctrl_rdvc_sel=2;
      COP2_VXLMSUB:  ctrl_rdvc_sel=2;
      COP2_VXLMSUB_U:  ctrl_rdvc_sel=2;
      COP2_VINS_VV:  ctrl_rdvc_sel=2;
      COP2_VINS_VV:  ctrl_rdvc_sel=3;
      COP2_VINS_SV:  ctrl_rdvc_sel=3;
      COP2_VEXT_VV:  ctrl_rdvc_sel=3;
      COP2_VEXT_SV:  ctrl_rdvc_sel=3;
      COP2_VEXT_U_SV:  ctrl_rdvc_sel=3;
      COP2_VFINS:  ctrl_rdvc_sel=3;
      COP2_VHALFUP:  ctrl_rdvc_sel=3;
      COP2_VHALFDN:  ctrl_rdvc_sel=3;
      COP2_VHALF:
      begin
          ctrl_vc_writedatain_sel=1;
          ctrl_vcdest_sel=2;
          ctrl_vc_we=1;
      end
      COP2_VSATVL:
      begin
          ctrl_vc_writedatain_sel=2;
          ctrl_vcdest_sel=2;
          ctrl_vc_we=1;
      end
      COP2_VFLD,
      COP2_VLD_B,
      COP2_VLD_H,
      COP2_VLD_W,
      COP2_VLD_L,
      COP2_VLD_U_B,
      COP2_VLD_U_H,
      COP2_VLD_U_W,
      COP2_VLDS_B,
      COP2_VLDS_H,
      COP2_VLDS_W,
      COP2_VLDS_L,
      COP2_VLDS_U_B,
      COP2_VLDS_U_H,
      COP2_VLDS_U_W,
      COP2_VFST,
      COP2_VST_B,
      COP2_VST_H,
      COP2_VST_W,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
      COP2_VSTS_W,
      COP2_VSTS_L:
      begin
          ctrl_vbase_writedatain_sel=1;
          ctrl_vcdest_sel=1;
          ctrl_vc_we=1;
      end
      COP2_VMCTS:
      begin
        ctrl_rdctl_sel=1;
        ctrl_vsdest_sel=0;
        ctrl_vs_we=1;
      end
      COP2_VMSTC:
      begin
        ctrl__vc_writedatain_sel=1;
        ctrl_vcdest_sel=0;
        ctrl_vc_we=1;
      end
      COP2_CFC2:
      begin
        ctrl_scalar_out_en=1;
        ctrl_rdctl_sel=1;
      end
      COP2_CTC2:
      begin
        ctrl_scalarin_en=1;
        ctrl_vcdest_sel=0;
        ctrl_vc_we=1;
      end
      COP2_MTC2:
      begin
        ctrl_scalarin_en=1;
        ctrl_vsdest_sel=1;
        ctrl_vs_we=1;
      end
    endcase
  end

/******************************************************************************/
/************************** 2nd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[2]=internal_stall[3] |
                              (ctrl_scalarin_en_s2&~scalar_in_en) | 
                              haz_vs_WAW;
  assign stall2=internal_stall[2]; // | vlanes_stalled[2];
  assign squash2 = ((stall2&~stall3))|~resetn;

  // Stall scalar if a) we are stalled, are expecting a scalar, and the it is
  // available OR b) if we haven't gotten there yet.
  assign scalar_in_wait=(scalar_in_en&~ctrl_scalarin_en_s2) || 
                        (ctrl_scalarin_en_s2&scalar_in_en&stall2);

  assign vc_readdataout= (fwd_vc) ? fwddata : vc_a_readdataout;
  assign vl_readdataout= (fwd_vl) ? fwddata : vl;
  assign vbase_readdataout= (fwd_vbase) ? fwddata : vbase_a_readdataout;
  assign vinc_readdataout= (fwd_vinc) ? fwddata : vinc_a_readdataout;
  assign vstride_readdataout= (fwd_vstride) ? fwddata : vstride_a_readdataout;
  assign vs_readdataout= vs_a_readdataout;

  assign vc_combined_out=(!ir_vcr_r[5]) ? vc_readdataout :
                         (!ir_vcr_r[4]) ? vbase_readdataout :
                         (!ir_vcr_r[3]) ? vinc_readdataout : 
                         vstride_readdataout;

  pipereg_32 scalar_out_reg(
      vc_combined_out,clk,resetn,ctrl_scalar_out_en_s2,1'b1,vc_combined_out_s3);

  pipereg_1 scalar_out_en_reg(
      ctrl_scalar_out_en_s2,clk,~squash2,~stall2,1'b1,scalar_out_en_s3);


  /************** Datapath - Control Regs *******************/

  assign vc_c_we = ctrl_vc_we_s2&~vcdest_s2[5] & ~stall2;
  assign vc_c_reg = vcdest_s2[LOG2NUMNONMEMVCREGS-1:0]; 
  //temporary to feed to base,inc, and stride regfiles (without vl logic)
  assign _vc_c_writedatain= (ctrl__vc_writedatain_sel_s2) ? vs_a_readdataout :
                                                        scalar_in;
  assign vc_c_writedatain= (ctrl_vc_writedatain_sel_s2==0) ? _vc_c_writedatain :
                            //vhalf instruction
                            (ctrl_vc_writedatain_sel_s2==1) ? vl_readdataout>>1:
                            //vsatvl instruction
                            //(vl_readdataout>vc_readdataout) ? vc_readdataout :
                            (vl_readdataout>MVL) ? MVL : 
                            vl_readdataout;

  assign vbase_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:4]==2) & ~stall2;
  assign vbase_c_reg = vcdest_s2[LOG2NUMVBASEREGS-1:0];
  assign vbase_c_writedatain = (ctrl_vbase_writedatain_sel_s2) ?
                                vbase_plus_vinc :
                                _vc_c_writedatain;

  assign vbase_plus_vinc=vbase_readdataout+vinc_readdataout;

  assign vinc_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==6) & ~stall2;
  assign vinc_c_reg = vcdest_s2[LOG2NUMVINCREGS-1:0];
  assign vinc_c_writedatain = _vc_c_writedatain;

  assign vstride_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==7) & ~stall2;
  assign vstride_c_reg = vcdest_s2[LOG2NUMVSTRIDEREGS-1:0];
  assign vstride_c_writedatain = _vc_c_writedatain;

  //FIXME - OR vector writes in, but check for RAW & WAW hazards
  assign vs_c_we = vlanes_vs_we || (ctrl_vs_we_s2 & ~stall2);
  assign vs_c_reg = (vlanes_vs_we) ? vlanes_vs_dst : vsdest_s2;
  assign vs_c_writedatain = (vlanes_vs_we) ? vlanes_vs_writedata : 
                    (ctrl_vsdest_sel_s2) ? scalar_in : vc_combined_out;


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[3]=scalar_out_en_s3&scalar_out_wait;
  assign stall3=internal_stall[3]; // | vlanes_stalled[3];

  assign scalar_out=vc_combined_out_s3;
  assign scalar_out_en=scalar_out_en_s3&~cfc_satisfied;

  always@(posedge clk)
    cfc_satisfied<=scalar_out_en_s3 & ~scalar_out_wait;

/******************************************************************************/
/************************ Instantiate Vector Lanes ****************************/
/******************************************************************************/

  vlanes_8_3_8_3_8_64_6_2_1_32_5_2_1_0_128_7_128_7_32_32_32_5 vlanes(
    .clk(clk),
    .resetn(resetn),

    // Instruction interface
    .instr(ir),
    .instr_en(is_cop2_s1),    // tells when instr is valid and available
    .instr_wait(),            // if high says vpu is not ready to receive

    .stall_in({internal_stall,1'b0}),
    .is_stalled(vlanes_stalled),
    .has_memop(has_memop),

    // Control register values - 2nd stage
    .vc_in(vc_readdataout),
    .vl_in(vl_readdataout),
    .vbase_in(vbase_readdataout),
    .vinc_in(vinc_readdataout),
    .vstride_in(vstride_readdataout),
    .vs_in(vs_readdataout),
    .matmul_masks_in(matmul_masks),

    // vs Writeback
    .vs_dst(vlanes_vs_dst),
    .vs_wetrack(vlanes_vs_wetrack),  //1-bit for each pipe-stage
    .vs_we(vlanes_vs_we),
    .vs_writedata(vlanes_vs_writedata),

    // Data memory interface
    .dbus_address(dbus_address),
    .dbus_en(dbus_en),
    .dbus_we(dbus_we),
    .dbus_byteen(dbus_byteen),
    .dbus_writedata(dbus_writedata),
    .dbus_readdata(dbus_readdata),
    .dbus_cachematch(dbus_cachematch),
    .dbus_cachemiss(dbus_cachemiss),
    .dbus_prefetch(dbus_prefetch),
    .dbus_wait(dbus_wait)
    );
  
///////////////////////////////////////////////////
//    vlanes parameter sequence for reference
//
//    vlanes.NUMLANES=NUMLANES,
//    vlanes.LOG2NUMLANES=LOG2NUMLANES,
//    vlanes.NUMMEMPARALLELLANES=NUMMEMLANES,
//    vlanes.LOG2NUMMEMPARALLELLANES=LOG2NUMMEMLANES,
//    vlanes.NUMMULLANES=NUMMULLANES,
//    vlanes.MVL=MVL,
//    vlanes.LOG2MVL=LOG2MVL,
//    vlanes.VPW=VPW,
//    vlanes.LOG2VPW=LOG2VPW,
//    vlanes.LANEWIDTH=LANEWIDTH,
//    vlanes.LOG2LANEWIDTH=LOG2LANEWIDTH,
//    vlanes.NUMBANKS=NUMBANKS,
//    vlanes.LOG2NUMBANKS=LOG2NUMBANKS,
//    vlanes.ALUPERBANK=ALUPERBANK,
//    vlanes.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, 
//    vlanes.LOG2DMEM_WRITEWIDTH=7,
//    vlanes.DMEM_READWIDTH=DMEM_READWIDTH,
//    vlanes.LOG2DMEM_READWIDTH=7,
//    vlanes.VCWIDTH=VCWIDTH,
//    vlanes.VSWIDTH=VSWIDTH,
//    vlanes.NUMVSREGS=NUMVSREGS,
//    vlanes.LOG2NUMVSREGS=LOG2NUMVSREGS;
//////////////////////////////////////////////////////////
endmodule

                 

module vdispatcher_shift (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 151=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  squash;

input [ 151-1:0 ] inshift_data;

input [ 2*151-1:0 ]  inparallel_data;
output [ 2*151-1:0 ] outparallel_data;

wire [ 151-1:0 ]  shiftin_left;
wire [ 151-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[2*151-1:(2-1)*151];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[151-1:0];

  velmshifter_2_151 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule


module vdispatcher_shift (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  squash;

input [ 1-1:0 ] inshift_data;

input [ 2*1-1:0 ]  inparallel_data;
output [ 2*1-1:0 ] outparallel_data;

wire [ 1-1:0 ]  shiftin_left;
wire [ 1-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[2*1-1:(2-1)*1];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[1-1:0];

  velmshifter_2_1 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule


 
// THIS UNIT SHOULD BE REDESIGNED!!!!!
// It started off simple with low performance and after adding a bunch of hacks
// to make it perform better it's gotten messy.  A new memory unit should
// special case strided loads and service them early and pipelined, and it
// should have a store buffer with variable width for writes.

/************************
 *
 * VPUWIDTH must equal CONTROLWIDTH must equal 32 
 * NUMPARALLELLANES must be a divisor of NUMLANES
 *
 * op[6] - memop 1-memory operation, 0-shift operation
 *
 * op[5] op[4]
 *   0     0    UNIT
 *   0     1    STRIDE
 *   1     0    INDEX
 *
 * size1 size0 signd ld/st
 * op[3] op[2] op[1] op[0]
 *   0     0     0     0    VLD.u.b
 *   0     1     0     0    VLD.u.h
 *   1     0     0     0    VLD.u.w
 *   0     0     1     0    VLD.b
 *   0     1     1     0    VLD.h
 *   1     0     1     0    VLD.w
 *   0     0     X     1    VST.b
 *   0     1     X     1    VST.h
 *   1     0     X     1    VST.w
 *
 *
 * Ported to handle element shifting as well:
 *
 * Data shifts
 * ===========
 * vins -   L shift vindex
 * vext -     R shift vindex
 * vcompress -  R
 * vexpand -  L 
 * vfins -  L vindex
 * vhalf -        R shift vl/2
 * vexthalf -     R shift vl/2
 * vhalfup -  R shift 2^vindex
 * vhalfdn -  L shift 2^vindex
 *
 * Mask shifting - much harder
 * =============
 *
 * Quartus 5.0: 919 LEs, 137 MHz (CritPath: vip_r to vreaddata)
 *************************/
`include "velmshifter_serial.v"
`include "vmem_crossbar.v"

module vmem_unit_32_8_3_8_3_32_128_7_128_7_-3_8 (
    clk,
    resetn,

    enable,       //Is this unit on? (is it less than VL, does the instr use it)
    en, //Is pipeline advancing
    squash, //Is pipeline squashing us
    op,
    stall,
    last_subvector,

    // Control ports
    cbase,
    cstride,
    cprefetch,

    // Vector ports
    vmask,
    vstrideoffset,  //pre-computed stride offsets for each lane (stride*laneid)
    vindex,
    vwritedata,
    voutput,
    voutput_we,

    // Writeback ports
    in_dst,
    in_dst_we,
    out_dst,
    out_dst_we,
    in_vs_dst_we,
    out_vs_dst_we,

    // Vector operations ports
    sa,
    dir_left,

    // Data memory interface
    dmem_en,
    dmem_address,
    dmem_we,
    dmem_byteen,
    dmem_writedata,
    dmem_readdata,
    dmem_cachematch,
    dmem_cachemiss,
    dmem_prefetch,
    dmem_wait
    );

parameter       MUNIT_IDLE=0,
                MUNIT_ISSUE=1,
                MUNIT_DONE=2,
                MUNIT_SHIFT=3, 
                MUNIT_XTRAWRITE=4; 
  
parameter ISSUE_IDLE=3'b000,
          ISSUE_INITIAL_INCREMENT=3'b001,
          ISSUE_INCREMENTED=3'b010,
          ISSUE_WAITONE=3'b011,
          ISSUE_WAITING=3'b100;

input clk;
input resetn;

input  enable;
input  [2:0] en; //Is pipeline one
input  [2:0] squash;
input [6:0] op;
output stall;
input last_subvector;

// Control ports
input [ 32-1 : 0 ] cbase;
input [ 32-1 : 0 ] cstride;
input [ 32-1 : 0 ] cprefetch;

// Vector ports
input  [          8-1 : 0 ]  vmask;
input  [ 8*32-1 : 0 ]  vstrideoffset;
input  [ 8*32-1 : 0 ]  vindex;
input  [ 8*32-1 : 0 ]  vwritedata;
output [ 8*32-1 : 0 ]  voutput;
output [          8-1 : 0 ]  voutput_we;

input   [8-1:0]  in_dst;
input                     in_dst_we;
output   [8-1:0] out_dst;
output                    out_dst_we;
input                     in_vs_dst_we;
output                    out_vs_dst_we;

input  [      3-1 : 0 ]  sa;
input                               dir_left;

// Data memory interface
output dmem_en;
output dmem_we;
output  [ 31 : 0 ]                  dmem_address;
output  [ 128/8-1 : 0 ] dmem_byteen;
output  [ 128-1 : 0 ]   dmem_writedata;
input   [ 128-1 : 0 ]    dmem_readdata;
input                               dmem_cachematch;
input                               dmem_cachemiss;
output  [ 31 : 0 ]                  dmem_prefetch;
input                               dmem_wait;

reg     [ 31 : 0 ]                  dmem_address;
reg     [ 128/8-1 : 0 ] dmem_byteen;
reg     [ 128-1 : 0 ]   dmem_writedata;
reg     [ 31 : 0 ]                  dmem_prefetch;

wire  [1:0]  op_pattern;      // 0-Unit, 1-Strided, 2-Indexed
wire  [1:0]  op_size;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
wire         op_signed;       // 0-unsigned, 1-signed
wire         op_we;         // 0-load, 1-store
wire         op_memop;         // 1-memory op, 0-vector shift op

wire  [ 8*32-1 : 0 ]  __vreaddata;
reg           [ 8*32-1 : 0 ]  address;
reg              [ 8*32-1:0]  vreaddata;
reg                    [ 8-1 : 0 ]  vreaddata_we;
reg   [ 8*(7-5)-1 : 0 ] crossbar_sel;
wire  [ 8*32-1 : 0 ] crossbar;
wire  [ 8*32-1 : 0 ]  _vwritedata;
wire   [ 8*4-1 : 0 ]  _vbyteen;

reg  [ 32-1 : 0 ] stride;
wire [ 32-1 : 0 ] stride_tmp;
reg  [ 32-1 : 0 ] prefetch;
wire [ 32-1 : 0 ] t_cprefetch;

wire           [         8-1 : 0 ]  vshifted_mask;
wire           [         8-1 : 0 ]  vshifted_masksave;
wire          [ 8*32-1 : 0 ]  vshifted_writedata;
wire          [ 8*32-1 : 0 ]  vshifted_address;


reg                           dmem_valid;
reg   [ 31 : 0 ]              dmem_readdata_address;
reg                           cachedata_stillvalid;

reg [2:0] munit_state;
reg [2:0] issue_state;

reg  [ 3-1 : 0 ] vpid;
reg  [ 8-1 : 0 ] done;
wire doneall;

wire  [         8-1 : 0 ]  _vreaddata_we;
reg   [ 8-1 : 0 ]  _parhits;
wire  [ 8-1 : 0 ]  parhits;
reg   [ 8-1 : 0 ]  parhits_done;
wire  parhits_doneall;
wire  parhits_all;
wire  parhits_none;
wire  parhits_some;

wire  shifter_dirleft;
wire  shifter_load;
wire  shifter_shift;
wire  shifter_jump;
wire  shifter_jump_s1;

wire addr_sneakyload;
wire addr_advance;
wire addr_rewind;

wire do_quick_load;
reg quick_loaded;

reg  [ 3-1 : 0 ] sa_count;
wire [ 3-1 : 0 ] next_sa_count;
wire                        next_sa_count_zero;
wire                        sa_zero;
reg  [ 8*32-1 : 0 ]  vshiftresult;

genvar i;
integer j;
genvar k;
integer l;
integer m;
integer n;
integer p;

  assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;

  assign stall= (munit_state!=MUNIT_IDLE && munit_state!=MUNIT_DONE);

/************************** Pipeline load stage *******************************/
  reg enable_s2;
  reg last_subvector_s2;
  reg [ 8-1 : 0 ] in_dst_s2;
  reg [      3-1 : 0 ] sa_s2;
  reg dir_left_s2;
  reg  [1:0]  op_pattern_s2;      // 0-Unit, 1-Strided, 2-Indexed
  reg  [1:0]  op_size_s2;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
  reg         op_signed_s2;       // 0-unsigned, 1-signed
  reg         op_we_s2;         // 0-load, 1-store
  reg         op_memop_s2;         // 1-memory op, 0-vector shift op

  always@(posedge clk)
    if (!resetn)
    begin
      enable_s2<=0;
      last_subvector_s2<=0;
      in_dst_s2<=0;
      sa_s2<=0;
      dir_left_s2<=0;
      {op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=0;
    end
    else if (!stall)
    begin
      enable_s2<=enable;
      last_subvector_s2<=last_subvector;
      if (en[0])
        in_dst_s2<=in_dst;
      sa_s2<=sa;
      dir_left_s2<=dir_left;
      {op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=op;
    end

/*************************** Vector op logic *********************************/

  assign next_sa_count=sa_count-1;
  assign next_sa_count_zero=(next_sa_count==0);
  assign sa_zero=(sa==0);

  always@(posedge clk)
  begin
    if (!resetn)
      sa_count<=0;
    else if (shifter_load)
      sa_count<=sa;
    else if (shifter_shift)
      sa_count<=next_sa_count;
  end

  // Detect double write one clock cycle early - also support 1 lane
  assign doublewrite=(~op_memop_s2) && (8>1) && ( (dir_left_s2) ? 
              (|vshifted_mask[((8>1) ? 8:2)-2:0]) && //support L=1
                (vshifted_mask[8-1]&(|vshifted_masksave)) :
              (|vshifted_mask[8-1:(8>1) ? 1 : 0]) && //support L=1
                (vshifted_mask[0]&(|vshifted_masksave)));

  assign out_dst = {in_dst_s2[8-1:-3],
                    (munit_state!=MUNIT_XTRAWRITE) ? in_dst_s2[-3-1:0] :
                     (dir_left_s2) ? in_dst_s2[-3-1:0]+1'b1 : 
                       in_dst_s2[-3-1:0]-1'b1};

  //Truncate shifted result to VPW's size
  always@*
    for (p=0; p<8; p=p+1)
      vshiftresult[p*32 +: 32]=
              vshifted_address[p*32 +: 32];

  assign voutput= (~enable_s2) ? 0 :
                  (op_memop_s2) ? vreaddata : 
                               vshiftresult;
 

  assign voutput_we=(op_memop_s2) ? vreaddata_we : 
                      (munit_state==MUNIT_XTRAWRITE) ? vshifted_masksave : 
                        (munit_state==MUNIT_DONE) ? vshifted_mask : 0;

/*************************** ISSUING LOGIC *********************************/

  assign shifter_load=enable && ~quick_loaded &&
                        (munit_state==MUNIT_IDLE || 
                         munit_state==MUNIT_DONE ||
                         do_quick_load);

  assign shifter_shift= (munit_state==MUNIT_SHIFT) || 
               ((munit_state==MUNIT_ISSUE) && parhits_doneall);

  assign shifter_dirleft=(op_memop_s2) ? 1'b0 : dir_left_s2;
  assign shifter_jump=op_memop_s2 && ~op_pattern_s2[1];
  assign shifter_jump_s1=op_memop && ~op_pattern[1];

  // Address Generator for each lane, or store vector to be shifted
  always@*
    for (m=0; m<8; m=m+1)
      address[m*32 +: 32] = ((op_memop) ? cbase : 0) + 
            ( (op_pattern[1] || ~op_memop)  ? vindex[m*32 +: 32] : 
               (vstrideoffset[m*32 +: 32]<<op_size));

  wire [8-1:0] vwritedata_shifter_squash_NC;
  velmshifter_jump_8_8_32 vwritedatashifter(
      .clk(clk),
      .resetn(resetn),    //Don't use if not a store
      .load(shifter_load && op_we),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vwritedata_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe(vwritedata),
      .outpipe(vshifted_writedata));

  wire [8-1:0] vaddress_shifter_squash_NC;
  velmshifter_jump_8_8_32 vaddressshifter(
      .clk(clk),
      .resetn(resetn), //consider forcing to 0 if not used
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vaddress_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe( address),
      .outpipe(vshifted_address));

  velmshifter_jump_8_8_1 vmaskshifter(
      .clk(clk),
      .resetn(resetn),
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(0),
      .shiftin_left( 1'b0),
      .shiftin_right( 1'b0 ),
      .inpipe(vmask),
      .outpipe(vshifted_mask));

  //Save spilled over masks for shifting instructions (can only shift by 1)
  wire    [8-1 : 0]  vmasks_save_inpipe_NC;
  velmshifter_8_1  vmaskssave(
      .clk(clk),
      .resetn(resetn && ~shifter_load),
      .load(1'b0),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .squash(0),
      .shiftin_left( vshifted_mask[0] ),
      .shiftin_right( vshifted_mask[8-1] ),
      .inpipe(vmasks_save_inpipe_NC),
      .outpipe(vshifted_masksave));

  //Stride is maximum of stride specified and cache line size 
  assign stride_tmp = (((op_pattern[0]) ? cstride : 1) << op_size);
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      stride<=0;
    else if (shifter_load)
      stride<= (stride_tmp>128/8) ? 
                      stride_tmp : 128/8;

  wire [15:0] constantprefetch;
  assign constantprefetch=`VECTORPREFETCHES;
  assign t_cprefetch=cprefetch[15:0]<<(65535-`VECTORPREFETCHES);

  //***********************  Prefetch Logic **************************
  //
  //Send stride and vector length to bus.  To do constant prefetching set
  //stride to cache line size and send constant as length
  //******************************************************************
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      prefetch<=0;
    else if (shifter_load)
      if (`VECTORPREFETCHES >= 65530)
        prefetch<= { stride_tmp[15:0], t_cprefetch[15:0] };
      else if (`VECTORPREFETCHES == 0)
        prefetch<= 0;
      else
        prefetch<= { 16'd0+2**(7-3),constantprefetch };

  // State machine for issuing addresses - this is really sneaky!!!!!
  // The cache takes 1 cycle to respond to a cache request and is generally
  // not pipelined - but here we treat it as if it is pipeline by changing
  // the address during a single cache access.  
  //
  // We speculatively increment to the next cache line before we know if the
  // previous cache request hit or missed.  If it hits then we're fine and
  // we get pipelined outputs from the cache, but if not, we have to rewind 
  // our increment and wait for the data to come in.
  //
  // NOTE: We only do this for loads - writes write on the second clock
  always@(posedge clk)
  begin
    if (!resetn)
      issue_state<=ISSUE_IDLE;
    else
      case(issue_state)
        ISSUE_IDLE:
          issue_state<= (addr_sneakyload) ? 
                          ISSUE_INITIAL_INCREMENT: MUNIT_IDLE;
        ISSUE_INITIAL_INCREMENT:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall && !doneall) ? ISSUE_INITIAL_INCREMENT :
                                            ISSUE_INCREMENTED;
        ISSUE_INCREMENTED:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_none) ? ISSUE_WAITONE : ISSUE_INCREMENTED;
        // This stage ignores the speculated result following a miss
        ISSUE_WAITONE:      
          issue_state<= ISSUE_WAITING;
        ISSUE_WAITING:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_some) ? ISSUE_INITIAL_INCREMENT : ISSUE_WAITING;
      endcase
  end

  assign addr_sneakyload=(shifter_load && op_memop && ~op_we && ~op_pattern[1] && enable);
                  //(shifter_shift && ~shifter_load && ~op_we_s2 && enable_s2));

  assign addr_advance=(issue_state==ISSUE_INITIAL_INCREMENT) ||
                      (issue_state==ISSUE_INCREMENTED && parhits_some) ||
                      (issue_state==ISSUE_WAITING && parhits_some) ||
                      (op_we_s2 && parhits_some && ~cachedata_stillvalid);

  assign addr_rewind=(issue_state==ISSUE_INCREMENTED && parhits_none);

  always@(posedge clk)
    if (!resetn || shifter_load || last_subvector_s2&~stall&~shifter_load )
      vpid<=0;
    else if (shifter_shift)
      vpid<=vpid + ((shifter_jump) ? 8 : 1'b1);

  // Loads in a new request if it is a memory request. quick_loaded is high 
  // the cycle after the new request has been loaded and instructs the
  // logic below to continue requesting from memory (even though munit_state
  // will be in the DONE state).
  // Note we don't do quickload on last subvector - a speculative miss can 
  // cause the quickloaded memory instruction to re-load its cache line from
  // memory - but if that line was dirty we would have missed the changes.
    // I don't think this is true anymore PY 07/2008
  assign do_quick_load=doneall&enable_s2&op_memop_s2&~op_we_s2&
                       ~last_subvector_s2&enable&op_memop&~op_we&~op_pattern[1];
  always@(posedge clk)
    quick_loaded<=shifter_load&do_quick_load;

  assign dmem_en = ((shifter_jump) ? (|vshifted_mask[8-1:0]) : 
                                      vshifted_mask[0]) && 
                    (munit_state==MUNIT_ISSUE || quick_loaded);
  assign dmem_we=op_we_s2;

  /*********************
  * We don't want cache miss signal to be an add/sub select signal or else it
  * will propagate through the adder.  We perform the addition/subtraction
  * in parallel with the miss calculation and use the miss as a mux select
  *********************/
  wire [ 32-1 : 0 ] dmem_address_next /* synthesis keep */;
  wire [ 32-1 : 0 ] dmem_address_prev /* synthesis keep */;
  assign dmem_address_next=dmem_address + stride;
  assign dmem_address_prev=dmem_address - stride;

  always@(posedge clk)
    if (!resetn)
      dmem_address<=0;
    else if (shifter_load)
      dmem_address<=address[32-1:0];
    else if (shifter_shift)
      dmem_address<= (!shifter_jump) ? 
              vshifted_address[((8>1) ? 1:0)
                *32 +: 32] :
              vshifted_address[
                ((8<=8) ? 0 : 8)
                  *32 +: 32];
    //Fetch next cache line if partial match on initial cache access
    //else if (parhits_some && ~cachedata_stillvalid)
    else if (addr_advance)
      dmem_address<=dmem_address_next;
    else if (addr_rewind)
      dmem_address<=dmem_address_prev;

  always@(posedge clk)
    if (!resetn)
      dmem_prefetch<=0;
    //else if (shifter_load)
    else
      dmem_prefetch<=prefetch[32-1:0];

/*************************** Mem Write LOGIC ********************************/

// Generate byte/halfword alignment circuitry for each word                  

       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[0*32 +: 32]}),
      .d_address(vshifted_address[0*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*0 +: 4]),  
      .d_writedataout(_vwritedata[0*32 +: 32]));

        


       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[1*32 +: 32]}),
      .d_address(vshifted_address[1*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*1 +: 4]),  
      .d_writedataout(_vwritedata[1*32 +: 32]));

        


       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[2*32 +: 32]}),
      .d_address(vshifted_address[2*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*2 +: 4]),  
      .d_writedataout(_vwritedata[2*32 +: 32]));

        


       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[3*32 +: 32]}),
      .d_address(vshifted_address[3*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*3 +: 4]),  
      .d_writedataout(_vwritedata[3*32 +: 32]));

        


       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[4*32 +: 32]}),
      .d_address(vshifted_address[4*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*4 +: 4]),  
      .d_writedataout(_vwritedata[4*32 +: 32]));

        


       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[5*32 +: 32]}),
      .d_address(vshifted_address[5*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*5 +: 4]),  
      .d_writedataout(_vwritedata[5*32 +: 32]));

        


       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[6*32 +: 32]}),
      .d_address(vshifted_address[6*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*6 +: 4]),  
      .d_writedataout(_vwritedata[6*32 +: 32]));

        


       vstore_data_translator vstore_data_translator_i(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[7*32 +: 32]}),
      .d_address(vshifted_address[7*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*7 +: 4]),  
      .d_writedataout(_vwritedata[7*32 +: 32]));

        


  always@*
  begin
    dmem_writedata=0;
    dmem_byteen=0;
    for (l=0; l<8; l=l+1)
      if (dmem_address[31:7-3] == 
          vshifted_address[32*l+7-3 +: 
                            32-7+3])
      begin
        dmem_writedata=dmem_writedata| (_vwritedata[l*32 +: 32] << 
            {vshifted_address[32*l+2 +: {7-5], {5{1'b0}}}});
        if (vshifted_mask[l] && (shifter_jump || (l==0)))
          dmem_byteen=dmem_byteen | (_vbyteen[4*l+:4]<<
            {vshifted_address[32*l+2 +: {7-5], {2{1'b0}}}});
      end
  end

/*************************** Mem Receive LOGIC ********************************/
  reg wastedcache;  //Identifies when a cache request is made but then aborted because the data was found in the previous cache request which is "stillvalid"

  // This is specific to cache timing - 1 cycle
  always@(posedge clk)
    if (!resetn) 
    begin
      dmem_valid<=0;
      dmem_readdata_address<=0;
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end
    else
    begin
      dmem_valid<=dmem_en;
      if (!wastedcache)
        dmem_readdata_address<=dmem_address;
      //WE know for sure data is still valid after the cache hit since it takes
      //two clock cyles to get new cacheline
      //wastedcache<=shifter_shift&cachedata_stillvalid;
      //cachedata_stillvalid<=dmem_valid&~dmem_wait;
      //Disable this since now we speculatively advance the address
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end

  // Find out which parallel lanes hit in the cache
  always@*
    for (j=0; j<8; j=j+1)
      //This signal tells us a cache request succeeded (for either load/store)
      //Each bit corresponds to a parallel lane
      _parhits[j]= vshifted_mask[j] &&
          ((dmem_valid&(dmem_cachematch&~op_we_s2 || op_we_s2&~dmem_wait)) || 
              (cachedata_stillvalid&~op_we_s2)) &&
            (vshifted_address[j*32+7-3 +: 
                             32-7+3] ==
               dmem_readdata_address[31:7-3]);

  //For operations that don't jump, just look at first bit of _parhits
  assign parhits=(shifter_jump) ? _parhits : {8{_parhits[0]}};

  // Detect all parallel lanes hitting
  assign parhits_all=&(parhits|~vshifted_mask[8-1:0]);
  // Detect cache misses
  assign parhits_none=~|(parhits);
  // Detect some misses - fetch next cache line
  assign parhits_some=~parhits_none && ~parhits_all;

  //If 8<=8 then we will never do a jump, so we make
  //compiler ignore this statement if that's the case
  always@(posedge clk)
    if (!resetn || shifter_load)
      parhits_done<= (shifter_jump_s1) ?  ~vmask : {8{~vmask[0]}};
    else if ( parhits_doneall )
      parhits_done<= (shifter_jump && (8>8)) ? 
                    ~vshifted_mask[ 
                        ((8>8) ? 8 : 0) 
                      +: 8] :
                    {8{(8>1) ? ~vshifted_mask[1] : 1'b0}};
    else           
      parhits_done<= parhits_done|parhits;

  assign parhits_doneall=&(parhits|parhits_done);

  assign _vreaddata_we= ((shifter_jump) ? _parhits : _parhits[0]) 
                          << vpid[3-1:0];

  always@(posedge clk)
    if (!resetn || ~enable_s2)  // Force to zero if unit not used
      vreaddata_we<= 0 ;
    else           // write to regfile if a) is a load, b) we haven't already
      vreaddata_we<= {8{~op_we_s2}} & _vreaddata_we & ~done;

  //Select signal for crossbar either uses bits from corresponding parallel
  //lane address or all selects are set to the same if we're not jumping
  always@*
    for (n=0; n<8; n=n+1)
      crossbar_sel[(7-5)*n +: (7-5)]=
            (!shifter_jump) ? 
              {8{vshifted_address[2 +: 7-5]}} :
              vshifted_address[n*32+2 +: (7-5)];

  vmem_crossbar_128_7_8_32_5 vmem_crossbar(
      .clk(), .resetn(),
      .sel(crossbar_sel),
      .in(dmem_readdata),
      .out(crossbar));

                // Generate byte/halfword alignment circuitry for each word                   

      vload_data_translator load_data_translator0(
      .d_readdatain(crossbar[32*(0+1)-1:32*0]),
      .d_address( (shifter_jump) ? vshifted_address[32*0 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(0+1)-1:32*0])
      );

        


      vload_data_translator load_data_translator1(
      .d_readdatain(crossbar[32*(1+1)-1:32*1]),
      .d_address( (shifter_jump) ? vshifted_address[32*1 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(1+1)-1:32*1])
      );

        


      vload_data_translator load_data_translator2(
      .d_readdatain(crossbar[32*(2+1)-1:32*2]),
      .d_address( (shifter_jump) ? vshifted_address[32*2 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(2+1)-1:32*2])
      );

        


      vload_data_translator load_data_translator3(
      .d_readdatain(crossbar[32*(3+1)-1:32*3]),
      .d_address( (shifter_jump) ? vshifted_address[32*3 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(3+1)-1:32*3])
      );

        


      vload_data_translator load_data_translator4(
      .d_readdatain(crossbar[32*(4+1)-1:32*4]),
      .d_address( (shifter_jump) ? vshifted_address[32*4 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(4+1)-1:32*4])
      );

        


      vload_data_translator load_data_translator5(
      .d_readdatain(crossbar[32*(5+1)-1:32*5]),
      .d_address( (shifter_jump) ? vshifted_address[32*5 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(5+1)-1:32*5])
      );

        


      vload_data_translator load_data_translator6(
      .d_readdatain(crossbar[32*(6+1)-1:32*6]),
      .d_address( (shifter_jump) ? vshifted_address[32*6 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(6+1)-1:32*6])
      );

        


      vload_data_translator load_data_translator7(
      .d_readdatain(crossbar[32*(7+1)-1:32*7]),
      .d_address( (shifter_jump) ? vshifted_address[32*7 +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(7+1)-1:32*7])
      );

        


always@(posedge clk)
    //zero if unit not used
    if (!resetn || ~enable_s2 || (~stall&~enable) || op_we_s2)
      vreaddata<= 0 ;
    else                // Don't write to regfile unless this is a load!
      vreaddata<= {8/8{__vreaddata}};

/*************************** DONE LOGIC *********************************/

  // Done register has 1 bit for each lane, we are finished when done is all 1's
  always@(posedge clk)
    if (!resetn)
      done<=0;
    //Don't check for MUNIT_DONE, because of quickload we are reading then
    //else if (shifter_load || (munit_state==MUNIT_DONE) )
    else if (shifter_load || doneall || (munit_state==MUNIT_DONE && (op_we_s2 || op_pattern_s2[1])))
      done<=~vmask;
    else 
      done<=done|_vreaddata_we;

  assign doneall=(&(done|_vreaddata_we));

  always@(posedge clk)
  begin
    if (!resetn)
      munit_state<=MUNIT_IDLE;
    else
      case(munit_state)
        MUNIT_IDLE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        MUNIT_ISSUE:
          munit_state<= doneall ? MUNIT_DONE : MUNIT_ISSUE;
        MUNIT_SHIFT:
          munit_state<= (next_sa_count_zero) ? 
                            (doublewrite) ? MUNIT_XTRAWRITE : MUNIT_DONE : 
                            MUNIT_SHIFT;
        MUNIT_XTRAWRITE:  //Spilled over subvector bounds when shifting 
          munit_state<= MUNIT_DONE;
        MUNIT_DONE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        default:
          munit_state<=MUNIT_IDLE;
      endcase
  end

  pipereg_1  dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_dst_we));

  pipereg_1 vsdstwepipe (
    .d( in_vs_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_vs_dst_we));

endmodule


module activation_32(
 input clk,
 input resetn,
 input en,
 input stall,
 input [32-1:0] a,
 output reg[32-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      if(a>0)
        out <= a;
      else
        out <= 0;
end

endmodule

        
