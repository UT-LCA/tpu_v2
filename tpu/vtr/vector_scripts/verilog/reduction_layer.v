
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
module reduction_layer_10_3_32_5_10 
(
  input clk,
  input resetn, //resets the processing elements
  input en, //indicates valid reduction operation
  input  [1:0] reduction_type, //can have 3 values: 0 (Add), 1 (Max), 2 (Min)
  input read,
  input  [8 * 10 -1:0] a, // input data to reduction logic
  output [8 * 10 -1:0] reduced_out, //output
  output reg done, //output is valid when done is 1
  output reg busy
);

wire [8 * 10 -1:0] reduced_out_1; //output
wire [8 * 10 -1:0] reduced_out_add; //output
wire [10 + 3-1:0] reduced_out_unrounded;

reduction_unit_10_3 ucu(
  .clk(clk),
  .reset(reset_reduction_unit),
  .inp0(a[1*10-1:0*10]), 
  .inp1(a[2*10-1:1*10]), 
  .inp2(a[3*10-1:2*10]), 
  .inp3(a[4*10-1:3*10]), 
  .inp4(a[5*10-1:4*10]), 
  .inp5(a[6*10-1:5*10]), 
  .inp6(a[7*10-1:6*10]), 
  .inp7(a[8*10-1:7*10]), 
  .mode(reduction_type),
  .outp(reduced_out_unrounded)
);

////////////////////////////////////////////////////////////////
// Rounding of the output of reduction unit (from 20 bits to 16 bits).
// This is required only when reduction type is "sum"
////////////////////////////////////////////////////////////////
rounding_13_10 u_round(.i_data(reduced_out_unrounded), .o_data(reduced_out_add));

assign reduced_out_1 = (reduction_type==2'b0) ? reduced_out_add : reduced_out_unrounded[10-1:0];
assign reduced_out = {8{reduced_out_1}};

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
