module trp_unit #(parameter WIDTH=4)(
 input clk,
 input resetn,
 input en,
 input [WIDTH-1:0] a,
 input [1:0] mode,
 input read,
 output busy,
 output reg valid,
 output reg[WIDTH-1:0] out
);

reg en_reduction;
reg en_transpose;
reg read_transpose;
reg read_reduction;
wire transpose_busy;
wire reduction_busy;
wire reduction_done;
wire [WIDTH-1:0] reduction_out;
wire [WIDTH-1:0] transpose_out;
wire reduction_valid; 
 
//assign busy = transpose_busy || reduction_busy;
assign busy =  reduction_busy;

always@(*)begin
//  if(mode == 2'b11)begin
//    en_transpose = en;
//    en_reduction = 1'b0;
//    read_transpose = read;
//    read_reduction = 1'b0;
//  end
//  else begin
//    en_transpose = 1'b0;
    en_reduction = en;
//    read_transpose = 1'b0;
//    read_reduction = read;
//  end
end

always@(*)begin
 // if(transpose_busy)begin
 //   out = transpose_out;
 //   valid = 1'b1;
 // end
 // else if(reduction_done)begin
    out = reduction_out;
    valid = reduction_valid;
 // end
 // else begin
 //   out = 'h0;
 //   valid = 1'b0;
 // end
end

reduction_layer u_reduction_layer(
  .clk(clk),
  .resetn(resetn),
  .en(en_reduction),
  .reduction_type(mode),
  .a(a),
  .reduced_out(reduction_out),
  .done(reduction_done),
  .busy(reduction_busy),
  .valid(reduction_valid)
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
