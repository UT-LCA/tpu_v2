module permute #(
  parameter WIDTH = 16,
  parameter NUMSTAGES = 8,
  parameter REGIDWIDTH = 8,
  parameter LOGNUMSTAGES = $clog2(NUMSTAGES)
)(
 input clk,
 input resetn,
 input read,
 input en,
 input row_col_op,
 input [NUMSTAGES*LOGNUMSTAGES-1:0] row_col_num,

 input load,
 input [NUMSTAGES * WIDTH-1:0]a,

 input [REGIDWIDTH-1:0] in_dst,
 input in_dst_we,
 input [NUMSTAGES-1:0] in_dst_mask,

 output reg [REGIDWIDTH-1:0] out_dst,
 output reg [NUMSTAGES-1:0] out_dst_mask,
 output reg out_dst_we,
 output reg [NUMSTAGES * WIDTH-1 : 0] out
);

parameter READ_STATE = 2'b00;
parameter ROWOP_STATE = 2'b01;
parameter COLOP_STATE = 2'b10;
parameter WRITE_STATE = 2'b11;

wire [NUMSTAGES * NUMSTAGES * WIDTH - 1:0] data;

reg [NUMSTAGES * LOGNUMSTAGES-1:0 ] rowsel;
reg [NUMSTAGES * LOGNUMSTAGES-1:0 ] colsel;

reg [LOGNUMSTAGES:0] count;
reg [31:0] i;

reg [1:0] p_state, n_state;
reg opsel;
reg [NUMSTAGES-1:0] load_en;
reg shuffle;
reg [NUMSTAGES * NUMSTAGES * WIDTH -1: 0] data_in;

always@(posedge clk)begin
  if(!resetn)begin
    out_dst <= 0;
    out_dst_we <= 0;
    out_dst_mask <= 0;
  end
  else begin
    out_dst <= in_dst;
    out_dst_we <= in_dst_we;
    out_dst_mask <= in_dst_mask;
  end
end


genvar g_i,g_j;

generate
  for(g_i = 0; g_i < NUMSTAGES; g_i = g_i + 1)begin
    for(g_j = 0; g_j < NUMSTAGES; g_j = g_j + 1)begin
       permute_elm #(.WIDTH(WIDTH), .MUXWIDTH(NUMSTAGES), .MUXSEL(LOGNUMSTAGES)) inst_elm(
         .clk(clk),
         .resetn(resetn),
         .row_sel(rowsel[g_i * LOGNUMSTAGES +: LOGNUMSTAGES]),
         .col_sel(colsel[g_j * LOGNUMSTAGES +: LOGNUMSTAGES]),
         .row_col_op(opsel),
         .in_row_data(data[((g_i+1) * NUMSTAGES * WIDTH)-1 : g_i * (NUMSTAGES * WIDTH)]),
         .in_col_data({data[(( 0 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 1 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 2 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 3 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 4 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 5 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 6 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 7 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ]
                      }),
         .en(shuffle),
         .load(load_en[g_i]),
         .data_in(data_in[g_i * (NUMSTAGES * WIDTH) + g_j * WIDTH +: WIDTH]),
         .out_data(data[(g_i * NUMSTAGES * WIDTH) + (g_j * WIDTH) +: WIDTH])
       );
    end
  end
endgenerate

always@(*) begin
  data_in = 'h0;
  case(count)
    4'h0:data_in[1 * (NUMSTAGES * WIDTH)-1: 0 * (NUMSTAGES*WIDTH)] = a;
    4'h1:data_in[2 * (NUMSTAGES * WIDTH)-1: 1 * (NUMSTAGES*WIDTH)] = a;
    4'h2:data_in[3 * (NUMSTAGES * WIDTH)-1: 2 * (NUMSTAGES*WIDTH)] = a;
    4'h3:data_in[4 * (NUMSTAGES * WIDTH)-1: 3 * (NUMSTAGES*WIDTH)] = a;
    4'h4:data_in[5 * (NUMSTAGES * WIDTH)-1: 4 * (NUMSTAGES*WIDTH)] = a;
    4'h5:data_in[6 * (NUMSTAGES * WIDTH)-1: 5 * (NUMSTAGES*WIDTH)] = a;
    4'h6:data_in[7 * (NUMSTAGES * WIDTH)-1: 6 * (NUMSTAGES*WIDTH)] = a;
    4'h7:data_in[8 * (NUMSTAGES * WIDTH)-1: 7 * (NUMSTAGES*WIDTH)] = a;
  endcase
end

always@(*) begin
  out = 0;
  case(count)
    4'h1: out = data[8*(NUMSTAGES * WIDTH) - 1 : 7 * (NUMSTAGES* WIDTH)];
    4'h2: out = data[7*(NUMSTAGES * WIDTH) - 1 : 6 * (NUMSTAGES* WIDTH)];
    4'h3: out = data[6*(NUMSTAGES * WIDTH) - 1 : 5 * (NUMSTAGES* WIDTH)];
    4'h4: out = data[5*(NUMSTAGES * WIDTH) - 1 : 4 * (NUMSTAGES* WIDTH)];
    4'h5: out = data[4*(NUMSTAGES * WIDTH) - 1 : 3 * (NUMSTAGES* WIDTH)];
    4'h6: out = data[3*(NUMSTAGES * WIDTH) - 1 : 2 * (NUMSTAGES* WIDTH)];
    4'h7: out = data[2*(NUMSTAGES * WIDTH) - 1 : 1 * (NUMSTAGES* WIDTH)];
    4'h8: out = data[1*(NUMSTAGES * WIDTH) - 1 : 0 * (NUMSTAGES* WIDTH)];
  endcase
end


always@(posedge clk)begin
  if(!resetn)
    count <= 'h0;
  else begin
    if(en & load)begin
      count <= count + 1;
    end
    else if(en & read)
      count <= count - 1 ;
  end
end

always@(*)begin
  load_en = 8'h0;
  if(en & load)begin
      case(count)
        3'h0: load_en = 8'h1;
        3'h1: load_en = 8'h2;
        3'h2: load_en = 8'h4;
        3'h3: load_en = 8'h8;
        3'h4: load_en = 8'h10;
        3'h5: load_en = 8'h20;
        3'h6: load_en = 8'h40;
        3'h7: load_en = 8'h80;
      endcase
  end
end


always@(*)begin
    shuffle = 1'b0;
    rowsel = {3'h7,3'h6,3'h5,3'h4,3'h3,3'h2,3'h1,3'h0};
    colsel = {3'h7,3'h6,3'h5,3'h4,3'h3,3'h2,3'h1,3'h0};
    if(en & ~load & ~read)begin
        shuffle = 1'b1;
        rowsel = row_col_num; 
        colsel = row_col_num;
        if(row_col_op)
            opsel = 1'b1; 
        else
            opsel = 1'b0; 
    end
end

endmodule


module permute_elm #(
  parameter MUXWIDTH = 8,
  parameter WIDTH = 16,
  parameter MUXSEL = $clog2(MUXWIDTH)
)(
  input clk,
  input resetn,
  input [MUXSEL-1:0] row_sel,
  input [MUXSEL-1:0] col_sel,
  input row_col_op,
  input [MUXWIDTH*WIDTH-1:0] in_row_data,  
  input [MUXWIDTH*WIDTH-1:0] in_col_data,  

  input load,
  input en,
  input [WIDTH-1:0] data_in,

  output reg [WIDTH-1:0] out_data
);

reg [WIDTH-1:0] row_data;
reg [WIDTH-1:0] col_data;
reg [WIDTH-1:0] n_data;

always@(*)begin
  case(row_sel)
    3'h0:begin row_data = in_row_data[1 * WIDTH -1: 0 * WIDTH]; end
    3'h1:begin row_data = in_row_data[2 * WIDTH -1: 1 * WIDTH]; end
    3'h2:begin row_data = in_row_data[3 * WIDTH -1: 2 * WIDTH]; end
    3'h3:begin row_data = in_row_data[4 * WIDTH -1: 3 * WIDTH]; end
    3'h4:begin row_data = in_row_data[5 * WIDTH -1: 4 * WIDTH]; end
    3'h5:begin row_data = in_row_data[6 * WIDTH -1: 5 * WIDTH]; end
    3'h6:begin row_data = in_row_data[7 * WIDTH -1: 6 * WIDTH]; end
    3'h7:begin row_data = in_row_data[8 * WIDTH -1: 7 * WIDTH]; end
  endcase
  case(col_sel)
    3'h0:begin col_data = in_col_data[1 * WIDTH -1: 0 * WIDTH]; end
    3'h1:begin col_data = in_col_data[2 * WIDTH -1: 1 * WIDTH]; end
    3'h2:begin col_data = in_col_data[3 * WIDTH -1: 2 * WIDTH]; end
    3'h3:begin col_data = in_col_data[4 * WIDTH -1: 3 * WIDTH]; end
    3'h4:begin col_data = in_col_data[5 * WIDTH -1: 4 * WIDTH]; end
    3'h5:begin col_data = in_col_data[6 * WIDTH -1: 5 * WIDTH]; end
    3'h6:begin col_data = in_col_data[7 * WIDTH -1: 6 * WIDTH]; end
    3'h7:begin col_data = in_col_data[8 * WIDTH -1: 7 * WIDTH]; end
  endcase
  
  if(row_col_op)
    n_data = row_data;
  else
    n_data = col_data; 
end

always@(posedge clk)begin
  if(~resetn)
    out_data <= 'h0;
  else
   if(load)
     out_data <= data_in;
   else if(en)
     out_data <= n_data;
end

endmodule
