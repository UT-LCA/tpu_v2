module matmul_8x8(
 clk,
 //Reset for the whole matmul
 reset,
 //Reset for the PEs. Typically connected to 'reset'
 pe_reset,
 //When this is asserted, the matmul operation starts. This can remain
 //asserted during the execution, but is not required to.
 start, 
 //This is asserted a cycle after the matmul start operation. It stays
 //asserted until the matmul starts shifting out the output data.
 in_progress,
 //This is not used any more. But it is a pulse, and is asserted when
 //the execution (including output shift out) is finished.
 done,
 //Input data matrix A. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 a_data,
 //Input data matrix B. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 b_data,
 //Output data matrix C. For MAT_MUL_SIZE=8, 64 values come out over 8 cycles.
 //8 values in one cycle.
 c_data_out, 
 //This isn't used anymore. It stays asserted when the output data matrix C
 //is being shifted out.
 c_data_available,
 //Masks for input matrices A and B. These are used when we want to use this 
 //unit to multiply matrices that are less than 8x8 (eg: 6x4 with a 4x5 matrix).
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols
);

 input clk;
 input reset;
 input pe_reset;
 input start;
 output in_progress;
 output done;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data_out;
 output c_data_available;

 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

//////////////////////////////////////////////////////////////////////////
// Logic for clock counting and when to assert done
//////////////////////////////////////////////////////////////////////////
reg in_progress;
reg done_mat_mul;
wire matmul_op_in_progress;
reg shift_out_data;
reg shift_in_data;
wire start_pulse;
wire [7:0] clk_cnt_for_done;
wire [7:0] clk_cnt_for_latching_c_data;
wire [7:0] clk_cnt_for_shifting_inputs;

assign done = done_mat_mul;

reg start_delayed;
always @(posedge clk) begin
  if (reset) begin
    start_delayed <= 0;
  end 
  else begin
    start_delayed <= start;
  end
end

assign start_pulse = start & (~start_delayed);

//This signal is used in other modules instantiated in this design.
//It stays high during the entire time matmul is working.
//                            _
//start_pulse           _____| |___________________________________________
//                              ___
//shift_in_data         _______|   |_______________________________________
//                                   ________________________
//in_progress           ____________|                        |______________
//                                                            _______
//shift_out_data        _____________________________________|       |______
//                            _______________________________________
//matmul_op_in_progress _____|                                       |______
assign matmul_op_in_progress = start_pulse|shift_in_data|in_progress|shift_out_data;

//This is 7 bits because the expectation is that clock count will be pretty
//small. For large matmuls, this will need to increased to have more bits.
//In general, a systolic multiplier takes 4*N-2+P cycles, where N is the size 
//of the matmul and P is the number of pipleine stages in the MAC block.
reg [7:0] clk_cnt;

//Number of cycles to assert matmul done. This includes the cycles to shift out the results. 
//This is hardcoded here, because this was generated using a script.
assign clk_cnt_for_done = 
                          (34);  

//Number of cycles at which we latch the output data and start shifting it out.
assign clk_cnt_for_latching_c_data =                        
                          (27);  

//Number of cycles at which we finish shifting inputs into the matmul.
//Note that while this shifting is happening, the matmul is calculating
//outputs in its PEs.
assign clk_cnt_for_shifting_inputs =                        
                          (1);  //Ideally this should have been 7, but if we keep this as
                                //7, then stall signal is asserted a bit later than required

always @(posedge clk) begin
  if (reset) begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (start_pulse == 1'b1) begin
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 1;
  end
  else if (clk_cnt == clk_cnt_for_shifting_inputs) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 1;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_latching_c_data) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 1;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_done) begin
    done_mat_mul <= 1;
    clk_cnt <= clk_cnt + 1;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if ((in_progress == 1) || (shift_out_data == 1) || (shift_in_data == 1)) begin
    clk_cnt <= clk_cnt + 1;
  end    
  else begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
  end
end

wire [`DWIDTH-1:0] a0_data;
wire [`DWIDTH-1:0] a1_data;
wire [`DWIDTH-1:0] a2_data;
wire [`DWIDTH-1:0] a3_data;
wire [`DWIDTH-1:0] a4_data;
wire [`DWIDTH-1:0] a5_data;
wire [`DWIDTH-1:0] a6_data;
wire [`DWIDTH-1:0] a7_data;
wire [`DWIDTH-1:0] b0_data;
wire [`DWIDTH-1:0] b1_data;
wire [`DWIDTH-1:0] b2_data;
wire [`DWIDTH-1:0] b3_data;
wire [`DWIDTH-1:0] b4_data;
wire [`DWIDTH-1:0] b5_data;
wire [`DWIDTH-1:0] b6_data;
wire [`DWIDTH-1:0] b7_data;
wire [`DWIDTH-1:0] a1_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_1;
wire [`DWIDTH-1:0] a3_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_1;
wire [`DWIDTH-1:0] a4_data_delayed_2;
wire [`DWIDTH-1:0] a4_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_1;
wire [`DWIDTH-1:0] a5_data_delayed_2;
wire [`DWIDTH-1:0] a5_data_delayed_3;
wire [`DWIDTH-1:0] a5_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_1;
wire [`DWIDTH-1:0] a6_data_delayed_2;
wire [`DWIDTH-1:0] a6_data_delayed_3;
wire [`DWIDTH-1:0] a6_data_delayed_4;
wire [`DWIDTH-1:0] a6_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_1;
wire [`DWIDTH-1:0] a7_data_delayed_2;
wire [`DWIDTH-1:0] a7_data_delayed_3;
wire [`DWIDTH-1:0] a7_data_delayed_4;
wire [`DWIDTH-1:0] a7_data_delayed_5;
wire [`DWIDTH-1:0] a7_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_7;
wire [`DWIDTH-1:0] b1_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_1;
wire [`DWIDTH-1:0] b3_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_1;
wire [`DWIDTH-1:0] b4_data_delayed_2;
wire [`DWIDTH-1:0] b4_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_1;
wire [`DWIDTH-1:0] b5_data_delayed_2;
wire [`DWIDTH-1:0] b5_data_delayed_3;
wire [`DWIDTH-1:0] b5_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_1;
wire [`DWIDTH-1:0] b6_data_delayed_2;
wire [`DWIDTH-1:0] b6_data_delayed_3;
wire [`DWIDTH-1:0] b6_data_delayed_4;
wire [`DWIDTH-1:0] b6_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_1;
wire [`DWIDTH-1:0] b7_data_delayed_2;
wire [`DWIDTH-1:0] b7_data_delayed_3;
wire [`DWIDTH-1:0] b7_data_delayed_4;
wire [`DWIDTH-1:0] b7_data_delayed_5;
wire [`DWIDTH-1:0] b7_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_7;


//////////////////////////////////////////////////////////////////////////
// Instantiation of systolic data setup
//////////////////////////////////////////////////////////////////////////
systolic_data_setup u_systolic_data_setup(
.clk(clk),
.reset(reset),
.matmul_op_in_progress(matmul_op_in_progress),
.a_data(a_data),
.b_data(b_data),
.clk_cnt(clk_cnt),
.a0_data(a0_data),
.b0_data(b0_data),
.a1_data_delayed_1(a1_data_delayed_1),
.b1_data_delayed_1(b1_data_delayed_1),
.a2_data_delayed_2(a2_data_delayed_2),
.b2_data_delayed_2(b2_data_delayed_2),
.a3_data_delayed_3(a3_data_delayed_3),
.b3_data_delayed_3(b3_data_delayed_3),
.a4_data_delayed_4(a4_data_delayed_4),
.b4_data_delayed_4(b4_data_delayed_4),
.a5_data_delayed_5(a5_data_delayed_5),
.b5_data_delayed_5(b5_data_delayed_5),
.a6_data_delayed_6(a6_data_delayed_6),
.b6_data_delayed_6(b6_data_delayed_6),
.a7_data_delayed_7(a7_data_delayed_7),
.b7_data_delayed_7(b7_data_delayed_7),

.validity_mask_a_rows(validity_mask_a_rows),
.validity_mask_a_cols(validity_mask_a_cols),
.validity_mask_b_rows(validity_mask_b_rows),
.validity_mask_b_cols(validity_mask_b_cols)

);

wire [`DWIDTH-1:0] a0;
wire [`DWIDTH-1:0] a1;
wire [`DWIDTH-1:0] a2;
wire [`DWIDTH-1:0] a3;
wire [`DWIDTH-1:0] a4;
wire [`DWIDTH-1:0] a5;
wire [`DWIDTH-1:0] a6;
wire [`DWIDTH-1:0] a7;
wire [`DWIDTH-1:0] b0;
wire [`DWIDTH-1:0] b1;
wire [`DWIDTH-1:0] b2;
wire [`DWIDTH-1:0] b3;
wire [`DWIDTH-1:0] b4;
wire [`DWIDTH-1:0] b5;
wire [`DWIDTH-1:0] b6;
wire [`DWIDTH-1:0] b7;

assign a0 = a0_data          ;
assign a1 = a1_data_delayed_1;
assign a2 = a2_data_delayed_2;
assign a3 = a3_data_delayed_3;
assign a4 = a4_data_delayed_4;
assign a5 = a5_data_delayed_5;
assign a6 = a6_data_delayed_6;
assign a7 = a7_data_delayed_7;

assign b0 = b0_data          ;
assign b1 = b1_data_delayed_1;
assign b2 = b2_data_delayed_2;
assign b3 = b3_data_delayed_3;
assign b4 = b4_data_delayed_4;
assign b5 = b5_data_delayed_5;
assign b6 = b6_data_delayed_6;
assign b7 = b7_data_delayed_7;

wire [`DWIDTH-1:0] matrixC0_0;
wire [`DWIDTH-1:0] matrixC0_1;
wire [`DWIDTH-1:0] matrixC0_2;
wire [`DWIDTH-1:0] matrixC0_3;
wire [`DWIDTH-1:0] matrixC0_4;
wire [`DWIDTH-1:0] matrixC0_5;
wire [`DWIDTH-1:0] matrixC0_6;
wire [`DWIDTH-1:0] matrixC0_7;
wire [`DWIDTH-1:0] matrixC1_0;
wire [`DWIDTH-1:0] matrixC1_1;
wire [`DWIDTH-1:0] matrixC1_2;
wire [`DWIDTH-1:0] matrixC1_3;
wire [`DWIDTH-1:0] matrixC1_4;
wire [`DWIDTH-1:0] matrixC1_5;
wire [`DWIDTH-1:0] matrixC1_6;
wire [`DWIDTH-1:0] matrixC1_7;
wire [`DWIDTH-1:0] matrixC2_0;
wire [`DWIDTH-1:0] matrixC2_1;
wire [`DWIDTH-1:0] matrixC2_2;
wire [`DWIDTH-1:0] matrixC2_3;
wire [`DWIDTH-1:0] matrixC2_4;
wire [`DWIDTH-1:0] matrixC2_5;
wire [`DWIDTH-1:0] matrixC2_6;
wire [`DWIDTH-1:0] matrixC2_7;
wire [`DWIDTH-1:0] matrixC3_0;
wire [`DWIDTH-1:0] matrixC3_1;
wire [`DWIDTH-1:0] matrixC3_2;
wire [`DWIDTH-1:0] matrixC3_3;
wire [`DWIDTH-1:0] matrixC3_4;
wire [`DWIDTH-1:0] matrixC3_5;
wire [`DWIDTH-1:0] matrixC3_6;
wire [`DWIDTH-1:0] matrixC3_7;
wire [`DWIDTH-1:0] matrixC4_0;
wire [`DWIDTH-1:0] matrixC4_1;
wire [`DWIDTH-1:0] matrixC4_2;
wire [`DWIDTH-1:0] matrixC4_3;
wire [`DWIDTH-1:0] matrixC4_4;
wire [`DWIDTH-1:0] matrixC4_5;
wire [`DWIDTH-1:0] matrixC4_6;
wire [`DWIDTH-1:0] matrixC4_7;
wire [`DWIDTH-1:0] matrixC5_0;
wire [`DWIDTH-1:0] matrixC5_1;
wire [`DWIDTH-1:0] matrixC5_2;
wire [`DWIDTH-1:0] matrixC5_3;
wire [`DWIDTH-1:0] matrixC5_4;
wire [`DWIDTH-1:0] matrixC5_5;
wire [`DWIDTH-1:0] matrixC5_6;
wire [`DWIDTH-1:0] matrixC5_7;
wire [`DWIDTH-1:0] matrixC6_0;
wire [`DWIDTH-1:0] matrixC6_1;
wire [`DWIDTH-1:0] matrixC6_2;
wire [`DWIDTH-1:0] matrixC6_3;
wire [`DWIDTH-1:0] matrixC6_4;
wire [`DWIDTH-1:0] matrixC6_5;
wire [`DWIDTH-1:0] matrixC6_6;
wire [`DWIDTH-1:0] matrixC6_7;
wire [`DWIDTH-1:0] matrixC7_0;
wire [`DWIDTH-1:0] matrixC7_1;
wire [`DWIDTH-1:0] matrixC7_2;
wire [`DWIDTH-1:0] matrixC7_3;
wire [`DWIDTH-1:0] matrixC7_4;
wire [`DWIDTH-1:0] matrixC7_5;
wire [`DWIDTH-1:0] matrixC7_6;
wire [`DWIDTH-1:0] matrixC7_7;

wire [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data_out;
wire [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data_out;

wire row_latch_en;
//////////////////////////////////////////////////////////////////////////
// Instantiation of the output logic
//////////////////////////////////////////////////////////////////////////
output_logic u_output_logic(
.matmul_op_in_progress(matmul_op_in_progress),
.done_mat_mul(done_mat_mul),
.c_data_out(c_data_out),
.c_data_available(c_data_available),
.clk_cnt(clk_cnt),
.row_latch_en(row_latch_en),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.clk(clk),
.reset(reset)
);

//////////////////////////////////////////////////////////////////////////
// Instantiations of the actual PEs
//////////////////////////////////////////////////////////////////////////
systolic_pe_matrix u_systolic_pe_matrix(
.clk(clk),
.reset(reset),
.pe_reset(pe_reset),
.a0(a0),
.a1(a1),
.a2(a2),
.a3(a3),
.a4(a4),
.a5(a5),
.a6(a6),
.a7(a7),
.b0(b0),
.b1(b1),
.b2(b2),
.b3(b3),
.b4(b4),
.b5(b5),
.b6(b6),
.b7(b7),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.a_data_out(a_data_out),
.b_data_out(b_data_out)
);

endmodule