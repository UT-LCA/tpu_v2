module tmj_portmux(
      input [31:0] count,
      input count_ready,
      output reg count_want,
      output reg [0:0] procresetn_in,
      input procreset_want,
      output reg procreset_ready,
      output reg [0:0] bootloadresetn_in,
      input bootloadreset_want,
      output reg bootloadreset_ready,
      output reg [0:0] traceactivate_in,
      input traceactivate_want,
      output reg traceactivate_ready,
      output reg [31:0] data,
      input datawrite_want,
      output reg datawrite_ready,
      input [8:0] fs_writedata,
      input fs_writedata_ready,
      output reg fs_writedata_want,
      output reg [7:0] fs_readdata,
      input fs_readdata_want,
      output reg fs_readdata_ready,
      input [4:0] trc_addr,
      input trc_addr_ready,
      output reg trc_addr_want,
      input [31:0] trc_data,
      input trc_data_ready,
      output reg trc_data_want,
      input clk
      );

parameter TMJ_MAX_TO_USER_PORT_WIDTH = 32;
parameter TMJ_MAX_FROM_USER_PORT_WIDTH = 32;
parameter TMJ_PORT0_WIDTH = 32;
parameter TMJ_PORT0_PADDED_WIDTH = 32;
parameter TMJ_PORT0_ADDRESS = 1;
parameter TMJ_PORT1_WIDTH = 1;
parameter TMJ_PORT1_PADDED_WIDTH = 8;
parameter TMJ_PORT1_ADDRESS = 2;
parameter TMJ_PORT2_WIDTH = 1;
parameter TMJ_PORT2_PADDED_WIDTH = 8;
parameter TMJ_PORT2_ADDRESS = 3;
parameter TMJ_PORT3_WIDTH = 1;
parameter TMJ_PORT3_PADDED_WIDTH = 8;
parameter TMJ_PORT3_ADDRESS = 4;
parameter TMJ_PORT4_WIDTH = 32;
parameter TMJ_PORT4_PADDED_WIDTH = 32;
parameter TMJ_PORT4_ADDRESS = 5;
parameter TMJ_PORT5_WIDTH = 9;
parameter TMJ_PORT5_PADDED_WIDTH = 16;
parameter TMJ_PORT5_ADDRESS = 6;
parameter TMJ_PORT6_WIDTH = 8;
parameter TMJ_PORT6_PADDED_WIDTH = 8;
parameter TMJ_PORT6_ADDRESS = 7;
parameter TMJ_PORT7_WIDTH = 5;
parameter TMJ_PORT7_PADDED_WIDTH = 8;
parameter TMJ_PORT7_ADDRESS = 8;
parameter TMJ_PORT8_WIDTH = 32;
parameter TMJ_PORT8_PADDED_WIDTH = 32;
parameter TMJ_PORT8_ADDRESS = 9;

parameter TMJ_PORT_ADDRESS_WIDTH = 6;
parameter TMJ_TRANSFER_COUNT_WIDTH = 12;
parameter TMJ_SIGNATURE_WIDTH = 8;
parameter TMJ_ID_WIDTH = 8;
/* Pad the command out to the nearest nibble - 36 bits for now */
parameter TMJ_COMMAND_WIDTH = (TMJ_SIGNATURE_WIDTH
                        + TMJ_ID_WIDTH
                        + TMJ_TRANSFER_COUNT_WIDTH
                        + TMJ_PORT_ADDRESS_WIDTH + 1 + 1);

reg tmj_command_in_progress, tmj_new_command_available, tmj_send_nack;

reg [TMJ_PORT_ADDRESS_WIDTH-1:0] tmj_port_address;
reg tmj_write_command;

reg [TMJ_TRANSFER_COUNT_WIDTH-1:0] tmj_values_to_transfer;

reg [TMJ_MAX_FROM_USER_PORT_WIDTH-1:0] tmj_write_data;
reg [TMJ_MAX_TO_USER_PORT_WIDTH:0] tmj_read_data, tmj_read_data_shifter;
reg [14:0] tmj_write_data_bit_count, tmj_read_data_bit_count, tmj_read_data_shifter_count, tmj_port_data_width;
reg [14:0] tmj_write_data_bit_count_delay1, tmj_write_data_bit_count_delay2;
reg tmj_data_accepted, tmj_data_ready, tmj_read_data_ack;

reg [TMJ_COMMAND_WIDTH-1:0] tmj_from_user_buf;
reg [TMJ_COMMAND_WIDTH-1:0] tmj_new_command;


parameter TMJ_FROM_USER_RAM_END = 8191;
reg tmj_from_user_ram_data, tmj_from_user_ram_we;
reg signed [12:0] tmj_from_user_ram_read_addr, tmj_from_user_ram_write_addr;
wire tmj_from_user_ram_q;

simple_dual_port_ram_single_clock tmj_from_user_ram(
   .data(tmj_from_user_ram_data),
   .read_addr(tmj_from_user_ram_read_addr),
   .write_addr(tmj_from_user_ram_write_addr),
   .we(tmj_from_user_ram_we),
   .clk(clk),
   .q(tmj_from_user_ram_q)
   );

reg signed [12:0] tmj_last_from_user_ram_read_addr, tmj_last_from_user_ram_write_addr;

always @(posedge clk) begin

   if(!tmj_command_in_progress) begin

      /* Valid commands have hex "AC" in the signature field.  We also see spurious commands
       * from the virtual jtag interface, and this lets us ignore them.
       */

      if(tmj_new_command_available) begin
         tmj_send_nack <= 1;
         if (tmj_new_command[TMJ_COMMAND_WIDTH - 1 : TMJ_COMMAND_WIDTH - TMJ_SIGNATURE_WIDTH] == 8'hAC) begin

            tmj_command_in_progress <= 1;
            tmj_values_to_transfer <= tmj_new_command[TMJ_TRANSFER_COUNT_WIDTH-1:0];
            tmj_write_command <= tmj_new_command[TMJ_COMMAND_WIDTH - TMJ_SIGNATURE_WIDTH - TMJ_ID_WIDTH - 2];
            tmj_write_data_bit_count <= 0;
            tmj_write_data_bit_count_delay1 <= 0;
            tmj_write_data_bit_count_delay2 <= 0;
            tmj_last_from_user_ram_read_addr <= tmj_last_from_user_ram_write_addr + 1;
   
            tmj_port_address = tmj_new_command[TMJ_COMMAND_WIDTH - TMJ_SIGNATURE_WIDTH - TMJ_ID_WIDTH - 3
                                      :TMJ_COMMAND_WIDTH - TMJ_SIGNATURE_WIDTH - TMJ_ID_WIDTH - 2 - TMJ_PORT_ADDRESS_WIDTH];
      
            if(tmj_port_address == TMJ_PORT0_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT0_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT1_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT1_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT2_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT2_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT3_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT3_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT4_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT4_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT5_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT5_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT6_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT6_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT7_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT7_PADDED_WIDTH;
            end

            if(tmj_port_address == TMJ_PORT8_ADDRESS) begin
               tmj_port_data_width <= TMJ_PORT8_PADDED_WIDTH;
            end

         end
      end
   end

   else begin
      if(tmj_write_command) begin

         // The user is writing to the circuit

         if(tmj_send_nack) begin
            tmj_read_data[0] <= 0;
            tmj_read_data_bit_count <= 1;
            tmj_send_nack <= 0;
         end

         if(tmj_write_data_bit_count == 0) begin
            tmj_from_user_ram_read_addr = tmj_last_from_user_ram_read_addr
                                        - (tmj_values_to_transfer * tmj_port_data_width);

            if(tmj_from_user_ram_read_addr < 0) begin
               tmj_from_user_ram_read_addr = tmj_from_user_ram_read_addr
                                              + (TMJ_FROM_USER_RAM_END + 1);
            end
         end

         if(tmj_write_data_bit_count == tmj_port_data_width) begin

            tmj_data_accepted = 0;
      
            if(tmj_write_data_bit_count_delay2 == tmj_port_data_width) begin
               if(tmj_port_address == TMJ_PORT1_ADDRESS) begin
                  if(procreset_want && !procreset_ready) begin
                     tmj_data_accepted = 1;
                     procresetn_in <= tmj_write_data[TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT1_PADDED_WIDTH+TMJ_PORT1_WIDTH-1
                                        :TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT1_PADDED_WIDTH];
                     procreset_ready <= 1;
                  end
               end

               if(tmj_port_address == TMJ_PORT2_ADDRESS) begin
                  if(bootloadreset_want && !bootloadreset_ready) begin
                     tmj_data_accepted = 1;
                     bootloadresetn_in <= tmj_write_data[TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT2_PADDED_WIDTH+TMJ_PORT2_WIDTH-1
                                        :TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT2_PADDED_WIDTH];
                     bootloadreset_ready <= 1;
                  end
               end

               if(tmj_port_address == TMJ_PORT3_ADDRESS) begin
                  if(traceactivate_want && !traceactivate_ready) begin
                     tmj_data_accepted = 1;
                     traceactivate_in <= tmj_write_data[TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT3_PADDED_WIDTH+TMJ_PORT3_WIDTH-1
                                        :TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT3_PADDED_WIDTH];
                     traceactivate_ready <= 1;
                  end
               end

               if(tmj_port_address == TMJ_PORT4_ADDRESS) begin
                  if(datawrite_want && !datawrite_ready) begin
                     tmj_data_accepted = 1;
                     data <= tmj_write_data[TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT4_PADDED_WIDTH+TMJ_PORT4_WIDTH-1
                                        :TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT4_PADDED_WIDTH];
                     datawrite_ready <= 1;
                  end
               end

               if(tmj_port_address == TMJ_PORT6_ADDRESS) begin
                  if(fs_readdata_want && !fs_readdata_ready) begin
                     tmj_data_accepted = 1;
                     fs_readdata <= tmj_write_data[TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT6_PADDED_WIDTH+TMJ_PORT6_WIDTH-1
                                        :TMJ_MAX_FROM_USER_PORT_WIDTH-TMJ_PORT6_PADDED_WIDTH];
                     fs_readdata_ready <= 1;
                  end
               end

               if(tmj_data_accepted) begin
         
                  if(tmj_values_to_transfer == 1) begin
                     tmj_command_in_progress <= 0;
                     tmj_read_data[0] <= 1;
                     tmj_read_data_bit_count <= 1;
                  end
         
                  tmj_values_to_transfer <= tmj_values_to_transfer - 1;
                  tmj_write_data_bit_count <= 0;
               end
               else begin
                  /* Otherwise, we do nothing until the circuit is ready to
                   * accept the data.  If the user asks for an acknowledgement,
                   * the wrapper will send 0.  There are no timeouts.
                   */
               end
            end
         end
         else begin
            // Read the next bit of the current data word from the ram
            tmj_write_data_bit_count <= tmj_write_data_bit_count + 1;
            tmj_from_user_ram_read_addr <= tmj_from_user_ram_read_addr + 1;
         end

         if(tmj_write_data_bit_count_delay2 != tmj_port_data_width) begin
            tmj_write_data <= {tmj_from_user_ram_q,
                        tmj_write_data[TMJ_MAX_FROM_USER_PORT_WIDTH-1:1]};
         end

         tmj_write_data_bit_count_delay1 <= tmj_write_data_bit_count;
         tmj_write_data_bit_count_delay2 <= tmj_write_data_bit_count_delay1;
      end

      else begin

         // The user is reading from the circuit

         if((tmj_values_to_transfer != 0)
                 && !tmj_read_data_ack
                 && (tmj_read_data_bit_count == 0)) begin
   
            tmj_data_ready = 0;

            if(tmj_port_address == TMJ_PORT0_ADDRESS) begin
               if(count_ready && !count_want) begin
                  count_want <= 1;
                  tmj_read_data[TMJ_PORT0_WIDTH:1] <= count;
                  tmj_data_ready = 1;
               end
            end

            if(tmj_port_address == TMJ_PORT5_ADDRESS) begin
               if(fs_writedata_ready && !fs_writedata_want) begin
                  fs_writedata_want <= 1;
                  tmj_read_data[TMJ_PORT5_WIDTH:1] <= fs_writedata;
                  tmj_read_data[16:10] <= { 7{ fs_writedata[8] } };
                  tmj_data_ready = 1;
               end
            end

            if(tmj_port_address == TMJ_PORT7_ADDRESS) begin
               if(trc_addr_ready && !trc_addr_want) begin
                  trc_addr_want <= 1;
                  tmj_read_data[TMJ_PORT7_WIDTH:1] <= trc_addr;
                  tmj_read_data[8:6] <= { 3{ trc_addr[4] } };
                  tmj_data_ready = 1;
               end
            end

            if(tmj_port_address == TMJ_PORT8_ADDRESS) begin
               if(trc_data_ready && !trc_data_want) begin
                  trc_data_want <= 1;
                  tmj_read_data[TMJ_PORT8_WIDTH:1] <= trc_data;
                  tmj_data_ready = 1;
               end
            end

            if(tmj_data_ready) begin
               tmj_read_data[0] <= 1;
               tmj_read_data_bit_count <= tmj_port_data_width + 1;
            end
            else begin
               /* Send an empty word, if the user circuit isn't ready
                * in time.
                */
               tmj_read_data[0] <= 0;
               tmj_read_data_bit_count <= tmj_port_data_width + 1;
            end
         
            if(tmj_values_to_transfer == 1) begin
               tmj_command_in_progress <= 0;
            end

            tmj_values_to_transfer <= tmj_values_to_transfer - 1;
         end
      end
   end

   if(count_want && !count_ready)
      count_want <= 0;

   if(procreset_ready && !procreset_want)
      procreset_ready <= 0;

   if(bootloadreset_ready && !bootloadreset_want)
      bootloadreset_ready <= 0;

   if(traceactivate_ready && !traceactivate_want)
      traceactivate_ready <= 0;

   if(datawrite_ready && !datawrite_want)
      datawrite_ready <= 0;

   if(fs_writedata_want && !fs_writedata_ready)
      fs_writedata_want <= 0;

   if(fs_readdata_ready && !fs_readdata_want)
      fs_readdata_ready <= 0;

   if(trc_addr_want && !trc_addr_ready)
      trc_addr_want <= 0;

   if(trc_data_want && !trc_data_ready)
      trc_data_want <= 0;

   if(tmj_read_data_ack)
      tmj_read_data_bit_count <= 0;

end
         

wire tmj_tck, tmj_shift_dr, tmj_capture_dr, tmj_e1dr;
wire [1:0]  tmj_ir;
reg tmj_bypass_reg, tmj_tdo;
wire tmj_tdi;

reg tmj_bitflip_tck;
reg tmj_bitflip_clk, tmj_bitflip_clk_delay1, tmj_bitflip_clk_delay2;
reg tmj_tdi_clk, tmj_shift_dr_clk, tmj_e1dr_clk;
reg [1:0] tmj_ir_clk;


always @(posedge clk) begin

   tmj_from_user_ram_we <= 0;
   tmj_new_command_available <= 0;

   if(tmj_bitflip_clk_delay2 != tmj_bitflip_clk_delay1) begin

      if(tmj_ir_clk == 2) begin
         if(tmj_shift_dr_clk || tmj_e1dr_clk) begin

            // Shift new bits into the tmj_from_user_buf.  Any bits that fall off
            // the end are shifted into the tmj_from_user_ram.

            tmj_from_user_ram_data <= tmj_from_user_buf[0];
            tmj_from_user_ram_we <= 1;
            if(tmj_from_user_ram_write_addr == TMJ_FROM_USER_RAM_END) begin
               tmj_from_user_ram_write_addr <= 0;
            end
            else begin
               tmj_from_user_ram_write_addr <= tmj_from_user_ram_write_addr + 1;
            end

            tmj_from_user_buf = { tmj_tdi_clk, tmj_from_user_buf[TMJ_COMMAND_WIDTH-1:1] };
         end

         if(tmj_e1dr_clk) begin

            // The last bit of the next command has arrived

            tmj_new_command = tmj_from_user_buf;
            tmj_new_command_available <= 1;
            tmj_last_from_user_ram_write_addr <= tmj_from_user_ram_write_addr;
         end
      end

      if(tmj_ir_clk == 2 && tmj_shift_dr_clk) begin
         tmj_read_data_ack <= 0;

         if(tmj_read_data_shifter_count != 0) begin
            tmj_tdo <= tmj_read_data_shifter[0];
            tmj_read_data_shifter[TMJ_MAX_TO_USER_PORT_WIDTH-1:0] <= tmj_read_data_shifter[TMJ_MAX_TO_USER_PORT_WIDTH:1];
            tmj_read_data_shifter_count <= tmj_read_data_shifter_count - 1;
         end

         else if(tmj_read_data_bit_count != 0) begin
            tmj_tdo <= tmj_read_data[0];
            tmj_read_data_shifter[TMJ_MAX_TO_USER_PORT_WIDTH] <= 1'b0;
            tmj_read_data_shifter[TMJ_MAX_TO_USER_PORT_WIDTH-1:0] <= tmj_read_data[TMJ_MAX_TO_USER_PORT_WIDTH:1];
            tmj_read_data_shifter_count <= tmj_read_data_bit_count - 1;
            tmj_read_data_ack <= 1;
         end

         else begin
            tmj_tdo <= tmj_read_data[0];
         end

      end
      else begin
         tmj_tdo <= tmj_bypass_reg;
      end
   
      tmj_bypass_reg <= tmj_tdi_clk;
   
   end

   tmj_bitflip_clk_delay2 <= tmj_bitflip_clk_delay1;
   tmj_bitflip_clk_delay1 <= tmj_bitflip_clk;

   tmj_ir_clk <= tmj_ir;
   tmj_tdi_clk <= tmj_tdi;
   tmj_shift_dr_clk <= tmj_shift_dr;
   tmj_e1dr_clk <= tmj_e1dr;
   tmj_bitflip_clk <= tmj_bitflip_tck;

end


always @(posedge tmj_tck) begin
   tmj_bitflip_tck <= !tmj_bitflip_tck;
end



virtual1   virtual1_inst (
//   .ir_out ( ),
   .tdo ( tmj_tdo),
   .ir_in ( tmj_ir ),
   .tck ( tmj_tck ),
   .tdi ( tmj_tdi ),
   .virtual_state_cdr ( tmj_capture_dr ),
//   .virtual_state_cir ( ),
   .virtual_state_e1dr ( tmj_e1dr ),
//   .virtual_state_e2dr ( virtual_state_e2dr_sig ),
//   .virtual_state_pdr ( virtual_state_pdr_sig ),
   .virtual_state_sdr ( tmj_shift_dr )
//   .virtual_state_udr ( update_dr ),
//   .virtual_state_uir ( )
   );

endmodule
