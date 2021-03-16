

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

                  