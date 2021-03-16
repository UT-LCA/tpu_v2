import parser

class div():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, idle, dividing, done):
        string = '''
module div_{IDLE}_{DIVIDING}_{DONE}(en,resetn,stalled,quotient,remainder,dividend,divider,sign,clk);

   input         clk;
   input         resetn;
   input         sign;
   input         en;
   input [31:0]  dividend, divider;
   output [31:0] quotient, remainder;
   output        stalled;

   reg [31:0]    quotient, quotient_temp;
   reg [63:0]    dividend_copy, divider_copy, diff;
   reg           negative_output;
   
   wire [31:0]   remainder = (!negative_output) ? 
                             dividend_copy[31:0] : 
                             ~dividend_copy[31:0] + 1'b1;

   reg [5:0]     bits; 

   reg [1:0] state;

   always@(posedge clk)
     if (!resetn)
       state<=0;
     else
       case(state)
        {IDLE}: state<=(en) ? {DIVIDING} : {IDLE};
        {DIVIDING}: state<=(bits==5'd1) ? {DONE} : {DIVIDING};
        {DONE}: state<= {IDLE};
        default: state<=0;
       endcase

   assign stalled = (state=={DIVIDING}) || (state=={IDLE} && en);
   //assign stalled = (bits==0 && en) || (|bits);

   always @( posedge clk ) 
     if (!resetn)
     begin

        bits = 0;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = 0;
        divider_copy = 0;
        negative_output =0;
        diff=0;

     end
     else if( en && state=={IDLE}) begin

        bits = 6'd32;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = (!sign || !dividend[31]) ? 
                        {{32'd0,dividend}} : 
                        {{32'd0,~dividend + 1'b1}};
        divider_copy = (!sign || !divider[31]) ? 
                       {{1'b0,divider,31'd0}} : 
                       {{1'b0,~divider + 1'b1,31'd0}};

        negative_output = sign &&
                          ((divider[31] && !dividend[31]) 
                        ||(!divider[31] && dividend[31]));
        
     end 
     else if ( bits > 0 ) begin

        diff = dividend_copy - divider_copy;

        if( !diff[63] ) begin
           dividend_copy = diff;
           quotient_temp = (quotient_temp << 1) | 1'd1;
        end
        else begin
           quotient_temp = quotient_temp << 1;
        end

        quotient = (!negative_output) ? 
                   quotient_temp : 
                   ~quotient_temp + 1'b1;

        divider_copy = divider_copy >> 1;
        bits = bits - 1'b1;

     end
endmodule
'''       

        return string.format(IDLE=idle, DIVIDING=dividing, DONE=done)

    def write (self, idle, dividing, done):
        self.fp.write(self.make_str(idle, dividing, done))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = div(fp)
    uut.write(0, 1, 2)
    fp.close()