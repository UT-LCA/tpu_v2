import parser

class register():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          Generic Register
****************************************************************************/
module register_{WIDTH}(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [{WIDTH}-1:0] d;
output [{WIDTH}-1:0] q;
reg [{WIDTH}-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))


class register_sync():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          Generic Register - synchronous reset
****************************************************************************/
module register_sync_{WIDTH}(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [{WIDTH}-1:0] d;
output [{WIDTH}-1:0] q;
reg [{WIDTH}-1:0] q;

always @(posedge clk)		//synchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

class pipereg():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_{WIDTH}(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [{WIDTH}-1:0] d;
output [{WIDTH}-1:0] q;
reg [{WIDTH}-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

class onecyclestall():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\
/****************************************************************************
          One cycle Stall circuit
****************************************************************************/
module onecyclestall(request,clk,resetn,stalled);
input request;
input clk;
input resetn;
output stalled;

  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(request or T)
  begin
    case(T) 
      1'b0: Tnext=request;
      1'b1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;
  assign stalled=(request&~T);
endmodule'''       

        return string

    def write (self):
        self.fp.write(self.make_str())

class multicyclestall():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\
/****************************************************************************
          Multi cycle Stall circuit - with wait signal

          - One FF plus one 2:1 mux to stall 1st cycle on request, then wait
          - this makes wait don't care for the first cycle
****************************************************************************/
module multicyclestall(request, devwait,clk,resetn,stalled);
input request;
input devwait;
input clk;
input resetn;
output stalled;

  reg T;

  always@(posedge clk)
    if (~resetn)
      T<=0;
    else
      T<=stalled;

  assign stalled=(T) ? devwait : request;
endmodule'''       

        return string

    def write (self):
        self.fp.write(self.make_str())

class pipedelayreg():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          One cycle - Pipeline delay register
****************************************************************************/
module pipedelayreg_{WIDTH}(d,en,clk,resetn,squashn,dst,stalled,q);

input [{WIDTH}-1:0] d;
input [4:0] dst;
input en;
input clk;
input resetn;
input squashn;
output stalled;
output [{WIDTH}-1:0] q;

  reg [{WIDTH}-1:0] q;
  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(en or T or dst)
  begin
    case(T) 
      0: Tnext=en&(|dst);
      1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;

  always @(posedge clk)   //synchronous reset
  begin
    if (resetn==0 || squashn==0)
      q<=0;
    else if (en==1)
      q<=d;
  end

  assign stalled=(en&~T&(|dst));
endmodule'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

class fakedelay():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          Fake Delay
****************************************************************************/
module fakedelay_{WIDTH}(d,clk,q);

input [{WIDTH}-1:0] d;
input clk;
output [{WIDTH}-1:0] q;

assign q=d;

endmodule'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

class zeroer():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          Zeroer
****************************************************************************/
module zeroer_{WIDTH}(d,en,q);

input en;
input [{WIDTH}-1:0] d;
output [{WIDTH}-1:0] q;
assign q= (en) ? d : 0;

endmodule'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

class nop():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          NOP - used to hack position of multiplexors
****************************************************************************/
module nop_{WIDTH}(d,q);

input [{WIDTH}-1:0] d;
output [{WIDTH}-1:0] q;

  assign q=d;

endmodule'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

class const():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width, val):
        string = '''\
/****************************************************************************
          Const
****************************************************************************/
module const_{WIDTH}_{VAL} (out);

output [{WIDTH}-1:0] out;

assign out={VAL};

endmodule'''       

        return string.format(WIDTH=width, VAL=val)

    def write (self, width, val):
        self.fp.write(self.make_str(width, val))

class branch_detector():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\
/****************************************************************************
          Branch detector
****************************************************************************/
module branch_detector(opcode, func, is_branch);
input [5:0] opcode;
input [5:0] func;
output is_branch;

wire is_special;
wire [5:0] func_local;

assign func_local = func & 6'b111000;

assign is_special=!(|opcode);
assign is_branch=((!(|opcode[5:3])) && !is_special) || 
                  ((is_special)&&(func_local==6'b001000));

endmodule'''       

        return string.format()

    def write (self):
        self.fp.write(self.make_str())

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = pipedelayreg(fp)
    uut.write(32)
    fp.close()