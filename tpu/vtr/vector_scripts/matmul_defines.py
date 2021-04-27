class matmul_defines():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\

`define DWIDTH 16
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

'''
        return string

    def write(self):
        self.fp.write(self.make_str())
