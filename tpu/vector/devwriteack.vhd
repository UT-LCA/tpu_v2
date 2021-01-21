---------------------------------------------------------------------------
--  TM4: Development Bus Parameterized Bus Abstraction Module            --
--  Author: Josh Fender                                                  --
--                                                                       --
--  Command: Acked Write (with dataNew output)                           --
--                                                                       --
--  Description:                                                         --
--    Provides a parameterized width register on a development FPGA      --
--    that user circuits can use as a flow controlled input port         --
--                                                                       --
--  User Signals:                                                        --
--     data : out std_logic_vector(dataWidth - 1 downto 0);              --
--        - parameterized width register's output                        --
--     dataReq : in std_logic;                                           --
--        - Asserted by user when they want new data                     --
--     dataNew : out std_logic;                                          --
--        - Asserted by module when data is valid and is held asserted   --
--          until the user deasserts dataReq                             --
--                                                                       --
--  Module Parameters:                                                   --
--     dataWidth    Width, in bits, of desired register                  --
--     writeCycles  Number of 32bit development bus transactions         --
--                  necessary for a dataWidth write                      --
--                  =  Ceiling(dataWidth/32)                             --
--     writePow2    Unused legacy parameter                              --
--     portAddr     An 6 bit std_logic_vector that indicates the         --
--                  address of the abstracted register port              --
--                                                                       --
--  Bus Protocol                                                         --
--    The acked write transaction consists of [writeCycles] different    --
--    32bit write cycles followed by an acknowledgement cycle.  Writes   --
--    are ordered from LSB to MSB.  The acked write transaction accepts  -- 
--    any number of idle bus states between different write cycles in    --
--    a multiwrite transaction.  The abstract port register is only      --
--    updated when all [writeCycles] write cycles have been completed    --
--    and the user circuit has asserted dataReq.                         --
--                                                                       --
--    A single write cycle consists of the bridge chip driving the       --
--    address and data lines as well as asserting the framen signal for  --
--    one cycle.                                                         --
--                                                                       --
--    An acknowledgement cycle must follow, without any idle cycles,     --
--    immediately after a write cycle.  The cycle consists of the master --
--    device holding the target address, write data and continuing to    --
--    assert frame until it detects a target acknowledgement.  The       --
--    master will then deassert frame for one cycle before continuing    --
--    with further transactions.                                         --
--                                                                       --
--  Waveform                                                             --
--     CLOCK    ____|````|____|````|____|````|____|````|____|~~~~|       --
--     ADDRESS  -[ Address ]--1--[ Address               ]--------       --
--     FRAMEn   `|_________|`````|_______________________|````````       --
--     DATA     -[Data LSB]------[        Data MSB       ]--------       --
--     TACKn    `````````````````````````````|________|```````````       --
--                                                                       --
--     Notes: 1 There can be any number of bus idle states between       --
--              different 32bit write cycles.  An idle state consists    --
--              of a bus cycle where FRAMEn is not asserted.             --
--                                                                       --
--  User Circuit Handshake                                               --
--     dataReq [User Driven]  _______``````````````````````________      --
--     dataNew [Port Driven]  _________________```````````````_____      --
--     dataOut [Port Driven]  -----------------<Data Valid>--------      --
--                                                                       --
--     - User circuit asserts dataReq                                    --
--     - When data is ready the port asserts dataNew and outputs the     --
--       newly received data                                             --
--     - Once the user circuit is finished with the data it deasserts    --
--       dataReq.  From this time dataOut is considered no longer valid  --
--     - Upon seeing that the user circuit has deasserted dataReq the    --
--       port will deassert dataNew                                      --
---------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

ENTITY DevWriteAck IS
  GENERIC (
     dataWidth : natural := 53;
     writeCycles : natural := 2;   -- = CEIL(Datawidth/32)
     writePow2 : natural := 1;     -- = 1 if writecycles in a power of 2
     portAddr : natural := 0
  );
  PORT (
     -- Development Bus Signals
     resetn : in std_logic;
     address : in std_logic_vector(5 downto 0);
     datain : in std_logic_vector(31 downto 0);
     dataout : out std_logic_vector(31 downto 0);
     oe : out std_logic;
     framen : in std_logic;
     tackn : out std_logic;
     devclk : in std_logic;
     -- User interface signals
     data : out std_logic_vector(datawidth - 1 downto 0);
     dataReq : in std_logic;
     dataNew : out std_logic
  );
END;

ARCHITECTURE rtl of DevWriteAck IS
  SIGNAL databuffer : std_logic_vector(writecycles*32-1 downto 0);
  SIGNAL writecount : natural range 0 to writecycles;
  SIGNAL writeEn, pause : boolean;
  SIGNAL dataReady : std_logic;
BEGIN
  data <= databuffer(datawidth - 1 downto 0);
  dataNew <= dataReady;

  -- Set Dev Bus outputs
  dataout <= (others => '0');
  oe <= '0';

  writeEn <= (framen = '0') AND (address = portaddr);
  tackn <= '0' WHEN (writeCount = writeCycles) AND (dataReq = '0') 
                AND (dataReady = '1') ELSE '1';

  PROCESS (resetn, devclk)
  BEGIN
    IF (resetn = '0') THEN
      databuffer <= (others => '0');
      writeCount <= 0;
      dataReady <= '0';
      pause <= false;
    ELSIF rising_edge(devclk) THEN
      -- Pause until the framen is deasserted
      IF pause THEN  
        pause <= (framen = '0');
      -- If we have data ready then handle the handshake
      ELSIF (writeCount = writeCycles) THEN
        dataReady <= dataReq;
        IF (dataReq = '0') AND (dataReady = '1') THEN
          writeCount <= 0;
          pause <= true;
        END IF;
      -- If the devbus is handling a write then buffer to the shift register
      ELSIF writeEn THEN
        writeCount <= writecount + 1;
        -- Load 32bit parallel shift register
        databuffer(writecycles*32-1 downto (writecycles -1)*32) <= dataIn;
        IF (writecycles > 1) THEN
          writeloop : FOR k IN 1 TO writecycles-1 LOOP
            databuffer((k*32)-1 downto (k-1)*32) <=
               databuffer((k+1)*32-1 downto k*32);
          END LOOP writeloop;
        END IF;
      END IF;
    END IF;
  END PROCESS;

END rtl;
