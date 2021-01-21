---------------------------------------------------------------------------
--  TM4: Development Bus Parameterized Bus Abstraction Module            --
--  Author: Josh Fender                                                  --
--                                                                       --
--  Command: Direct Write (no handshake)                                 --
--                                                                       --
--  Description:                                                         --
--    Provides a parameterized width register on a development FPGA      --
--    that user circuits can use as a handshake free input.              --
--                                                                       --
--  User Signals:                                                        --
--     data : out std_logic_vector(dataWidth - 1 downto 0);              --
--        - parameterized width register's output                        --
--     dataNew : out std_logic;                                          --
--        - Asserted by module for one cycle at the same time new data   --
--          is available on the data output lines                        --
--                                                                       --
--  Module Parameters:                                                   --
--     dataWidth    Width, in bits, of desired register                  --
--     writeCycles  Number of 32bit development bus transactions         --
--                  necessary for a dataWidth write                      --
--                  =  Ceiling(dataWidth/32)                             --
--     writePow2    A flag indicated if writeCycles is a power of 2      --
--                    - 1 if power of 2                                  --
--                    - 0 if not a power of 2                            --
--     portAddr     An 6 bit natural that indicates the         --
--                  address of the abstracted register port              --
--                                                                       --
--  Bus Protocol                                                         --
--    The direct write transaction consists of [writeCycles] different   --
--    32bit write cycles.  Writes are ordered from LSB to MSB.  The      --
--    direct write transaction accepts any number of idle bus states     --
--    between different write cycles in a multiwrite transaction.  The   --
--    abstract port register is only updated when all [writeCycles]      --
--    write cycles have been completed.                                  --
--                                                                       --
--    A single write cycle consists of the bridge chip driving the       --
--    address and data lines as well as asserting the framen signal for  --
--    one cycle.                                                         --
--                                                                       --
--  Waveform                                                             --
--     CLOCK    ____|````|____|````|____|````|____|````|                 --
--     ADDRESS  -[ Address ]--1--[ Address          ]---                 --
--     FRAMEn   `|_________|`````|__________________|```                 --
--     DATA     -[Data LSB]------[  Data  ][  Data  ]---                 --
--     TACKn    ````````````````````````````````````````                 --
--                                                                       --
--     Notes: 1 There can be any number of bus idle states between       --
--              different 32bit write cycles.  An idle state consists    --
--              of a bus cycle where FRAMEn is not asserted.             --
--                                                                       --
---------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;


ENTITY DevWrite IS
  GENERIC (
     dataWidth : natural;
     writeCycles : natural;   -- = CEIL(Datawidth/32)
     writePow2 : natural;     -- = 1 if writecycles in a power of 2
     portAddr : natural
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
     dataNew : out std_logic
  );
END;

ARCHITECTURE rtl of DevWrite IS
  -- Note: databuffer made one too large to handle case where no data
  --       buffer is required (ie single cycle transactions)
  SIGNAL databuffer : std_logic_vector((writecycles-1)*32 downto 0);
  SIGNAL writecount : natural range 0 to writecycles-1;
  SIGNAL writeEn : std_logic;
BEGIN
  -- Set Dev Bus outputs
  tackn <= '1';
  oe <= '0';
  dataout <= (others => '0');

  -- Control Logic
  writeEn <= '1' WHEN (framen = '0') AND (address = portaddr) ELSE '0';

  PROCESS (resetn,devclk,writeEn)
  BEGIN
    IF (resetn = '0') THEN
      databuffer <= (others => '0');
      writecount <= 0;
      data <= (others => '0');
      dataNew <= '0';
    ELSIF (rising_edge(devclk)) THEN
      dataNew <= '0';
      IF (writeEn = '1') THEN
        -- If multicycle write then buffer the initial cycles data
        IF (writecycles > 1) THEN
          writeloop : FOR k IN 0 TO writecycles-2 LOOP
            IF (writecount = k) THEN
              databuffer((k+1)*32-1 downto k*32) <= datain;
            END IF;
          END LOOP writeloop;
        END IF;

        -- If this is the last cycle of write update data output
        -- with buffered data and current bus transaction data
        IF (writecount = writecycles-1) THEN
          dataNew <= '1';
          data(datawidth-1 downto (writecycles-1)*32) <= 
                datain(datawidth - (writecycles-1)*32-1 downto 0);
          IF (writecycles > 1) THEN
            data((writecycles-1)*32-1 downto 0) <= 
                databuffer((writecycles-1)*32-1 downto 0);
          END IF;

          -- If writecycles is power of 2 the counter will auto wrap
          IF (writePow2 = 1) THEN
             writecount <= writecount + 1;
          ELSE
            writecount <= 0;
          END IF;
        ELSE
          writecount <= writecount + 1;
        END IF;
      END IF; 
    END IF;
  END PROCESS;

END rtl;
