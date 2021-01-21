---------------------------------------------------------------------------
--  TM4: Development Bus Parameterized Bus Abstraction Module            --
--  Author: Gary Pong / Josh Fender                                                  --
--                                                                       --
--  Command: Direct Read (no handshake)                                  --
--                                                                       --
--  Description:                                                         --
--    Provides a parameterized width register on a development FPGA      --
--    that user circuits can use as a handshake free output              --
--                                                                       --
--  User Signals:                                                        --
--     data : std_logic_vector(dataWidth - 1 downto 0)                   --
--        - parameterized width register's output                        --
--                                                                       --
--  Module Parameters:                                                   --
--     dataWidth    Width, in bits, of desired register                  --
--     readCycles  Number of 32bit development bus transactions          --
--                  necessary for a dataWidth read                       --
--                  =  Ceiling(dataWidth/32)                             --
--     readPow2    A flag indicated if readCycles is a power of 2        --
--                    - 1 if power of 2                                  --
--                    - 0 if not a power of 2                            --
--     portAddr     An 6 bit std_logic_vector that indicates the         --
--                  address of the abstracted register port              --
--                                                                       --
--  Bus Protocol                                                         --
--    The direct read transaction consists of [readCycles] different     --
--    32bit read cycles.  Reads are ordered from LSB to MSB.  The bus    --
--    master drives the address lines and asserts the FRAMEn signal for  --
--    one cycle.  After a variable number of idle bus cycles, possibly   --
--    zero, the target device assert TACKn, latches the data input port  --
--    into an internal buffer and transmits the values 32 bits at a time --
--    across the development bus.                                        --
--                                                                       --
---------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;


ENTITY DevRead IS
  GENERIC (
     dataWidth : natural;
     readCycles : natural;   -- = CEIL(Datawidth/32)
     readPow2 : natural;     -- = 1 if readCycles in a power of 2
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
     data : in std_logic_vector(datawidth - 1 downto 0)
  );
END;

ARCHITECTURE rtl of DevRead IS
  SIGNAL databuffer : std_logic_vector( (readCycles-1)*32 downto 0);
  SIGNAL readcount : natural range 0 to readCycles-1;
  SIGNAL readEn : std_logic;
BEGIN
  -- Control Logic
  readEn <= '1' WHEN (framen = '0') AND (address = portaddr) ELSE '0';

  PROCESS (resetn,devclk,readEn)
  BEGIN
    IF (resetn = '0') THEN
      readcount <= 0;
      databuffer <= (others => '0');
      tackn <= '1';
      oe <= '0';
      dataout <= (others => '0');
    ELSIF (rising_edge(devclk)) THEN
      IF (readEn = '1' or readcount /= 0) THEN
        tackn <= '0';
        oe <= '1';

        -- Handle data transmission and buffering
        IF (readcount = 0) THEN
          IF ( readCycles > 1) THEN
            -- if multiple cycles required, buffer data
            databuffer(datawidth - 1 - 32 downto 0) <= data(datawidth - 1 downto 32);
            dataout <= data(31 downto 0);
          ELSE
            -- directly output all data
            dataout(datawidth - 1 downto 0) <= data;
          END IF;
        ELSE
          -- map databuffer contents to dataout, 32 bits at a time
          readloop : FOR k IN 1 TO readCycles-1 LOOP
            IF (readcount = k) THEN
              dataout <= databuffer(k*32-1 downto (k-1)*32);
            END IF;
          END LOOP readloop;
        END IF;

        -- Update the read cycle counters
        IF (readcount = readCycles-1) THEN
          -- If readCycles is power of 2 the counter will auto wrap
          IF (readPow2 = 1) THEN
            readcount <= readcount + 1;
          ELSE
            readcount <= 0;
          END IF;
        ELSE
          readcount <= readcount + 1;
        END IF;
      ELSE
        tackn <= '1';
        oe <= '0';
        dataout <= (others => '0');        
      END IF; 
    END IF;
  END PROCESS;

END rtl;
