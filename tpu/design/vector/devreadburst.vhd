-- Notes: Fix top comment block

---------------------------------------------------------------------------
--  TM4: Development Bus Parameterized Bus Abstraction Module            --
--  Author: Gary Pong / Josh Fender                                      --
--                                                                       --
--  Command: Direct Read (with handshake)                                --
--                                                                       --
--  Description:                                                         --
--    Provides a parameterized width port on a development FPGA          --
--    that user circuits can use as an output with flow control.         --
--                                                                       --
--  User Signals:                                                        --
--     data : std_logic_vector(dataWidth - 1 downto 0);                  --
--        - parameterized width register's output                        --
--                                                                       --
--  Module Parameters:                                                   --
--     dataWidth   Width, in bits, of desired register                   --
--     readCycles  Number of 32bit development bus transactions          --
--                  necessary for a dataWidth write                      --
--                  =  Ceiling(dataWidth/32)                             --
--     readPow2    A flag indicated if readCycles is a power of 2        --
--                    - 1 if power of 2                                  --
--                    - 0 if not a power of 2                            --
--     portAddr     An 6 bit std_logic_vector that indicates the         --
--                  address of the abstracted register port              --
--                                                                       --
--  Bus Protocol                                                         --
--    The direct read transaction consists of [readCycles] different     --
--    32bit read cycles.  Reads are ordered from LSB to MSB.  The        --
--    direct read transaction continuously                               --
---------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.std_logic_unsigned.all;

ENTITY DevReadBurst IS
  GENERIC (
     dataWidth : natural := 64;
     readCycles : natural := 2;   -- = CEIL(Datawidth/32)
     readPow2 : natural := 1;     -- = 1 if readCycles in a power of 2
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
     data : in std_logic_vector(datawidth - 1 downto 0);
     dataReq : out std_logic;
     dataReady : in std_logic
  );
END;

ARCHITECTURE rtl of DevReadBurst IS
  PROCEDURE BufferData(SIGNAL data : IN std_logic_vector(datawidth - 1 downto 0);
                       SIGNAL readcount : INOUT natural range 0 to readCycles-1;
                       SIGNAL dataout : OUT std_logic_vector(31 downto 0);
                       SIGNAL databuffer : OUT std_logic_vector( (readCycles-1)*32 downto 0)) IS
  BEGIN
    IF readCycles > 1 THEN
      -- Need to buffer a multicycle transfer
      databuffer(datawidth - 1 - 32 downto 0) <= data(datawidth - 1 downto 32);
      dataout <= data(31 downto 0);
      readcount <= 1;
    ELSE
      dataout(datawidth - 1 downto 0) <= data;
      databuffer(0) <= '-';
      readcount <= 0;
    END IF;
  END BufferData;

  PROCEDURE SendData( SIGNAL databuffer : IN std_logic_vector( (readCycles-1)*32 downto 0);
                      SIGNAL readcount : INOUT natural range 0 to readCycles-1;
                      SIGNAL dataout : OUT std_logic_vector(31 downto 0)) IS
  BEGIN
    SendLoop : FOR k IN 1 TO readCycles-1 LOOP
      IF (readcount = k) THEN
        dataout <= databuffer(k*32-1 downto (k-1)*32);
      END IF;
    END LOOP SendLoop;
  
    IF readcount = (readCycles - 1) THEN
      -- We need to reset the counter to zero
      IF (ReadPow2 = 1) THEN
        readcount <= readcount+1;
      ELSE
        readcount <= 0;
      END IF;
    ELSE
      readcount <= readcount+1;
    END IF;

  END SendData;

  TYPE states IS (S_IDLE, S_FRAME, S_HANDSHAKEWAIT, S_TRANSFER);

  SIGNAL curr_state : states;

  SIGNAL framecycle : BOOLEAN;
  SIGNAL cyclelength : STD_LOGIC_VECTOR(15 downto 0);
  SIGNAL databuffer : std_logic_vector( (readCycles-1)*32 downto 0);
  SIGNAL readcount : natural range 0 to readCycles-1;
BEGIN
  framecycle <= (framen = '0') AND (address = portaddr);

  PROCESS (resetn,devclk)
  BEGIN
    IF (resetn = '0') THEN
      curr_state <= S_IDLE;
      cyclelength <= (others => '0');
      dataReq <= '0';
      oe <= '0';
      tackn <= '1';
    ELSIF (rising_edge(devclk)) THEN
      oe <= '0';
      tackn <= '1';
      dataout <= (others => '0');
      CASE (curr_state) IS
        WHEN S_IDLE => 
          IF framecycle THEN
            IF dataReady = '0' THEN
              curr_state <= S_FRAME;
              cyclelength <= datain(15 downto 0);
            ELSE
              cyclelength <= datain(15 downto 0) - CONV_STD_LOGIC_VECTOR(1,16);
              dataReq <= '1';
              -- Perform transfer
              oe <= '1';
              tackn <= '0';
              BufferData(data,readcount,dataout,databuffer);
              IF readCycles > 1 THEN
                curr_state <= S_TRANSFER;
              ELSE
                curr_state <= S_HANDSHAKEWAIT;
              END IF;
            END IF;    
          END IF;
        WHEN S_FRAME =>
          IF dataReady = '1' THEN
            dataReq <= '1';
            cyclelength <= cyclelength - 1;
            -- Perform transfer
            oe <= '1';
            tackn <= '0';
            BufferData(data,readcount,dataout,databuffer);
            IF readCycles > 1 THEN
              curr_state <= S_TRANSFER;
            ELSE
              curr_state <= S_HANDSHAKEWAIT;
            END IF;
          END IF;
        WHEN S_TRANSFER =>
          oe <= '1';
          tackn <= '0';
          cyclelength <= cyclelength - 1;
          SendData(databuffer,readcount,dataout);
          IF readcount = (readCycles - 1)THEN
            -- We are at the end of a transfer
            IF dataReady = '1' THEN
              curr_state <= S_HANDSHAKEWAIT;
            ELSE
              dataReq <= '0';
              IF cyclelength = CONV_STD_LOGIC_VECTOR(0,16) THEN
                curr_state <= S_IDLE;
              ELSE
                curr_state <= S_FRAME;
              END IF;
            END IF; 
          END IF;
        WHEN S_HANDSHAKEWAIT =>
          dataReq <= dataReady;
          IF dataReady = '0' THEN
            IF cyclelength = CONV_STD_LOGIC_VECTOR(0,16) THEN
              curr_state <= S_IDLE;
            ELSE
              curr_state <= S_FRAME;
            END IF;
          END IF;
      END CASE;
    END IF;
  END PROCESS;

END rtl;
