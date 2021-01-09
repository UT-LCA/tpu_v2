library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;


-- This circuit talks to the development bus, and to
-- the user's circuit.

entity tm4_portmux is port(
   tm4_devbus : inout std_logic_vector(40 downto 0);
   count : in std_logic_vector(31 downto 0);
   count_ready : in std_logic;
   count_want : out std_logic;
   procresetn_in : out std_logic_vector(0 downto 0);
   procreset_want : in std_logic;
   procreset_ready : out std_logic;
   bootloadresetn_in : out std_logic_vector(0 downto 0);
   bootloadreset_want : in std_logic;
   bootloadreset_ready : out std_logic;
   traceactivate_in : out std_logic_vector(0 downto 0);
   traceactivate_want : in std_logic;
   traceactivate_ready : out std_logic;
   data : out std_logic_vector(31 downto 0);
   datawrite_want : in std_logic;
   datawrite_ready : out std_logic;
   fs_writedata : in std_logic_vector(8 downto 0);
   fs_writedata_ready : in std_logic;
   fs_writedata_want : out std_logic;
   fs_readdata : out std_logic_vector(7 downto 0);
   fs_readdata_want : in std_logic;
   fs_readdata_ready : out std_logic;
   trc_addr : in std_logic_vector(4 downto 0);
   trc_addr_ready : in std_logic;
   trc_addr_want : out std_logic;
   trc_data : in std_logic_vector(31 downto 0);
   trc_data_ready : in std_logic;
   trc_data_want : out std_logic;
   tm4_glbclk0 : in std_logic
   );
end;

architecture arch_portmux of tm4_portmux is


component DevRead
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
END COMPONENT;


COMPONENT DevWrite
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
END COMPONENT;


COMPONENT DevReadBurst
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
     data : in std_logic_vector(datawidth - 1 downto 0);
     dataReq : out std_logic;
     dataReady : in std_logic
  );
END COMPONENT;


COMPONENT DevWriteAck
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
     dataReq : in std_logic;
     dataNew : out std_logic
  );
END COMPONENT;


   constant TM_num_ports : natural := 9;

   signal TM_to_devbus0 : std_logic_vector(31 downto 0);
   signal TM_to_devbus1 : std_logic_vector(31 downto 0);
   signal TM_to_devbus2 : std_logic_vector(31 downto 0);
   signal TM_to_devbus3 : std_logic_vector(31 downto 0);
   signal TM_to_devbus4 : std_logic_vector(31 downto 0);
   signal TM_to_devbus5 : std_logic_vector(31 downto 0);
   signal TM_to_devbus6 : std_logic_vector(31 downto 0);
   signal TM_to_devbus7 : std_logic_vector(31 downto 0);
   signal TM_to_devbus8 : std_logic_vector(31 downto 0);
   signal TM_devbus_oe : std_logic_vector(TM_num_ports-1 downto 0);
   signal TM_devbus_all_tackn : std_logic_vector(TM_num_ports-1 downto 0);
   signal TM_devbus_reg : std_logic_vector(40 downto 0);

begin

TM_port0: DevReadBurst generic map(dataWidth => 32, readCycles => 1, readPow2 => 0, portAddr => 0)
port map (
   dataReady => count_ready,
   dataReq => count_want,
   data => count,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus0,
   oe => TM_devbus_oe(0),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(0),
   devclk => tm4_glbclk0
   );

TM_port1: DevWriteAck generic map(dataWidth => 1, writeCycles => 1, writePow2 => 0, portAddr => 1)
port map (
   dataReq => procreset_want,
   dataNew => procreset_ready,
   data => procresetn_in,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus1,
   oe => TM_devbus_oe(1),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(1),
   devclk => tm4_glbclk0
   );

TM_port2: DevWriteAck generic map(dataWidth => 1, writeCycles => 1, writePow2 => 0, portAddr => 2)
port map (
   dataReq => bootloadreset_want,
   dataNew => bootloadreset_ready,
   data => bootloadresetn_in,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus2,
   oe => TM_devbus_oe(2),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(2),
   devclk => tm4_glbclk0
   );

TM_port3: DevWriteAck generic map(dataWidth => 1, writeCycles => 1, writePow2 => 0, portAddr => 3)
port map (
   dataReq => traceactivate_want,
   dataNew => traceactivate_ready,
   data => traceactivate_in,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus3,
   oe => TM_devbus_oe(3),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(3),
   devclk => tm4_glbclk0
   );

TM_port4: DevWriteAck generic map(dataWidth => 32, writeCycles => 1, writePow2 => 0, portAddr => 4)
port map (
   dataReq => datawrite_want,
   dataNew => datawrite_ready,
   data => data,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus4,
   oe => TM_devbus_oe(4),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(4),
   devclk => tm4_glbclk0
   );

TM_port5: DevReadBurst generic map(dataWidth => 9, readCycles => 1, readPow2 => 0, portAddr => 5)
port map (
   dataReady => fs_writedata_ready,
   dataReq => fs_writedata_want,
   data => fs_writedata,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus5,
   oe => TM_devbus_oe(5),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(5),
   devclk => tm4_glbclk0
   );

TM_port6: DevWriteAck generic map(dataWidth => 8, writeCycles => 1, writePow2 => 0, portAddr => 6)
port map (
   dataReq => fs_readdata_want,
   dataNew => fs_readdata_ready,
   data => fs_readdata,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus6,
   oe => TM_devbus_oe(6),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(6),
   devclk => tm4_glbclk0
   );

TM_port7: DevReadBurst generic map(dataWidth => 5, readCycles => 1, readPow2 => 0, portAddr => 7)
port map (
   dataReady => trc_addr_ready,
   dataReq => trc_addr_want,
   data => trc_addr,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus7,
   oe => TM_devbus_oe(7),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(7),
   devclk => tm4_glbclk0
   );

TM_port8: DevReadBurst generic map(dataWidth => 32, readCycles => 1, readPow2 => 0, portAddr => 8)
port map (
   dataReady => trc_data_ready,
   dataReq => trc_data_want,
   data => trc_data,
   resetn => TM_devbus_reg(40),
   address => TM_devbus_reg(37 downto 32),
   datain => TM_devbus_reg(31 downto 0),
   dataout => TM_to_devbus8,
   oe => TM_devbus_oe(8),
   framen => TM_devbus_reg(38),
   tackn => TM_devbus_all_tackn(8),
   devclk => tm4_glbclk0
   );

process(tm4_glbclk0)
   begin
   if (rising_edge(tm4_glbclk0)) then
      if(conv_integer(TM_devbus_oe) = 0) then
         tm4_devbus(31 downto 0) <= (others => 'Z');
      else
         tm4_devbus(31 downto 0) <= TM_to_devbus0
   		or TM_to_devbus1
   		or TM_to_devbus2
   		or TM_to_devbus3
   		or TM_to_devbus4
   		or TM_to_devbus5
   		or TM_to_devbus6
   		or TM_to_devbus7
   		or TM_to_devbus8
   			;
      end if;

      if(conv_integer(not TM_devbus_all_tackn) /= 0) then
         tm4_devbus(39) <= '0';
      else
         tm4_devbus(39) <= '1';
      end if;

      TM_devbus_reg <= tm4_devbus;

      --PETES CHANGES - modelsim requires this end of the devbus to be driven
      tm4_devbus(40) <= 'Z'; --reset signal (input only)
      tm4_devbus(38) <= 'Z'; --framen signal (input only)
      tm4_devbus(37 downto 32) <= (others => 'Z'); --address (input only)

   end if;

end process;

end arch_portmux;
