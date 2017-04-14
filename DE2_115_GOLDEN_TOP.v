// --------------------------------------------------------------------
// Copyright (c) 2010 by Terasic Technologies Inc.
// --------------------------------------------------------------------
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development
//   Kits made by Terasic.  Other use of this code, including the selling
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use
//   or functionality of this code.
//
// --------------------------------------------------------------------
//
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// --------------------------------------------------------------------
//
// Major Functions:  DE2_115 Top Entity Reference Design
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author                    :| Mod. Date :| Changes Made:
//   V1.0 :| Richard                   :| 07/09/10  :| Initial Revision
// --------------------------------------------------------------------
module DE2_115_GOLDEN_TOP(

  //////// CLOCK //////////
  CLOCK_50,
  CLOCK2_50,
  CLOCK3_50,
  ENETCLK_25,

  //////// Sma //////////
  SMA_CLKIN,
  SMA_CLKOUT,

  //////// LED //////////
  LEDG,
  LEDR,

  //////// KEY //////////
  KEY,

  //////// SW //////////
  SW,

  //////// SEG7 //////////
  HEX0,
  HEX1,
  HEX2,
  HEX3,
  HEX4,
  HEX5,
  HEX6,
  HEX7,

  //////// LCD //////////
  LCD_BLON,
  LCD_DATA,
  LCD_EN,
  LCD_ON,
  LCD_RS,
  LCD_RW,

  //////// RS232 //////////
  UART_CTS,
  UART_RTS,
  UART_RXD,
  UART_TXD,

  //////// PS2 //////////
  PS2_CLK,
  PS2_DAT,
  PS2_CLK2,
  PS2_DAT2,

  //////// SDCARD //////////
  SD_CLK,
  SD_CMD,
  SD_DAT,
  SD_WP_N,

  //////// VGA //////////
  VGA_B,
  VGA_BLANK_N,
  VGA_CLK,
  VGA_G,
  VGA_HS,
  VGA_R,
  VGA_SYNC_N,
  VGA_VS,

  //////// Audio //////////
  AUD_ADCDAT,
  AUD_ADCLRCK,
  AUD_BCLK,
  AUD_DACDAT,
  AUD_DACLRCK,
  AUD_XCK,

  //////// I2C for EEPROM //////////
  EEP_I2C_SCLK,
  EEP_I2C_SDAT,

  //////// I2C for Audio and Tv-Decode //////////
  I2C_SCLK,
  I2C_SDAT,

  //////// Ethernet 0 //////////
  ENET0_GTX_CLK,
  ENET0_INT_N,
  ENET0_MDC,
  ENET0_MDIO,
  ENET0_RST_N,
  ENET0_RX_CLK,
  ENET0_RX_COL,
  ENET0_RX_CRS,
  ENET0_RX_DATA,
  ENET0_RX_DV,
  ENET0_RX_ER,
  ENET0_TX_CLK,
  ENET0_TX_DATA,
  ENET0_TX_EN,
  ENET0_TX_ER,
  ENET0_LINK100,

  //////// Ethernet 1 //////////
  ENET1_GTX_CLK,
  ENET1_INT_N,
  ENET1_MDC,
  ENET1_MDIO,
  ENET1_RST_N,
  ENET1_RX_CLK,
  ENET1_RX_COL,
  ENET1_RX_CRS,
  ENET1_RX_DATA,
  ENET1_RX_DV,
  ENET1_RX_ER,
  ENET1_TX_CLK,
  ENET1_TX_DATA,
  ENET1_TX_EN,
  ENET1_TX_ER,
  ENET1_LINK100,

  //////// TV Decoder //////////
  TD_CLK27,
  TD_DATA,
  TD_HS,
  TD_RESET_N,
  TD_VS,

  /////// USB OTG controller
  OTG_DATA,
  OTG_ADDR,
  OTG_CS_N,
  OTG_WR_N,
  OTG_RD_N,
  OTG_INT,
  OTG_RST_N,
  OTG_DREQ,
  OTG_DACK_N,
  OTG_FSPEED,
  OTG_LSPEED,

  //////// IR Receiver //////////
  IRDA_RXD,

  //////// SDRAM //////////
  DRAM_ADDR,
  DRAM_BA,
  DRAM_CAS_N,
  DRAM_CKE,
  DRAM_CLK,
  DRAM_CS_N,
  DRAM_DQ,
  DRAM_DQM,
  DRAM_RAS_N,
  DRAM_WE_N,

  //////// SRAM //////////
  SRAM_ADDR,
  SRAM_CE_N,
  SRAM_DQ,
  SRAM_LB_N,
  SRAM_OE_N,
  SRAM_UB_N,
  SRAM_WE_N,

  //////// Flash //////////
  FL_ADDR,
  FL_CE_N,
  FL_DQ,
  FL_OE_N,
  FL_RST_N,
  FL_RY,
  FL_WE_N,
  FL_WP_N,

  //////// GPIO //////////
  GPIO,

  //////// HSMC (LVDS) //////////
//  HSMC_CLKIN_N1,
//  HSMC_CLKIN_N2,
  HSMC_CLKIN_P1,
  HSMC_CLKIN_P2,
  HSMC_CLKIN0,
//  HSMC_CLKOUT_N1,
//  HSMC_CLKOUT_N2,
  HSMC_CLKOUT_P1,
  HSMC_CLKOUT_P2,
  HSMC_CLKOUT0,
  HSMC_D,
//  HSMC_RX_D_N,
  HSMC_RX_D_P,
//  HSMC_TX_D_N,
  HSMC_TX_D_P,

  //////// EXTEND IO //////////
  EX_IO
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input                   CLOCK_50;
input                   CLOCK2_50;
input                   CLOCK3_50;
input                   ENETCLK_25;

//////////// Sma //////////
input                   SMA_CLKIN;
output                  SMA_CLKOUT;

//////////// LED //////////
output         [8:0]    LEDG;
output        [17:0]    LEDR;

//////////// KEY //////////
input         [3:0]     KEY;

//////////// SW //////////
input        [17:0]     SW;

//////////// SEG7 //////////
output         [6:0]    HEX0;
output         [6:0]    HEX1;
output         [6:0]    HEX2;
output         [6:0]    HEX3;
output         [6:0]    HEX4;
output         [6:0]    HEX5;
output         [6:0]    HEX6;
output         [6:0]    HEX7;

//////////// LCD //////////
output                  LCD_BLON;
inout         [7:0]     LCD_DATA;
output                  LCD_EN;
output                  LCD_ON;
output                  LCD_RS;
output                  LCD_RW;

//////////// RS232 //////////
output                  UART_CTS;
input                   UART_RTS;
input                   UART_RXD;
output                  UART_TXD;

//////////// PS2 //////////
inout                   PS2_CLK;
inout                   PS2_DAT;
inout                   PS2_CLK2;
inout                   PS2_DAT2;

//////////// SDCARD //////////
output                  SD_CLK;
inout                   SD_CMD;
inout         [3:0]     SD_DAT;
input                   SD_WP_N;

//////////// VGA //////////
output         [7:0]    VGA_B;
output                  VGA_BLANK_N;
output                  VGA_CLK;
output         [7:0]    VGA_G;
output                  VGA_HS;
output         [7:0]    VGA_R;
output                  VGA_SYNC_N;
output                  VGA_VS;

//////////// Audio //////////
input                   AUD_ADCDAT;
inout                   AUD_ADCLRCK;
inout                   AUD_BCLK;
output                  AUD_DACDAT;
inout                   AUD_DACLRCK;
output                  AUD_XCK;

//////////// I2C for EEPROM //////////
output                  EEP_I2C_SCLK;
inout                   EEP_I2C_SDAT;

//////////// I2C for Audio and Tv-Decode //////////
output                  I2C_SCLK;
inout                   I2C_SDAT;

//////////// Ethernet 0 //////////
output                  ENET0_GTX_CLK;
input                   ENET0_INT_N;
output                  ENET0_MDC;
inout                   ENET0_MDIO;
output                  ENET0_RST_N;
input                   ENET0_RX_CLK;
input                   ENET0_RX_COL;
input                   ENET0_RX_CRS;
input          [3:0]    ENET0_RX_DATA;
input                   ENET0_RX_DV;
input                   ENET0_RX_ER;
input                   ENET0_TX_CLK;
output         [3:0]    ENET0_TX_DATA;
output                  ENET0_TX_EN;
output                  ENET0_TX_ER;
input                   ENET0_LINK100;

//////////// Ethernet 1 //////////
output                  ENET1_GTX_CLK;
input                   ENET1_INT_N;
output                  ENET1_MDC;
inout                   ENET1_MDIO;
output                  ENET1_RST_N;
input                   ENET1_RX_CLK;
input                   ENET1_RX_COL;
input                   ENET1_RX_CRS;
input          [3:0]    ENET1_RX_DATA;
input                   ENET1_RX_DV;
input                   ENET1_RX_ER;
input                   ENET1_TX_CLK;
output         [3:0]    ENET1_TX_DATA;
output                  ENET1_TX_EN;
output                  ENET1_TX_ER;
input                   ENET1_LINK100;

//////////// TV Decoder 1 //////////
input                   TD_CLK27;
input          [7:0]    TD_DATA;
input                   TD_HS;
output                  TD_RESET_N;
input                   TD_VS;


//////////// USB OTG controller //////////
inout            [15:0]     OTG_DATA;
output           [1:0]      OTG_ADDR;
output                      OTG_CS_N;
output                      OTG_WR_N;
output                      OTG_RD_N;
input            [1:0]      OTG_INT;
output                      OTG_RST_N;
input            [1:0]      OTG_DREQ;
output           [1:0]      OTG_DACK_N;
inout                       OTG_FSPEED;
inout                       OTG_LSPEED;

//////////// IR Receiver //////////
input                   IRDA_RXD;

//////////// SDRAM //////////
output        [12:0]    DRAM_ADDR;
output         [1:0]    DRAM_BA;
output                  DRAM_CAS_N;
output                  DRAM_CKE;
output                  DRAM_CLK;
output                  DRAM_CS_N;
inout         [31:0]    DRAM_DQ;
output         [3:0]    DRAM_DQM;
output                  DRAM_RAS_N;
output                  DRAM_WE_N;

//////////// SRAM //////////
output        [19:0]    SRAM_ADDR;
output                  SRAM_CE_N;
inout         [15:0]    SRAM_DQ;
output                  SRAM_LB_N;
output                  SRAM_OE_N;
output                  SRAM_UB_N;
output                  SRAM_WE_N;

//////////// Flash //////////
output        [22:0]    FL_ADDR;
output                  FL_CE_N;
inout          [7:0]    FL_DQ;
output                  FL_OE_N;
output                  FL_RST_N;
input                   FL_RY;
output                  FL_WE_N;
output                  FL_WP_N;

//////////// GPIO //////////
inout         [35:0]    GPIO;

//////////// HSMC (LVDS) //////////

//input                  HSMC_CLKIN_N1;
//input                  HSMC_CLKIN_N2;
input                   HSMC_CLKIN_P1;
input                   HSMC_CLKIN_P2;
input                   HSMC_CLKIN0;
//output                  HSMC_CLKOUT_N1;
//output                  HSMC_CLKOUT_N2;
output                  HSMC_CLKOUT_P1;
output                  HSMC_CLKOUT_P2;
output                  HSMC_CLKOUT0;
inout          [3:0]    HSMC_D;
//input        [16:0]    HSMC_RX_D_N;
input         [16:0]    HSMC_RX_D_P;
//output        [16:0]    HSMC_TX_D_N;
output        [16:0]    HSMC_TX_D_P;

//////// EXTEND IO //////////
inout          [6:0]    EX_IO;


// Clock divider to slow down 50 MHz clock to 1MHz
wire CLOCK_1;
clk clk_main (
  .inclk0 (CLOCK_50),
  .c0     (CLOCK_1)
);


// Counter to make clock even slow
wire clk_slow;
cntr_clk #(
  .CNT_VAL(10000)
) clk_slow_main (
  .clk_i (CLOCK_1),
  .clk_o (clk_slow)
);
assign LEDR[17] = clk_slow;

wire clk_slow2;
wire le, oen, sdo;
ss ss0 (
  .clk_i   (clk_slow),
  .clk_o   (clk_slow2),
  .btn     (KEY[0]),
  .row     (8'b11111111),

  .sdo     (sdo),
  .le      (le),
  .oen     (oen),

  .cnt_led (LEDR[6:0])
);

assign LEDG[7]  = clk_slow2;
assign GPIO[15] = clk_slow2;

//assign le = SW[0];
assign LEDG[1]  = le;
assign GPIO[13] = le;

//assign oen = SW[1];
assign LEDG[2]  = oen;
assign GPIO[11] = oen;

assign LEDG[0]  = sdo;
assign GPIO[17] = sdo;

test test0 (
   .in0 (  SW[16]),
  .out0 (HEX0[0]),
   .in1 (  SW[17]),
  .out1 (HEX0[1])
);

endmodule



module test (
  input   in0,
  output out0,
  input   in1,
  output reg out1
);

assign out0 = in0;

always begin
  out1 <= in1;
end

endmodule


// Counter to slow clock even more
module cntr_clk (
  input  clk_i,
  output clk_o
);

parameter CNT_VAL = 1000000;

wire [31:0] val;
wire        clr;
reg         tgl = 1;

cntr cntr0 (
  .clock (clk_i),
  .sclr  (clr),
  .q     (val)
);

assign clr   = ( val == (CNT_VAL - 1) ) ? 1 : 0;
assign clk_o = tgl;

always @(posedge clk_i) begin
  if (clr == 1) begin
    tgl <= ~tgl;
  end else begin
    tgl <= tgl;
  end
end

endmodule


module ss (
  input        clk_i,
  output       clk_o,
  input        btn,
  input [7:0]  row,
  output reg   le,
  output reg   oen,
  output reg   sdo,
  output [7:0] cnt_led
);

localparam BTN_ON  = 1;
localparam BTN_OFF = 1;
localparam OEN_ON  = 0;
localparam OEN_OFF = 1;

localparam SIZE = 3;
localparam IDLE = 0;
localparam SEND = 1;
localparam DONE = 2;
reg [SIZE-1:0] currSt = IDLE;
reg [SIZE-1:0] nextSt = IDLE;

localparam CNT_RST_VAL = 8'b11111111;
reg [7:0] cnt = CNT_RST_VAL;
assign cnt_led = cnt;

localparam red = 8'b00000111;
localparam grn = 8'b00111000;
localparam blu = 8'b11000000;
reg  [23:0] rgb = 24'b1;
wire [31:0] srl = {rgb, row};

assign clk_g = (currSt == SEND) ? 0 : 1;
assign clk_o = (clk_g == 1) ? 0 : clk_i;

localparam CNT_ST_SEND = 31;
localparam CNT_ST_DONE = 2;

// FSM

// Combinational logic to determine next state
always @(*) begin
  nextSt = currSt;

  case(currSt)
    IDLE: begin
      if (btn == BTN_ON) begin
        nextSt = SEND;
      end
    end

    SEND: begin
      if (cnt == CNT_ST_SEND) begin
        nextSt = DONE;
      end
    end

    DONE: begin
      if (cnt == CNT_ST_DONE) begin
        nextSt = IDLE;
      end
    end
  endcase
end

// Register the next state
always @(posedge clk_i) begin
  currSt <= nextSt;
end

// Sequential logic for counters and shifting
always @(posedge clk_i) begin
  case(currSt)
    IDLE: begin
      if (nextSt == SEND) begin
        cnt <= 0;
      end
    end

    SEND: begin
      if (nextSt == DONE) begin
        cnt <= 0;
      end else begin
        cnt <= cnt + 1;
      end
    end

    DONE: begin
      if (nextSt == IDLE) begin
        cnt <= 0;
        rgb <= {rgb[22:0], rgb[23]};
      end else begin
        cnt <= cnt + 1;
      end
    end
  endcase
end

// Combinational logic for serial data out
always @(*) begin
  oen = OEN_ON;
  sdo = 0;
  le  = 0;

  case(currSt)
    SEND: begin
      sdo = srl[cnt];
    end

    DONE: begin
      oen = OEN_OFF;
      le  = ~cnt[0];
    end
  endcase
end


endmodule
