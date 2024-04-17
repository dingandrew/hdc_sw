// `define DELAY 0.13ns // for port b
`define DELAY 0.1ns // for port a
// `define DELAY 0

// Memory wrapper around tsmc dual port memory
module memory_wrapper # (
  DATA_WIDTH                        = 32,
  ADDR_WIDTH                        = 32,
  M_ADDR_WIDTH                      = 16,
  logic [ADDR_WIDTH-1:0] ADDR_MAP_A = '0,
  logic [ADDR_WIDTH-1:0] ADDR_MAP_B = '0
) (
  input  logic                    clk,
  input  logic                    data_req_a,
  input  logic [ADDR_WIDTH-1:0]   data_addr_a,
  input  logic                    data_we_a,
  input  logic [DATA_WIDTH/8-1:0] data_be_a,
  input  logic [DATA_WIDTH-1:0]   data_wdata_a,
  output logic                    data_gnt_a,
  output logic                    data_rvalid_a,
  output logic                    data_err_a,
  output logic [DATA_WIDTH-1:0]   data_rdata_a,

  input  logic                    data_req_b,
  input  logic [ADDR_WIDTH-1:0]   data_addr_b,
  input  logic                    data_we_b,
  input  logic [DATA_WIDTH/8-1:0] data_be_b,
  input  logic [DATA_WIDTH-1:0]   data_wdata_b,
  output logic                    data_gnt_b,
  output logic                    data_rvalid_b,
  output logic                    data_err_b,
  output logic [DATA_WIDTH-1:0]   data_rdata_b
);
  timeunit 1ns;
  timeprecision 1ps;

  logic ceba;
  logic [DATA_WIDTH-1:0] bweba;
  logic data_err_next_a;

  logic [M_ADDR_WIDTH-1:0] data_addr_aw;
  logic [DATA_WIDTH-1:0] data_wdata_aw;
  logic data_we_aw;

  logic cebb;
  logic [DATA_WIDTH-1:0] bwebb;
  logic data_err_next_b;

  logic [M_ADDR_WIDTH-1:0] data_addr_bw;
  logic [DATA_WIDTH-1:0] data_wdata_bw;
  logic data_we_bw;

  // adding delays to signals to pass hold time check in tsmc memory
  always_comb begin
    data_addr_aw <= #`DELAY data_addr_a[M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)-1:$clog2(DATA_WIDTH/8)];
    data_wdata_aw <= #`DELAY data_wdata_a;
    data_we_aw <= #`DELAY data_we_a;

    data_addr_bw <= #`DELAY data_addr_b[M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)-1:$clog2(DATA_WIDTH/8)];
    data_wdata_bw <= #`DELAY data_wdata_b;
    data_we_bw <= #`DELAY data_we_b;
  end

  // use byte enable to get bit write enable
  genvar i, j;
  for (i = 0; i < DATA_WIDTH/8; i++) begin
    for (j = 0; j < 8; j++) begin
      always_comb begin
        bweba[i*8+j] <= #`DELAY ~data_be_a[i];
        bwebb[i*8+j] <= #`DELAY ~data_be_b[i];
      end
    end
  end

  // chip enable also gets latched at clk edge
  always_comb begin
    ceba <= #`DELAY ~data_req_a;
    cebb <= #`DELAY ~data_req_b;
  end


  // memory instantiation
  // currently only using one memory port
  TSDN28HPCPUHDB512X64M4MWA mem (
    .CLK(clk),
    .RTSEL(2'b0), .WTSEL(2'b0), .PTSEL(2'b0),
    .AWT(1'b0),
    // inputs
    .AA(data_addr_aw), .DA(data_wdata_aw),
    .BWEBA(bweba), .WEBA(~data_we_aw), .CEBA(ceba),
    .AB(data_addr_bw), .DB(data_wdata_bw),
    .BWEBB(bwebb), .WEBB(~data_we_bw), .CEBB(cebb),
    // outputs
    .QA(data_rdata_a), .QB(data_rdata_b)
  );
  
  // OUTPUTS

  // fine if no setup/hold violations
  // data_gnt indicates next clock cycle inputs can change so at the clock edge, inputs
  //   will be latched by the memory already
  always_comb begin
    data_gnt_a <= data_req_a; 
    data_gnt_b <= data_req_b; 
  end

  // data is ready clock cycle after the data_req is made
  always_ff @(posedge clk) begin
    // data is read clk cycle after data_req made
    // so it should be latched on clk edges
    data_rvalid_a <= data_req_a;
    data_rvalid_b <= data_req_b;
  end

  // (todo) handle same addr access error
  // handle addr mapping only if addr is wider than memory addr
  // error code is only checked on data_rvalid so I can just have it always assigned to
  generate
    if (ADDR_WIDTH > M_ADDR_WIDTH) begin
        // error when address map is different from expected
        // error when two port same addr access and one of them is a write access
        assign data_err_next_a = (ADDR_MAP_A[ADDR_WIDTH-1:M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)] != data_addr_a[ADDR_WIDTH-1:M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)]
                || (data_addr_aw == data_addr_bw && ~ceba && ~cebb && (data_we_a || data_we_b))
        );
        assign data_err_next_b = (ADDR_MAP_B[ADDR_WIDTH-1:M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)] != data_addr_b[ADDR_WIDTH-1:M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)]
                || (data_addr_aw == data_addr_bw && ~ceba && ~cebb && (data_we_a || data_we_b))
        );
    end
    else begin
      assign data_err_next_a = data_addr_aw == data_addr_bw && ~ceba && ~cebb && (data_we_a || data_we_b);
      assign data_err_next_b = data_addr_aw == data_addr_bw && ~ceba && ~cebb && (data_we_a || data_we_b);
    end
  endgenerate

  always_ff @ (posedge clk) begin
    data_err_a <= data_err_next_a;
    data_err_b <= data_err_next_b;
  end
endmodule