// Memory wrapper around tsmc dual port memory
module memory_wrapper # (
  DATA_WIDTH                        = 32,
  ADDR_WIDTH                        = 32,
  M_ADDR_WIDTH                      = 16,
  logic [ADDR_WIDTH-1:0] ADDR_MAP_A = '0,
  logic [ADDR_WIDTH-1:0] ADDR_MAP_B = '0
) (
  input  logic                    clk,
  input  logic                    rst,
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

  // latch inputs at clk negedge to avoid hold violations
  always_ff @(negedge clk) begin
    if (!rst) begin
      data_addr_aw <= '0;
      data_wdata_aw <= '0;
      data_we_aw <= '0;
      data_addr_bw <= '0;
      data_wdata_bw <= '0;
      data_we_bw <= '0;

      ceba <= '1;
      cebb <= '1;
    end else begin
      // memory address assumes that the processor is using byte addressing
      data_addr_aw <= data_addr_a[M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)-1:$clog2(DATA_WIDTH/8)];
      data_wdata_aw <= data_wdata_a;
      data_we_aw <= data_we_a;

      data_addr_bw <= data_addr_b[M_ADDR_WIDTH+$clog2(DATA_WIDTH/8)-1:$clog2(DATA_WIDTH/8)];
      data_wdata_bw <= data_wdata_b;
      data_we_bw <= data_we_b;

      ceba <= ~data_req_a;
      cebb <= ~data_req_b;
    end
  end

  // use byte enable to get bit write enable
  genvar i, j;
  for (i = 0; i < DATA_WIDTH/8; i++) begin
    for (j = 0; j < 8; j++) begin
      always_ff @(negedge clk) begin
        if (!rst) begin
          bweba[i*8+j] <= '0;
          bweba[i*8+j] <= '0;
        end else begin
          bweba[i*8+j] <= ~data_be_a[i];
          bwebb[i*8+j] <= ~data_be_b[i];
        end
      end
    end
  end

  // sram instantiation
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

  // data_gnt indicates next clock cycle inputs can change so at the clock edge, inputs
  //   will be latched by the memory already
  always_comb begin
    data_gnt_a <= data_req_a; 
    data_gnt_b <= data_req_b; 
  end

  // data is ready clock cycle after the data_req is made
  always_ff @(posedge clk or negedge rst) begin
    // data is read clk cycle after data_req made
    // so it should be latched on clk edges
    if (!rst) begin
      data_rvalid_a <= 0;
      data_rvalid_b <= 0;
    end else begin
      data_rvalid_a <= data_req_a;
      data_rvalid_b <= data_req_b;
    end
  end

  // handle addr mapping only if addr is wider than memory addr
  // error code is only checked on data_rvalid high so I can just have it always assigned to
  generate
    if (ADDR_WIDTH > M_ADDR_WIDTH) begin
        // error when address map is different from expected
        // error when two port same addr access and at least one of them is a write access
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

  // set error output
  always_ff @ (posedge clk or negedge rst) begin
    if (!rst) begin
      data_err_a <= 0;
      data_err_b <= 0;
    end else begin
      data_err_a <= data_err_next_a;
      data_err_b <= data_err_next_b;
    end
  end
endmodule