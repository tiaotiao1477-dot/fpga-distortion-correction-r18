`timescale 1ns/1ps
// Source-domain bundled-data snapshot bridge. The payload is captured on one
// source clock edge and held stable until the next acknowledged request.
module camera_diag_snapshot_cdc #(
  parameter integer WIDTH = 32
)(
  input                  src_clk,
  input                  src_rst_n,
  input                  request_toggle_async,
  input      [WIDTH-1:0] counter_bus,
  output reg [WIDTH-1:0] snapshot_bus,
  output reg             ack_toggle
);
  reg request_meta;
  reg request_sync;
  reg request_seen;

  always @(posedge src_clk or negedge src_rst_n) begin
    if (!src_rst_n) begin
      request_meta <= 1'b0;
      request_sync <= 1'b0;
      request_seen <= 1'b0;
      snapshot_bus <= {WIDTH{1'b0}};
      ack_toggle <= 1'b0;
    end else begin
      request_meta <= request_toggle_async;
      request_sync <= request_meta;
      if (request_sync != request_seen) begin
        request_seen <= request_sync;
        snapshot_bus <= counter_bus;
        ack_toggle <= request_sync;
      end
    end
  end
endmodule

// System-domain coordinator for camera, DDR, pixel, and two color-statistics
// acknowledgements. Each fresh bit requires a new ack transition matching A5.
module camera_diag_snapshot_status (
  input            sys_clk,
  input            rst_n,
  input            request_toggle,
  input            ack_toggle_cam_async,
  input            ack_toggle_ddr_async,
  input            ack_toggle_pix_async,
  input            ack_toggle_cam_color_async,
  input            ack_toggle_disp_color_async,
  output reg [4:0] fresh_mask,
  output wire      snapshot_valid
);
  reg ack_cam_meta, ack_cam_sync, ack_cam_sync_d;
  reg ack_ddr_meta, ack_ddr_sync, ack_ddr_sync_d;
  reg ack_pix_meta, ack_pix_sync, ack_pix_sync_d;
  reg ack_cam_color_meta, ack_cam_color_sync, ack_cam_color_sync_d;
  reg ack_disp_color_meta, ack_disp_color_sync, ack_disp_color_sync_d;
  reg request_seen;
  reg request_active;

  assign snapshot_valid = &fresh_mask;

  always @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) begin
      ack_cam_meta <= 1'b0; ack_cam_sync <= 1'b0; ack_cam_sync_d <= 1'b0;
      ack_ddr_meta <= 1'b0; ack_ddr_sync <= 1'b0; ack_ddr_sync_d <= 1'b0;
      ack_pix_meta <= 1'b0; ack_pix_sync <= 1'b0; ack_pix_sync_d <= 1'b0;
      ack_cam_color_meta <= 1'b0; ack_cam_color_sync <= 1'b0; ack_cam_color_sync_d <= 1'b0;
      ack_disp_color_meta <= 1'b0; ack_disp_color_sync <= 1'b0; ack_disp_color_sync_d <= 1'b0;
      request_seen <= 1'b0;
      request_active <= 1'b0;
      fresh_mask <= 5'b00000;
    end else begin
      ack_cam_meta <= ack_toggle_cam_async;
      ack_cam_sync <= ack_cam_meta;
      ack_cam_sync_d <= ack_cam_sync;
      ack_ddr_meta <= ack_toggle_ddr_async;
      ack_ddr_sync <= ack_ddr_meta;
      ack_ddr_sync_d <= ack_ddr_sync;
      ack_pix_meta <= ack_toggle_pix_async;
      ack_pix_sync <= ack_pix_meta;
      ack_pix_sync_d <= ack_pix_sync;
      ack_cam_color_meta <= ack_toggle_cam_color_async;
      ack_cam_color_sync <= ack_cam_color_meta;
      ack_cam_color_sync_d <= ack_cam_color_sync;
      ack_disp_color_meta <= ack_toggle_disp_color_async;
      ack_disp_color_sync <= ack_disp_color_meta;
      ack_disp_color_sync_d <= ack_disp_color_sync;

      if (request_toggle != request_seen) begin
        request_seen <= request_toggle;
        request_active <= 1'b1;
        fresh_mask <= 5'b00000;
      end else if (request_active) begin
        if ((ack_cam_sync == request_seen) && (ack_cam_sync != ack_cam_sync_d))
          fresh_mask[0] <= 1'b1;
        if ((ack_ddr_sync == request_seen) && (ack_ddr_sync != ack_ddr_sync_d))
          fresh_mask[1] <= 1'b1;
        if ((ack_pix_sync == request_seen) && (ack_pix_sync != ack_pix_sync_d))
          fresh_mask[2] <= 1'b1;
        if ((ack_cam_color_sync == request_seen) &&
            (ack_cam_color_sync != ack_cam_color_sync_d))
          fresh_mask[3] <= 1'b1;
        if ((ack_disp_color_sync == request_seen) &&
            (ack_disp_color_sync != ack_disp_color_sync_d))
          fresh_mask[4] <= 1'b1;
        if (((fresh_mask[0] || ((ack_cam_sync == request_seen) && (ack_cam_sync != ack_cam_sync_d))) &&
             (fresh_mask[1] || ((ack_ddr_sync == request_seen) && (ack_ddr_sync != ack_ddr_sync_d))) &&
             (fresh_mask[2] || ((ack_pix_sync == request_seen) && (ack_pix_sync != ack_pix_sync_d))) &&
             (fresh_mask[3] || ((ack_cam_color_sync == request_seen) &&
                                (ack_cam_color_sync != ack_cam_color_sync_d))) &&
             (fresh_mask[4] || ((ack_disp_color_sync == request_seen) &&
                                (ack_disp_color_sync != ack_disp_color_sync_d)))))
          request_active <= 1'b0;
      end
    end
  end
endmodule
