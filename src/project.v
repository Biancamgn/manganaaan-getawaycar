/*
 * Endless Runner v2 - VGA playground
 * 640x480 @ 60Hz, 25.175 MHz pixel clock, TinyVGA Pmod pinout.
 *
 * Nothing about the world is stored. The road centre, the obstacles and
 * every tree, bush and rock are pure functions of position, so the course
 * is infinite, repeatable, and costs no memory.
 *
 * Controls:
 *   ui[0] steer left
 *   ui[1] steer right
 *   ui[2] restart
 *
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_endless_runner (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // ==================================================================
  // VGA timing
  // ==================================================================
  localparam H_VIS = 640, H_FP = 16, H_SY = 96, H_TOT = 800;
  localparam V_VIS = 480, V_FP = 10, V_SY = 2,  V_TOT = 525;

  reg [9:0] hpos, vpos;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hpos <= 10'd0; vpos <= 10'd0;
    end else if (hpos == H_TOT - 1) begin
      hpos <= 10'd0;
      vpos <= (vpos == V_TOT - 1) ? 10'd0 : vpos + 1'b1;
    end else begin
      hpos <= hpos + 1'b1;
    end
  end

  wire hsync     = ~((hpos >= H_VIS + H_FP) && (hpos < H_VIS + H_FP + H_SY));
  wire vsync     = ~((vpos >= V_VIS + V_FP) && (vpos < V_VIS + V_FP + V_SY));
  wire visible   = (hpos < H_VIS) && (vpos < V_VIS);
  wire frame_end = (vpos == V_VIS) && (hpos == 10'd0);

  // ==================================================================
  // Game state
  // ==================================================================
  localparam CAR_W   = 10'd28;
  localparam CAR_TOP = 10'd396;
  localparam CAR_H   = 10'd44;
  localparam CAR_MID = 10'd418;

  reg [15:0] scroll;
  reg [9:0]  car_x;
  reg        crashed;
  reg [5:0]  crash_t;
  reg [3:0]  sc0, sc1, sc2;
  reg [2:0]  tick;

  // ==================================================================
  // Track centre: triangle wave with its 3rd and 5th harmonics
  // subtracted to round off the corners. Both harmonics are shift-adds.
  //    centre(w) = T(w) - T(3w)/8 - T(5w)/8  + slow sweep
  // ==================================================================
  wire [15:0] w = scroll - {6'd0, vpos};

  function [9:0] trackc;
    input [15:0] ww;
    reg [15:0] a3, a5;
    reg [6:0]  q1, q3, q5, qs;
    reg [8:0]  f;
    begin
      a3 = ww + {ww[14:0], 1'b0};
      a5 = ww + {ww[13:0], 2'b00};
      q1 = ww[8] ? ~ww[7:1] : ww[7:1];
      q3 = a3[8] ? ~a3[7:1] : a3[7:1];
      q5 = a5[8] ? ~a5[7:1] : a5[7:1];
      f  = {2'd0, q1} + 9'd32 - {5'd0, q3[6:3]} - {5'd0, q5[6:3]};
      qs = ww[11] ? ~ww[10:4] : ww[10:4];
      trackc = 10'd320 + {1'd0, f} + {3'd0, qs} - 10'd144;
    end
  endfunction

  wire [9:0] centre = trackc(w);

  // road is wide to begin with and narrows with distance
  wire [3:0] lvl    = scroll[15:12];
  wire [6:0] shrink = (lvl > 4'd10) ? 7'd50 : {2'd0, lvl, 1'd0} + {3'd0, lvl};
  wire [6:0] halfw  = 7'd110 - shrink;

  wire [9:0] road_l = centre - {3'd0, halfw};
  wire [9:0] road_r = centre + {3'd0, halfw};
  wire       on_road = (hpos >= road_l) && (hpos < road_r);

  // ==================================================================
  // Procedural scatter hash (10-bit xorshift, pure wiring + XOR)
  // ==================================================================
  function [9:0] scramble;
    input [9:0] k;
    reg   [9:0] a;
    begin
      a = k;
      a = a ^ {a[6:0], 3'd0};
      a = a ^ {5'd0, a[9:5]};
      a = a ^ {a[7:0], 2'd0};
      scramble = a;
    end
  endfunction

  // ------------------------------------------------------------------
  // Obstacles in the road: one band every 64 world units
  // ------------------------------------------------------------------
  wire [9:0] o_band = w[15:6];
  wire [9:0] o_h    = scramble(o_band ^ 10'h1A7);

  wire       o_here = (o_h[3:0] < 4'd5);          // ~31% of bands
  wire [2:0] o_lane = o_h[6:4];                   // which of 8 lanes
  wire [9:0] o_off  = {3'd0, halfw} * {7'd0, o_lane} >> 2;
  wire [9:0] o_anchor = trackc({o_band, 6'd32}) - {3'd0, halfw};
  wire [9:0] o_x0   = o_anchor + o_off;
  wire [9:0] o_x1   = o_x0 + 10'd36;

  wire o_rows = (w[5:0] >= 6'd16) && (w[5:0] < 6'd48);
  wire [5:0] o_gy = w[5:0] - 6'd16;
  wire [9:0] o_gx = hpos - o_x0;

  wire [9:0] o_dx = (o_gx >= 10'd18) ? (o_gx - 10'd18) : (10'd18 - o_gx);
  wire [5:0] o_dy = (o_gy >= 6'd16)  ? (o_gy - 6'd16)  : (6'd16 - o_gy);

  wire obstacle = o_here && o_rows && on_road &&
                  (hpos >= o_x0) && (hpos < o_x1) &&
                  ((o_dx + {4'd0, o_dy}) < 10'd18);

  // ------------------------------------------------------------------
  // Scenery in the grass: 32x32 cells, hashed for type and presence
  // ------------------------------------------------------------------
  wire [4:0] g_cx = hpos[9:5];
  wire [4:0] g_cy = w[9:5];
  wire [9:0] g_h  = scramble({g_cx, g_cy} ^ 10'h2C9);

  wire       g_here = (g_h[2:0] == 3'd0);   // ~1 in 8 cells
  wire [1:0] g_type = g_h[4:3];                   // 0,1 = tree; 2 = bush; 3 = rock

  wire [4:0] g_x = hpos[4:0];
  wire [4:0] g_y = ~w[4:0];   // w decreases down-screen, so mirror

  // diamond distance from a centre
  wire [4:0] dx12 = (g_x >= 5'd16) ? (g_x - 5'd16) : (5'd16 - g_x);

  wire [4:0] dyT  = (g_y >= 5'd12) ? (g_y - 5'd12) : (5'd12 - g_y);
  wire [4:0] dyB  = (g_y >= 5'd19) ? (g_y - 5'd19) : (5'd19 - g_y);
  wire [4:0] dyR  = (g_y >= 5'd22) ? (g_y - 5'd22) : (5'd22 - g_y);

  wire is_tree = (g_type[1] == 1'b0);
  wire is_bush = (g_type == 2'd2);
  wire is_rock = (g_type == 2'd3);

  wire canopy = g_here && is_tree && ((dx12 + dyT) < 5'd9);
  wire trunk  = g_here && is_tree && (g_x >= 5'd14) && (g_x < 5'd18)
                                  && (g_y >= 5'd17) && (g_y < 5'd28);
  wire bush   = g_here && is_bush && ((dx12 + dyB) < 5'd8);
  wire rock   = g_here && is_rock && ((dx12 + dyR) < 5'd7);

  wire scenery = !on_road && (canopy || trunk || bush || rock);

  // ==================================================================
  // Collision: latch the road edges and the obstacle at the car's row
  // ==================================================================
  reg [9:0] hit_l, hit_r, hit_o0, hit_o1;
  reg       hit_obs;

  always @(posedge clk) begin
    if (vpos == CAR_MID && hpos == 10'd0) begin
      hit_l   <= road_l;
      hit_r   <= road_r;
      hit_o0  <= o_x0;
      hit_o1  <= o_x1;
      hit_obs <= o_here && o_rows;
    end
  end

  wire off_road = (car_x < hit_l) || ((car_x + CAR_W) > hit_r);
  wire bumped   = hit_obs && !((car_x + CAR_W) <= hit_o0 || (car_x >= hit_o1));

  // ==================================================================
  // Per-frame update
  // ==================================================================
  wire [2:0] speed = 3'd2 + {1'd0, scroll[15:14]};

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      scroll  <= 16'd0;
      car_x   <= 10'd306;
      crashed <= 1'b0;
      crash_t <= 6'd0;
      sc0 <= 4'd0; sc1 <= 4'd0; sc2 <= 4'd0;
      tick <= 3'd0;
    end else if (frame_end) begin
      if (ui_in[2]) begin
        scroll  <= 16'd0;
        car_x   <= 10'd306;
        crashed <= 1'b0;
        crash_t <= 6'd0;
        sc0 <= 4'd0; sc1 <= 4'd0; sc2 <= 4'd0;
      end else if (crashed) begin
        if (crash_t == 6'd0) begin
          scroll  <= 16'd0;
          car_x   <= 10'd306;
          crashed <= 1'b0;
          sc0 <= 4'd0; sc1 <= 4'd0; sc2 <= 4'd0;
        end else begin
          crash_t <= crash_t - 1'b1;
        end
      end else begin
        scroll <= scroll + {13'd0, speed};

        if (ui_in[0] && car_x > 10'd12)
          car_x <= car_x - 10'd3;
        else if (ui_in[1] && car_x < (10'd640 - CAR_W - 10'd12))
          car_x <= car_x + 10'd3;

        if (off_road || bumped) begin
          crashed <= 1'b1;
          crash_t <= 6'd45;
        end

        tick <= tick + 1'b1;
        if (tick == 3'd7) begin
          if (sc0 == 4'd9) begin
            sc0 <= 4'd0;
            if (sc1 == 4'd9) begin
              sc1 <= 4'd0;
              sc2 <= (sc2 == 4'd9) ? 4'd0 : sc2 + 1'b1;
            end else sc1 <= sc1 + 1'b1;
          end else sc0 <= sc0 + 1'b1;
        end
      end
    end
  end

  // ==================================================================
  // Score digits
  // ==================================================================
  function [6:0] seg7;
    input [3:0] d;
    begin
      case (d)
        4'd0: seg7 = 7'b0111111;  4'd1: seg7 = 7'b0000110;
        4'd2: seg7 = 7'b1011011;  4'd3: seg7 = 7'b1001111;
        4'd4: seg7 = 7'b1100110;  4'd5: seg7 = 7'b1101101;
        4'd6: seg7 = 7'b1111101;  4'd7: seg7 = 7'b0000111;
        4'd8: seg7 = 7'b1111111;  4'd9: seg7 = 7'b1101111;
        default: seg7 = 7'b0000000;
      endcase
    end
  endfunction

  wire s_row = (vpos >= 10'd16) && (vpos < 10'd56);
  wire s_d0  = (hpos >= 10'd16) && (hpos < 10'd44);
  wire s_d1  = (hpos >= 10'd48) && (hpos < 10'd76);
  wire s_d2  = (hpos >= 10'd80) && (hpos < 10'd108);

  reg [9:0] slx;
  always @(*) begin
    if      (s_d0) slx = hpos - 10'd16;
    else if (s_d1) slx = hpos - 10'd48;
    else           slx = hpos - 10'd80;
  end
  wire [9:0] sly = vpos - 10'd16;

  wire qL = (slx < 10'd6);
  wire qR = (slx >= 10'd22);
  wire qM = (slx >= 10'd6)  && (slx < 10'd22);
  wire qT = (sly < 10'd6);
  wire qB = (sly >= 10'd34);
  wire qI = (sly >= 10'd17) && (sly < 10'd23);
  wire qU = (sly >= 10'd6)  && (sly < 10'd17);
  wire qD = (sly >= 10'd23) && (sly < 10'd34);

  wire [6:0] sshape = { qI&qM, qL&qU, qL&qD, qB&qM, qR&qD, qR&qU, qT&qM };

  reg [3:0] sdig;
  always @(*) begin
    if      (s_d0) sdig = sc2;
    else if (s_d1) sdig = sc1;
    else           sdig = sc0;
  end

  wire score_lit = s_row && (s_d0 || s_d1 || s_d2) && |(sshape & seg7(sdig));

  // ==================================================================
  // Scene composition
  // ==================================================================
  wire kerb     = ((hpos >= road_l) && (hpos < road_l + 10'd8)) ||
                  ((hpos >= road_r - 10'd8) && (hpos < road_r));
  wire dash     = (hpos >= centre - 10'd3) && (hpos < centre + 10'd3) && w[4];
  wire grass_lt = w[5];

  wire car_body  = (hpos >= car_x) && (hpos < car_x + CAR_W) &&
                   (vpos >= CAR_TOP) && (vpos < CAR_TOP + CAR_H);
  wire car_glass = (hpos >= car_x + 10'd5) && (hpos < car_x + CAR_W - 10'd5) &&
                   (vpos >= CAR_TOP + 10'd7) && (vpos < CAR_TOP + 10'd19);

  wire flash = crashed && crash_t[2];

  reg [1:0] R, G, B;
  always @(*) begin
    if (!visible) begin
      R = 2'b00; G = 2'b00; B = 2'b00;
    end else if (flash) begin
      R = 2'b11; G = 2'b00; B = 2'b00;
    end else if (score_lit) begin
      R = 2'b11; G = 2'b11; B = 2'b11;
    end else if (car_glass) begin
      R = 2'b01; G = 2'b10; B = 2'b11;
    end else if (car_body) begin
      R = 2'b11; G = 2'b00; B = 2'b01;
    end else if (obstacle) begin
      R = 2'b11; G = 2'b10; B = 2'b00;            // orange hazard
    end else if (canopy && !on_road) begin
      R = 2'b00; G = 2'b11; B = 2'b01;            // bright canopy
    end else if (trunk && !on_road) begin
      R = 2'b10; G = 2'b01; B = 2'b00;            // brown trunk
    end else if (bush && !on_road) begin
      R = 2'b01; G = 2'b11; B = 2'b01;
    end else if (rock && !on_road) begin
      R = 2'b01; G = 2'b01; B = 2'b10;            // slate rock
    end else if (kerb) begin
      R = 2'b11;
      G = w[4] ? 2'b11 : 2'b00;
      B = w[4] ? 2'b11 : 2'b00;
    end else if (dash) begin
      R = 2'b11; G = 2'b11; B = 2'b11;
    end else if (on_road) begin
      R = 2'b01; G = 2'b01; B = 2'b01;
    end else begin
      R = 2'b00; G = grass_lt ? 2'b01 : 2'b10; B = 2'b00;
    end
  end

  assign uo_out  = { hsync, B[0], G[0], R[0], vsync, B[1], G[1], R[1] };
  assign uio_out = 8'd0;
  assign uio_oe  = 8'd0;

  wire _unused = &{ena, uio_in, ui_in[7:3], scenery, 1'b0};

endmodule
