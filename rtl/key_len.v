//============================================================================
// key_len.v —— 把"短按/长按"分开（拓展功能用按键代替拨码）
//
// 背景：板载 2 位拨码是蓝色 DIP，拨片很小、很难拨。改为用 SW3/SW4 按键：
//   SW3 短按 -> 目标 +STEP
//   SW4 短按 -> 目标 -STEP
//   SW3 长按 -> 直立标定（保留）
//   SW4 长按 -> 下垂标定（保留）
//
// 本模块负责测量按键按住时长，松开时按阈值给出 short_p / long_p 脉冲。
//============================================================================
`timescale 1ns/1ps

module key_len #(
    parameter integer LONG_TICKS = 1200      // 长按阈值 1200 ms
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick,                 // 1 ms
    input  wire [1:0]  level,                // keys[3:2]   （1 = 按下）
    input  wire [1:0]  press,                // pressed[3:2]（按下瞬间的 1 拍脉冲）
    output reg  [1:0]  short_p,              // 短按松开 -> 1 拍脉冲
    output reg  [1:0]  long_p                // 长按松开 -> 1 拍脉冲
);

    reg [1:0]  lvl_d;
    reg [15:0] cnt0, cnt1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lvl_d   <= 2'b00;
            cnt0    <= 16'd0;
            cnt1    <= 16'd0;
            short_p <= 2'b00;
            long_p  <= 2'b00;
        end else begin
            short_p <= 2'b00;                // 默认无脉冲
            long_p  <= 2'b00;
            lvl_d   <= level;

            // ---- SW3 (bit0) ----
            if (press[0])                 cnt0 <= 16'd0;
            else if (tick && level[0])    begin if (cnt0 < 16'hFFFF) cnt0 <= cnt0 + 16'd1; end
            if (lvl_d[0] && !level[0]) begin         // 松开沿
                if (cnt0 >= LONG_TICKS) long_p[0]  <= 1'b1;
                else                    short_p[0] <= 1'b1;
            end

            // ---- SW4 (bit1) ----
            if (press[1])                 cnt1 <= 16'd0;
            else if (tick && level[1])    begin if (cnt1 < 16'hFFFF) cnt1 <= cnt1 + 16'd1; end
            if (lvl_d[1] && !level[1]) begin
                if (cnt1 >= LONG_TICKS) long_p[1]  <= 1'b1;
                else                    short_p[1] <= 1'b1;
            end
        end
    end

endmodule
