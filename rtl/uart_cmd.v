//============================================================================
// uart_cmd.v —— 用串口命令控制拓展功能（替代难拨的拨码/失灵按键）
//
// 因为板载 2 位拨码是蓝色 DIP、拨片极小，且 4 个按键在实测中读回恒为 0，
// 所以改用【串口下发单字节命令】控制目标位置。L20 是空闲的 UART RX，
// 与遥测 TX（K20）共用同一根 USB 线，不需要额外接线。
//
// 命令（单字节，115200 8N1）：
//   '+' (0x2B)  目标 +STEP_COUNTS
//   '-' (0x2D)  目标 -STEP_COUNTS
//   '0' (0x30)  回原点
//   'p'/'P'     往返演示开关（在 ±STEP 之间自动往复）
//   '?' (0x3F)  ping（保留）
//
// 按键仍然接在 traj_gen 上（如果将来按键修好，两者可并存）。
//============================================================================
`timescale 1ns/1ps

module uart_cmd #(
    parameter integer UART_BPS = 115200,
    parameter integer CLK_FREQ = 50000000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire rx,                 // L20
    output reg  step_plus,          // 1 拍脉冲
    output reg  step_minus,
    output reg  go_home,
    output reg  pingpong_toggle,
    output reg  [7:0] last_byte     // 诊断：最后一个收到的字节
);

    wire [7:0] data;
    wire       valid;

    uart_rx #(.UART_BPS(UART_BPS), .CLK_FREQ(CLK_FREQ)) rxd(
        clk, rst_n, rx, data, valid);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step_plus       <= 1'b0;
            step_minus      <= 1'b0;
            go_home         <= 1'b0;
            pingpong_toggle <= 1'b0;
            last_byte       <= 8'h00;
        end else begin
            step_plus       <= 1'b0;
            step_minus      <= 1'b0;
            go_home         <= 1'b0;
            pingpong_toggle <= 1'b0;
            if (valid) begin
                last_byte <= data;
                case (data)
                    8'h2B: step_plus       <= 1'b1;   // '+'
                    8'h2D: step_minus      <= 1'b1;   // '-'
                    8'h30: go_home         <= 1'b1;   // '0'
                    8'h50: pingpong_toggle <= 1'b1;   // 'P'
                    8'h70: pingpong_toggle <= 1'b1;   // 'p'
                    default: begin end
                endcase
            end
        end
    end

endmodule
