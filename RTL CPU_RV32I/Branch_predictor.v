/*-------------------------------------------------------------------------
This module implements a 2-bit bimodal branch predictor using a 256-entry table of saturating counters.
It predicts branch direction based on the most significant bit (MSB) of each counter. On each clock cy-
-cle, the predictor updates its state according to the actual branch outcome if update_en is high.

      Strongly not taken(00) -- Weakly not taken(01) -- Weakly taken(10) -- Strongly taken(11)
-------------------------------------------------------------------------*/
module branch_predictor (
    input           clk,
    input           rst,    
    input           insBRA,
    input  [31:0]   pc_pred_in,         
    input           update_en,    
    input           actual_taken,  
    output          predict_taken   //0: not take, 1: take
);

    // Bảng 2-bit saturating counter: 256 dòng
    (* ram_style = "block" *) reg [1:0] BHT [255:0];  // 2-bit per entry. (Branch history table)
           
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) BHT[i] <= 2'b01; 
    end

    wire [7:0] index = pc_pred_in[9:2]; // Chọn index từ PC; 8 bit -> 255 trường hợp

    // Đọc dự đoán
    assign predict_taken = (insBRA)?BHT[index][1]:1'b1;

    // Cập nhật theo kết quả thật
    
    always @(posedge clk) begin
        if (!rst) begin
            for (i = 0; i < 256; i = i + 1) BHT[i] <= 2'b01;
        end 
        else if (update_en) begin
            case ({actual_taken, BHT[index]})
                3'b0_00: BHT[index] <= 2'b00; // Giữ SN
                3'b0_01: BHT[index] <= 2'b00; // WN -> SN
                3'b0_10: BHT[index] <= 2'b01; // WT -> WN
                3'b0_11: BHT[index] <= 2'b10; // ST -> WT
                3'b1_00: BHT[index] <= 2'b01; // SN -> WN
                3'b1_01: BHT[index] <= 2'b10; // WN -> WT
                3'b1_10: BHT[index] <= 2'b11; // WT -> ST
                3'b1_11: BHT[index] <= 2'b11; // Giữ ST
            endcase
        end
    end

endmodule
