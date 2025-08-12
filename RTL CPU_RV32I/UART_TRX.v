/*                  DTP B21DT166
Module chức năng UART hỗ trợ truyền song công
Để gửi dữ liệu cần set bit yêu cầu truyền và reset sau đó
Để nhận dữ liệu cần kiểm tra bit RXNE và đọc dữ liệu từ
thanh ghi data_rx
Không có ngắt nên cần lưu ý khi kiểm tra bit RXNE chỉ được
set trong 1 chu kỳ.
*/

/*
Thanh ghi cấu hình
    B[0]: EN
    B[1]: STRTX
    B[7:4]: BR: bandrate 
        0000: 600
        0001: 1200
        0010: 2400
        0011: 4800
        0100: 9600
        0101: 14400
        0110: 19200
        0111: 38400
        1000: 56000
        1001: 57600
        1111: 115200
    B[15:8]: CLK: chứa tốc hệ thống khai báo. Không hỗ trợ chia tần
Thanh ghi trạng thái
    B[0]: TBUSY
    B[1]: RXNE

Thanh ghi dữ liệu TX
Thanh ghi dữ liệu RX
*/
module UART
	#(parameter	CLOCK = 2_700_000,
			BAUD_RATE = 115_200) (
	input       i_clk,                      //CLOCK hệ thống
    input       i_rst,                      //Reset đồng bộ tích cực thấp
    input       i_en,                       //Cờ bật trạng thái hoạt động của ngoại vi UART 
    input       i_str_tx,                   //Cờ bật yêu cầu truyền cho ngoại vi UART
    input [7:0] i_data_tx,                  //Bit_f chứa dữ liệu 8 bit muốn truyền đi  
    input [3:0] i_br,                       //Bit_f chứa tốc độ band của ngoại vi UART
    input [7:0] i_clk_dec,                  //Bit_f khai báo tần số hệ thống. Chỉ sử dụng tần số hệ thống không hỗ trợ chia tần
    
	input       i_RX,
	output      o_TX,

    output      o_busy_tx,                  //Cờ báo hiệu ngoại vi UART đang bận truyền
    output      o_RXNE,                     //Cờ báo hiệu ngoại vi UART nhận được đủ 8bit từ bên truyền
    output [7:0]o_data_rx                   //Bit_f chứa dữ liệu 8 bit nhận được từ bên truyền

	//output      o_blink_led               //Thiết kế để TEST
);
`define MAKEBAND
wire [31:0] BAUDR;
`ifdef MAKEBAND
    assign BAUDR = (i_br == 0)?600:
                   (i_br == 1)?1200:
                   (i_br == 2)?2400:
                   (i_br == 3)?4800:
                   (i_br == 4)?9600:
                   (i_br == 5)?14400:
                   (i_br == 6)?19200:
                   (i_br == 7)?38400:
                   (i_br == 8)?56000:
                   (i_br == 9)?57600:
                   115200;
`endif


    localparam FRAMES = CLOCK / BAUD_RATE;
	localparam HALF_FRAME = FRAMES / 2;
    
    wire [31:0] BAND_CNT = (CLOCK)/BAUDR;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////UART_RX/////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
	reg r_RX_ready;
	reg r_blink_led;
    reg [15:0] r_RX_counter = 0;
    reg [3:0] bit_idx = 0;
    reg [7:0] rx_shift = 0;
    reg [3:0] RX_state = 0;
    reg [7:0] ascii_data;


    always @(posedge i_clk) begin
        if (!i_rst || !i_en) begin
            RX_state <= 0;
            r_RX_counter <= 0;
            bit_idx <= 0;
            r_RX_ready <= 0;
        end else begin
            case (RX_state)
                0: begin // Wait for start bit
                    r_RX_ready <= 0;
                    if (!i_RX) begin
                        r_RX_counter <= BAND_CNT >> 1;
                        RX_state <= 1;
                    end
                end
                1: begin // Start bit midpoint
                    if (r_RX_counter == 0) begin
                        r_RX_counter <= BAND_CNT;
                        RX_state <= 2;
                        bit_idx <= 0;
                    end else
                        r_RX_counter <= r_RX_counter - 1;
                end
                2: begin // Receive 8 bits
                    if (r_RX_counter == 0) begin
                        rx_shift[bit_idx] <= i_RX;
                        if (bit_idx == 7) begin
                            RX_state <= 3;
                        end else
                            bit_idx <= bit_idx + 1;
                        r_RX_counter <= BAND_CNT;
                    end else
                        r_RX_counter <= r_RX_counter - 1;
                end
                3: begin // Stop bit
                    if (r_RX_counter == 0) begin
                        ascii_data <= rx_shift;
                        r_RX_ready <= 1;
                        RX_state <= 4;
                    end 
                    else r_RX_counter <= r_RX_counter - 1;
                end
                4: begin
                    r_RX_ready <= 1;
                    RX_state <= 0;
                end
            endcase
        end
    end

//////////////////////////////////////////ASS////////////////////////////////////////
    assign o_RXNE       =   r_RX_ready;
    assign o_data_rx    =   ascii_data;


///////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////UART_TX////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
	reg [25:0] r_TX_counter;
	reg [7:0] r_TX_byte;
	reg [2:0] r_TX_index;
	reg r_TX;
    reg r_busy_tx;
    reg r_busy_txfast;
    reg r_busy_txnot = 1'b0;

    assign o_busy_tx = r_busy_txfast || r_busy_tx;
    always @(posedge i_clk) begin
        if (!i_rst || !i_en) begin
            r_busy_txfast = 0;
		end
        else begin
            r_busy_txfast = 0;
            if(i_str_tx && !r_busy_tx && !r_busy_txnot) r_busy_txfast = 1;
        end
    end

	integer TX_state;
	localparam	TX_IDLE = 0;
	localparam  TX_START = 1;
	localparam	TX_WRITE = 2;
	localparam	TX_STOP = 3;
	localparam	TX_DONE = 4;

	always@(posedge i_clk) begin
		if (!i_rst || !i_en) begin
            r_TX_counter <= 1;
			r_TX_byte <= 0;
			r_TX_index <= 0;
			r_TX <= 1;
			TX_state <= TX_IDLE;
            r_busy_tx <= 0;
            r_busy_txnot <= 0;
		end
		else begin
			case (TX_state)
				TX_IDLE : begin
                    r_busy_tx <= 0;
                    if (i_str_tx) begin
                        r_TX <= 0;
                        r_busy_tx <= 1;
                        TX_state <= TX_START;
                    end
				end

				TX_START : begin
					if (r_TX_counter == BAND_CNT) begin
						r_TX_counter <= 1;
						r_TX_byte <= i_data_tx;
						TX_state <= TX_WRITE;
					end
					else r_TX_counter <= r_TX_counter + 1;
				end

				TX_WRITE : begin
					r_TX <= r_TX_byte[r_TX_index];
					if (r_TX_counter == BAND_CNT) begin
						r_TX_counter <= 1;
						if (r_TX_index == 3'd7) begin
							r_TX_index <= 0;
							TX_state <= TX_STOP;
						end
						else r_TX_index <= r_TX_index + 1;
					end
					else r_TX_counter <= r_TX_counter + 1;
				end

				TX_STOP : begin
					r_TX <= 1;
					if (r_TX_counter == BAND_CNT) begin
						r_TX_counter <= 1;
						TX_state <= TX_DONE;
					end
					else r_TX_counter <= r_TX_counter + 1;
				end

				TX_DONE : begin
                    r_busy_tx <= 0;
                    r_busy_txnot <= 1'b1;
					if (!i_str_tx) begin
                        r_busy_txnot <= 1'b0;
                        TX_state <= TX_IDLE;
                    end
				end
			endcase
		end
	end

//////////////////////////////////////////ASS//////////////////////////////////////////
	assign o_TX = r_TX;
endmodule


///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////FOR TEST//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/*
	always@(posedge i_clk, negedge i_rst) begin
		if (!i_rst)
			r_blink_led <= 0;
		else if (r_RX_ready) begin
			if (ascii_data == 8'h31)
				r_blink_led <= 1;
			else if (ascii_data == 8'h30)
				r_blink_led <= 0;
		end
	end

	assign o_blink_led = r_blink_led;
*/