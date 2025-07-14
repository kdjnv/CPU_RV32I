module Peripheral_unit(
    input           clk,
    input           rst,

//bus
    input   [31:0 ] add_w,
    input   [31:0 ] data_w,
    input           wen,
    input   [ 3:0 ] wmask,
    output          wready,

    input   [31:0 ] add_r,
    output  [31:0 ] data_r,
    output          ren,
    
    output          rvalid,

//Peripheral
    inout   [15:0 ] GPIO
);

    regs_GPIO gpio_unit(
        .clk(clk),
        .rst(rst),

        // GPIO IO
        .csr_gpio_io_gpio_0_in (leds[0]),
        .csr_gpio_io_gpio_0_out(gpio_out[0]),
        .csr_gpio_io_gpio_1_in (leds[1]),
        .csr_gpio_io_gpio_1_out(gpio_out[1]),
        .csr_gpio_io_gpio_2_in (leds[2]),
        .csr_gpio_io_gpio_2_out(gpio_out[2]),
        .csr_gpio_io_gpio_3_in (leds[3]),
        .csr_gpio_io_gpio_3_out(gpio_out[3]),
        .csr_gpio_io_gpio_4_in (leds[4]),
        .csr_gpio_io_gpio_4_out(gpio_out[4]),
        .csr_gpio_io_gpio_5_in (leds[5]),
        .csr_gpio_io_gpio_5_out(gpio_out[5]),
        .csr_gpio_io_gpio_6_in (leds[6]),
        .csr_gpio_io_gpio_6_out(gpio_out[6]),
        .csr_gpio_io_gpio_7_in (leds[7]),
        .csr_gpio_io_gpio_7_out(gpio_out[7]),
        .csr_gpio_io_gpio_8_in (leds[8]),
        .csr_gpio_io_gpio_8_out(gpio_out[8]),
        .csr_gpio_io_gpio_9_in (leds[9]),
        .csr_gpio_io_gpio_9_out(gpio_out[9]),
        .csr_gpio_io_gpio_10_in (leds[10]),
        .csr_gpio_io_gpio_10_out(gpio_out[10]),
        .csr_gpio_io_gpio_11_in (leds[11]),
        .csr_gpio_io_gpio_11_out(gpio_out[11]),
        .csr_gpio_io_gpio_12_in (leds[12]),
        .csr_gpio_io_gpio_12_out(gpio_out[12]),
        .csr_gpio_io_gpio_13_in (leds[13]),
        .csr_gpio_io_gpio_13_out(gpio_out[13]),
        .csr_gpio_io_gpio_14_in (leds[14]),
        .csr_gpio_io_gpio_14_out(gpio_out[14]),
        .csr_gpio_io_gpio_15_in (leds[15]),
        .csr_gpio_io_gpio_15_out(gpio_out[15]),

        // GPIO Config (Direction)
        .csr_gpio_config_gpio_0_config_out (gpio_dir[0]),
        .csr_gpio_config_gpio_1_config_out (gpio_dir[1]),
        .csr_gpio_config_gpio_2_config_out (gpio_dir[2]),
        .csr_gpio_config_gpio_3_config_out (gpio_dir[3]),
        .csr_gpio_config_gpio_4_config_out (gpio_dir[4]),
        .csr_gpio_config_gpio_5_config_out (gpio_dir[5]),
        .csr_gpio_config_gpio_6_config_out (gpio_dir[6]),
        .csr_gpio_config_gpio_7_config_out (gpio_dir[7]),
        .csr_gpio_config_gpio_8_config_out (gpio_dir[8]),
        .csr_gpio_config_gpio_9_config_out (gpio_dir[9]),
        .csr_gpio_config_gpio_10_config_out(gpio_dir[10]),
        .csr_gpio_config_gpio_11_config_out(gpio_dir[11]),
        .csr_gpio_config_gpio_12_config_out(gpio_dir[12]),
        .csr_gpio_config_gpio_13_config_out(gpio_dir[13]),
        .csr_gpio_config_gpio_14_config_out(gpio_dir[14]),
        .csr_gpio_config_gpio_15_config_out(gpio_dir[15]),

        // Local Bus
        .waddr ({4'h0, mem_addr[27:0]}),
        .wdata (mem_wdata),
        .wen   (s1_sel_gpio & (|mem_wstrb)),
        .wstrb (mem_wstrb),
        .wready(),

        .raddr ({4'h0, mem_addr[27:0]}),
        .ren   (s1_sel_gpio & mem_rstrb),
        .rdata (rdata_gpio),
        .rvalid()
    );
endmodule