# CPU_RV32I
Dự án viết RTL và mô phỏng một CPU đơn giản sử dụng kiến trúc tập lệnh RV32I. Ngôn ngữ sử dụng là Verilog.

## Mục tiêu

- Mô phỏng pipeline hoặc single-cycle CPU cơ bản theo ISA RV32I.
- Có thể thực thi một tập lệnh cơ bản (add, sub, lw, sw, beq, jal, ...).
- Hướng đến kiểm thử bằng testbench và thực thi chương trình RISC-V dạng `.hex`.

##cấu trúc 
  - CPU_RV32I: chưa mã RTL
  - Simulation: chứa thông tin mô phỏng (các mã lệnh để thử điều được gen ra từ RISC_V GNU toolchain)
     + Image: chứa hình ảnh mô phỏng đượng dạng wave và text
     + Testbench and wave: chưa testbench và file waveform


