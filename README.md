# CPU_RV32I
Dự án viết RTL và mô phỏng một CPU đơn giản sử dụng kiến trúc tập lệnh RV32I. Ngôn ngữ sử dụng là Verilog.

## Mục tiêu

- Mô phỏng pipeline hoặc single-cycle CPU cơ bản theo ISA RV32I.
- Có thể thực thi một tập lệnh cơ bản (add, sub, lw, sw, beq, jal, ...).
- Hướng đến kiểm thử bằng testbench và thực thi chương trình RISC-V dạng `.hex`.

##cấu trúc thư mục
  - RTL CPU_RV32I: chưa mã RTL
  - Simulation: chứa thông tin mô phỏng (các mã lệnh để thử điều được gen ra từ RISC_V GNU toolchain) -> Mô phỏng bao gồm mô phỏng Decoder, ALU_init, Instruction và Data memory
      + CPU_final: chưa hình ảnh, wave và testbench cho mô phỏng cấp CPU.
        * 5 cycles per instruction: chế độ hoạt động cơ bản (không dùng kĩ thuật đường ống) - 5 chu kì 1 lệnh.
        * 3 cycles per instruction: chế độ hoạt động nhanh (không dùng kĩ thuật đường ống) - 3 chu kì 1 lệnh.
        * 1 cycles per instruction: chế độ hoạt động nâng cao (sử dụng kĩ thuật đường ống) - 1 chu kì 1 lệnh.
      + Decoder: Chứa hình ảnh, wave và testbench cho mô phỏng bộ giải mã.
      + ALU: Chứa hình ảnh, wave và testbench cho mô phỏng bộ số học và logic.
      + Instruction memory: Chứa hình ảnh, wave và testbench cho mô phỏng bộ nhớ lệnh (Read only memory).
      + Data memory: chứa hình ảnh, wave và testbench cho mô phỏng bộ nhớ dữ liệu.
  - Firmwave RV32I: Chứa mã chương trình (C), phân vùng bộ nhớ(lds), các thanh ghi ngoại vi, start.s(asm), MakeFile, ... cùng các file build ra được như file mã chương trình (hex), map, asm...
  - Bao cao tien do: Chứa pdf mô tả công việc đã làm mỗ tuần, mỗi buổi.
  - References: Chứa tài liệu tham khảo.
      
  

  



