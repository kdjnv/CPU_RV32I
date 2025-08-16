# CPU_RV32I
Dự án viết RTL và mô phỏng một CPU đơn giản và nâng cao sử dụng kiến trúc tập lệnh RV32I. Ngôn ngữ sử dụng là Verilog.

## Mục tiêu

- Mô phỏng pipeline hoặc single-cycle CPU cơ bản theo ISA RV32I.
- Có thể thực thi một tập lệnh cơ bản (add, sub, lw, sw, beq, jal, ...).
- Hướng đến kiểm thử bằng testbench và thực thi chương trình RISC-V dạng `.hex`.

## cấu trúc thư mục
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

## Đặc điểm các chế độ hoạt động
  - Chế độ cơ bản – 5 state single cycle
      + CPU được tổ chức thành 5 trạng thái tuần tự: IF → ID → EX → MEM → WB. Mỗi trạng thái tiêu tốn một chu kỳ clock, do đó một lệnh cần 4–5 chu kỳ để hoàn thành. Đây là cách thực thi đơn giản, dễ thiết kế, ít rủi ro sai sót. Nhược điểm: tốc độ thực thi chậm, trung bình chỉ đạt 0.2–0.25 CPI (chu kỳ/lệnh).
  - Chế độ nhanh – 3 state single cycle
      + CPU được rút gọn thành 3 trạng thái chính: IF → EX → WB. Các bước decode, đọc thanh ghi, và memory access được gộp hợp lý để giảm số chu kỳ. Mỗi lệnh chỉ cần 3 chu kỳ clock để hoàn thành. Tốc độ cải thiện đáng kể so với 5 state, nhưng độ phức tạp trong thiết kế tăng lên. Thích hợp để tối ưu giữa tài nguyên và hiệu năng.
  - Chế độ nâng cao – Pipeline (tiếp cận 1 chu kỳ/lệnh)
      + CPU được chia thành 5 stage pipeline: IF → ID → EX → MEM → WB. Các lệnh được chồng lấn trong pipeline, sau khi pipeline đầy thì mỗi chu kỳ hoàn thành 1 lệnh (≈ 1 CPI).
      + Khi CPU gặp lệnh rẽ nhánh (BEQ, BNE, BLT, JAL, JALR, …), thay vì chờ tới giai đoạn EX mới biết rẽ hay không, CPU dùng bộ dự đoán rẽ nhánh (branch predictor) để tạm thời dự đoán hướng nhảy. Sử dụng bộ dự đoán rẽ nhánh 2-bit saturating counter, kết hợp với Branch History Table (BHT). Mỗi khi phát hiện dự đoán rẽ nhánh sai-Branch misprediction: thì pipeline phải hủy và khởi động lại → mất 2 chu kỳ.
       <img width="1200" height="247" alt="image" src="https://github.com/user-attachments/assets/5c517223-cbaf-44bb-89bf-bdc001dd9491" />
      + Hazard đặc biệt với JALR: Nếu JALR cần dữ liệu từ thanh ghi (rs1), thì dữ liệu này phải có ngay tại giai đoạn IF để tính toán PC next. Vì bình thường nếu không pipeline tính toán PC sẽ ở giai đoạn EX, nhưng ở đây tính PCnext quá sơm nên JALR không thể tránh stall. Áp dụng forward data cho trường hợp này cũng chỉ làm giảm số lần stall cần thiệt. Nếu JALR phụ thuộc giá trị từ lệnh trước (hazard 1-cycle) → stall 2 chu kỳ. Nếu JALR phụ thuộc giá trị từ lệnh cách 1 lệnh ở giữa (hazard 2-cycle) → stall 1 chu kỳ. Còn lại hazard 3-cycle và hazard 4-cycle cho JALR đều sẽ được forward mà không cần stall.
      + Tất cả các hazard khác đều có thể xử lý bằng forwarding, không gây stall.
      
  

  



