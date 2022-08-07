module cpu6502
    ( input  logic        clock,
      input  logic        reset,
      input  logic        io_enable,
      input  logic        io_irq,
      input  logic        io_nmi,
      input  logic        io_set_overflow,
      input  logic [7:0]  io_data_in,

      output logic [15:0] io_address,
      output logic [7:0]  io_data_out,
      output logic        io_write_enable,
      output logic        io_sync,

      output logic [7:0]  io_debug_opcode,
      output logic [15:0] io_debug_pc,
      output logic [7:0]  io_debug_a,
      output logic [7:0]  io_debug_x,
      output logic [7:0]  io_debug_y,
      output logic [7:0]  io_debug_s
      );

    // State type
    typedef enum {
        byte1,
        byte2,
        zeropage1,
        zeropage2,
        absolute1,
        absolute2,
        indirect1,
        indirect2,
        indexed,
        implied,
        modify1,
        modify2,
        branch1,
        branch2,
        branch3,
        stack1,
        stack2,
        stack3,
        stuck
    } state;

    // Registers
    var logic [15:0] reg_pc;
    var logic [7:0]  reg_a, reg_x, reg_y, reg_s;
    var logic        flag_n, flag_v, flag_d, flag_i, flag_z, flag_c;

    var logic [7:0]  reg_opcode;
    var state        reg_state;
    var logic [15:0] reg_addr;
    var logic [8:0]  reg_index; // result of address index calculation, with carry
    var logic [9:0]  reg_branch; // result of branch target calculation, with carry
    var logic [7:0]  reg_data_in; // registered input
    // TODO: registered input for irq, nmi, set_overflow

    uwire logic [7:0] data_in = reg_data_in;

    // Instruction decoding
    uwire logic opcode_alu          = reg_opcode ==? 8'b???_???_?1; // ALU (unless load/store/implied)
    uwire logic opcode_rmw          = reg_opcode ==? 8'b???_???_1?; // RMW (unless load/store/implied)
    uwire logic opcode_1_byte       = reg_opcode ==? 8'b???_?10_?0; // implied
    uwire logic opcode_relative     = reg_opcode ==? 8'b???_100_00;

    uwire logic opcode_indirect_any = reg_opcode ==? 8'b???_?00_?1; // ($00,X) or ($00),Y
    uwire logic opcode_zeropage_any = reg_opcode ==? 8'b???_?01_??; // $00 or $00,X
    uwire logic opcode_absolute_any = reg_opcode ==? 8'b???_?11_??; // $0000 or $0000,X

    uwire logic opcode_indirect_x   = reg_opcode ==? 8'b???_000_?1; // ($00,X)
    uwire logic opcode_zeropage     = reg_opcode ==? 8'b???_001_??; // $00
    uwire logic opcode_immediate    = reg_opcode ==? 8'b???_010_?1; // #$00
    uwire logic opcode_absolute     = reg_opcode ==? 8'b???_011_??; // $0000
    uwire logic opcode_indirect_y   = reg_opcode ==? 8'b???_100_?1; // ($00),Y
    uwire logic opcode_zeropage_x   = reg_opcode ==? 8'b???_101_??; // $00,X
    uwire logic opcode_absolute_y   = reg_opcode ==? 8'b???_110_?1; // $0000,Y
    uwire logic opcode_absolute_x   = reg_opcode ==? 8'b???_111_??; // $0000,X

    uwire logic opcode_acc          = reg_opcode ==? 8'b0??_010_10; // ASL/ROL/LSR/ROR accumulator
    uwire logic opcode_shift        = reg_opcode ==? 8'b0??_???_1?; // ASL/ROL/LSR/ROR (unless implied/immediate)
    uwire logic opcode_dec_inc      = reg_opcode ==? 8'b11?_???_1?; // INC/DEC/isb/dcp (unless implied/immediate)
    uwire logic opcode_load_store   = reg_opcode ==? 8'b10?_???_??; // ST* or LD* (unless BCC/BCS/CLV)
    uwire logic opcode_store        = reg_opcode ==? 8'b100_???_??; // ST*
    uwire logic opcode_load         = reg_opcode ==? 8'b101_???_??; // LD*
    uwire logic opcode_ldx          = reg_opcode ==? 8'b101_???_1?; // LDX/TAX/TSX/lax/lxa/lae
    uwire logic opcode_lda          = reg_opcode ==? 8'b101_???_?1; // LDA/lax
    uwire logic opcode_ldy          = reg_opcode ==? 8'b101_???_00; // LDY/TAY (unless BCS/CLV)
    uwire logic opcode_adc_sbc      = reg_opcode ==? 8'b?11_???_?1; // ADC/SBC/rra/isb
    uwire logic opcode_bit          = reg_opcode ==? 8'b001_0?1_00; // BIT $00 or BIT $0000
    uwire logic opcode_tax_tay      = reg_opcode ==? 8'b101_010_?0; // TAX/TAY

    uwire logic opcode_rti          = reg_opcode == 8'h40;
    uwire logic opcode_plp          = reg_opcode == 8'h28;
    uwire logic opcode_tax          = reg_opcode == 8'hAA;
    uwire logic opcode_tay          = reg_opcode == 8'hA8;
    uwire logic opcode_tsx          = reg_opcode == 8'hBA;
    uwire logic opcode_dex          = reg_opcode == 8'hCA;
    uwire logic opcode_inx          = reg_opcode == 8'hE8;
    uwire logic opcode_dey          = reg_opcode == 8'h88;
    uwire logic opcode_iny          = reg_opcode == 8'hC8;
    uwire logic opcode_txa          = reg_opcode == 8'h8A;
    uwire logic opcode_tya          = reg_opcode == 8'h98;
    uwire logic opcode_clv          = reg_opcode == 8'hB8;
    uwire logic opcode_cld_sed      = reg_opcode ==? 8'b11?_110_00; // CLD/SED
    uwire logic opcode_cli_sei      = reg_opcode ==? 8'b01?_110_00; // CLI/SEI
    uwire logic opcode_clc_sec      = reg_opcode ==? 8'b00?_110_00; // CLC/SEC

    uwire logic opcode_3_byte       = opcode_absolute_any | opcode_absolute_y;
    uwire logic opcode_modify       = opcode_rmw & !opcode_load_store;

    // Intermediate values
    var logic [7:0] offset; // for address calculations

    // Conditional branches (opcode xxx_100_00)
    // 000 = BPL, 001 = BMI
    // 010 = BVC, 011 = BVS
    // 100 = BCC, 101 = BCS
    // 110 = BNE, 111 = BEQ
    var logic branch_taken;
    var logic [9:0] next_branch;
    always_comb begin
        var logic flag;
        case (reg_opcode[7:6])
            2'b00: flag = flag_n;
            2'b01: flag = flag_v;
            2'b10: flag = flag_c;
            2'b11: flag = flag_z;
        endcase // case (reg_opcode[7:6])
        branch_taken = (flag == reg_opcode[5]);
        next_branch = {2'b0, reg_pc[7:0]} + {{2{data_in[7]}}, data_in};
    end

    // Indexed address calculations
    var logic [8:0] next_index;
    always_comb begin
        next_index = data_in + offset;
    end

    // Address generation
    //uwire logic [15:0] addr_pc    = new_pc;
    //uwire logic [15:0] addr_zp    = {8'h00, data_in};
    //uwire logic [15:0] addr_zpx   = {8'h00, reg_index[7:0]};
    //uwire logic [15:0] addr_index = {data_in, reg_index[7:0]};
    //uwire logic [15:0] addr_hold  = reg_addr;
    //uwire logic [15:0] addr_inc_page = {reg_addr[15:8] + 8'h1, reg_addr[7:0]};
    //uwire logic [15:0] addr_dec_page = {reg_addr[15:8] - 8'h1, reg_addr[7:0]};

    // ALU inputs are A, B, op, carry_in, decimal
    // ALU outputs are result, overflow, carry

    // ALU
    uwire logic [7:0] alu_out;
    uwire logic alu_c_out, alu_v_out, alu_n_out, alu_z_out;
    // operands come from accumulator and data bus
    cpu6502_alu alu
        ( .a_in(reg_a),
          .b_in(data_in),
          .c_in(flag_c),
          .d_in(flag_d),
          .op(reg_opcode[7:5]),
          .c_out(alu_c_out),
          .v_out(alu_v_out),
          .n_out(alu_n_out),
          .z_out(alu_z_out),
          .result(alu_out) );

    // Accumulator shift unit
    uwire logic [7:0] shift_out;
    uwire logic shift_n_out, shift_z_out, shift_c_out;
    cpu6502_shift shift
        ( .data_in(reg_a),
          .c_in(flag_c),
          .op(reg_opcode[7:5]),
          .c_out(shift_c_out),
          .n_out(shift_n_out),
          .z_out(shift_z_out),
          .data_out(shift_out) );

    // Index register increment/decrement
    // ca DEX, e8 INX, 88 DEY, c8 INY
    uwire logic [7:0] dec_x = reg_x - 1'b1;
    uwire logic [7:0] inc_x = reg_x + 1'b1;
    uwire logic [7:0] dec_y = reg_y - 1'b1;
    uwire logic [7:0] inc_y = reg_y + 1'b1;

    // RMW unit
    uwire logic [7:0] rmw_out;
    uwire logic rmw_c_out, rmw_z_out, rmw_n_out;
    cpu6502_shift rmw
        ( .data_in(data_in),
          .c_in(flag_c),
          .op(reg_opcode[7:5]),
          .c_out(rmw_c_out),
          .n_out(rmw_n_out),
          .z_out(rmw_z_out),
          .data_out(rmw_out) );

    // Value stored by ST* instruction
    var logic [7:0] store_out;
    always_comb begin
        case (reg_opcode[1:0])
            2'b00: store_out = reg_y;
            2'b01: store_out = reg_a;
            2'b10: store_out = reg_x;
            2'b11: store_out = reg_a & reg_x;
        endcase // case (reg_opcode[1:0])
    end

    // Stack pointer
    var logic [7:0] next_s;
    always_comb begin
        var logic [7:0] inc_s = reg_s + 1'b1;
        var logic [7:0] dec_s = reg_s - 1'b1;
        // default to previous value
        next_s = reg_s;
        // When is S updated?
        // TXS (implied)
        // PHA/PHP
        // PLA/PLP
        // BRK
        // JSR
        // RTI
        // RTS
        // TODO: implement all these!
    end

    // New values for Registers and Flags
    var logic [7:0] next_a, next_x, next_y;
    var logic next_n, next_v, next_d, next_i, next_z, next_c;
    always_comb begin
        // When is entire P register updated?
        var logic load_p
            = (opcode_plp & (reg_state == byte1)) |
              (opcode_rti & (reg_state == stack3));
        // default to previous values
        next_a = reg_a;
        next_x = reg_x;
        next_y = reg_y;
        next_n = flag_n;
        next_v = flag_v;
        next_d = flag_d;
        next_i = flag_i;
        next_z = flag_z;
        next_c = flag_c;
        if (reg_state == byte1) begin
            // Load instructions
            if (opcode_lda) next_a = data_in;
            if (opcode_ldx) next_x = data_in;
            if (opcode_ldy) next_y = data_in;
            if (opcode_load) begin
                next_n = data_in[7];
                next_z = data_in == 8'h00;
            end

            if (opcode_bit) begin
                next_n = data_in[7];
                next_v = data_in[6];
                next_z = (reg_a & data_in) == 8'h00;
            end
            if (opcode_alu & !opcode_load_store) begin
                next_a = alu_out;
            end
            if (opcode_adc_sbc) begin
                next_v = alu_v_out;
                next_c = alu_c_out;
            end
            // TODO: CPX/CPY
        end

        if (reg_state == modify2) begin
            next_n = rmw_n_out;
            next_z = rmw_z_out;
            if (opcode_shift) next_c = rmw_c_out;
        end

        if (reg_state == implied) begin
            //     00  20  40  60  80  a0  c0  e0
            // 08                  DEY TAY INY INX
            // 18  CLC SEC CLI SEI TYA CLV CLD SED
            // 0a  ASL ROL LSR ROR TXA TAX DEX NOP
            // 1a  nop nop nop nop TXS TSX nop nop
            // 08                  Z-- Z-- Z-- Z--
            // 18  --- --- --- --- Z-- --- --- ---
            // 0a  ZC- ZC- ZC- ZC- Z-- Z-- Z-- ---
            // 1a  --- --- --- --- --- Z-- --- ---

            if (opcode_clv)     next_v = 0;
            if (opcode_cld_sed) next_d = reg_opcode[5];
            if (opcode_cli_sei) next_i = reg_opcode[5];
            if (opcode_clc_sec) next_c = reg_opcode[5];
            if (opcode_txa) next_a = reg_x;
            if (opcode_tya) next_a = reg_y;
            if (opcode_tax) next_x = reg_a;
            if (opcode_tsx) next_x = reg_s;
            if (opcode_dex) next_x = dec_x;
            if (opcode_inx) next_x = inc_x;
            if (opcode_tay) next_y = reg_a;
            if (opcode_dey) next_y = dec_y;
            if (opcode_iny) next_y = inc_y;

            if (opcode_txa | opcode_tya) begin
                next_n = next_a[7];
                next_z = next_a == 8'h00;
            end
            if (opcode_tax | opcode_tsx | opcode_dex | opcode_inx) begin
                next_n = next_x[7];
                next_z = next_x == 8'h00;
            end
            if (opcode_tay | opcode_dey | opcode_iny) begin
                next_n = next_y[7];
                next_z = next_y == 8'h00;
            end
            if (opcode_acc) begin
                next_a = shift_out;
                next_n = shift_n_out;
                next_z = shift_z_out;
                next_c = shift_c_out;
            end
        end
        if (load_p) begin
            next_n = data_in[7];
            next_v = data_in[6];
            next_d = data_in[3];
            next_i = data_in[2];
            next_z = data_in[1];
            next_c = data_in[0];
        end
        // TODO: implement PLA
    end

    // State machine
    var logic [15:0] next_pc;
    var logic [15:0] next_addr;
    var logic        next_write_enable;
    var logic [7:0]  next_data_out;
    var state        next_state;

    // Possible values for next_pc
    uwire logic [15:0] pc_inc = reg_pc + 1'b1;
    uwire logic [15:0] pc_branch = {reg_pc[15:8], reg_branch[7:0]};
    uwire logic [7:0]  branch_carry = {{7{reg_branch[9]}}, reg_branch[8]};
    uwire logic [15:0] pc_fix_page = {reg_pc[15:8] + branch_carry, reg_pc[7:0]};

    // Possible values for next_addr
    uwire logic [15:0] addr_stack = {8'h01, reg_s};
    uwire logic [15:0] addr_zp1   = {8'h00, data_in};
    uwire logic [15:0] addr_zp2   = {8'h00, reg_index[7:0]};
    uwire logic [15:0] addr_abs   = {data_in, reg_index[7:0]};
    uwire logic [15:0] addr_fffe  = 16'hfffe;
    uwire logic [15:0] addr_hold  = reg_addr;
    uwire logic [15:0] addr_inc   = {reg_addr[15:8], reg_addr[7:0] + 8'h1};
    uwire logic [15:0] addr_inc_page = {reg_addr[15:8] + 8'h1, reg_addr[7:0]};
    uwire logic [15:0] addr_dec_page = {reg_addr[15:8] - 8'h1, reg_addr[7:0]};
    uwire logic [15:0] addr_carry = {reg_addr[15:8] + {7'h0, reg_index[8]}, reg_addr[7:0]};

    always_comb begin
        // Default to reading from memory
        next_write_enable = 0;
        next_data_out = 0;
        // Writes could happen from states zeropage1, zeropage2, absolute2, indirect2 or indexed.
        // Writes will happen from states modify1 and modify2.

        // Default to no address indexing calculation
        offset = 0;

        unique case (reg_state)

            byte2:
                begin
                    next_pc = pc_inc;
                    next_addr = next_pc;

                    casez (data_in)
                        8'b???_?01_??: next_state = zeropage1; // $00 or $00,X
                        8'b???_?11_??: next_state = absolute1; // $0000 or $0000,X
                        8'b???_?00_?1: next_state = zeropage1; // ($00,X) or ($00),Y
                        8'b???_110_?1: next_state = absolute1; // $0000,Y
                        8'b???_010_?1: next_state = byte1;     // immediate
                        8'b???_?10_?0: next_state = implied;   // implied or push/pull
                        8'b???_100_00: next_state = branch1;   // relative
                        8'b1??_000_?0: next_state = byte1;     // immediate
                        8'b0??_000_00: next_state = stack1;    // BRK/JSR/RTI/RTS
                        8'b???_100_10: next_state = stuck;
                        8'b0??_000_10: next_state = stuck;
                    endcase
                end

            zeropage1:
                // Read from zero page address
                // ($00,X) or ($00),Y or $00 or $00,X
                // xxx_x00_x1, xxx_x01_xx
                //     00  20  40  60  80  a0  c0  e0
                // 01  ORA AND EOR ADC STA LDA CMP SBC  ($00,X)
                // 03  slo rla sre rra sax lax dcp isb  ($00,X)
                // 04  nop BIT nop nop STY LDY CPY CPX  $00
                // 05  ORA AND EOR ADC STA LDA CMP SBC  $00
                // 06  ASL ROL LSR ROR STX LDX DEC INC  $00
                // 07  slo rla sre rra sax lax dcp isb  $00
                // 11  ORA AND EOR ADC STA LDA CMP SBC  ($00),Y
                // 13  slo rla sre rra sha lax dcp isb  ($00),Y
                // 14  nop nop nop nop STY LDY nop nop  $00,X
                // 15  ORA AND EOR ADC STA LDA CMP SBC  $00,X
                // 16  ASL ROL LSR ROR STX LDX DEC INC  $00,X
                // 17  slo rla sre rra sax lax dcp isb  $00,X
                begin
                    next_pc = reg_pc;
                    next_addr = addr_zp1;
                    next_state
                        = opcode_zeropage_x ? zeropage2 : // xxx_101_xx (14,15,16,17)
                          opcode_indirect_x ? zeropage2 : // xxx_000_x1 (01,03)
                          opcode_indirect_y ? indirect1 : // xxx_100_x1 (11,13)
                          byte1;                          // xxx_001_xx (04,05,06,07)
                    if (opcode_indirect_x) begin
                        // 01,03
                        offset = reg_x;
                    end
                    if (opcode_zeropage_x) begin
                        // 14,15,16,17
                        // 10x_101_1x have swapped (Y-indexed) addressing
                        offset = (opcode_load_store & reg_opcode[1]) ? reg_y : reg_x;
                    end
                end

            zeropage2:
                // Read from zero page address + X
                // xxx_000_x1, xxx_101_xx (01,03,14,15,16,17)
                //     00  20  40  60  80  a0  c0  e0
                // 01  ORA AND EOR ADC STA LDA CMP SBC  ($00,X)
                // 03  slo rla sre rra sax lax dcp isb  ($00,X)
                // 14  nop nop nop nop STY LDY nop nop  $00,X
                // 15  ORA AND EOR ADC STA LDA CMP SBC  $00,X
                // 16  ASL ROL LSR ROR STX LDX DEC INC  $00,X
                // 17  slo rla sre rra sax lax dcp isb  $00,X
                begin
                    next_pc = reg_pc;
                    next_addr = addr_zp2;
                    next_state
                        = opcode_indirect_x ? indirect1 : // xxx_000_x1 (01,03)
                          byte1;                          // xxx_101_xx (14,15,16,17)
                end

            absolute1:
                // Read the low byte of a 16-bit address
                // xxx_x11_xx, xxx_110_x1 (_c,_d,_e,_f,19,1b)
                //     00  20  40  60  80  a0  c0  e0
                // 0c  nop BIT JMP JMP'STY LDY CPY CPX  $0000
                // 0d  ORA AND EOR ADC STA LDA CMP SBC  $0000
                // 0e  ASL ROL LSR ROR STX LDX DEC INC  $0000
                // 0f  slo rla sre rra sax lax dcp isb  $0000
                // 19  ORA AND EOR ADC STA LDA CMP SBC  $0000,Y  1001
                // 1b  slo rla sre rra shs lae dcp isb  $0000,Y  1011
                // 1c  nop nop nop nop shy LDY nop nop  $0000,X  1100
                // 1d  ORA AND EOR ADC STA LDA CMP SBC  $0000,X  1101
                // 1e  ASL ROL LSR ROR shx LDX DEC INC  $0000,X  1110 (Y for shx/LDX)
                // 1f  slo rla sre rra sha lax dcp isb  $0000,X  1111 (Y for sha/lax)
                begin
                    next_pc = pc_inc;
                    next_addr = next_pc;
                    next_state = absolute2;
                    if (reg_opcode[4]) begin
                        // indexing 19,1b,1c,1d,1e,1f
                        if (reg_opcode[3]) begin
                            // 1c,1d,1e,1f
                            // 9e,9f,be,bf have swapped (Y-indexed) addressing
                            offset = (opcode_load_store & reg_opcode[1]) ? reg_y : reg_x;
                        end else begin
                            // 19,1b
                            offset = reg_y;
                        end
                    end
                end

            absolute2:
                // Read the high byte of a 16-bit address
                // $0000 or $0000,X or $0000,Y
                // xxx_x11_xx, xxx_110_x1 (_c,_d,_e,_f,19,1b)
                //     00  20  40  60  80  a0  c0  e0
                // 0c  nop BIT JMP JMP'STY LDY CPY CPX  $0000
                // 0d  ORA AND EOR ADC STA LDA CMP SBC  $0000
                // 0e  ASL ROL LSR ROR STX LDX DEC INC  $0000
                // 0f  slo rla sre rra sax lax dcp isb  $0000
                // 19  ORA AND EOR ADC STA LDA CMP SBC  $0000,Y
                // 1b  slo rla sre rra shs lae dcp isb  $0000,Y
                // 1c  nop nop nop nop shy LDY nop nop  $0000,X
                // 1d  ORA AND EOR ADC STA LDA CMP SBC  $0000,X
                // 1e  ASL ROL LSR ROR shx LDX DEC INC  $0000,X
                // 1f  slo rla sre rra sha lax dcp isb  $0000,X
                // state "indexed" only if address calculation carried
                // or if the instruction was indexed write or modify
                begin
                    next_pc = reg_pc;
                    next_addr = addr_abs;
                    if (reg_opcode[4]) begin
                        // indexed
                        // Read instructions go to state 'indexed' iff address carried
                        var state maybe_indexed = reg_index[8] ? indexed : byte1;
                        next_state
                            = opcode_store ? indexed :
                              opcode_load  ? maybe_indexed :
                              opcode_rmw   ? indexed :
                              maybe_indexed;
                    end else begin
                        // not indexed
                        next_state = opcode_modify ? modify1 : byte1;
                    end
                end

            indirect1:
                // Read first byte of vector
                // ($00,X) or ($00),Y
                // xxx_x00_x1 (_1,_3)
                //     00  20  40  60  80  a0  c0  e0
                // 01  ORA AND EOR ADC STA LDA CMP SBC  ($00,X)
                // 03  slo rla sre rra sax lax dcp isb  ($00,X)
                // 11  ORA AND EOR ADC STA LDA CMP SBC  ($00),Y
                // 13  slo rla sre rra sha lax dcp isb  ($00),Y
                begin
                    next_pc = reg_pc;
                    next_addr = addr_inc;
                    next_state = indirect2;
                    if (reg_opcode[4]) begin
                        // ($00),Y (11,13)
                        offset = reg_y;
                    end
                end

            indirect2:
                // Read second byte of vector
                // ($00,X) or ($00),Y
                // xxx_x00_x1 (_1,_3)
                //     00  20  40  60  80  a0  c0  e0
                // 01  ORA AND EOR ADC STA LDA CMP SBC  ($00,X)
                // 03  slo rla sre rra sax lax dcp isb  ($00,X)
                // 11  ORA AND EOR ADC STA LDA CMP SBC  ($00),Y
                // 13  slo rla sre rra sha lax dcp isb  ($00),Y
                begin
                    next_pc = reg_pc;
                    next_addr = addr_abs;
                    // state "indexed" only if address calculation carried
                    // or if the instruction was indexed write or modify
                    if (reg_opcode[4]) begin
                        // ($00),Y
                        // Read instructions go to state 'indexed' iff address carried
                        var state maybe_indexed = reg_index[8] ? indexed : byte1;
                        next_state
                            = opcode_store   ? indexed :
                              opcode_load    ? maybe_indexed :
                              opcode_rmw     ? modify1 :
                              maybe_indexed;
                    end else begin
                        // ($00,X)
                        next_state
                            = opcode_load_store ? byte1 :
                              opcode_rmw        ? modify1 :
                              byte1;
                    end
                end

            indexed:
                // Propagate carry to high address byte and read again
                // ($00),Y or $0000,Y or $0000,X
                //     00  20  40  60  80  a0  c0  e0
                // 11  ORA AND EOR ADC STA LDA CMP SBC  ($00),Y
                // 13  slo rla sre rra sha lax dcp isb  ($00),Y
                // 19  ORA AND EOR ADC STA LDA CMP SBC  $0000,Y
                // 1b  slo rla sre rra shs lae dcp isb  $0000,Y
                // 1c  nop nop nop nop shy LDY nop nop  $0000,X
                // 1d  ORA AND EOR ADC STA LDA CMP SBC  $0000,X
                // 1e  ASL ROL LSR ROR shx LDX DEC INC  $0000,X
                // 1f  slo rla sre rra sha lax dcp isb  $0000,X
                begin
                    next_pc = reg_pc;
                    next_addr = addr_carry;
                    next_state
                        = opcode_load_store ? byte1 :
                          opcode_rmw        ? modify1 :
                          byte1;
                end

            byte1:
                begin
                    next_state = byte2;
                end

            modify1:
                begin
                    next_state = modify2;
                    next_write_enable = 1;
                    next_data_out = data_in;
                end

            modify2:
                begin
                    next_state = byte1;
                    next_write_enable = 1;
                    next_data_out = rmw_out;
                end

            branch1:
                // Read branch offset
                // xxx_100_00 (10)
                // 10  BPL BMI BVC BVS BCC BCS BNE BEQ
                // 12   -   -   -   -   -   -   -   -
                begin
                    next_pc = pc_inc;
                    if (branch_taken) begin
                        next_state = branch2;
                    end else begin
                        next_state = byte2;
                    end
                    next_addr = next_pc;
                end

            branch2:
                // Ignore read from byte following branch instruction
                begin
                    next_pc = pc_branch;
                    next_addr = next_pc;
                    if (reg_branch[9] | reg_branch[8]) begin
                        next_state = branch3;
                    end else begin
                        next_state = byte2;
                    end
                end

            branch3:
                begin
                    next_pc = pc_fix_page;
                    next_addr = next_pc;
                    next_state = byte2;
                end

            stack1:
                // BRK JSR RTI RTS
                // maybe we should have PHP PLP PHA PLA in this state too
                begin
                    next_state = stack2;
                end

            stack2:
                begin
                end

            stack3:
                begin
                end

            implied:
                // xxx_x10_x0: implied (_8,_a)
                //     00  20  40  60  80  a0  c0  e0
                // 08  PHP PLP PHA PLA DEY TAY INY INX
                // 0a  ASL ROL LSR ROR TXA TAX DEX NOP
                // 18  CLC SEC CLI SEI TYA CLV CLD SED
                // 1a  nop nop nop nop TXS TSX nop nop
                // all except PHP PLP PHA PLA are done right away
                begin
                    next_state = byte2;
                end // case: implied

            stuck:
                begin
                    next_pc = reg_pc;
                    next_addr = next_pc;
                    next_state = stuck;
                end

        endcase // case (reg_state)

        if (opcode_store & (next_state == byte1)) begin
            // FIXME: suppress write if the instruction is immediate mode
            next_write_enable = 1;
            next_data_out = store_out;
        end
    end

    // Register updates
    always_ff @(posedge clock) begin
        if (reset) begin
            reg_pc <= 16'hfffc;
            reg_opcode <= 8'h6c; // JMP ($fffc)
            reg_state <= absolute1;
        end

        else if (io_enable) begin
            if (reg_state == byte2) begin
                reg_opcode <= data_in;
            end
            reg_pc     <= next_pc;
            reg_a      <= next_a;
            reg_x      <= next_x;
            reg_y      <= next_y;
            reg_s      <= next_s;
            flag_n     <= next_n;
            flag_v     <= next_v;
            flag_d     <= next_d;
            flag_i     <= next_i;
            flag_z     <= next_z;
            flag_c     <= next_c;

            reg_state  <= next_state;
            reg_addr   <= next_addr;
            reg_index  <= next_index;
            reg_branch <= next_branch;

            // after we've done a write, data_in should reflect the
            // previous data_out.
            if (next_write_enable) begin
                reg_data_in <= next_data_out;
            end else begin
                reg_data_in <= io_data_in;
            end
        end
    end

    // Module outputs
    assign io_address      = next_addr;
    assign io_data_out     = next_data_out;
    assign io_write_enable = next_write_enable;
    assign io_sync         = (reg_state == byte1);

    assign io_debug_opcode = reg_opcode;
    assign io_debug_pc     = reg_pc;
    assign io_debug_a      = reg_a;
    assign io_debug_x      = reg_x;
    assign io_debug_y      = reg_y;
    assign io_debug_s      = reg_s;

endmodule: cpu6502

module cpu6502_alu
    ( input logic [7:0] a_in,
      input logic [7:0] b_in,
      input logic c_in,
      input logic d_in,
      input logic [2:0] op, // ORA,AND,EOR,ADC,STA,LDA,CMP,SBC

      output logic c_out,
      output logic v_out,
      output logic n_out,
      output logic z_out,
      output logic [7:0] result
      );

    // CMP and SBC do subtraction
    uwire logic sub = op[2];

    // complement 2nd operand if subtracting
    uwire logic [7:0] addend = sub ? ~b_in : b_in;

    // binary mode adder
    uwire logic [8:0] bin_add = a_in + addend + 8'(c_in);
    uwire logic [7:0] add_result = bin_add[7:0]; // TODO: decimal mode

    always_comb begin
        case (op)
            3'h0: result = a_in | b_in; // ORA
            3'h1: result = a_in & b_in; // AND
            3'h2: result = a_in ^ b_in; // EOR
            3'h3: result = add_result;  // ADC
            3'h4: result = 8'h0;        // STA
            3'h5: result = b_in;        // LDA
            3'h6: result = a_in;        // CMP
            3'h7: result = add_result;  // SBC
        endcase // case (op)
    end

    assign n_out = result[7];
    assign v_out = (a_in[7] ^ result[7]) & (addend[7] ^ result[7]);
    assign z_out = result == 8'h00;
    assign c_out = bin_add[8];

endmodule: cpu6502_alu

module cpu6502_shift
    ( input logic [7:0] data_in,
      input logic c_in,
      input logic [2:0] op, // 0=ASL, 1=ROL, 2=LSR, 3=ROR, 6=DEC, 7=INC

      output logic c_out,
      output logic n_out,
      output logic z_out,
      output logic [7:0] data_out
      );

    uwire logic rotate = op[0];
    uwire logic right = op[1];
    uwire logic inc_dec = op[2];
    uwire logic carry = c_in & rotate;

    always_comb begin
        if (inc_dec) begin
            data_out = op[0] ? (data_in + 1'b1) : (data_in - 1'b1);
            c_out = 1'b0;
        end else begin
            if (right)
                {data_out, c_out} = {carry, data_in};
            else
                {c_out, data_out} = {data_in, carry};
        end
    end

    assign n_out = data_out[7];
    assign z_out = data_out == 8'h0;

endmodule: cpu6502_shift
