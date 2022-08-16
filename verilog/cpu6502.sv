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
    typedef struct packed {
        logic byte1;       // Read 1st instruction byte (opcode)
        logic byte2;       // Read 2nd instruction byte (low arg)
        logic byte3;       // Read 3rd instruction byte (high arg)
        logic zeropage1;   // Read from zero-page address
        logic zeropage2;   // Read from zero-page address plus offset
        logic absolute;    // Read from 16-bit address (plus offset, if any)
        logic indirect;    // Read from 2nd half of indirect vector
        logic fixpage;     // Read from 16-bit address with fixed-up page
        logic modify1;     // First write cycle of RMW instruction
        logic modify2;     // Second write cycle of RMW instruction
        logic branch1;     // Update PCL to PCL+offset for taken branch
        logic branch2;     // Update PCH for taken branch to adjacent page
        logic stack1;      // 3rd cycle of BRK/JSR/RTI/RTS/PHP/PLP/PHA/PLA
        logic stack2;      // 4th cycle of BRK/JSR/RTI/RTS/PLP/PLA
        logic stack3;      // 5th cycle of BRK/JSR/RTI/RTS
        logic stack4;      // 6th cycle of BRK/JSR/RTI/RTS
        logic stack5;      // 7th cycle of BRK
    } state;

    // Registers
    var logic [15:0] reg_pc;
    var logic [7:0]  reg_a, reg_x, reg_y, reg_s;
    var logic        flag_n, flag_v, flag_d, flag_i, flag_z, flag_c;

    var logic [7:0]  reg_opcode;
    var state        reg_state;
    var logic [15:0] reg_addr;
    var logic [8:0]  reg_index; // result of address index calculation, with carry
    var logic [7:0]  reg_fixpage; // high byte of address index calculation
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
    uwire logic opcode_immediate_a  = reg_opcode ==? 8'b???_010_?1; // #$00
    uwire logic opcode_absolute     = reg_opcode ==? 8'b???_011_??; // $0000
    uwire logic opcode_indirect_y   = reg_opcode ==? 8'b???_100_?1; // ($00),Y
    uwire logic opcode_zeropage_x   = reg_opcode ==? 8'b???_101_??; // $00,X
    uwire logic opcode_absolute_y   = reg_opcode ==? 8'b???_110_?1; // $0000,Y
    uwire logic opcode_absolute_x   = reg_opcode ==? 8'b???_111_??; // $0000,X

    uwire logic opcode_stack        = reg_opcode ==? 8'b0??_0?0_00; // BRK/JSR/RTI/RTS/PHP/PLP/PHA/PLA
    uwire logic opcode_immediate_xy = reg_opcode ==? 8'b1??_000_?0; // LDX/LDY/CPX/CPY #$00
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

    uwire logic opcode_php_pha      = reg_opcode ==? 8'b0?0_010_00; // PHP/PHA
    uwire logic opcode_plp_pla      = reg_opcode ==? 8'b0?1_010_00; // PLP/PLA
    uwire logic opcode_brk_jsr      = reg_opcode ==? 8'b00?_000_00; // BRK/JSR
    uwire logic opcode_rti_rts      = reg_opcode ==? 8'b01?_000_00; // RTI/RTS
    uwire logic opcode_brk          = reg_opcode == 8'h00;
    uwire logic opcode_jsr          = reg_opcode == 8'h20;
    uwire logic opcode_rti          = reg_opcode == 8'h40;
    uwire logic opcode_rts          = reg_opcode == 8'h60;
    uwire logic opcode_plp          = reg_opcode == 8'h28;
    uwire logic opcode_pha          = reg_opcode == 8'h48;
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
    uwire logic opcode_jmp_abs      = reg_opcode == 8'h4C;
    uwire logic opcode_cld_sed      = reg_opcode ==? 8'b11?_110_00; // CLD/SED
    uwire logic opcode_cli_sei      = reg_opcode ==? 8'b01?_110_00; // CLI/SEI
    uwire logic opcode_clc_sec      = reg_opcode ==? 8'b00?_110_00; // CLC/SEC

    uwire logic opcode_3_byte       = opcode_absolute_any | opcode_absolute_y;
    uwire logic opcode_modify       = opcode_rmw & !opcode_load_store;
    uwire logic opcode_imm_any      = opcode_immediate_a | opcode_immediate_xy;
    uwire logic opcode_2_cycle      = opcode_imm_any | (opcode_1_byte & ~opcode_stack);

    // Conditional branches (opcode xxx_100_00)
    // 000 = BPL, 001 = BMI
    // 010 = BVC, 011 = BVS
    // 100 = BCC, 101 = BCS
    // 110 = BNE, 111 = BEQ
    var logic branch_taken;
    var logic [9:0] branch_result;
    always_comb begin
        var logic flag;
        case (reg_opcode[7:6])
            2'b00: flag = flag_n;
            2'b01: flag = flag_v;
            2'b10: flag = flag_c;
            2'b11: flag = flag_z;
        endcase // case (reg_opcode[7:6])
        branch_taken = (flag == reg_opcode[5]);
        branch_result = {2'b0, reg_pc[7:0]} + {{2{data_in[7]}}, data_in};
    end

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

    // New values for Registers and Flags
    var logic [7:0] next_a, next_x, next_y;
    always_comb begin
        // When is entire P register updated?
        var logic load_p
            = (opcode_plp & reg_state.byte1) |
              (opcode_rti & reg_state.stack3);
        // default to previous values
        next_a = reg_a;
        next_x = reg_x;
        next_y = reg_y;
        if (reg_state.byte1) begin
            // Load instructions
            if (opcode_lda) next_a = data_in;
            if (opcode_ldx) next_x = data_in;
            if (opcode_ldy) next_y = data_in;

            if (opcode_alu & !opcode_load_store) begin
                next_a = alu_out;
            end
            // TODO: CPX/CPY
        end

        if (reg_state.byte1) begin
            //     00  20  40  60  80  a0  c0  e0
            // 08                  DEY TAY INY INX
            // 18  CLC SEC CLI SEI TYA CLV CLD SED
            // 0a  ASL ROL LSR ROR TXA TAX DEX NOP
            // 1a  nop nop nop nop TXS TSX nop nop
            // 08                  Z-- Z-- Z-- Z--
            // 18  --- --- --- --- Z-- --- --- ---
            // 0a  ZC- ZC- ZC- ZC- Z-- Z-- Z-- ---
            // 1a  --- --- --- --- --- Z-- --- ---

            if (opcode_txa) next_a = reg_x;
            if (opcode_tya) next_a = reg_y;
            if (opcode_tax) next_x = reg_a;
            if (opcode_tsx) next_x = reg_s;
            if (opcode_dex) next_x = dec_x;
            if (opcode_inx) next_x = inc_x;
            if (opcode_tay) next_y = reg_a;
            if (opcode_dey) next_y = dec_y;
            if (opcode_iny) next_y = inc_y;

            if (opcode_acc) begin
                next_a = shift_out;
            end
        end
        if (load_p) begin
        end
        // TODO: implement PLA
    end

    // Control signals
    var struct packed {
        logic pc_increment;  // increment pc by 1
        logic pc_branch1;    // set pcl to result of branch index calculation
        logic pc_branch2;    // set pch to fixup branch page carry
        logic pc_vector;     // set pc from last two bytes read
        logic addr_stack;    // ADH = 01, ADL = S
        logic addr_zp1;      // ADH = 00, ADL = data_in
        logic addr_zp2;      // ADH = 00, ADL = indexed
        logic addr_abs;      // ADH = data_in, ADL = indexed
        logic addr_fffe;     // address = $fffe
        logic addr_hold;     // keep address unchanged
        logic addr_inc;      // ADH unchanged, increment ADL
        logic addr_carry;    // ADH += indexing carry, ADL unchanged
        logic write_enable;  // 0 = read, 1 = write
        logic write_same;    // copy data_out from data_in
        logic write_rmw;     // write data from rmw unit
        logic write_store;   // write data from store instruction
        logic write_a;       // write data from accumulator
        logic index_xy;      // index with x or y register
        logic index_y;       // index with y register
        logic stack_inc;     // increment stack pointer
        logic stack_dec;     // decrement stack pointer
        logic db_data_in;    // set DB from data_in
        logic db_rmw;        // set DB from RMW unit
        logic db_alu;        // set DB from ALU unit
        logic db_shift;      // set DB from accumulator RMW unit
        logic db_next_a;     // set DB from A register
        logic db_next_x;     // set DB from X register
        logic db_next_y;     // set DB from Y register
        logic n_db7;         // set N flag from bit 7 of DB
        logic v_di6;         // set V flag from bit 6 of data_in
        logic v_ir5;         // set V flag from bit 5 of Instruction Register
        logic v_alu;         // set V flag from ALU unit
        logic d_di3;         // set D flag from bit 3 of data_in
        logic d_ir5;         // set D flag from bit 5 of Instruction Register
        logic i_di2;         // set I flag from bit 2 of data_in
        logic i_ir5;         // set I flag from bit 5 of Instruction Register
        logic z_dbz;         // set Z flag from DB == 0
        logic c_di0;         // set C flag from bit 0 of data_in
        logic c_ir5;         // set C flag from bit 5 of Instruction Register
        logic c_alu;         // set C flag from ALU carry out
        logic c_rmw;         // set C flag from RMW carry out
        logic c_shift;       // set C flag from accumulator RMW carry out
    } control;

    // State machine
    var state next_state;

    always_comb begin
        // Default all control signals to 0
        control = '{ default: 0 };
        next_state = '{ default: 0 };

        if (reg_state.byte1)
        begin
            // Requesting opcode byte
            next_state.byte2 = 1;
            control.pc_increment = 1;

            // Set flags and registers based on previous instruction
            if (opcode_load) begin
                control.db_data_in = 1;
            end
            if (opcode_bit) begin
                control.db_alu = 1;
                control.n_db7 = 1; //FIXME next_n = data_in[7], not alu output;
                control.v_di6 = 1;
                control.z_dbz = 1;
            end
            if (opcode_adc_sbc) begin
                control.v_alu = 1;
                control.c_alu = 1;
            end
            if (opcode_clv)     control.v_ir5 = 1;
            if (opcode_cld_sed) control.d_ir5 = 1;
            if (opcode_cli_sei) control.i_ir5 = 1;
            if (opcode_clc_sec) control.c_ir5 = 1;
            if (opcode_txa | opcode_tya) begin
                control.db_next_a = 1;
            end
            if (opcode_tax | opcode_tsx | opcode_dex | opcode_inx) begin
                control.db_next_x = 1;
            end
            if (opcode_tay | opcode_dey | opcode_iny) begin
                control.db_next_y = 1;
            end
            if (opcode_acc) begin
                control.db_shift = 1;
                control.c_shift = 1;
            end

            if (opcode_txa | opcode_tax | opcode_inx | opcode_dex |
                opcode_tya | opcode_tay | opcode_iny | opcode_dey |
                opcode_load | opcode_acc | opcode_tsx
                )
            begin
                control.n_db7 = 1;
                control.z_dbz = 1;
            end
        end

        if (reg_state.byte2)
        begin
            // Requesting byte after opcode byte
            // Opcode available on data_in and reg_opcode
            control.pc_increment = ~opcode_1_byte;

            next_state.zeropage1 = opcode_zeropage_any | opcode_indirect_any;
            next_state.byte3 = opcode_3_byte;
            next_state.byte1 = opcode_2_cycle | (opcode_relative & ~branch_taken);
            next_state.branch1 = opcode_relative & branch_taken;
            next_state.stack1 = opcode_stack;
        end

        if (reg_state.zeropage1)
        begin
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

            control.addr_zp1 = 1;

            if (opcode_indirect_x)
            begin
                // 8'b???_000_?1
                // 01,03
                control.index_xy = 1;
                next_state.zeropage2 = 1;
            end
            if (opcode_zeropage)
            begin
                // 8'b???_001_??
                // 04,05,06,07
                next_state.modify1 = opcode_rmw;
                next_state.byte1 = ~opcode_rmw;
            end
            if (opcode_zeropage_x)
            begin
                // 8'b???_101_??
                // 14,15,16,17
                // 10x_101_1x have swapped (Y-indexed) addressing
                control.index_xy = 1;
                control.index_y = opcode_load_store & reg_opcode[1];
                next_state.zeropage2 = 1;
            end
            if (opcode_indirect_y)
            begin
                // 8'b???_100_?1
                // 11, 13
                next_state.indirect = 1;
            end
        end

        if (reg_state.zeropage2)
        begin
            // Read from zero page address + X
            // xxx_000_x1, xxx_101_xx (01,03,14,15,16,17)
            //     00  20  40  60  80  a0  c0  e0
            // 01  ORA AND EOR ADC STA LDA CMP SBC  ($00,X)
            // 03  slo rla sre rra sax lax dcp isb  ($00,X)
            // 14  nop nop nop nop STY LDY nop nop  $00,X
            // 15  ORA AND EOR ADC STA LDA CMP SBC  $00,X
            // 16  ASL ROL LSR ROR STX LDX DEC INC  $00,X
            // 17  slo rla sre rra sax lax dcp isb  $00,X
            control.addr_zp2 = 1;
            next_state.indirect = opcode_indirect_x; // xxx_000_x1 (01,03)
            next_state.byte1 = ~opcode_indirect_x;    // xxx_101_xx (14,15,16,17)
        end

        if (reg_state.byte3)
        begin
            // Request the final byte of a 3-byte instruction
            // data_in contains the low byte of a 16-bit address
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
            control.pc_increment = 1;
            if (opcode_jmp_abs) begin
                next_state.byte1 = 1;
                control.pc_vector = 1;
            end else begin
                next_state.absolute = 1;
            end
            if (reg_opcode[4]) begin
                // indexing 19,1b,1c,1d,1e,1f
                if (reg_opcode[2]) begin
                    // 1c,1d,1e,1f
                    // 9e,9f,be,bf have swapped (Y-indexed) addressing
                    control.index_xy = 1;
                    control.index_y = opcode_load_store & reg_opcode[1];
                end else begin
                    // 19,1b
                    control.index_xy = 1;
                    control.index_y = 1;
                end
            end
        end

        if (reg_state.absolute)
        begin
            // Read from 16-bit address (plus offset, if any)
            // reg_index contains LSB, data_in contains MSB
            // $0000 or $0000,X or $0000,Y or ($00,X) or ($00),Y
            // xxx_x00_x1, xxx_x11_xx, xxx_110_x1 (_1,_3,_c,_d,_e,_f,19,1b)
            //     00  20  40  60  80  a0  c0  e0
            // 01  ORA AND EOR ADC STA LDA CMP SBC  ($00,X)
            // 03  slo rla sre rra sax lax dcp isb  ($00,X) (rmw)
            // 0c  nop BIT JMP JMP'STY LDY CPY CPX  $0000
            // 0d  ORA AND EOR ADC STA LDA CMP SBC  $0000
            // 0e  ASL ROL LSR ROR STX LDX DEC INC  $0000   (rmw)
            // 0f  slo rla sre rra sax lax dcp isb  $0000   (rmw)
            // 11  ORA AND EOR ADC STA LDA CMP SBC  ($00),Y
            // 13  slo rla sre rra sha lax dcp isb  ($00),Y (rmw)
            // 19  ORA AND EOR ADC STA LDA CMP SBC  $0000,Y
            // 1b  slo rla sre rra shs lae dcp isb  $0000,Y (rmw)
            // 1c  nop nop nop nop shy LDY nop nop  $0000,X
            // 1d  ORA AND EOR ADC STA LDA CMP SBC  $0000,X
            // 1e  ASL ROL LSR ROR shx LDX DEC INC  $0000,X (rmw)
            // 1f  slo rla sre rra sha lax dcp isb  $0000,X (rmw)
            // state "fixpage" only if address calculation carried
            // or if the instruction was indexed write or modify
            control.addr_abs = 1;

            if (reg_opcode[4]) begin
                // indexed
                // Read instructions go to state 'fixpage' iff address carried
                if (opcode_store | opcode_modify | reg_index[8])
                    next_state.fixpage = 1;
                else
                    next_state.byte1 = 1;
            end
            else begin
                // not indexed
                next_state.modify1 = opcode_modify;
                next_state.byte1 = ~opcode_modify;
            end
        end

        if (reg_state.indirect)
        begin
            // Read first byte of vector
            // ($00,X) or ($00),Y
            // xxx_x00_x1 (_1,_3)
            //     00  20  40  60  80  a0  c0  e0
            // 01  ORA AND EOR ADC STA LDA CMP SBC  ($00,X)
            // 03  slo rla sre rra sax lax dcp isb  ($00,X)
            // 11  ORA AND EOR ADC STA LDA CMP SBC  ($00),Y
            // 13  slo rla sre rra sha lax dcp isb  ($00),Y
            control.addr_inc = 1;
            next_state.absolute = 1;
            if (reg_opcode[4]) begin
                // ($00),Y (11,13)
                control.index_xy = 1;
                control.index_y = 1;
            end
        end

        if (reg_state.fixpage)
        begin
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
            control.addr_carry = 1;
            if (opcode_modify) next_state.modify1 = 1;
            else next_state.byte1 = 1;
        end

        if (reg_state.modify1)
        begin
            next_state.modify2 = 1;
            control.addr_hold = 1;
            control.write_enable = 1;
            control.write_same = 1;
        end

        if (reg_state.modify2)
        begin
            next_state.byte1 = 1;
            control.addr_hold = 1;
            control.write_enable = 1;
            control.write_rmw = 1;

            control.db_rmw = 1;
            control.n_db7 = 1;
            control.z_dbz = 1;
            control.c_rmw = opcode_shift;
        end

        if (reg_state.branch1)
        begin
            // Adjust low byte of PC for branch
            // xxx_100_00 (10)
            // 10  BPL BMI BVC BVS BCC BCS BNE BEQ
            // 12   -   -   -   -   -   -   -   -
            control.pc_branch1 = 1;
            if (branch_result[9:8] == 2'b00)
                next_state.byte1 = 1;
            else
                next_state.branch2 = 1;
        end

        if (reg_state.branch2)
        begin
            // Fixup high byte of PC for branch
            control.pc_branch2 = 1;
            next_state.byte1 = 1;
        end

        if (reg_state.stack1)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            // 08 PHP  28 PLP  48 PHA  68 PLA
            next_state.byte1 = opcode_php_pha;
            next_state.stack2 = ~opcode_php_pha;
            control.addr_stack = 1;
            control.stack_inc = opcode_rti_rts | opcode_plp_pla;
            control.stack_dec = opcode_brk | opcode_php_pha;
            control.write_enable = opcode_brk | opcode_php_pha;
            // All stack reads in this cycle are ignored
            // TODO: specify what value to write
            // brk writes pch
            // php writes p
            control.write_a = opcode_pha;
        end

        if (reg_state.stack2)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            //         28 PLP          68 PLA
            next_state.byte1 = opcode_plp_pla;
            next_state.stack3 = ~opcode_plp_pla;
            control.addr_stack = 1;
            control.stack_inc = opcode_rti_rts;
            control.stack_dec = opcode_brk_jsr;
            // TODO: specify what values are read
            // rti/plp read p
            // rts reads pcl
            // pla reads a
            // TODO: specify what values are written
            // brk writes pcl
            // jsr writes pch
        end

        if (reg_state.stack3)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            next_state.stack4 = 1;
            control.addr_stack = 1;
            control.stack_inc = opcode_rti;
            control.stack_dec = opcode_brk_jsr;
            control.pc_vector = opcode_rts;
            control.write_enable = opcode_brk_jsr;
            // TODO: specify what values are written
            // brk writes p
            // jsr writes pcl
        end

        if (reg_state.stack4)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            next_state.byte1 = opcode_rti_rts | opcode_jsr;
            next_state.stack5 = opcode_brk;
            control.addr_stack = opcode_rti_rts | opcode_jsr;
            control.addr_fffe = opcode_brk;
            control.pc_vector = opcode_rti;
        end

        if (reg_state.stack5)
        begin
            // 00 BRK
            next_state.byte1 = 1;
            control.addr_inc = 1;
            control.pc_vector = 1;
        end

        if (opcode_store
           & next_state.byte1
           & !(reg_opcode ==? 8'b???_??0_?0)
           & !(reg_opcode ==? 8'b???_010_?1))
        begin
            control.write_enable = 1;
            control.write_store = 1;
        end
    end

    // Internal Data Bus
    uwire logic [7:0] db =
        control.db_data_in ? data_in :
        control.db_rmw     ? rmw_out :
        control.db_alu     ? alu_out :
        control.db_shift   ? shift_out :
        control.db_next_a  ? next_a :
        control.db_next_x  ? next_x :
        control.db_next_y  ? next_y :
        0'h00;

    // N Flag
    uwire logic next_n =
        control.n_db7 ? db[7] :
        flag_n;

    // V Flag
    uwire logic next_v =
        control.v_di6 ? data_in[6] :
        control.v_ir5 ? reg_opcode[5] :
        control.v_alu ? alu_v_out :
        flag_v;

    // D Flag
    uwire logic next_d =
        control.d_di3 ? data_in[3] :
        control.d_ir5 ? reg_opcode[5] :
        flag_d;

    // I Flag
    uwire logic next_i =
        control.i_di2 ? data_in[2] :
        control.i_ir5 ? reg_opcode[5] :
        flag_i;

    // Z Flag
    uwire logic next_z =
        control.z_dbz ? (db == 8'h00) :
        flag_z;

    // C Flag
    uwire logic next_c =
        control.c_di0 ? data_in[0] :
        control.c_ir5 ? reg_opcode[5] :
        control.c_alu ? alu_c_out :
        control.c_rmw ? rmw_c_out :
        control.c_shift ? shift_c_out :
        flag_c;

    // Program counter
    uwire logic [15:0] pc_inc = reg_pc + 16'(control.pc_increment);
    uwire logic [7:0]  branch_carry = {{7{reg_branch[9]}}, reg_branch[8]};
    uwire logic [15:0] next_pc =
        control.pc_branch1 ? {reg_pc[15:8], branch_result[7:0]} :
        control.pc_branch2 ? {reg_pc[15:8] + branch_carry, reg_pc[7:0]} :
        control.pc_vector  ? {io_data_in, reg_data_in} :
        pc_inc;

    // Stack register
    uwire logic [7:0] next_s =
        control.stack_inc ? reg_s + 1'b1 :
        control.stack_dec ? reg_s - 1'b1 :
        reg_s;

    // Indexing calculations
    uwire logic [7:0] index = control.index_y ? reg_y : reg_x;
    uwire logic [7:0] offset = control.index_xy ? index : 8'h00;
    // 9-bit value includes carry bit
    uwire logic [8:0] next_index = data_in + offset;
    uwire logic [7:0] next_fixpage = data_in + 8'(reg_index[8]);

    // Bus address
    uwire logic [15:0] address_out =
        control.addr_stack ? {8'h01, reg_s} :
        control.addr_zp1   ? {8'h00, data_in} :
        control.addr_zp2   ? {8'h00, reg_index[7:0]} :
        control.addr_abs   ? {data_in, reg_index[7:0]} :
        control.addr_fffe  ? 16'hfffe :
        control.addr_hold  ? reg_addr :
        control.addr_inc   ? {reg_addr[15:8], reg_addr[7:0] + 8'h1} :
        control.addr_carry ? {reg_fixpage, reg_addr[7:0]} :
        reg_pc;

    // Data out
    uwire logic [7:0] data_out =
        control.write_same  ? data_in :
        control.write_rmw   ? rmw_out :
        control.write_store ? store_out :
        control.write_a     ? reg_a :
        8'h00;

    // Register updates
    always_ff @(posedge clock) begin
        if (reset) begin
            reg_pc <= 16'hfffc;
            reg_opcode <= 8'h6c; // JMP ($fffc)
            reg_state <= '{ byte3: 1, default: 0 };
        end

        else if (io_enable) begin
            if (reg_state.byte1) begin
                reg_opcode <= io_data_in;
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
            reg_addr   <= address_out;
            reg_index  <= next_index;
            reg_fixpage <= next_fixpage;
            reg_branch <= branch_result;

            // after we've done a write, data_in should reflect the
            // previous data_out.
            if (control.write_enable) begin
                reg_data_in <= data_out;
            end else begin
                reg_data_in <= io_data_in;
            end
        end
    end

    // Module outputs
    assign io_address      = address_out;
    assign io_data_out     = data_out;
    assign io_write_enable = control.write_enable;
    assign io_sync         = reg_state.byte1;

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
