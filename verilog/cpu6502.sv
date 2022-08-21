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
    var logic [9:0]  reg_branch; // result of branch target calculation, with carry
    var logic [7:0]  reg_data_in; // registered input
    // TODO: registered input for irq, nmi, set_overflow

    wire logic [7:0] data_in = reg_data_in;

    // Instruction decoding
    wire logic opcode_alu          = reg_opcode ==? 8'b???_???_?1; // ALU (unless load/store/implied)
    wire logic opcode_rmw          = reg_opcode ==? 8'b???_???_1?; // RMW (unless load/store/implied)
    wire logic opcode_1_byte       = reg_opcode ==? 8'b???_?10_?0; // implied
    wire logic opcode_relative     = reg_opcode ==? 8'b???_100_00;

    wire logic opcode_indirect_any = reg_opcode ==? 8'b???_?00_?1; // ($00,X) or ($00),Y
    wire logic opcode_zeropage_any = reg_opcode ==? 8'b???_?01_??; // $00 or $00,X
    wire logic opcode_absolute_any = reg_opcode ==? 8'b???_?11_??; // $0000 or $0000,X

    wire logic opcode_indirect_x   = reg_opcode ==? 8'b???_000_?1; // ($00,X)
    wire logic opcode_zeropage     = reg_opcode ==? 8'b???_001_??; // $00
    wire logic opcode_immediate_a  = reg_opcode ==? 8'b???_010_?1; // #$00
    wire logic opcode_absolute     = reg_opcode ==? 8'b???_011_??; // $0000
    wire logic opcode_indirect_y   = reg_opcode ==? 8'b???_100_?1; // ($00),Y
    wire logic opcode_zeropage_x   = reg_opcode ==? 8'b???_101_??; // $00,X
    wire logic opcode_absolute_y   = reg_opcode ==? 8'b???_110_?1; // $0000,Y
    wire logic opcode_absolute_x   = reg_opcode ==? 8'b???_111_??; // $0000,X

    wire logic opcode_stack        = reg_opcode ==? 8'b0??_0?0_00; // BRK/JSR/RTI/RTS/PHP/PLP/PHA/PLA
    wire logic opcode_immediate_xy = reg_opcode ==? 8'b1??_000_?0; // LDX/LDY/CPX/CPY #$00
    wire logic opcode_acc          = reg_opcode ==? 8'b0??_010_10; // ASL/ROL/LSR/ROR accumulator
    wire logic opcode_shift        = reg_opcode ==? 8'b0??_???_1?; // ASL/ROL/LSR/ROR (unless implied/immediate)
    wire logic opcode_dec_inc      = reg_opcode ==? 8'b11?_???_1?; // INC/DEC/isb/dcp (unless implied/immediate)
    wire logic opcode_load_store   = reg_opcode ==? 8'b10?_???_??; // ST* or LD* (unless BCC/BCS/CLV)
    wire logic opcode_store        = reg_opcode ==? 8'b100_???_??; // ST*
    wire logic opcode_load         = reg_opcode ==? 8'b101_???_??; // LD*
    wire logic opcode_ldx          = reg_opcode ==? 8'b101_???_1?; // LDX/TAX/TSX/lax/lxa/lae
    wire logic opcode_lda          = reg_opcode ==? 8'b101_???_?1; // LDA/lax
    wire logic opcode_ldy          = reg_opcode ==? 8'b101_??1_00; // LDY ($00/$0000/$00,X/$0000,X)
    wire logic opcode_ldy_tay      = reg_opcode ==? 8'b101_0?0_00; // LDY #$00 / TAY
    wire logic opcode_lda_sbc      = reg_opcode ==? 8'b1?1_???_?1; // LDA/SBC/lax/isb
    wire logic opcode_adc_sbc      = reg_opcode ==? 8'b?11_???_?1; // ADC/SBC/rra/isb
    wire logic opcode_bit          = reg_opcode ==? 8'b001_0?1_00; // BIT $00 or BIT $0000
    wire logic opcode_tax_tay      = reg_opcode ==? 8'b101_010_?0; // TAX/TAY
    wire logic opcode_txa          = reg_opcode ==? 8'b100_010_1?; // TXA/ane

    wire logic opcode_php_pha      = reg_opcode ==? 8'b0?0_010_00; // PHP/PHA
    wire logic opcode_plp_pla      = reg_opcode ==? 8'b0?1_010_00; // PLP/PLA
    wire logic opcode_brk_jsr      = reg_opcode ==? 8'b00?_000_00; // BRK/JSR
    wire logic opcode_rti_rts      = reg_opcode ==? 8'b01?_000_00; // RTI/RTS
    wire logic opcode_brk          = reg_opcode == 8'h00;
    wire logic opcode_jsr          = reg_opcode == 8'h20;
    wire logic opcode_rti          = reg_opcode == 8'h40;
    wire logic opcode_rts          = reg_opcode == 8'h60;
    wire logic opcode_php          = reg_opcode == 8'h08;
    wire logic opcode_plp          = reg_opcode == 8'h28;
    wire logic opcode_pha          = reg_opcode == 8'h48;
    wire logic opcode_pla          = reg_opcode == 8'h68;
    wire logic opcode_tax          = reg_opcode == 8'hAA;
    wire logic opcode_tay          = reg_opcode == 8'hA8;
    wire logic opcode_tsx          = reg_opcode == 8'hBA;
    wire logic opcode_dex          = reg_opcode == 8'hCA;
    wire logic opcode_inx          = reg_opcode == 8'hE8;
    wire logic opcode_dey          = reg_opcode == 8'h88;
    wire logic opcode_iny          = reg_opcode == 8'hC8;
    wire logic opcode_tya          = reg_opcode == 8'h98;
    wire logic opcode_clv          = reg_opcode == 8'hB8;
    wire logic opcode_jmp_abs      = reg_opcode == 8'h4C;
    wire logic opcode_cld_sed      = reg_opcode ==? 8'b11?_110_00; // CLD/SED
    wire logic opcode_cli_sei      = reg_opcode ==? 8'b01?_110_00; // CLI/SEI
    wire logic opcode_clc_sec      = reg_opcode ==? 8'b00?_110_00; // CLC/SEC
    wire logic opcode_alu4         = reg_opcode ==? 8'b0??_???_?1; // ORA/AND/EOR/ADC
    wire logic opcode_cmp          = reg_opcode ==? 8'b110_???_?1; // CMP
    wire logic opcode_cpxy_inxy    = reg_opcode ==? 8'b11?_0??_00; // CPX/CPY/INX/INY

    wire logic opcode_3_byte       = opcode_absolute_any | opcode_absolute_y;
    wire logic opcode_arith        = opcode_alu & ~opcode_load_store; // ORA/AND/EOR/ADC/CMP/SBC
    wire logic opcode_modify       = opcode_rmw & ~opcode_load_store; // ASL/ROL/LSR/ROR/DEC/INC
    wire logic opcode_imm_any      = opcode_immediate_a | opcode_immediate_xy;
    wire logic opcode_2_cycle      = opcode_imm_any | (opcode_1_byte & ~opcode_stack);

    wire logic opcode_update_a =
        opcode_alu4 | opcode_lda_sbc | opcode_acc | opcode_txa | opcode_tya | opcode_pla;

    wire logic opcode_update_x =
        opcode_ldx | opcode_inx | opcode_dex;

    wire logic opcode_update_y =
        opcode_ldy | opcode_ldy_tay | opcode_dey | opcode_iny;

    wire logic opcode_update_nz =
        opcode_update_a | opcode_update_x | opcode_update_y | opcode_cmp | opcode_cpxy_inxy;

    // Conditional branches (opcode xxx_100_00)
    // 000 = BPL, 001 = BMI
    // 010 = BVC, 011 = BVS
    // 100 = BCC, 101 = BCS
    // 110 = BNE, 111 = BEQ
    wire logic [3:0] branch_flags = {flag_z, flag_c, flag_v, flag_n};
    wire logic branch_flag = branch_flags[reg_opcode[7:6]];
    wire logic branch_taken = (branch_flag == reg_opcode[5]);
    wire logic [9:0] branch_result = {2'b0, reg_pc[7:0]} + {{2{data_in[7]}}, data_in};

    // ALU inputs are A, B, op, carry_in, decimal
    // ALU outputs are result, overflow, carry

    // ALU
    wire logic [7:0] alu_out;
    wire logic alu_c_out, alu_v_out;
    // operands come from accumulator and data bus
    cpu6502_alu alu
        ( .a_in(reg_a),
          .b_in(data_in),
          .c_in(flag_c),
          .d_in(flag_d),
          .op(reg_opcode[7:5]),
          .c_out(alu_c_out),
          .v_out(alu_v_out),
          .result(alu_out) );

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

    // Control signals
    var struct packed {
        struct packed {
            logic increment; // increment pc by 1
            logic branch1;   // set pcl to result of branch index calculation
            logic branch2;   // set pch to fixup branch page carry
            logic vector;    // set pc from last two bytes read
        } pc;
        struct packed {
            logic pc;        // ADH = PCH, ADL = PCL
            logic stack;     // ADH = 01, ADL = S
            logic zp1;       // ADH = 00, ADL = data_in
            logic zp2;       // ADH = 00, ADL = indexed
            logic abs;       // ADH = data_in, ADL = indexed
            logic fffe;      // address = $fffe
            logic hold;      // keep address unchanged
            logic inc;       // ADH unchanged, increment ADL
            logic carry;     // ADH += indexing carry, ADL unchanged
        } addr;
        struct packed {
            logic enable;    // 0 = read, 1 = write
            logic same;      // copy data_out from data_in
            logic rmw;       // write data from rmw unit
            logic store;     // write data from store instruction
            logic a;         // write data from accumulator
            logic p;         // write data from status register
            logic pch;       // write high byte of PC
            logic pcl;       // write low byte of PC
        } write;
        struct packed {
            logic xy;        // index with x or y register
            logic y;         // index with y register
            logic carry;     // add carry bit from previous cycle
        } index;
        struct packed {
            logic inc;       // increment stack pointer
            logic dec;       // decrement stack pointer
        } stack;
        struct packed {
            logic data;      // send RMW input from data_in
        } rmw;
        struct packed {
            logic x;         // send counter input from X register
            logic y;         // send counter input from Y register
            logic inc;       // increment counter
            logic dec;       // decrement counter
        } count;
        struct packed {
            logic data_in;   // set DB from data_in
            logic rmw;       // set DB from RMW unit
            logic alu;       // set DB from ALU unit
            logic a;         // set DB from A register
            logic s;         // set DB from S register
            logic count;     // set DB from counter unit
        } db;
        struct packed {
            logic di7;       // set N flag from bit 7 of data_in
            logic db7;       // set N flag from bit 7 of DB
        } n;
        struct packed {
            logic di6;       // set V flag from bit 6 of data_in
            logic ir5;       // set V flag from bit 5 of Instruction Register
            logic alu;       // set V flag from ALU unit
        } v;
        struct packed {
            logic di3;       // set D flag from bit 3 of data_in
            logic ir5;       // set D flag from bit 5 of Instruction Register
        } d;
        struct packed {
            logic di2;       // set I flag from bit 2 of data_in
            logic ir5;       // set I flag from bit 5 of Instruction Register
        } i;
        struct packed {
            logic di1;       // set Z flag from bit 1 of data_in
            logic dbz;       // set Z flag from DB == 0
        } z;
        struct packed {
            logic di0;       // set C flag from bit 0 of data_in
            logic ir5;       // set C flag from bit 5 of Instruction Register
            logic alu;       // set C flag from ALU carry out
            logic rmw;       // set C flag from RMW carry out
        } c;
        logic a;             // set A register from DB
        logic x;             // set X register from DB
        logic y;             // set Y register from DB
    } control;

    // State machine
    var state next_state;

    always_comb begin
        // Default all control signals to 0
        control = '0;
        next_state = '0;

        if (reg_state.byte1)
        begin
            // Requesting opcode byte
            next_state.byte2 = 1;
            control.addr.pc = 1;
            control.pc.increment = 1;

            // Set flags and registers based on previous instruction
            control.count.x = opcode_txa | opcode_inx | opcode_dex;
            control.count.y = opcode_tya | opcode_iny | opcode_dey;
            control.count.inc = opcode_inx | opcode_iny;
            control.count.dec = opcode_dex | opcode_dey;
            control.db.data_in = (opcode_load & ~opcode_1_byte) | opcode_pla;
            control.db.a = opcode_tax_tay;
            control.db.s = opcode_tsx;
            control.db.alu = opcode_arith | opcode_bit;
            control.db.rmw = opcode_acc;
            control.db.count = opcode_txa | opcode_inx | opcode_dex | opcode_tya | opcode_iny | opcode_dey;

            control.n.di7 = opcode_plp | opcode_bit;
            control.v.di6 = opcode_plp | opcode_bit;
            control.d.di3 = opcode_plp;
            control.i.di2 = opcode_plp;
            control.z.di1 = opcode_plp;
            control.c.di0 = opcode_plp;

            control.v.ir5 = opcode_clv;
            control.d.ir5 = opcode_cld_sed;
            control.i.ir5 = opcode_cli_sei;
            control.c.ir5 = opcode_clc_sec;

            control.v.alu = opcode_adc_sbc;
            control.c.alu = opcode_adc_sbc;
            control.c.rmw = opcode_acc;

            control.n.db7 = opcode_update_nz;
            control.z.dbz = opcode_update_nz | opcode_bit;

            control.a = opcode_update_a;
            control.x = opcode_update_x;
            control.y = opcode_update_y;
        end

        if (reg_state.byte2)
        begin
            // Requesting byte after opcode byte
            // Opcode available on data_in and reg_opcode
            control.addr.pc = 1;
            control.pc.increment = ~opcode_1_byte;

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

            control.addr.zp1 = 1;

            if (opcode_indirect_x)
            begin
                // 8'b???_000_?1
                // 01,03
                control.index.xy = 1;
                next_state.zeropage2 = 1;
            end
            if (opcode_zeropage)
            begin
                // 8'b???_001_??
                // 04,05,06,07
                next_state.modify1 = opcode_modify;
                next_state.byte1 = ~opcode_modify;
            end
            if (opcode_zeropage_x)
            begin
                // 8'b???_101_??
                // 14,15,16,17
                // 10x_101_1x have swapped (Y-indexed) addressing
                control.index.xy = 1;
                control.index.y = opcode_load_store & reg_opcode[1];
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
            control.addr.zp2 = 1;
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
            control.addr.pc = 1;
            control.pc.increment = 1;
            if (opcode_jmp_abs) begin
                next_state.byte1 = 1;
                control.pc.vector = 1;
            end else begin
                next_state.absolute = 1;
            end
            if (reg_opcode[4]) begin
                // indexing 19,1b,1c,1d,1e,1f
                if (reg_opcode[2]) begin
                    // 1c,1d,1e,1f
                    // 9e,9f,be,bf have swapped (Y-indexed) addressing
                    control.index.xy = 1;
                    control.index.y = opcode_load_store & reg_opcode[1];
                end else begin
                    // 19,1b
                    control.index.xy = 1;
                    control.index.y = 1;
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
            control.addr.abs = 1;
            control.index.carry = 1;

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
            control.addr.inc = 1;
            next_state.absolute = 1;
            if (reg_opcode[4]) begin
                // ($00),Y (11,13)
                control.index.xy = 1;
                control.index.y = 1;
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
            control.addr.carry = 1;
            if (opcode_modify) next_state.modify1 = 1;
            else next_state.byte1 = 1;
        end

        if (reg_state.modify1)
        begin
            next_state.modify2 = 1;
            control.addr.hold = 1;
            control.rmw.data = 1;
            control.write.enable = 1;
            control.write.same = 1;

            control.db.rmw = 1;
            control.n.db7 = 1;
            control.z.dbz = 1;
            control.c.rmw = opcode_shift;
        end

        if (reg_state.modify2)
        begin
            next_state.byte1 = 1;
            control.addr.hold = 1;
            control.rmw.data = 1; // FIXME: We should write RMW output computed in the previous cycle
            control.write.enable = 1;
            control.write.rmw = 1;
        end

        if (reg_state.branch1)
        begin
            // Adjust low byte of PC for branch
            // xxx_100_00 (10)
            // 10  BPL BMI BVC BVS BCC BCS BNE BEQ
            // 12   -   -   -   -   -   -   -   -
            control.addr.pc = 1;
            control.pc.branch1 = 1;
            if (branch_result[9:8] == 2'b00)
                next_state.byte1 = 1;
            else
                next_state.branch2 = 1;
        end

        if (reg_state.branch2)
        begin
            // Fixup high byte of PC for branch
            control.addr.pc = 1;
            control.pc.branch2 = 1;
            next_state.byte1 = 1;
        end

        if (reg_state.stack1)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            // 08 PHP  28 PLP  48 PHA  68 PLA
            next_state.byte1 = opcode_php_pha;
            next_state.stack2 = ~opcode_php_pha;
            control.addr.stack = 1;
            control.stack.inc = opcode_rti_rts | opcode_plp_pla;
            control.stack.dec = opcode_brk | opcode_php_pha;
            control.write.enable = opcode_brk | opcode_php_pha;
            // All stack reads in this cycle are ignored
            control.write.pch = opcode_brk;
            control.write.p = opcode_php;
            control.write.a = opcode_pha;
        end

        if (reg_state.stack2)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            //         28 PLP          68 PLA
            next_state.byte1 = opcode_plp_pla;
            next_state.stack3 = ~opcode_plp_pla;
            control.addr.stack = 1;
            control.stack.inc = opcode_rti_rts;
            control.stack.dec = opcode_brk_jsr;
            // TODO: specify what values are read
            // rti/plp read p
            // rts reads pcl
            // pla reads a
            control.write.enable = opcode_brk_jsr;
            control.write.pcl = opcode_brk;
            control.write.pch = opcode_jsr;
        end

        if (reg_state.stack3)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            next_state.stack4 = 1;
            control.addr.stack = 1;
            control.stack.inc = opcode_rti;
            control.stack.dec = opcode_brk_jsr;
            control.pc.vector = opcode_rts;
            control.write.enable = opcode_brk_jsr;
            control.write.p = opcode_brk;
            control.write.pcl = opcode_jsr;
            control.n.di7 = opcode_rti;
            control.v.di6 = opcode_rti;
            control.d.di3 = opcode_rti;
            control.i.di2 = opcode_rti;
            control.z.di1 = opcode_rti;
            control.c.di0 = opcode_rti;
        end

        if (reg_state.stack4)
        begin
            // 00 BRK  20 JSR  40 RTI  60 RTS
            next_state.byte1 = opcode_rti_rts | opcode_jsr;
            next_state.stack5 = opcode_brk;
            control.addr.stack = opcode_rti_rts | opcode_jsr;
            control.addr.fffe = opcode_brk;
            control.pc.vector = opcode_rti;
        end

        if (reg_state.stack5)
        begin
            // 00 BRK
            next_state.byte1 = 1;
            control.addr.inc = 1;
            control.pc.vector = 1;
        end

        if (opcode_store
           & next_state.byte1
           & !(reg_opcode ==? 8'b???_??0_?0)
           & !(reg_opcode ==? 8'b???_010_?1))
        begin
            control.write.enable = 1;
            control.write.store = 1;
        end
    end

    // RMW unit
    wire logic [7:0] rmw_data_in = control.rmw.data ? data_in : reg_a;
    wire logic [7:0] rmw_out;
    wire logic rmw_c_out;
    cpu6502_shift rmw
        ( .data_in(rmw_data_in),
          .c_in(flag_c),
          .op(reg_opcode[7:5]),
          .c_out(rmw_c_out),
          .data_out(rmw_out) );

    // Counter unit (increment/decrement)
    wire logic [7:0] count_in =
        control.count.x ? reg_x : control.count.y ? reg_y : data_in;
    wire logic [7:0] count_out =
        count_in + {8{control.count.dec}} + 8'(control.count.inc);

    // Internal Data Bus
    wire logic [7:0] db =
        (control.db.data_in ? data_in   : '0) |
        (control.db.rmw     ? rmw_out   : '0) |
        (control.db.alu     ? alu_out   : '0) |
        (control.db.a       ? reg_a     : '0) |
        (control.db.count   ? count_out : '0);

    // P register
    wire logic flag_b = 1'b1; // TODO: set this from a control bit.
    wire logic [7:0] status_out =
        {flag_n, flag_v, 1'b1, flag_b, flag_d, flag_i, flag_z, flag_c};

    // N Flag
    wire logic next_n =
        control.n.di7 ? data_in[7] :
        control.n.db7 ? db[7] :
        flag_n;

    // V Flag
    wire logic next_v =
        control.v.di6 ? data_in[6] :
        control.v.ir5 ? reg_opcode[5] :
        control.v.alu ? alu_v_out :
        flag_v;

    // D Flag
    wire logic next_d =
        control.d.di3 ? data_in[3] :
        control.d.ir5 ? reg_opcode[5] :
        flag_d;

    // I Flag
    wire logic next_i =
        control.i.di2 ? data_in[2] :
        control.i.ir5 ? reg_opcode[5] :
        flag_i;

    // Z Flag
    wire logic next_z =
        control.z.di1 ? data_in[1] :
        control.z.dbz ? (db == 8'h00) :
        flag_z;

    // C Flag
    wire logic next_c =
        control.c.di0 ? data_in[0] :
        control.c.ir5 ? reg_opcode[5] :
        control.c.alu ? alu_c_out :
        control.c.rmw ? rmw_c_out :
        flag_c;

    // Accumulator register
    wire logic [7:0] next_a = control.a ? db : reg_a;

    // X index register
    wire logic [7:0] next_x = control.x ? db : reg_x;

    // Y index register
    wire logic [7:0] next_y = control.y ? db : reg_y;

    // Program counter
    wire logic [15:0] pc_inc = reg_pc + 16'(control.pc.increment);
    wire logic [7:0]  branch_carry = {{7{reg_branch[9]}}, reg_branch[8]};
    wire logic [15:0] next_pc =
        control.pc.branch1 ? {reg_pc[15:8], branch_result[7:0]} :
        control.pc.branch2 ? {reg_pc[15:8] + branch_carry, reg_pc[7:0]} :
        control.pc.vector  ? {io_data_in, reg_data_in} :
        pc_inc;

    // Stack register
    wire logic [7:0] next_s =
        reg_s + {8{control.stack.dec}} + 8'(control.stack.inc);

    // Indexing calculations
    wire logic [7:0] index = control.index.y ? reg_y : reg_x;
    wire logic [7:0] offset = control.index.xy ? index : 8'h00;
    // 9-bit value includes carry bit
    wire logic [8:0] next_index = data_in + (control.index.carry ? 8'(reg_index[8]) : offset);

    // Bus address
    wire logic [15:0] address_out =
        (control.addr.pc    ? reg_pc                                 : '0) |
        (control.addr.stack ? {8'h01, reg_s}                         : '0) |
        (control.addr.zp1   ? {8'h00, data_in}                       : '0) |
        (control.addr.zp2   ? {8'h00, reg_index[7:0]}                : '0) |
        (control.addr.abs   ? {data_in, reg_index[7:0]}              : '0) |
        (control.addr.fffe  ? 16'hfffe                               : '0) |
        (control.addr.hold  ? reg_addr                               : '0) |
        (control.addr.inc   ? {reg_addr[15:8], reg_addr[7:0] + 8'h1} : '0) |
        (control.addr.carry ? {reg_index[7:0], reg_addr[7:0]}        : '0);

    // Data out
    wire logic [7:0] data_out =
        (control.write.same  ? data_in      : '0) |
        (control.write.rmw   ? rmw_out      : '0) |
        (control.write.store ? store_out    : '0) |
        (control.write.a     ? reg_a        : '0) |
        (control.write.p     ? status_out   : '0) |
        (control.write.pch   ? reg_pc[15:8] : '0) |
        (control.write.pcl   ? reg_pc[7:0]  : '0);

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
            reg_branch <= branch_result;

            // after we've done a write, data_in should reflect the
            // previous data_out.
            if (control.write.enable) begin
                reg_data_in <= data_out;
            end else begin
                reg_data_in <= io_data_in;
            end
        end
    end

    // Module outputs
    assign io_address      = address_out;
    assign io_data_out     = data_out;
    assign io_write_enable = control.write.enable;
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
      output logic [7:0] result
      );

    // CMP and SBC do subtraction
    wire logic sub = op[2];
    // CMP always uses C=1
    wire logic carry = c_in | ~op[0];

    // complement 2nd operand if subtracting
    wire logic [7:0] addend = sub ? ~b_in : b_in;

    // binary mode adder
    wire logic [8:0] bin_add = a_in + addend + 8'(carry);
    wire logic [7:0] add_result = bin_add[7:0]; // TODO: decimal mode

    always_comb begin
        case (op)
            3'h0: result = a_in | b_in; // ORA
            3'h1: result = a_in & b_in; // AND
            3'h2: result = a_in ^ b_in; // EOR
            3'h3: result = add_result;  // ADC
            3'h4: result = 8'h0;        // STA
            3'h5: result = b_in;        // LDA
            3'h6: result = add_result;  // CMP
            3'h7: result = add_result;  // SBC
        endcase // case (op)
    end

    assign v_out = (a_in[7] ^ result[7]) & (addend[7] ^ result[7]);
    assign c_out = bin_add[8];

endmodule: cpu6502_alu

module cpu6502_shift
    ( input logic [7:0] data_in,
      input logic c_in,
      input logic [2:0] op, // 0=ASL, 1=ROL, 2=LSR, 3=ROR, 6=DEC, 7=INC

      output logic c_out,
      output logic [7:0] data_out
      );

    wire logic rotate = op[0];
    wire logic right = op[1];
    wire logic inc_dec = op[2];
    wire logic carry = c_in & rotate;

    always_comb begin
        if (inc_dec) begin
            data_out = data_in + (op[0] ? 8'h01 : 8'hff);
            c_out = 1'b0;
        end else begin
            if (right)
                {data_out, c_out} = {carry, data_in};
            else
                {c_out, data_out} = {data_in, carry};
        end
    end
endmodule: cpu6502_shift
