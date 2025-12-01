`timescale 1 ns / 1 ps

/***************************************************************
 * Testbench for PicoRV32 with AES Decryption Co-Processor
 *
 * This testbench runs a RISC-V program that:
 * 1. Loads 128-bit ciphertext (4 x 32-bit words)
 * 2. Loads 128-bit key (4 x 32-bit words)
 * 3. Starts AES decryption
 * 4. Polls for completion
 * 5. Reads and stores the decrypted plaintext result
 *
 * AES-128 Test Vector (same as encryption test, reversed):
 *   Plaintext:  0x00112233445566778899aabbccddeeff
 *   Key:        0x000102030405060708090a0b0c0d0e0f
 *   Ciphertext: 0x69c4e0d86a7b0430d8cdb78070b4c55a
 ***************************************************************/

module tb_picorv32_aes_decryption;
    reg clk = 1;
    reg resetn = 0;
    wire trap;

    // Clock generation (100 MHz = 10ns period)
    always #5 clk = ~clk;

    // Dump VCD for waveform viewing
    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_picorv32_aes_decryption.vcd");
            $dumpvars(0, tb_picorv32_aes_decryption);
        end
    end

    // Reset and simulation control
    initial begin
        $display("==============================================");
        $display("PicoRV32 AES Decryption Co-Processor Testbench");
        $display("==============================================");

        // Hold reset for 20 cycles
        repeat (20) @(posedge clk);
        resetn <= 1;
        $display("[%0t] Reset released", $time);

        // Run for enough cycles for AES to complete
        repeat (5000) @(posedge clk);

        // Check results
        check_results();

        $display("==============================================");
        $display("Simulation complete");
        $display("==============================================");
        $finish;
    end

    // Memory interface signals
    wire        mem_valid;
    wire        mem_instr;
    reg         mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    reg  [31:0] mem_rdata;

    // Memory: 1KB (256 x 32-bit words)
    reg [31:0] memory [0:255];

    // Loop variable for memory initialization
    integer i;

    // Memory read/write behavior
    always @(posedge clk) begin
        mem_ready <= 0;
        if (mem_valid && !mem_ready) begin
            if (mem_addr < 1024) begin
                mem_ready <= 1;
                mem_rdata <= memory[mem_addr >> 2];
                if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
                if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
                if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
                if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
            end
        end
    end

    // PicoRV32 CPU instance with AES decryption enabled
    picorv32 #(
        .ENABLE_COUNTERS      (0),
        .ENABLE_COUNTERS64    (0),
        .ENABLE_REGS_16_31    (1),
        .ENABLE_REGS_DUALPORT (1),
        .ENABLE_MUL           (0),
        .ENABLE_DIV           (0),
        .ENABLE_AES           (0),
        .ENABLE_AES_DEC       (1),  // Enable AES decryption co-processor
        .ENABLE_IRQ           (0),
        .ENABLE_TRACE         (0),
        .CATCH_MISALIGN       (0),
        .CATCH_ILLINSN        (0)
    ) uut (
        .clk         (clk),
        .resetn      (resetn),
        .trap        (trap),
        .mem_valid   (mem_valid),
        .mem_instr   (mem_instr),
        .mem_ready   (mem_ready),
        .mem_addr    (mem_addr),
        .mem_wdata   (mem_wdata),
        .mem_wstrb   (mem_wstrb),
        .mem_rdata   (mem_rdata),
        // Unused PCPI external interface (internal PCPI cores are used)
        .pcpi_wr     (1'b0),
        .pcpi_rd     (32'b0),
        .pcpi_wait   (1'b0),
        .pcpi_ready  (1'b0),
        // Unused IRQ
        .irq         (32'b0)
    );

    // Monitor trap
    always @(posedge clk) begin
        if (trap) begin
            $display("[%0t] TRAP detected!", $time);
        end
    end

    /*******************************************************************
     * RISC-V Program to test AES Decryption
     *
     * Custom instruction encoding (R-type):
     *   [31:25] funct7  [24:20] rs2  [19:15] rs1  [14:12] funct3  [11:7] rd  [6:0] opcode
     *
     * AES Decryption Instructions (opcode=0001011, funct3=000):
     *   AES_DEC_LOAD_CT  : funct7=0101000 (0x28)
     *   AES_DEC_LOAD_KEY : funct7=0101001 (0x29)
     *   AES_DEC_START    : funct7=0101010 (0x2A)
     *   AES_DEC_READ     : funct7=0101011 (0x2B)
     *   AES_DEC_STATUS   : funct7=0101100 (0x2C)
     *******************************************************************/

    initial begin
        // Initialize memory to NOPs
        for (i = 0; i < 256; i = i + 1)
            memory[i] = 32'h00000013;  // NOP (addi x0, x0, 0)

        //=============================================================
        // CODE SECTION (starts at address 0x00)
        //=============================================================

        // Setup index registers
        memory[0]  = 32'h00100093;  // addi x1, x0, 1       ; x1 = 1
        memory[1]  = 32'h00200113;  // addi x2, x0, 2       ; x2 = 2
        memory[2]  = 32'h00300193;  // addi x3, x0, 3       ; x3 = 3

        memory[3]  = 32'h10000313;  // addi x6, x0, 0x100   ; x6 = CT base addr

        memory[4]  = 32'h11000213;  // addi x4, x0, 0x110   ; x4 = KEY base addr

        // Load Ciphertext into AES decryption co-processor
        memory[5]  = 32'h00032283;  // lw x5, 0(x6)         ; x5 = CT[0]
        memory[6]  = 32'h5050000B;  // AES_DEC_LOAD_CT x0, x5, x0  ; CT[31:0] = x5

        memory[7]  = 32'h00432283;  // lw x5, 4(x6)         ; x5 = CT[1]
        memory[8]  = 32'h5050800B;  // AES_DEC_LOAD_CT x0, x5, x1  ; CT[63:32] = x5

        memory[9]  = 32'h00832283;  // lw x5, 8(x6)         ; x5 = CT[2]
        memory[10] = 32'h5051000B;  // AES_DEC_LOAD_CT x0, x5, x2  ; CT[95:64] = x5

        memory[11] = 32'h00C32283;  // lw x5, 12(x6)        ; x5 = CT[3]
        memory[12] = 32'h5051800B;  // AES_DEC_LOAD_CT x0, x5, x3  ; CT[127:96] = x5

        // Load Key into AES decryption co-processor
        memory[13] = 32'h00022283;  // lw x5, 0(x4)         ; x5 = KEY[0]
        memory[14] = 32'h5250000B;  // AES_DEC_LOAD_KEY x0, x5, x0 ; KEY[31:0] = x5
        memory[15] = 32'h00422283;  // lw x5, 4(x4)         ; x5 = KEY[1]
        memory[16] = 32'h5250800B;  // AES_DEC_LOAD_KEY x0, x5, x1 ; KEY[63:32] = x5
        memory[17] = 32'h00822283;  // lw x5, 8(x4)         ; x5 = KEY[2]
        memory[18] = 32'h5251000B;  // AES_DEC_LOAD_KEY x0, x5, x2 ; KEY[95:64] = x5
        memory[19] = 32'h00C22283;  // lw x5, 12(x4)        ; x5 = KEY[3]
        memory[20] = 32'h5251800B;  // AES_DEC_LOAD_KEY x0, x5, x3 ; KEY[127:96] = x5

        // Start AES decryption
        memory[21] = 32'h5400000B;  // AES_DEC_START

        // Poll for completion (loop until status != 0)
        memory[22] = 32'h5800038B;  // AES_DEC_STATUS x7, x0, x0  ; x7 = status
        memory[23] = 32'hFE038EE3;  // beq x7, x0, -4             ; if busy, loop back

        // Read decrypted results
        memory[24] = 32'h5600040B;  // AES_DEC_READ x8,  x0, x0   ; x8  = PT[31:0]
        memory[25] = 32'h5600848B;  // AES_DEC_READ x9,  x0, x1   ; x9  = PT[63:32]
        memory[26] = 32'h5601050B;  // AES_DEC_READ x10, x0, x2   ; x10 = PT[95:64]
        memory[27] = 32'h5601858B;  // AES_DEC_READ x11, x0, x3   ; x11 = PT[127:96]

        // Store results to memory at 0x120
        memory[28] = 32'h12000613;  // addi x12, x0, 0x120   ; x12 = result addr
        memory[29] = 32'h00862023;  // sw x8, 0(x12)         ; store PT[0]
        memory[30] = 32'h00962223;  // sw x9, 4(x12)         ; store PT[1]
        memory[31] = 32'h00A62423;  // sw x10, 8(x12)        ; store PT[2]
        memory[32] = 32'h00B62623;  // sw x11, 12(x12)       ; store PT[3]

        // Infinite loop (end of program)
        memory[33] = 32'h0000006F;  // jal x0, 0             ; jump to self

        //=============================================================
        // DATA SECTION
        //=============================================================

        // Ciphertext at 0x100 (memory index 64)
        // Full ciphertext: 0x69c4e0d8_6a7b0430_d8cdb780_70b4c55a
        memory[64] = 32'h70b4c55a;  // CT word 0 (CT[31:0])
        memory[65] = 32'hd8cdb780;  // CT word 1 (CT[63:32])
        memory[66] = 32'h6a7b0430;  // CT word 2 (CT[95:64])
        memory[67] = 32'h69c4e0d8;  // CT word 3 (CT[127:96])

        // Key at 0x110 (memory index 68)
        // Full key: 0x00010203_04050607_08090a0b_0c0d0e0f
        memory[68] = 32'h0c0d0e0f;  // KEY word 0 (KEY[31:0])
        memory[69] = 32'h08090a0b;  // KEY word 1 (KEY[63:32])
        memory[70] = 32'h04050607;  // KEY word 2 (KEY[95:64])
        memory[71] = 32'h00010203;  // KEY word 3 (KEY[127:96])

        // Results will be stored at 0x120 (memory index 72)
        // Expected plaintext: 0x00112233_44556677_8899aabb_ccddeeff
        // memory[72] = PT[31:0]   = 0xccddeeff
        // memory[73] = PT[63:32]  = 0x8899aabb
        // memory[74] = PT[95:64]  = 0x44556677
        // memory[75] = PT[127:96] = 0x00112233
    end

    // Task to check results
    task check_results;
        begin
            $display("");
            $display("=== Checking AES Decryption Results ===");
            $display("");
            $display("Input Ciphertext: 0x%08x_%08x_%08x_%08x",
                     memory[67], memory[66], memory[65], memory[64]);
            $display("Input Key:        0x%08x_%08x_%08x_%08x",
                     memory[71], memory[70], memory[69], memory[68]);
            $display("");
            $display("Output Plaintext (from memory 0x120):");
            $display("  PT[127:96] = 0x%08x", memory[75]);
            $display("  PT[95:64]  = 0x%08x", memory[74]);
            $display("  PT[63:32]  = 0x%08x", memory[73]);
            $display("  PT[31:0]   = 0x%08x", memory[72]);
            $display("");
            $display("Full plaintext:   0x%08x_%08x_%08x_%08x",
                     memory[75], memory[74], memory[73], memory[72]);
            $display("");

            // Expected values
            if (memory[72] == 32'hccddeeff &&
                memory[73] == 32'h8899aabb &&
                memory[74] == 32'h44556677 &&
                memory[75] == 32'h00112233) begin
                $display("*** TEST PASSED! Plaintext matches expected value ***");
            end else begin
                $display("*** TEST RESULT: Check plaintext against your AES decryption core ***");
                $display("Expected: 0x00112233_44556677_8899aabb_ccddeeff");
            end
            $display("");
        end
    endtask

    // Optional: Monitor memory transactions
    always @(posedge clk) begin
        if (mem_valid && mem_ready) begin
            if (mem_instr)
                $display("[%0t] FETCH  PC=0x%08x  INSN=0x%08x", $time, mem_addr, mem_rdata);
            else if (|mem_wstrb)
                $display("[%0t] WRITE  addr=0x%08x data=0x%08x strb=%b",
                         $time, mem_addr, mem_wdata, mem_wstrb);
            else
                $display("[%0t] READ   addr=0x%08x data=0x%08x", $time, mem_addr, mem_rdata);
        end
    end

endmodule

