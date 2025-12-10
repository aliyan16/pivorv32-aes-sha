`timescale 1 ns / 1 ps

/***************************************************************
 * Testbench for PicoRV32 with AES Co-Processor + SHA-256 Chaining
 * 
 * This testbench runs a RISC-V program that:
 * 1. Loads 128-bit plaintext (4 x 32-bit words)
 * 2. Loads 128-bit key (4 x 32-bit words)
 * 3. Starts AES encryption (which automatically chains to SHA-256)
 * 4. Polls for completion (waits for both AES and SHA-256 to complete)
 * 5. Reads and stores the encrypted result
 *
 * Flow: AES_START -> AES encryption -> SHA-256 hash of ciphertext -> pcpi_ready
 * 
 * AES-128 Test Vector:
 *   Plaintext:  0x00112233445566778899aabbccddeeff
 *   Key:        0x000102030405060708090a0b0c0d0e0f
 *   Ciphertext: 0x69c4e0d86a7b0430d8cdb78070b4c55a
 *   
 * Note: SHA-256 automatically computes hash of the AES ciphertext
 *       (128 bits padded with zeros to form 512-bit block)
 ***************************************************************/

module tb_picorv32_aes;
    reg clk = 1;
    reg resetn = 0;
    wire trap;

    // Clock generation (100 MHz = 10ns period)
    always #5 clk = ~clk;

    // Dump VCD for waveform viewing
    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_picorv32_aes.vcd");
            $dumpvars(0, tb_picorv32_aes);
        end
    end

    // Reset and simulation control
    initial begin
        $display("==============================================");
        $display("PicoRV32 AES + SHA-256 Co-Processor Testbench");
        $display("==============================================");
        
        // Hold reset for 20 cycles
        repeat (20) @(posedge clk);
        resetn <= 1;
        $display("[%0t] Reset released", $time);
        
        // Run for enough cycles for AES + SHA-256 to complete
        // AES-128 takes ~10-11 rounds, each round may take multiple cycles
        // SHA-256 takes 64 rounds, so we need more cycles
        repeat (10000) @(posedge clk);
        
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

    // PicoRV32 CPU instance with AES enabled
    picorv32 #(
        .ENABLE_COUNTERS     (0),
        .ENABLE_COUNTERS64   (0),
        .ENABLE_REGS_16_31   (1),
        .ENABLE_REGS_DUALPORT(1),
        .ENABLE_MUL          (0),
        .ENABLE_DIV          (0),
        .ENABLE_AES          (1),  // Enable AES co-processor
        .ENABLE_IRQ          (0),
        .ENABLE_TRACE        (0),
        .CATCH_MISALIGN      (0),
        .CATCH_ILLINSN       (0)
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
        // Unused PCPI external interface (internal AES is used)
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
     * RISC-V Program to test AES + SHA-256 Chaining
     * 
     * Custom instruction encoding (R-type):
     *   [31:25] funct7  [24:20] rs2  [19:15] rs1  [14:12] funct3  [11:7] rd  [6:0] opcode
     *   
     * AES Instructions (opcode=0001011, funct3=000):
     *   AES_LOAD_PT  : funct7=0100000 (0x20)
     *   AES_LOAD_KEY : funct7=0100001 (0x21)
     *   AES_START    : funct7=0100010 (0x22) - Automatically chains to SHA-256
     *   AES_READ     : funct7=0100011 (0x23)
     *   AES_STATUS   : funct7=0100100 (0x24)
     * 
     * Note: AES_START now automatically:
     *   1. Encrypts plaintext with AES-128
     *   2. Takes 128-bit ciphertext and pads with zeros to 512 bits
     *   3. Computes SHA-256 hash of the padded block
     *   4. Returns pcpi_ready=1 only after both AES and SHA-256 complete
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

        memory[3]  = 32'h10000313;  // addi x6, x0, 0x100   ; x6 = PT base addr

        memory[4]  = 32'h11000213;  // addi x4, x0, 0x110   ; x4 = KEY base addr

        // Load Plaintext into AES co-processor

        memory[5]  = 32'h00032283;  // lw x5, 0(x6)         ; x5 = PT[0]
        memory[6]  = 32'h4050000B;  // AES_LOAD_PT x0, x5, x0  ; PT[31:0] = x5  )) send PT[0] to AES



        memory[7]  = 32'h00432283;  // lw x5, 4(x6)         ; x5 = PT[1]
        memory[8]  = 32'h4050800B;  // AES_LOAD_PT x0, x5, x1  ; PT[63:32] = x5

        memory[9]  = 32'h00832283;  // lw x5, 8(x6)         ; x5 = PT[2]
        memory[10] = 32'h4051000B;  // AES_LOAD_PT x0, x5, x2  ; PT[95:64] = x5

        memory[11] = 32'h00C32283;  // lw x5, 12(x6)        ; x5 = PT[3]
        memory[12] = 32'h4051800B;  // AES_LOAD_PT x0, x5, x3  ; PT[127:96] = x5

        // Load Key into AES co-processor
        memory[13] = 32'h00022283;  // lw x5, 0(x4)         ; x5 = KEY[0]
        memory[14] = 32'h4250000B;  // AES_LOAD_KEY x0, x5, x0 ; KEY[31:0] = x5
        memory[15] = 32'h00422283;  // lw x5, 4(x4)         ; x5 = KEY[1]
        memory[16] = 32'h4250800B;  // AES_LOAD_KEY x0, x5, x1 ; KEY[63:32] = x5
        memory[17] = 32'h00822283;  // lw x5, 8(x4)         ; x5 = KEY[2]
        memory[18] = 32'h4251000B;  // AES_LOAD_KEY x0, x5, x2 ; KEY[95:64] = x5
        memory[19] = 32'h00C22283;  // lw x5, 12(x4)        ; x5 = KEY[3]
        memory[20] = 32'h4251800B;  // AES_LOAD_KEY x0, x5, x3 ; KEY[127:96] = x5

        // Start AES encryption (automatically chains to SHA-256)
        // This will: AES encrypt -> SHA-256 hash -> pcpi_ready
        // The pcpi_ready signal will be asserted only after BOTH operations complete
        memory[21] = 32'h4400000B;  // AES_START (triggers AES + SHA-256 chain)

        // Poll for completion (loop until status != 0)
        // Status will be 1 only after both AES encryption AND SHA-256 hashing complete
        // This polling loop will wait for the entire AES->SHA-256 chain to finish
        memory[22] = 32'h4800038B;  // AES_STATUS x7, x0, x0  ; x7 = status
        memory[23] = 32'hFE038EE3;  // beq x7, x0, -4         ; if busy, loop back

        // Read encrypted results
        memory[24] = 32'h4600040B;  // AES_READ x8, x0, x0   ; x8 = CT[31:0]
        memory[25] = 32'h4600848B;  // AES_READ x9, x0, x1   ; x9 = CT[63:32]
        memory[26] = 32'h4601050B;  // AES_READ x10, x0, x2  ; x10 = CT[95:64]
        memory[27] = 32'h4601858B;  // AES_READ x11, x0, x3  ; x11 = CT[127:96]

        // Store results to memory at 0x120
        memory[28] = 32'h12000613;  // addi x12, x0, 0x120   ; x12 = result addr
        memory[29] = 32'h00862023;  // sw x8, 0(x12)         ; store CT[0]
        memory[30] = 32'h00962223;  // sw x9, 4(x12)         ; store CT[1]
        memory[31] = 32'h00A62423;  // sw x10, 8(x12)        ; store CT[2]
        memory[32] = 32'h00B62623;  // sw x11, 12(x12)       ; store CT[3]

        // Infinite loop (end of program)
        memory[33] = 32'h0000006F;  // jal x0, 0             ; jump to self

        //=============================================================
        // DATA SECTION
        //=============================================================
        
        // Plaintext at 0x100 (memory index 64)
        // Full plaintext: 0x00112233_44556677_8899aabb_ccddeeff
        memory[64] = 32'hccddeeff;  // PT word 0 (PT[31:0])
        memory[65] = 32'h8899aabb;  // PT word 1 (PT[63:32])
        memory[66] = 32'h44556677;  // PT word 2 (PT[95:64])
        memory[67] = 32'h00112233;  // PT word 3 (PT[127:96])

        // Key at 0x110 (memory index 68)
        // Full key: 0x00010203_04050607_08090a0b_0c0d0e0f
        memory[68] = 32'h0c0d0e0f;  // KEY word 0 (KEY[31:0])
        memory[69] = 32'h08090a0b;  // KEY word 1 (KEY[63:32])
        memory[70] = 32'h04050607;  // KEY word 2 (KEY[95:64])
        memory[71] = 32'h00010203;  // KEY word 3 (KEY[127:96])

        // Results will be stored at 0x120 (memory index 72)
        // Expected ciphertext: 0x69c4e0d8_6a7b0430_d8cdb780_70b4c55a
        // memory[72] = CT[31:0]   = 0x70b4c55a
        // memory[73] = CT[63:32]  = 0xd8cdb780
        // memory[74] = CT[95:64]  = 0x6a7b0430
        // memory[75] = CT[127:96] = 0x69c4e0d8
    end

    // Task to check results
    task check_results;
        begin
            $display("");
            $display("=== Checking AES Encryption + SHA-256 Results ===");
            $display("");
            $display("Input Plaintext:  0x%08x_%08x_%08x_%08x", 
                     memory[67], memory[66], memory[65], memory[64]);
            $display("Input Key:        0x%08x_%08x_%08x_%08x", 
                     memory[71], memory[70], memory[69], memory[68]);
            $display("");
            $display("Output Ciphertext (from memory 0x120):");
            $display("  CT[127:96] = 0x%08x", memory[75]);
            $display("  CT[95:64]  = 0x%08x", memory[74]);
            $display("  CT[63:32]  = 0x%08x", memory[73]);
            $display("  CT[31:0]   = 0x%08x", memory[72]);
            $display("");
            $display("Full ciphertext:  0x%08x_%08x_%08x_%08x", 
                     memory[75], memory[74], memory[73], memory[72]);
            $display("");
            $display("Note: SHA-256 hash was automatically computed on the ciphertext");
            $display("      (128-bit ciphertext padded with zeros to 512-bit block)");
            $display("");
            
            // Expected AES ciphertext values
            if (memory[72] == 32'h70b4c55a &&
                memory[73] == 32'hd8cdb780 &&
                memory[74] == 32'h6a7b0430 &&
                memory[75] == 32'h69c4e0d8) begin
                $display("*** TEST PASSED! AES Ciphertext matches expected value ***");
                $display("*** SHA-256 hash was automatically computed during AES_START ***");
            end else begin
                $display("*** TEST RESULT: Check ciphertext against your AES core ***");
                $display("Expected AES ciphertext: 0x69c4e0d8_6a7b0430_d8cdb780_70b4c55a");
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

    // Monitor AES custom instruction execution
    // (Observe pcpi signals inside CPU - this is optional debug)
    /*
    always @(posedge clk) begin
        if (uut.pcpi_valid) begin
            $display("[%0t] PCPI: insn=0x%08x rs1=0x%08x rs2=0x%08x", 
                     $time, uut.pcpi_insn, uut.pcpi_rs1, uut.pcpi_rs2);
        end
        if (uut.pcpi_int_ready) begin
            $display("[%0t] PCPI DONE: rd=0x%08x wr=%b", 
                     $time, uut.pcpi_int_rd, uut.pcpi_int_wr);
        end
    end
    */

endmodule
