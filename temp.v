/***************************************************************
 * pcpi_aes - AES Co-Processor for PicoRV32
 * 
 * Custom Instructions (opcode = 0001011, funct3 = 000):
 *   funct7=0100000: AES_LOAD_PT  - Load plaintext word  (rs1=index 0-3, rs2=data)
 *   funct7=0100001: AES_LOAD_KEY - Load key word        (rs1=index 0-3, rs2=data)
 *   funct7=0100010: AES_START    - Start encryption     (no operands needed)
 *   funct7=0100011: AES_READ     - Read result word     (rs1=index 0-3) -> rd
 *   funct7=0100100: AES_STATUS   - Check status         () -> rd (1=done, 0=busy)
 ***************************************************************/

module pcpi_aes (
	input clk, resetn,

	input             pcpi_valid,
	input      [31:0] pcpi_insn,
	input      [31:0] pcpi_rs1,
	input      [31:0] pcpi_rs2,
	output reg        pcpi_wr,
	output reg [31:0] pcpi_rd,
	output reg        pcpi_wait,
	output reg        pcpi_ready
);

	// Instruction decode
	wire [6:0] opcode = pcpi_insn[6:0];
	wire [2:0] funct3 = pcpi_insn[14:12];
	wire [6:0] funct7 = pcpi_insn[31:25];

	// Detect our custom instructions (opcode = custom-0 = 0001011, funct3 = 000)
	wire is_custom = (opcode == 7'b0001011) && (funct3 == 3'b000);

	wire instr_load_pt  = is_custom && (funct7 == 7'b0100000);
	wire instr_load_key = is_custom && (funct7 == 7'b0100001);
	wire instr_start    = is_custom && (funct7 == 7'b0100010);
	wire instr_read     = is_custom && (funct7 == 7'b0100011);
	wire instr_status   = is_custom && (funct7 == 7'b0100100);

	wire instr_any = instr_load_pt | instr_load_key | instr_start | instr_read | instr_status;

	// Internal data registers
	reg [127:0] PT;      // Plaintext
	reg [127:0] KEY;     // Key
	reg [127:0] RESULT;  // Encrypted result

	// AES control
	reg aes_running;
	reg aes_encrypt;     // Trigger signal for AES
	wire aes_done;
	wire [127:0] Dout;

	// Instantiate AES core
	ASMD_Encryption aes_core (
		.done          (aes_done),
		.Dout          (Dout),
		.plain_text_in (PT),
		.key_in        (KEY),
		.encrypt       (aes_encrypt),
		.clock         (clk),
		.reset         (~resetn)
	);

	// FSM states
	localparam IDLE       = 3'd0;
	localparam EXECUTE    = 3'd1;
	localparam WAIT_AES   = 3'd2;

	reg [2:0] state;
	reg [1:0] word_index;

	always @(posedge clk) begin
		if (!resetn) begin
			state       <= IDLE;
			pcpi_ready  <= 0;
			pcpi_wr     <= 0;
			pcpi_wait   <= 0;
			pcpi_rd     <= 0;
			PT          <= 128'b0;
			KEY         <= 128'b0;
			RESULT      <= 128'b0;
			aes_running <= 0;
			aes_encrypt <= 0;
		end
		else begin
			// Default: clear single-cycle signals
			pcpi_wr     <= 0;
			pcpi_ready  <= 0;
			aes_encrypt <= 0;

			case (state)

			IDLE: begin
				pcpi_wait <= 0;
				if (pcpi_valid && instr_any) begin
					word_index <= pcpi_rs1[1:0];  // Word index from rs1
					pcpi_wait  <= 1;
					state      <= EXECUTE;
				end
			end

			EXECUTE: begin
				if (instr_load_pt) begin
					// Load plaintext word: PT[index] = rs2
					case (word_index)
						2'd0: PT[31:0]    <= pcpi_rs2;
						2'd1: PT[63:32]   <= pcpi_rs2;
						2'd2: PT[95:64]   <= pcpi_rs2;
						2'd3: PT[127:96]  <= pcpi_rs2;
					endcase
					pcpi_rd    <= 32'd0;  // Return 0 (success)
					pcpi_wr    <= 1;
					pcpi_ready <= 1;
					pcpi_wait  <= 0;
					state      <= IDLE;
				end
				else if (instr_load_key) begin
					// Load key word: KEY[index] = rs2
					case (word_index)
						2'd0: KEY[31:0]    <= pcpi_rs2;
						2'd1: KEY[63:32]   <= pcpi_rs2;
						2'd2: KEY[95:64]   <= pcpi_rs2;
						2'd3: KEY[127:96]  <= pcpi_rs2;
					endcase
					pcpi_rd    <= 32'd0;
					pcpi_wr    <= 1;
					pcpi_ready <= 1;
					pcpi_wait  <= 0;
					state      <= IDLE;
				end
				else if (instr_start) begin
					// Start AES encryption
					aes_encrypt <= 1;
					aes_running <= 1;
					state       <= WAIT_AES;
				end
				else if (instr_read) begin
					// Read result word
					case (word_index)
						2'd0: pcpi_rd <= RESULT[31:0];
						2'd1: pcpi_rd <= RESULT[63:32];
						2'd2: pcpi_rd <= RESULT[95:64];
						2'd3: pcpi_rd <= RESULT[127:96];
					endcase
					pcpi_wr    <= 1;
					pcpi_ready <= 1;
					pcpi_wait  <= 0;
					state      <= IDLE;
				end
				else if (instr_status) begin
					// Return status: 1 = done/idle, 0 = busy
					pcpi_rd    <= aes_running ? 32'd0 : 32'd1;
					pcpi_wr    <= 1;
					pcpi_ready <= 1;
					pcpi_wait  <= 0;
					state      <= IDLE;
				end
			end

			WAIT_AES: begin
				if (aes_done) begin
					RESULT      <= Dout;      // Capture result
					aes_running <= 0;
					pcpi_rd     <= 32'd0;     // Return 0 (success)
					pcpi_wr     <= 1;
					pcpi_ready  <= 1;
					pcpi_wait   <= 0;
					state       <= IDLE;
				end
			end

			default: state <= IDLE;

			endcase
		end
	end
endmodule