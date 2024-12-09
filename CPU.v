// Define opcodes for each instruction
	`define NOP 4'b0000
	`define BRA 4'b0001
	`define LD  4'b0010
	`define STR 4'b0011
	`define ADD 4'b0100
	`define MUL 4'b0101
	`define CMP 4'b0110
	`define SHF 4'b0111
	`define ROT 4'b1000
	`define HLT 4'b1001

	///new inst.////////////////////////////////
	`define OR 4'b1010
	`define XOR 4'b1011

	`define S0 2'd0
	`define FETCH 2'd1
	`define EXECUTE 2'd2
	`define WRITE 2'd3

    `define CCC 1 // Result has carry
	`define CCE 2 // Result is even
	`define CCP 3 // Result is odd parity
	`define CCZ 4 // Result is Zero
	`define CCN 5 // Result is Negative
	`define CCA 0 // Always



    `define WIDTH 32  // Width of data paths
	`define ADDRSIZE  12 // Size of address field
	`define MAXREGS = 16 

	`define CMD_RESET      2'b00
	`define CMD_HOLD       2'b01
	`define CMD_LOAD       2'b10
	`define CMD_INCREMENT  2'b11


	`define LOAD_or_renew       2'b10
	`define INCREMENT  2'b11
    `define HOLD       2'b10
  
	`define OPCODE         c_IR[31:28]

	`define SRC_TYPE       c_IR[27]
	`define DST_TYPE       c_IR[26]

	`define SRC_addr       c_IR[23:12]
    `define DST_addr       c_IR[11:0]

	`define SRC_reg_addr   c_IR[15:12]
	`define DST_reg_addr   c_IR[3:0]

	`define IMMEDIATE   1

	`define CONDITION_CODES c_IR[27:24]



module FSM (
    input clk,
    input reset,
    output reg [1:0] c_state
);

    reg [1:0] n_state;

    always @(posedge clk) begin
        if (reset) begin
            c_state <= `S0;
        end else begin
            c_state <= n_state;
        end
    end

	always @(*) begin
		case (c_state)
		`S0: n_state=`FETCH;
		`FETCH: n_state=`EXECUTE;
		`EXECUTE:n_state=`WRITE;
		//`WRITE
		default: n_state=`FETCH;
		endcase
	end
endmodule

module IR (
    input clk,
    input reset,
    input [1:0] com,
    input [31:0] inst_in,
    output reg [31:0] inst_out
);
    always @(posedge clk) begin
        if (reset) begin
            inst_out <= 32'd0;
        end else begin
            case (com)
                `LOAD_or_renew: begin
                    inst_out <= inst_in;
                end
                //hold
                default: begin
                    inst_out <= inst_out;
                end
            endcase
        end
    end
endmodule

module PC (
    input clk,
    input reset,
    input [1:0] com,
    input [11:0] PC_in,
    output reg [11:0] PC_out
);
   //negedge give PC
    always @(negedge clk) begin
        if (reset) begin
            PC_out <= 12'd0;
        end else begin
            case (com)
                `LOAD_or_renew: PC_out <= PC_in;
                `INCREMENT: PC_out <= PC_out + 1;
                default: PC_out <=PC_out;
            endcase
        end
    end
endmodule

module ALU (
    input clk,
    input [3:0] operation,
    input [31:0] op1,
    input [31:0] op2,
    output reg [32:0] ALU_result
);


     reg [31:0] operand1,operand2,i;

	always @(*) begin
		 operand1=op1;
         operand2=op2;
		 i = (operand1[31]) ?  (~operand1) + 1 : operand1;
	end

    always @(*) begin
        case (operation)
            `SHF: begin
				ALU_result=(operand1[31])? {1'b0,operand2 << i}: {1'b0,operand2 >> i};
            end
            `ROT: begin 
                // Carry is the last bit shifted out
				ALU_result[31:0]=(operand1[31])? ((operand2 <<i) | (operand2 << (32 - i))): ((operand2 >> i) | (operand2 << (32 - i)));
                ALU_result[32]=(operand1[31])? operand2[32 - i]:operand2[i - 1];
            end
            `ADD: begin
                ALU_result = operand1 + operand2;
            end
            `MUL: begin
                ALU_result = operand1 * operand2;
            end
            `CMP: begin
                ALU_result = {1'b0, ~operand1};
            end
            `XOR: begin
                ALU_result = {1'b0, operand1 ^ operand2};
            end
			`OR: begin
                ALU_result = {1'b0, operand1 | operand2};
            end
            //NOP
            default: begin
			    ALU_result = ALU_result;
            end
        endcase
    end

endmodule

module RF (
    input clk,
    input w_en,           
    input [3:0] RF_w_addr,    
    input [31:0] RF_w_data,      
    input [3:0] RF_r_addr1,       
    input [3:0] RF_r_addr2, 
	
	output [31:0] RF_r_data1,   
    output [31:0] RF_r_data2     
);


    parameter WIDTH=32 ; // Width of data paths
	parameter MAXREGS = 16 ; // Maximum # of registers

    reg [WIDTH-1:0] RFILE [0:MAXREGS-1];

	 always @(negedge clk) begin//negedge give data
        if (w_en) begin
            RFILE[RF_w_addr] <= RF_w_data;
        end
    end


    assign RF_r_data1 = RFILE[RF_r_addr1];
    assign RF_r_data2 = RFILE[RF_r_addr2];

   
endmodule



module CONTROL (
    input [11:0] pc_current,
    input [1:0] c_state,
    input [4:0] psr_current,
    input [31:0] c_IR, reg_data1, reg_data2, mem_data_out,
    input [32:0] alu_result,

    output reg branch,
    output halt_signal,
    output reg [31:0] mem_datain,
    output reg reg_w_en,
    output reg mem_rw, 
    output reg [1:0] pc_com, 
    output reg [1:0] psr_com,
    output reg [1:0] ir_com,
    output reg [3:0] reg_write_addr, reg_read_addr1, reg_read_addr2,
    output reg [3:0] alu_op,
    output reg [4:0] psr_input,
    output reg [11:0] pc_next,
    output reg [11:0]mem_addr,
    output reg [31:0] ir_next,
    output reg [31:0] reg_RF_w_data, 
    output reg [31:0]alu_op1, alu_op2
);

    
    assign halt_signal = (`OPCODE == `HLT) ? 1 : 0;
    assign c_flag = alu_result[32];
    assign e_flag = ~(alu_result[0]);
    assign p_flag = ^(alu_result[31:0]);
    assign z_flag = ~(|alu_result[31:0]);
    assign n_flag =alu_result[31];





    //memory
    always @(*) begin
         if(c_state == `FETCH)begin
            mem_addr=pc_current;
           // $display(pc_current);
         end
         else if(`OPCODE == `STR)begin
            mem_addr=`DST_addr;
         end
         else if(`OPCODE == `LD)begin
            mem_addr=`SRC_addr;
         end
         else begin
            mem_addr=mem_addr;
         end


         if ((c_state == `EXECUTE || c_state == `WRITE) && `OPCODE == `STR && `SRC_TYPE != `IMMEDIATE) begin
             mem_datain = reg_data1;
         end 
         else begin 
            mem_datain = 32'hz;
         end


         mem_rw = ((c_state == `WRITE) && (`OPCODE == `STR) ) ? 1'b1 : 1'b0;
    end





    //ALU
    always @(*) begin
        if ((`OPCODE != `HLT) && (`OPCODE >= `ADD && `OPCODE <= `XOR)) begin
            alu_op=`OPCODE;
        end 
        else begin
            alu_op=`NOP;
        end  

        if (`SRC_TYPE == `IMMEDIATE) begin
            alu_op1={{20{c_IR[23]}}, `SRC_addr};
        end else begin
            alu_op1=reg_data1;
        end

        alu_op2 = reg_data2;

    end

    //reg
    always @(*) begin
         
         reg_read_addr1 = `SRC_reg_addr;
         reg_read_addr2 = `DST_reg_addr;

         if ((c_state == `WRITE) && (`OPCODE != `HLT) && ((`OPCODE >= `ADD && `OPCODE <= `XOR) ||(`OPCODE == `LD))) begin
            reg_w_en=1'd1;
         end else begin
            reg_w_en=1'd0;
         end
         reg_write_addr = `DST_reg_addr;

         if ((`OPCODE == `LD) && (`SRC_TYPE == `IMMEDIATE)) begin
            reg_RF_w_data={{20{c_IR[23]}}, `SRC_addr} ;
         end 
         else if((`OPCODE == `LD) && (`SRC_TYPE != `IMMEDIATE)) begin
            reg_RF_w_data=mem_data_out;
         end
         else begin
            reg_RF_w_data=alu_result;
         end

    end
    
   //IR
   always @(*) begin
     if (c_state == `FETCH) begin
         ir_com=`LOAD_or_renew;
         ir_next=mem_data_out;   
     end else begin
         ir_com=`HOLD;
         ir_next=c_IR;
     end
   end


   //branch
   always @(*) begin
    if ((c_state==`WRITE) &&(`OPCODE == `BRA)) begin
        case (`CONDITION_CODES)
            `CCA: branch=1'd1;
            `CCC: branch=psr_current[0];
            `CCE: branch=psr_current[1];
            `CCP: branch=psr_current[2];
            `CCZ: branch=psr_current[3];
            //CCN
            default: 
            branch=psr_current[4];
        endcase
    end else begin
        branch=1'd0;
    end
   end


  //PSR
  always @(*) begin
     if ((c_state == `WRITE)&&(`OPCODE >= `LD && `OPCODE <= `XOR)&&(`OPCODE !=`HLT)) begin
        psr_com=`LOAD_or_renew;
        if ((`OPCODE == `LD)&&(`SRC_TYPE == `IMMEDIATE)) begin
            psr_input={c_IR[23], ~(|c_IR[23:12]), ^(c_IR[23:12]), ~c_IR[12], 1'b0};
        end else if((`OPCODE == `LD)&&(`SRC_TYPE != `IMMEDIATE)) begin
            psr_input={mem_data_out[31], ~(|mem_data_out), ^(mem_data_out), ~mem_data_out[0], 1'b0};
        end
        else if(`OPCODE == `STR)
         begin
            psr_input={reg_data1[31], ~(|reg_data1), ^(reg_data1), ~reg_data1[0], 1'b0};
        end
        else begin
            psr_input={n_flag,z_flag, p_flag, e_flag,c_flag};
        end
     end else begin
        psr_com=`HOLD;
        psr_input=psr_input;
     end
    
  end
   
    // //  PC
  always @(*) begin
    if (c_state == `WRITE) begin
        if(branch) begin
          pc_com=`LOAD_or_renew;
          pc_next=`DST_addr;
        end
        else begin
          pc_com=`INCREMENT;
          pc_next=pc_next;
        end

    end else begin
        pc_com=`HOLD;
        pc_next=pc_current;
    end
    
  end

endmodule

module stat_reg (
    input clk,
    input reset,
    input [1:0] com,
    input [4:0] PSR_in,
    output reg [4:0] PSR_out
);

    always @(posedge clk) begin
        if (reset) begin
            PSR_out <= 5'd0;
        end else begin
            case (com)
                `LOAD_or_renew: PSR_out <= PSR_in; 
                //HOLD
                default: PSR_out <= PSR_out; 
            endcase
        end
    end

endmodule



module CPU (
    input clk,
    input reset,
    input [31:0] mem_data_out,
    output mem_rw,
    output [11:0] mem_addr,
    output [31:0] mem_datain,
	output halt
);

    parameter ADDR_SIZE = 12;
    parameter DATA_WIDTH = 32;

   
    wire branch;
    wire regfile_w_en,halt_signal;
    wire [1:0] ir_com, pc_com, psr_com, c_state;
    wire [3:0] alu_op, regfile_write_addr, regfile_read_addr1, regfile_read_addr2;
    wire [4:0] psr_output, psr_input;
    wire [ADDR_SIZE-1:0] pc_current, pc_next;
    wire [DATA_WIDTH-1:0] c_IR, ir_next, regfile_RF_w_data, regfile_data1, regfile_data2, alu_operand1, alu_operand2;
    wire [DATA_WIDTH:0] alu_result;


     assign halt=halt_signal;
	
    PC PC (
        .clk(clk),
        .reset(reset),
        .com(pc_com),
        .PC_in(pc_next),
        .PC_out(pc_current)
    );

    IR IR (
        .clk(clk),
        .reset(reset),
        .com(ir_com),
        .inst_in(ir_next),
        .inst_out(c_IR)
    );

	 RF RF (
        .clk(clk),
        .w_en(regfile_w_en),
        .RF_w_addr(regfile_write_addr),
        .RF_w_data(regfile_RF_w_data),
        .RF_r_addr1(regfile_read_addr1),
        .RF_r_data1(regfile_data1),
        .RF_r_addr2(regfile_read_addr2),
        .RF_r_data2(regfile_data2)
    );

    stat_reg PSR (
        .clk(clk),
        .reset(reset),
        .com(psr_com),
        .PSR_in(psr_input),
        .PSR_out(psr_output)
    );

    ALU ALU (
        .clk(clk),
        .operation(alu_op),
        .op1(alu_operand1),
        .op2(alu_operand2),
        .ALU_result(alu_result)
    );

    CONTROL CONTROLLER (
		.halt_signal(halt_signal),
		.c_state(c_state),
		.mem_datain(mem_datain),
		.mem_data_out(mem_data_out),
		.mem_addr(mem_addr),
		.mem_rw(mem_rw),
		.reg_RF_w_data(regfile_RF_w_data),
		.reg_write_addr(regfile_write_addr),
		.reg_w_en(regfile_w_en),
		.reg_data1(regfile_data1),
		.reg_read_addr1(regfile_read_addr1),
		.reg_data2(regfile_data2),
		.reg_read_addr2(regfile_read_addr2),
		.pc_current(pc_current),
		.pc_next(pc_next),
		.pc_com(pc_com),
		.c_IR(c_IR),
		.ir_next(ir_next),
		.ir_com(ir_com),
		.psr_current(psr_output),
		.psr_input(psr_input),
		.psr_com(psr_com),
		.alu_result(alu_result),
		.alu_op(alu_op),
		.alu_op1(alu_operand1),
		.alu_op2(alu_operand2),
        .branch(branch)
	);


    FSM FSM (
        .clk(clk),
        .reset(reset),
        .c_state(c_state)
    );

endmodule



