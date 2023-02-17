/*************************************************************************************|
|   1. YOU ARE NOT ALLOWED TO SHARE/PUBLISH YOUR CODE (e.g., post on piazza or online)|
|   2. Fill mipssim.c                                                                 |
|   3. Do not use any other .c files neither alter mipssim.h or parser.h              |
|   4. Do not include any other library files                                         |
|*************************************************************************************/

#include "mipssim.h"

#define BREAK_POINT 200000 // exit after so many cycles -- useful for debugging

// Global variables
char mem_init_path[1000];
char reg_init_path[1000];

uint32_t cache_size = 0;
struct architectural_state arch_state;




static inline uint8_t get_instruction_type(int opcode)
{
    switch (opcode) {
        /// opcodes are defined in mipssim.h

        case SPECIAL:
            return R_TYPE;
        case ADD:
            return R_TYPE;
        case ADDI:
            return R_TYPE;
        case LW:
            return LW;
        case SW:
            return SW;
        case BEQ:
            return BEQ;
        case J:
            return J;
        case SLT:
            return BEQ;
        case EOP:
            return EOP_TYPE;

        ///@students: fill in the rest

        default:
            assert(false);
    }
    assert(false);
}


//Instruction types
#define R_TYPE 1
//#define LW 2
//#define SW 3
//#define BEQ 4
//#define J 5
#define EOP_TYPE 6


// OPCODES
#define SPECIAL 0 // 000000
#define ADD 32    // 100000
#define ADDI 8    // 001000
#define LW 35     // 100011
#define SW 43     // 101011
#define BEQ  4    // 000100
#define J 2       // 000010
#define SLT 42    // 101010
#define EOP 63    // 111111
#define ADDU 33   // 100001


void FSM()
{
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;

    //reset control signals
    memset(control, 0, (sizeof(struct ctrl_signals)));
    int opcode = arch_state.IR_meta.opcode;
    //printf("\n！！！FSM opcode: %d ********************************\n\n", opcode);
    int state = arch_state.state;
    //printf("！！！FSM state: %d \n", state);
    switch (state) {
        case INSTR_FETCH:
            control->MemRead = 1;
            control->ALUSrcA = 0;
            control->IorD = 0;
            control->IRWrite = 1;
            control->ALUSrcB = 1;
            control->ALUOp = 0;
            control->PCWrite = 1;
            control->PCSource = 0;
            state = DECODE;
            break;
        case DECODE:
            control->ALUSrcA = 0;
            control->ALUSrcB = 3;
            control->ALUOp = 0;
            if (opcode == SPECIAL) state = EXEC;
            if (opcode == ADDI) state = I_TYPE_EXEC;
            if (opcode == LW || opcode == SW) state = MEM_ADDR_COMP;
            if (opcode == BEQ) state = BRANCH_COMPL;
            if (opcode == J) state = JUMP_COMPL;
            else if (opcode == EOP) state = EXIT_STATE;
            //else assert(false);
            break;
        case EXEC:
            control->ALUSrcA = 1;
            control->ALUSrcB = 0;
            control->ALUOp = 2;
            state = R_TYPE_COMPL;
            break;
        case R_TYPE_COMPL:
            control->RegDst = 1;
            control->RegWrite = 1;
            control->MemtoReg = 0;
            state = INSTR_FETCH;
            break;
        case I_TYPE_EXEC:
            control->ALUSrcA = 1;
            control->ALUSrcB = 2;
            control->ALUOp = 0;
            state = I_TYPE_COMPL;
            break;
        case I_TYPE_COMPL:
            control->RegDst = 0;
            control->RegWrite = 1;
            control->MemtoReg = 0;
            state = INSTR_FETCH;
            break;
        case MEM_ADDR_COMP:
            control->ALUSrcA = 1;
            control->ALUSrcB = 2;
            control->ALUOp = 0;
            if (opcode == LW) state = MEM_ACCESS_LD;
            else if (opcode == SW) state = MEM_ACCESS_ST;
            else if (opcode == EOP) state = EXIT_STATE;
            else assert(false);
            break;
        case MEM_ACCESS_LD:
            control->MemRead = 1;
            control->IorD = 1;
            state = WB_STEP;
            break;
        case MEM_ACCESS_ST:
            control->MemWrite = 1;
            control->IorD = 1;
            state = INSTR_FETCH;
            break;
        case WB_STEP:
            control->RegDst = 0;
            control->RegWrite = 1;
            control->MemtoReg = 1;
            state = INSTR_FETCH;
            break;
        case BRANCH_COMPL:
            control->ALUSrcA = 1;
            control->ALUSrcB = 0;
            control->ALUOp = 1;
            control->PCWriteCond = 1;
            control->PCSource = 1;
            state = INSTR_FETCH;
            break;
        case JUMP_COMPL:
            control->PCWrite = 1;
            control->PCSource = 2;
            state = INSTR_FETCH;
            break;
        default: assert(false);
    }
    arch_state.state = state;
}


void instruction_fetch()
{
    if (arch_state.control.MemRead && arch_state.state != 3 && arch_state.state != 4) {
        int address = arch_state.curr_pipe_regs.pc;
        //printf("！！！instruction_fetch address: %d \n", address);
        arch_state.next_pipe_regs.IR = memory_read(address);
        //printf("！！！instruction_fetch arch_state.next_pipe_regs.IR: %d \n", arch_state.next_pipe_regs.IR);
    }
}

void decode_and_read_RF()
{   unsigned functopPart	= 0x0000003f;
	unsigned jsecPart	    = 0x03ffffff;
    arch_state.IR_meta.opcode  = (arch_state.next_pipe_regs.IR >> 26)   & functopPart;
    arch_state.IR_meta.function  = arch_state.next_pipe_regs.IR  & functopPart;
    unsigned rPart	    = 0x1f;
    arch_state.IR_meta.reg_11_15 = (arch_state.next_pipe_regs.IR >> 11)   & rPart;
    arch_state.IR_meta.reg_21_25 = (arch_state.next_pipe_regs.IR >> 21) & rPart;
	unsigned offsetPart	= 0x0000ffff;
    arch_state.IR_meta.immediate = arch_state.next_pipe_regs.IR  & offsetPart;
    int read_register_1 = arch_state.IR_meta.reg_21_25;
    //printf("！！！decode_and_read_RF read_register_1: %d \n", read_register_1);
    arch_state.IR_meta.reg_16_20 = (arch_state.next_pipe_regs.IR >> 16) & rPart;
    int read_register_2 = arch_state.IR_meta.reg_16_20;
    //printf("！！！decode_and_read_RF read_register_2: %d \n", read_register_2);
    check_is_valid_reg_id(read_register_1);
    check_is_valid_reg_id(read_register_2);
    arch_state.next_pipe_regs.A = arch_state.registers[read_register_1];
    //printf("！！！decode_and_read_RF arch_state.next_pipe_regs.A: %d \n", arch_state.next_pipe_regs.A);
    arch_state.next_pipe_regs.B = arch_state.registers[read_register_2];
    //printf("！！！decode_and_read_RF arch_state.next_pipe_regs.B: %d \n",  arch_state.next_pipe_regs.B);
}

void execute()
{
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct pipe_regs *curr_pipe_regs = &arch_state.curr_pipe_regs;
    struct pipe_regs *next_pipe_regs = &arch_state.next_pipe_regs;

    //if (arch_state.state == EXEC || arch_state.state == I_TYPE_EXEC 
    //|| arch_state.state == MEM_ADDR_COMP){
        //printf("！！！control->ALUSrcA: %d \n",  control->ALUSrcA);
        int alu_opA = control->ALUSrcA == 1 ? curr_pipe_regs->A : curr_pipe_regs->pc; 
        //ALU mux 1: RegA or PC curr_pipe_regs->A
        //printf("！！！execute alu_opA: %d \n",  alu_opA);
        int alu_opB = 0;
        //printf("！！！execute alu_opB: %d \n",  alu_opB);
        int immediate = IR_meta->immediate;
        //printf("！！！execute immediate: %d \n",  immediate);
        int shifted_immediate = (immediate) << 2;
        //printf("！！！execute shifted_immediate: %d \n",  shifted_immediate);
        switch (control->ALUSrcB) { //ALU mux 2
            case 0: // RegB
                alu_opB = curr_pipe_regs->B;
                //printf("！！！execute alu_opB RegB: %d \n",  alu_opB);
                break;
            case 1: //+4
                alu_opB = WORD_SIZE;
                //printf("！！！execute alu_opB +4: %d \n",  alu_opB);
                break;
            case 2: //sign-extended immediate
                alu_opB = immediate;
                //printf("！！！execute alu_opB sign-extended immediate: %d \n",  alu_opB);
                break;
            case 3: //sign-extended shifted immediate
                alu_opB = shifted_immediate;
                //printf("！！！execute alu_opB sign-extended shifted immediate: %d \n",  alu_opB);
                break;
            default:
                assert(false);
        }


        switch (control->ALUOp) {
            case 0://ALU performs Add
                //printf("！！！AT THIS POINT ALU_OPA IS: %d \n",  alu_opA);
                //printf("！！！AT THIS POINT ALU_OPB IS: %d \n",  alu_opB);
                next_pipe_regs->ALUOut = alu_opA + alu_opB;
                //printf("！！！execute next_pipe_regs->ALUOut ALU performs Add case 0: %d \n",  next_pipe_regs->ALUOut);
                break;
            case 1://ALU performs Subtract
                next_pipe_regs->ALUOut = alu_opA - alu_opB;
                //printf("！！！execute next_pipe_regs->ALUOut ALU performs Subract case 1: %d \n",  next_pipe_regs->ALUOut);
                break;
            case 2://ALU action determined by FUNCT field
                //printf("！！！IR_meta->function case 2: %d \n",  IR_meta->function);
                if (IR_meta->function == ADD){
                    next_pipe_regs->ALUOut = alu_opA + alu_opB;
                    //printf("！！！execute next_pipe_regs->ALUOut ALU action determined by FUNCT field case 2: %d \n",  next_pipe_regs->ALUOut);
                }
                if (IR_meta->opcode == ADDI){
                    next_pipe_regs->ALUOut = alu_opA + alu_opB;
                    //printf("！！！execute: %d next_pipe_regs->ALUOut ALU action determined by FUNCT field",  next_pipe_regs->ALUOut);
                }
                if (IR_meta->opcode == ADDU){
                    next_pipe_regs->ALUOut = alu_opA + alu_opB;
                    //printf("！！！execute: %d next_pipe_regs->ALUOut ALU action determined by FUNCT field",  next_pipe_regs->ALUOut);
                }
                //else assert(false); */
                break;
            default:
                assert(false);
        }

        // PC calculation
        
        switch (control->PCSource) {
            case 0:
                next_pipe_regs->pc = next_pipe_regs->ALUOut;
                //printf("！！！execute next_pipe_regs->pc PC calculation case 0: %d \n",  next_pipe_regs->pc);
                break;
            case 1:
                if (arch_state.control.PCWriteCond)
                    if((next_pipe_regs->ALUOut == 0))
                        curr_pipe_regs->pc = curr_pipe_regs->ALUOut;
                break;
            case 2:
                next_pipe_regs->pc = (arch_state.IR_meta.jmp_offset << 2) + ((curr_pipe_regs->pc >> 28) << 28);
                //offset times 4 + first 4 bits of the current pc value
                //printf("！！！execute next_pipe_regs->pc PC calculation case 2: %d \n",  next_pipe_regs->pc);
                break;
            default:
                assert(false);
        }
}


void memory_access() {
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct pipe_regs *curr_pipe_regs = &arch_state.curr_pipe_regs;
    struct pipe_regs *next_pipe_regs = &arch_state.next_pipe_regs;
    if(arch_state.control.IorD && arch_state.control.MemRead) 
    {
        arch_state.next_pipe_regs.MDR = memory_read(arch_state.curr_pipe_regs.ALUOut);
        //printf("！！！memory_access next_pipe_regs->MDR : %d \n", next_pipe_regs->MDR);
    }
    if(arch_state.control.IorD && arch_state.control.MemWrite)
    {
        int write_data = arch_state.curr_pipe_regs.B;
        memory_write(arch_state.curr_pipe_regs.ALUOut, write_data);
        //printf("！！！memory_access write_data : %d \n", write_data);
    }

}

void write_back()
{
    
    if (arch_state.control.RegWrite) {
        int write_reg_id;
        if(arch_state.IR_meta.opcode == 0) {
            write_reg_id =  arch_state.IR_meta.reg_11_15;
            //printf("opcode is 0\n");
        }
        if(arch_state.IR_meta.opcode == 8) {
            write_reg_id =  arch_state.IR_meta.reg_16_20;
            //printf("opcode is 8\n");
        }
        //if(arch_state.IR_meta.opcode == 35) {
        if((arch_state.control.MemtoReg==1) && (arch_state.control.RegDst==0)) {
            write_reg_id = arch_state.IR_meta.reg_16_20;
        }
        //printf("！！！write_back write_reg_id: %d \n",  write_reg_id);
        check_is_valid_reg_id(write_reg_id);
        int write_data;
        if(arch_state.IR_meta.opcode == 0 || arch_state.IR_meta.opcode == 8){
        write_data =  arch_state.curr_pipe_regs.ALUOut;
        //printf("！！！write_back write_data: %d \n",  write_data);
        }
        if((arch_state.control.RegDst==0) && (arch_state.control.MemtoReg==1)){
            write_data =  arch_state.curr_pipe_regs.MDR; 
            //printf("！！！write_back write_data: %d \n",  write_data);
        }
        if (write_reg_id > 0 ) {
            arch_state.registers[write_reg_id] = write_data;
            //printf("***********Reg $%u = %d ************** \n", write_reg_id, write_data);
        } else printf("Attempting to write reg_0. That is likely a mistake \n");
    }
}


void set_up_IR_meta(int IR, struct instr_meta *IR_meta)
{
    IR_meta->opcode = get_piece_of_a_word(IR, OPCODE_OFFSET, OPCODE_SIZE);
    IR_meta->immediate = get_sign_extended_imm_id(IR, IMMEDIATE_OFFSET);
    IR_meta->function = get_piece_of_a_word(IR, 0, 6);
    IR_meta->jmp_offset = get_piece_of_a_word(IR, 0, 26);
    IR_meta->reg_11_15 = (uint8_t) get_piece_of_a_word(IR, 11, REGISTER_ID_SIZE);
    IR_meta->reg_16_20 = (uint8_t) get_piece_of_a_word(IR, 16, REGISTER_ID_SIZE);
    IR_meta->reg_21_25 = (uint8_t) get_piece_of_a_word(IR, 21, REGISTER_ID_SIZE);
    IR_meta->type = get_instruction_type(IR_meta->opcode);

    switch (IR_meta->opcode) {
        case SPECIAL:
            if (IR_meta->function == ADD)
                printf("Executing ADD(%d), $%u = $%u + $%u (function: %u) \n",
                       IR_meta->opcode,  IR_meta->reg_11_15, IR_meta->reg_21_25,  IR_meta->reg_16_20, IR_meta->function);
            else assert(false);
            break;
        case ADDI:
            printf("Executing ADDI \n");
            break;
        case LW:
            printf("Executing LW \n");
            break;
        case SW:
            printf("Executing LW \n");
        case J:
            printf("Executing J \n");
        case EOP:
            printf("Executing EOP(%d) \n", IR_meta->opcode);
        case BEQ:
            printf("Executing BEQ \n");
            break;
        default: assert(false);
    }
}

void assign_pipeline_registers_for_the_next_cycle()
{
    struct ctrl_signals *control = &arch_state.control;
    struct instr_meta *IR_meta = &arch_state.IR_meta;
    struct pipe_regs *curr_pipe_regs = &arch_state.curr_pipe_regs;
    struct pipe_regs *next_pipe_regs = &arch_state.next_pipe_regs;
        //printf("!!!!!!INTO ASSIGN_PIPELINE_REGISTERS FUNCTION!!!!!! \n");
        if (control->IRWrite) {
            curr_pipe_regs->IR = next_pipe_regs->IR;
            //printf("PC %d: ", curr_pipe_regs->pc / 4);
            set_up_IR_meta(curr_pipe_regs->IR, IR_meta);
        }
        curr_pipe_regs->ALUOut = next_pipe_regs->ALUOut;
        curr_pipe_regs->A = next_pipe_regs->A;
        curr_pipe_regs->B = next_pipe_regs->B;
        if (control->PCWrite) {
            check_address_is_word_aligned(next_pipe_regs->pc);
            curr_pipe_regs->pc = next_pipe_regs->pc;
        }
}


int main(int argc, const char* argv[])
{
    /*--------------------------------------
    /------- Global Variable Init ----------
    /--------------------------------------*/
    parse_arguments(argc, argv);
    arch_state_init(&arch_state);
    ///@students WARNING: Do NOT change/move/remove main's code above this point!
    while (true) {

        ///@students: Fill/modify the function bodies of the 7 functions below,
        /// Do NOT modify the main() itself, you only need to
        /// write code inside the definitions of the functions called below.

        FSM();

        instruction_fetch();

        decode_and_read_RF();

        execute();

        memory_access();

        write_back();

        assign_pipeline_registers_for_the_next_cycle();


       ///@students WARNING: Do NOT change/move/remove code below this point!
        marking_after_clock_cycle();
        arch_state.clock_cycle++;
        // Check exit statements
        if (arch_state.state == EXIT_STATE) { // I.E. EOP instruction!
            printf("Exiting because the exit state was reached \n");
            break;
        }
        if (arch_state.clock_cycle == BREAK_POINT) {
            printf("Exiting because the break point (%u) was reached \n", BREAK_POINT);
            break;
        }
    }
    marking_at_the_end();
}

