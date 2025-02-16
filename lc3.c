#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
#include <Windows.h>
#include <conio.h>

#define PC_START 0x3000
#define MEMORY_MAX (1 << 16)

//--------------------------
// MEMORY STORAGE
//--------------------------

uint16_t memory[MEMORY_MAX];

//--------------------------
// REGISTER STORAGE
//--------------------------

enum {
    R_R0 = 0, /* global registers from R0 to R7 */
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC, /* program count register */
    R_COND, /* condition flags register */
    R_COUNT /* number of all registers */
};

uint16_t reg[R_COUNT];

//--------------------------
// MEMORY MAPPED REGISTERS
//--------------------------

enum
{
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02  /* keyboard data */
};

//--------------------------
// CONDITION FLAGS
//--------------------------

enum {
    FL_POS = 1 << 0, /* p - positive */
    FL_ZRO = 1 << 1, /* z - zero */
    FL_NEG = 1 << 2, /* n - negative */
};

//--------------------------
// OPCODES
//--------------------------

enum {
    OP_BR = 0, /* branch */
    OP_ADD,    /* add  */
    OP_LD,     /* load */
    OP_ST,     /* store */
    OP_JSR,    /* jump register (jump subroutine) */
    OP_AND,    /* bitwise and */
    OP_LDR,    /* load register */
    OP_STR,    /* store register */
    OP_RTI,    /* unused */
    OP_NOT,    /* bitwise not */
    OP_LDI,    /* load indirect */
    OP_STI,    /* store indirect */
    OP_JMP,    /* jump */
    OP_RES,    /* reserved (unused) */
    OP_LEA,    /* load effective address */
    OP_TRAP    /* execute trap */
};

//--------------------------
// TRAP CODES
//--------------------------

enum
{
    TRAP_GETC = 0x20,  /* get character from keyboard, not echoed onto the terminal */
    TRAP_OUT = 0x21,   /* output a character */
    TRAP_PUTS = 0x22,  /* output a word string */
    TRAP_IN = 0x23,    /* get character from keyboard, echoed onto the terminal */
    TRAP_PUTSP = 0x24, /* output a byte string */
    TRAP_HALT = 0x25   /* halt the program */
};


//------------------------------------------------------------------------------


void load_arguments(int argc, const char *argv[]);
void read_image_file(FILE* file);
int read_image(const char* image_path);
void handle_interrupt(int signal);
uint16_t sign_extend(uint16_t x, int bit_count);
void set_condition_flags(uint16_t r);
uint16_t swap16(uint16_t x);
void mem_write(uint16_t address, uint16_t value);
uint16_t mem_read(uint16_t address);
void disable_input_buffering();
void restore_input_buffering();
uint16_t check_key();


HANDLE hStdin = INVALID_HANDLE_VALUE;
DWORD fdwMode, fdwOldMode;


int main(int argc, const char *argv[]) {

  load_arguments(argc, argv);
  signal(SIGINT, handle_interrupt);
  disable_input_buffering();

  reg[R_COND] = FL_ZRO;
  reg[R_PC] = PC_START;

  bool running = true;
  while (running) {

    uint16_t instruction = mem_read(reg[R_PC]);
    reg[R_PC]++;
    uint16_t opcode = instruction >> 12;

    switch (opcode) {

      case OP_ADD:
        {
          uint16_t DR = (instruction >> 9) & 0x7;
          uint16_t SR1 = (instruction >> 6) & 0x7;
          uint16_t immediate_mode = (instruction >> 5) & 0x1;

          if (immediate_mode) {
            uint16_t imm5 = sign_extend(instruction & 0x1F, 5);
            reg[DR] = reg[SR1] + imm5; 
          }
          else {
            uint16_t SR2 = instruction & 0x7;
            reg[DR] = reg[SR1] + reg[SR2];
          }

          set_condition_flags(DR);
        }
        break;

      case OP_AND:
        {
          uint16_t DR = (instruction >> 9) & 0x7;
          uint16_t SR1 = (instruction >> 6) & 0x7;
          uint16_t immediate_mode = (instruction >> 5) & 0x1;

          if (immediate_mode) {
            uint16_t imm5 = sign_extend(instruction & 0x1F, 5);
            reg[DR] = reg[SR1] & imm5;
          }
          else {
            uint16_t SR2 = instruction & 0x7;
            reg[DR] = reg[SR1] & reg[SR2];
          }
          
          set_condition_flags(DR);
        }
        break;

      case OP_BR:
        {
          uint16_t tested = (instruction >> 9) & 0x7;
          uint16_t pc_offset = sign_extend(instruction & 0x1FF, 9);

          if (tested & reg[R_COND]) {
            reg[R_PC] += pc_offset;
          }

        }
        break;

      case OP_JMP:
        {
          uint16_t BaseR = (instruction >> 6) & 0x7;
          reg[R_PC] = reg[BaseR];
        }
        break;

      case OP_JSR:
        {
          reg[R_R7] = reg[R_PC];
          uint16_t flag = (instruction >> 11) & 0x1;

          if (flag) {
            uint16_t pc_offset = sign_extend(instruction & 0x7FF, 11);
            reg[R_PC] += pc_offset;
          }
          else {
            uint16_t BaseR = (instruction >> 6) & 0x7;
            reg[R_PC] = reg[BaseR];
          }
        }
        break;

      case OP_LD:
        {
          uint16_t pc_offset = sign_extend(instruction & 0x1FF, 9);
          uint16_t DR = (instruction >> 9) & 0x7;

          reg[DR] = mem_read(reg[R_PC] + pc_offset);

          set_condition_flags(DR);
        }
        break;
        
      case OP_LDI:
        {
          uint16_t pc_offset = sign_extend(instruction & 0x1FF, 9);
          uint16_t DR = (instruction >> 9) & 0x7;

          reg[DR] = mem_read(mem_read(reg[R_PC] + pc_offset));

          set_condition_flags(DR);
        }
        break;

      case OP_LDR:
        {
          uint16_t offset = sign_extend(instruction & 0x3F, 6);
          uint16_t BaseR = (instruction >> 6) & 0x7;
          uint16_t DR = (instruction >> 9) & 0x7;

          reg[DR] = mem_read(reg[BaseR] + offset);

          set_condition_flags(DR);
        }
        break;

      case OP_LEA:
        {
          uint16_t pc_offset = sign_extend(instruction & 0x1FF, 9);
          uint16_t DR = (instruction >> 9) & 0x7;
          
          reg[DR] =  reg[R_PC] + pc_offset;

          set_condition_flags(DR);
        }
        break;

      case OP_NOT:
        {
          uint16_t SR = (instruction >> 6) & 0x7;
          uint16_t DR = (instruction >> 9) & 0x7;

          reg[DR] = ~reg[SR];

          set_condition_flags(DR);
        }
        break;

      case OP_ST:
        {
          uint16_t SR = (instruction >> 9) & 0x7;
          uint16_t pc_offset = sign_extend(instruction & 0x1FF, 9);

          mem_write(reg[R_PC] + pc_offset, reg[SR]);
        }
        break;

      case OP_STI:
        {
          uint16_t SR = (instruction >> 9) & 0x7;
          uint16_t pc_offset = sign_extend(instruction & 0x1FF, 9);

          mem_write(mem_read(reg[R_PC] + pc_offset), reg[SR]);
        }
        break;

      case OP_STR:
        {
          uint16_t SR = (instruction >> 9) & 0x7;
          uint16_t BaseR = (instruction >> 6) & 0x7;
          uint16_t offset = sign_extend(instruction & 0x3F, 6);

          mem_write(reg[BaseR] + offset, reg[SR]);
        }
        break;

      case OP_TRAP:
        {
          reg[R_R7] = reg[R_PC];
          uint16_t trapvector = instruction & 0xFF;

          switch (trapvector) {

            case TRAP_GETC:
              {
                char c = getchar();
                reg[R_R0] = (uint16_t) c;
                set_condition_flags(R_R0);
              }
              break;

            case TRAP_OUT:
              {
                uint16_t c = reg[R_R0] & 0xFF;
                putc((char) c, stdout);
                fflush(stdout);
              }
              break;

            case TRAP_PUTS:
              {
                uint16_t* c = memory + reg[R_R0];
                while (*c)
                {
                    putc((char) *c, stdout);
                    c++;
                }
                fflush(stdout);
              }
              break;
            
            case TRAP_IN:
              {
                puts("LC3 | Enter a character: ");
                char c = getchar();
                putc(c, stdout);
                fflush(stdout);

                reg[R_R0] = (uint16_t) c;
                set_condition_flags(R_R0);
              }
              break;
            
            case TRAP_PUTSP:
              {
                uint16_t* c = memory + reg[R_R0];
                while (*c)
                {
                    char c1 = (*c) & 0xFF;
                    char c2 = (*c) >> 8;

                    putc(c1, stdout);
                    if (c2) putc(c2, stdout);
                    c++;
                }
                fflush(stdout);
              }
              break;

            case TRAP_HALT:
              {
                running = false;
                puts("LC3 | Halt");
                fflush(stdout);
              }
              break;
            
            default:
              abort();
          }

        }
        break;

      case OP_RES:
        abort();

      case OP_RTI:
        abort();

      default:
        abort();
    }

  }

  restore_input_buffering();

}


//--------------------------
// HELPER FUNCTIONS
//--------------------------


void load_arguments(int argc, const char *argv[]) {

  if (argc < 2) {
    printf("LC3 | Usage: ./lc3 [image-file1] ...\n");
    exit(2);
  }

  for (int j = 1; j < argc; j++) {
      if (!read_image(argv[j])) {
          printf("LC3 | Couldn't find the file: %s\n", argv[j]);
          exit(1);
      }
  }

}

void handle_interrupt(int signal) {

  restore_input_buffering();
  printf("\n");
  exit(-2);

}

uint16_t sign_extend(uint16_t x, int bit_count) {

  if ((x >> (bit_count - 1)) & 1) {
      x |= (0xFFFF << bit_count);
  }
  return x;

}

void set_condition_flags(uint16_t r) {

  if (reg[r] == 0) {
      reg[R_COND] = FL_ZRO;
  }
  else if (reg[r] >> 15) { /* a 1 in the left-most bit indicates negative */
      reg[R_COND] = FL_NEG;
  }
  else {
      reg[R_COND] = FL_POS;
  }

}

int read_image(const char* image_path) {

  FILE* file = fopen(image_path, "rb");
  if (!file) return 0;

  read_image_file(file);
  fclose(file);
  return 1;

}

void read_image_file(FILE* file) {

  uint16_t origin;
  fread(&origin, sizeof(uint16_t), 1, file);
  origin = swap16(origin);

  uint16_t max_read = MEMORY_MAX - origin;
  uint16_t* p = memory + origin;
  size_t read = fread(p, sizeof(uint16_t), max_read, file);

  while (read > 0) {
      *p = swap16(*p);
      p++;
      read--;
  }

}

uint16_t swap16(uint16_t x) {
  
  return (x << 8) | (x >> 8);

}

void mem_write(uint16_t address, uint16_t value) {

  memory[address] = value;

}

uint16_t mem_read(uint16_t address) {

  if (address == MR_KBSR) {
      if (check_key()) {
          memory[MR_KBSR] = (1 << 15);
          memory[MR_KBDR] = getchar();
      }
      else {
          memory[MR_KBSR] = 0;
      }
  }
  return memory[address];

}

void disable_input_buffering() {

  hStdin = GetStdHandle(STD_INPUT_HANDLE);
  GetConsoleMode(hStdin, &fdwOldMode);
  fdwMode = fdwOldMode
          ^ ENABLE_ECHO_INPUT
          ^ ENABLE_LINE_INPUT; 

  SetConsoleMode(hStdin, fdwMode);
  FlushConsoleInputBuffer(hStdin);

}

void restore_input_buffering() {

  SetConsoleMode(hStdin, fdwOldMode);

}

uint16_t check_key() {

  return WaitForSingleObject(hStdin, 1000) == WAIT_OBJECT_0 && _kbhit();

}