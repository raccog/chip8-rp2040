#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"

#include "ssd1306.h"

#define FLASH_TARGET_OFFSET (256 * 1024)

#define DISPLAY_WIDTH 64
#define DISPLAY_HEIGHT 32
#define DISPLAY_SIZE DISPLAY_WIDTH * DISPLAY_HEIGHT

#define ONE_HERTZ 1000000
#define CLOCK_CYCLE_LENGTH 2000

#define ROM_START 0x200
#define MEMORY_SIZE 0x1000
#define STACK_SIZE 0x100
#define VARIABLE_COUNT 0x10
#define KEY_COUNT 0x10
#define FONT_START 0x50
#define FONT_SIZE 0x50
#define FONT_CHAR_SIZE 0x5

#define N_MASK 0b10000000
#define NN_MASK 0b01000000
#define NNN_MASK 0b00100000
#define X_MASK 0b00010000
#define Y_MASK 0b00001000
#define NONE_MASK 0

#define VX processor->V[get_X(instruction)]
#define VY processor->V[get_Y(instruction)]

typedef uint8_t byte;
typedef uint16_t word;
typedef uint32_t dword;
typedef uint64_t qword;

#define CLEAR_SCREEN 0x00e0
#define RETURN 0x00ee
#define JUMP 0x1000
#define CALL 0x2000
#define SKIP_EQ_NN 0x3000
#define SKIP_NEQ_NN 0x4000
#define SKIP_EQ_VY 0x5000
#define VX_SET_NN 0x6000
#define VX_ADD_NN 0x7000
#define VX_SET_VY 0x8000
#define BIT_OR 0x8001
#define BIT_AND 0x8002
#define BIT_XOR 0x8003
#define VX_ADD_VY 0x8004
#define VX_SUB_VY 0x8005
#define VX_SHIFT_RIGHT 0x8006
#define VY_SUB_VX 0x8007
#define VX_SHIFT_LEFT 0x800e
#define SKIP_NEQ_VY 0x9000
#define I_SET_NNN 0xa000
#define JUMP_SKIP 0xb000
#define VX_RAND_NN 0xc000
#define DRAW 0xd000
#define KEY_PRESSED 0xe09e
#define KEY_RELEASED 0xe0a1
#define VX_SET_DELAY 0xf007
#define WAIT_KEY_PRESS 0xf00a
#define DELAY_SET_VX 0xf015
#define SOUND_SET_VX 0xf018
#define I_ADD_VX 0xf01e
#define I_SET_CHAR 0xf029
#define BCD 0xf033
#define STORE_MEMORY 0xf055
#define LOAD_MEMORY 0xf065

const uint8_t *flash_target_contents = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

const uint8_t GPIO_KEYS[] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
const uint8_t GPIO_KEY_MAP[] = {0, 0, 0, 0, 0xa, 0x0, 0x7, 0x8, 0x4, 0x5, 0x1, 0x2, 0x3, 0xc, 0x6, 0xd, 0x9, 0xe, 0xb, 0xf};

typedef struct {
    byte memory[MEMORY_SIZE];
    byte display[DISPLAY_SIZE];
    byte full_display[DISPLAY_SIZE * 4];
    word PC;
    word I;
    byte V[VARIABLE_COUNT];
    word stack[STACK_SIZE];
    byte SP;
    byte delay;
    byte sound;
    bool key_buffer[KEY_COUNT];
    uint64_t key_last_irq[KEY_COUNT];
    bool keys[KEY_COUNT];
    bool should_draw;
    uint64_t last_cycle_time;
    uint64_t last_timer_decrement_time;
} Processor;

typedef struct {
    uint16_t size;
    uint8_t title_size;
    char *title;
    uint8_t *rom;
} Rom;

typedef struct {
    uint16_t rom_count;
    Rom *roms;
} RomList;

RomList read_metadata_uart() {
    RomList metadata;
    metadata.roms = malloc(sizeof(Rom));
    uart_read_blocking(uart0, (uint8_t *)&metadata.rom_count, 2);
    uart_read_blocking(uart0, (uint8_t *)&metadata.roms->size, 2);
    uart_read_blocking(uart0, &metadata.roms->title_size, 1);
    metadata.roms->title = malloc(metadata.roms->title_size + 1);
    metadata.roms->rom = malloc(metadata.roms->size);
    uart_read_blocking(uart0, metadata.roms->title, metadata.roms->title_size);
    *(metadata.roms->title + metadata.roms->title_size) = '\0';
    uart_read_blocking(uart0, metadata.roms->rom, metadata.roms->size);
    return metadata;
}

word get_opcode(word instruction) {
    switch (instruction >> 12) {
        case 0: 
            return instruction;
        case 0xe: case 0xf:
            return instruction & 0xf0ff;
        case 0x5: case 0x8: case 0x9:
            return instruction & 0xf00f;
        default:
            return instruction & 0xf000;
    }
}

void assert_operand_exists(word instruction, byte operand_mask) {
#ifdef CHIP8_DEBUG
    byte valid_operands;
    switch (get_opcode(instruction)) {
        case KEY_PRESSED:
        case KEY_RELEASED:
        case DELAY_SET_VX:
        case VX_SET_DELAY:
        case WAIT_KEY_PRESS:
        case SOUND_SET_VX:
        case I_ADD_VX:
        case I_SET_CHAR:
        case BCD:
        case STORE_MEMORY:
        case LOAD_MEMORY:
            valid_operands = X_MASK;
            break;
        case I_SET_NNN:
        case CALL:
        case JUMP:
        case JUMP_SKIP:
            valid_operands = NNN_MASK;
            break;
        case VX_SET_NN:
        case VX_ADD_NN:
        case SKIP_EQ_NN:
        case SKIP_NEQ_NN:
        case VX_RAND_NN:
            valid_operands = X_MASK | NN_MASK;
            break;
        case DRAW:
            valid_operands = X_MASK | Y_MASK | N_MASK;
            break;
        case SKIP_EQ_VY:
        case VX_SET_VY:
        case BIT_OR:
        case BIT_AND:
        case BIT_XOR:
        case VX_ADD_VY:
        case VX_SUB_VY:
        case VX_SHIFT_RIGHT:
        case VY_SUB_VX:
        case VX_SHIFT_LEFT:
        case SKIP_NEQ_VY:
            valid_operands = X_MASK | Y_MASK;
            break;
        case CLEAR_SCREEN:
        case RETURN:
        default:
            valid_operands = NONE_MASK;
    }
    assert((valid_operands & operand_mask) == operand_mask);
#endif
}

byte get_N(word instruction) {
    assert_operand_exists(instruction, N_MASK);
    return instruction & 0xf;
}

byte get_NN(word instruction) {
    assert_operand_exists(instruction, NN_MASK);
    return instruction & 0xff;
}

word get_NNN(word instruction) {
    assert_operand_exists(instruction, NNN_MASK);
    return instruction & 0xfff;
}

byte get_X(word instruction) {
    assert_operand_exists(instruction, X_MASK);
    return (instruction >> 8) & 0xf;
}

byte get_Y(word instruction) {
    assert_operand_exists(instruction, Y_MASK);
    return (instruction >> 4) & 0xf;
}

void processor_load_font(Processor *processor) {
    const byte FONT[] = {
        0xf0, 0x90, 0x90, 0x90, 0xf0, // 0
        0x20, 0x60, 0x20, 0x20, 0x70, // 1
        0xf0, 0x10, 0xf0, 0x80, 0xf0, // 2
        0xf0, 0x10, 0xf0, 0x10, 0xf0, // 3
        0x90, 0x90, 0xf0, 0x10, 0x10, // 4
        0xf0, 0x80, 0xf0, 0x10, 0xf0, // 5
        0xf0, 0x80, 0xf0, 0x90, 0xf0, // 6
        0xf0, 0x10, 0x20, 0x40, 0x40, // 7
        0xf0, 0x90, 0xf0, 0x90, 0xf0, // 8
        0xf0, 0x90, 0xf0, 0x10, 0xf0, // 9
        0xf0, 0x90, 0xf0, 0x90, 0x90, // A
        0xe0, 0x90, 0xe0, 0x90, 0xe0, // B
        0xf0, 0x80, 0x80, 0x80, 0xf0, // C
        0xe0, 0x90, 0x90, 0x90, 0xe0, // D
        0xf0, 0x80, 0xf0, 0x80, 0xf0, // E
        0xf0, 0x80, 0xf0, 0x80, 0x80  // F
    };
    memcpy(processor->memory + FONT_START, FONT, FONT_SIZE);
}

void processor_clear_displays(Processor *processor) {
    memset(processor->display, 0, DISPLAY_SIZE);
    memset(processor->full_display, 0, DISPLAY_SIZE * 4);
}

void processor_reset(Processor *processor) {
    memset(processor->memory, 0, MEMORY_SIZE);
    memset(processor->V, 0, VARIABLE_COUNT);
    memset(processor->stack, 0, STACK_SIZE);
    memset(processor->keys, 0, KEY_COUNT);
    processor_clear_displays(processor);
    processor->PC = ROM_START;
    processor->I = 0;
    processor->SP = 0;
    processor->delay = 0;
    processor->sound = 0;
    processor->should_draw = false;
    processor_load_font(processor);
}

Processor processor_init() {
    Processor processor;
    processor_reset(&processor);
    return processor;
}

word processor_next_instruction(Processor *processor) {
    word instruction = ((word)processor->memory[processor->PC] << 8) | processor->memory[processor->PC + 1];
    processor->PC += 2;
    return instruction;
}

void processor_update_pixel(Processor *processor, word instruction, byte x, byte y, byte pixel) {
    processor->display[VX + x + VY / 8 * DISPLAY_WIDTH] ^= pixel << y;

    for (byte i = 0; i < 2; ++i) {
        const size_t idx = (VX + x) * 2 + i + (VY + y) / 4 * DISPLAY_WIDTH * 2;
        const byte byte_shift = (VY + y) % 4 * 2;
        processor->full_display[idx] &= ~(0x3 << byte_shift);
        processor->full_display[idx] |= ((pixel << 1) | pixel) << byte_shift;
    }
}

void processor_run_instruction(Processor *processor, word instruction) {
    switch (get_opcode(instruction)) {
        case CLEAR_SCREEN:
            processor_clear_displays(processor);
            break;
        case RETURN:
            --processor->SP;
            processor->PC = processor->stack[processor->SP];
            break;
        case JUMP:
            processor->PC = get_NNN(instruction);
            break;
        case CALL:
            processor->stack[processor->SP] = processor->PC;
            ++processor->SP;
            processor-> PC = get_NNN(instruction);
            break;
        case SKIP_EQ_NN:
            if (VX == get_NN(instruction)) processor->PC += 2;
            break;
        case SKIP_NEQ_NN:
            if (VX != get_NN(instruction)) processor->PC += 2;
            break;
        case SKIP_EQ_VY:
            if (VX == VY) processor->PC += 2;
            break;
        case VX_SET_NN:
            VX = get_NN(instruction);
            break;
        case VX_ADD_NN:
            VX += get_NN(instruction);
            break;
        case VX_SET_VY:
            VX = VY;
            break;
        case BIT_OR:
            VX |= VY;
            break;
        case BIT_AND:
            VX &= VY;
            break;
        case BIT_XOR:
            VX ^= VY;
            break;
        case VX_ADD_VY:
            VX += VY;
            break;
        case VX_SUB_VY:
            VX -= VY;
            break;
        case VX_SHIFT_RIGHT:
            VX >>= VY;
            break;
        case VY_SUB_VX:
            VY -= VX;
            break;
        case VX_SHIFT_LEFT:
            VX <<= VY;
            break;
        case SKIP_NEQ_VY:
            if (VX != VY) processor->PC += 2;
            break;
        case I_SET_NNN:
            processor->I = get_NNN(instruction);
            break;
        case JUMP_SKIP:
            processor->PC = get_NN(instruction) + processor->V[0];
            break;
        case VX_RAND_NN:
            VX = get_NN(instruction) & (rand() % 0xff);
            break;
        case DRAW:
            for (byte y = 0; y < get_N(instruction); ++y) {
                for (byte x = 0; x < 8; ++x) {
                    const byte pixel = (processor->memory[processor->I + y] >> (7 - x)) & 0x1;
                    processor_update_pixel(processor, instruction, x, y, pixel);
                }
            }
            processor->should_draw = true;
            break;
        case KEY_PRESSED:
            if (!processor->keys[VX]) processor->PC += 2;
            break;
        case KEY_RELEASED:
            if (processor->keys[VX]) processor->PC += 2;
            break;
        case VX_SET_DELAY:
            VX = processor->delay;
            break;
        case WAIT_KEY_PRESS:
            break;
        case DELAY_SET_VX:
            processor->delay = VX;
            break;
        case SOUND_SET_VX:
            processor->sound = VX;
            break;
        case I_ADD_VX:
            processor->I += VX;
            break;
        case I_SET_CHAR:
            processor->I = VX * FONT_CHAR_SIZE + FONT_START;
            break;
        case BCD:
            processor->memory[processor->I] = VX / 100;
            processor->memory[processor->I + 1] = (VX / 10) % 10;
            processor->memory[processor->I + 3] = VX % 10;
            break;
        case STORE_MEMORY:
            memcpy(processor->memory + processor->I, processor->V, VARIABLE_COUNT);
            break;
        case LOAD_MEMORY:
            memcpy(processor->V, processor->memory + processor->I, VARIABLE_COUNT);
            break;
        default:
            printf("Unknown instruction at address 0x%x: 0x%x\n", processor->PC - 2, instruction);
            break;
    }
}

void processor_decrement_timers(Processor *processor) {
    uint64_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - processor->last_timer_decrement_time > ONE_HERTZ) {
        processor->last_timer_decrement_time = current_time;
        if (processor->delay > 0) {
            --processor->delay;
        }
        if (processor->sound > 0) {
            --processor->sound;
        }
    }
}

void processor_update_keys(Processor *processor) {
    for (uint i = 0; i < KEY_COUNT; ++i) gpio_set_irq_enabled(GPIO_KEYS[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    for (uint i = 0; i < KEY_COUNT; ++i) {
        uint gpio = GPIO_KEYS[i];
        uint key = GPIO_KEY_MAP[gpio];
        uint gpio_state = gpio_get(gpio);
        if (gpio_state != processor->key_buffer[key]) {
            processor->key_buffer[key] = gpio_state;
        }
    }
    memcpy(processor->keys, processor->key_buffer, KEY_COUNT);
    for (uint i = 0; i < KEY_COUNT; ++i) gpio_set_irq_enabled(GPIO_KEYS[i], GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void processor_run_cycle(Processor *processor) {
    processor->last_cycle_time = to_us_since_boot(get_absolute_time());
    processor_update_keys(processor);
    const word instruction = processor_next_instruction(processor);
    processor_run_instruction(processor, instruction);
    processor_decrement_timers(processor);
#ifdef CHIP8_DEBUG
    printf("Instruction ran at 0x%x: 0x%x\n", processor->PC - 2, instruction);
#endif
}

Processor processor;

void gpio_irq_callback(uint gpio, uint32_t events) {
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    uint key = GPIO_KEY_MAP[gpio];
    if (events == GPIO_IRQ_EDGE_RISE) {
        processor.key_buffer[key] = true;
    } else if (events == GPIO_IRQ_EDGE_FALL) {
        processor.key_buffer[key] = false;
    }
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

int main() {
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    stdio_init_all();
    printf("UART initialized\n");

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    for (size_t i = 0; i < KEY_COUNT; ++i) {
        uint gpio = GPIO_KEYS[i];
        gpio_init(gpio);
        gpio_set_dir(gpio, GPIO_IN);
        gpio_pull_down(gpio);
        gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    }

    SSD1306 ssd;
    ssd1306_init(&ssd, 0x3c, i2c1, BLACK);
    ssd1306_set_full_rotation(&ssd, true);
    printf("Screen initialized\n");

    processor = processor_init();
    printf("Processor initialized\n");

    while (true) {
        processor_update_keys(&processor);
        sleep_us(CLOCK_CYCLE_LENGTH);
    }

/*
    RomList metadata = read_metadata_uart();
    printf("Rom count: %x\n", metadata.rom_count);
    memcpy(processor.memory + ROM_START, metadata.roms->rom, metadata.roms->size);
    uint8_t *rom = malloc(256);
    memcpy(rom, metadata.roms->rom, metadata.roms->size);
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, rom, 256);
*/
    memcpy(processor.memory + ROM_START, flash_target_contents, 132);

    while (true) {
        processor_run_cycle(&processor);
        if (processor.should_draw) {
            processor.should_draw = false;
            ssd1306_buffer_set_pixels_direct(&ssd, processor.full_display);
            ssd1306_set_pixels(&ssd);
        }

        sleep_us(CLOCK_CYCLE_LENGTH);
    }

    return 0;
}
