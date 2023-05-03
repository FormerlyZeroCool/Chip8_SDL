#include <vector>
#include <fstream>
#include <array>
#include <exception>
#include <string>
#include <iostream>
#include <cmath>
#include <random>
#include <thread>
#include <mutex>
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <string>
#include "CrossPlatform.hpp"
template <typename P1>
P1 clamp(P1 val, P1 min, P1 max)
{
    return std::min(std::max(min, val), max);
}
class ViewTransformation {
    public:
    float x_scale;
    float y_scale;
    float y_translation;
    float x_translation;
    float x_min;
    float x_max;
    float deltaX;
    float y_min;
    float y_max;
    float deltaY;
    std::array<float, 2> velocity;
    std::array<float, 2> acceleration;
    ViewTransformation(float x_scale, float y_scale, float x_translation, float y_translation): velocity {0}, acceleration {0}
    {
        this->x_scale = x_scale;
        this->y_scale = y_scale;
        this->x_translation = x_translation;
        this->y_translation = y_translation;
        recalc();
    }

    SDL_Rect SDL_view()
    {
        return {(int32_t)(x_min), (int32_t)(y_min), (int32_t)(deltaX), (int32_t)(deltaY) };
    }
    bool compare(ViewTransformation& target_bounds) 
    {
        return target_bounds.x_scale == this->x_scale && target_bounds.x_translation == this->x_translation && 
            target_bounds.y_scale == this->y_scale && target_bounds.y_translation == this->y_translation;
    }
    void update_state(float delta_time)
    {
        const float mult = delta_time / 1000;
        this->x_translation = clamp(this->x_translation + this->velocity[0] * mult, -std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        this->y_translation = clamp(this->y_translation + this->velocity[1] * mult, -std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        const float velx_bounds = this->deltaX * 10;
        const float vely_bounds = this->deltaY * 10;
        this->velocity[0] = clamp(this->velocity[0] + this->acceleration[0] * mult, -velx_bounds, velx_bounds);
        this->velocity[1] = clamp(this->velocity[1] + this->acceleration[1] * mult, -vely_bounds, vely_bounds);
        this->recalc(this->x_scale, this->y_scale, this->x_translation, this->y_translation);
    }
    void stop_motion()
    {
        this->velocity[0] = 0;
        this->velocity[1] = 0;
        this->acceleration[0] = 0;
        this->acceleration[1] = 0;
    }
    void recalc(float x_scale, float y_scale, float x_translation, float y_translation)
    {
        this->x_scale = x_scale;
        this->y_scale = y_scale;
        this->x_translation = x_translation;
        this->y_translation = y_translation;
        this->x_min = this->x_translation - 1 / this->x_scale;
        this->x_max = this->x_translation + 1 / this->x_scale;
        this->deltaX = this->x_max - this->x_min;
        this->y_min = this->y_translation - 1 / this->y_scale;
        this->y_max = this->y_translation + 1 / this->y_scale;
        this->deltaY = this->y_max - this->y_min;
    }
    void recalc()
    {
        this->recalc(x_scale, y_scale, x_translation, y_translation);
    }
    ViewTransformation& copy(ViewTransformation& other)
    {
        this->recalc(other.x_scale, other.y_scale, other.x_translation, other.y_translation);
        return *this;
    }
};
class Window {	
    SDL_Window* window;
	SDL_Renderer* sdl_renderer;
    bool alive = true;
    public:
	int32_t width, height;
    std::array<int32_t, 2> touch_pos;
    std::array<int32_t, 2> delta_touch_pos;
    Window(std::string name, int32_t width, int32_t height): width(width), height(height)
    {
        window = SDL_CreateWindow(name.c_str(), SDL_WINDOWPOS_CENTERED_DISPLAY(1), SDL_WINDOWPOS_CENTERED, width, height, SDL_RENDERER_PRESENTVSYNC);
        if(window == nullptr)
            throw std::string("Error could not create SDL window.");

        sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
        if(sdl_renderer == nullptr)
            throw std::string("Error could not initialize hardware accelerated renderer.");
    }  
    SDL_Renderer& renderer()
    {
        return *sdl_renderer;
    }
    bool is_running()
    {
        return alive;
    } 
    struct PollRec {
        SDL_Event event;
        bool success;
    };
    PollRec poll()
    {
        PollRec rec;
        rec.success = SDL_PollEvent(&rec.event);
        if(rec.event.type == SDL_QUIT)
        {
            rec.success = false;
            alive = false;
        }
        else if(rec.event.type == SDL_MOUSEMOTION)
        {
            touch_pos[0] = rec.event.motion.x;
            touch_pos[1] = rec.event.motion.y;
            delta_touch_pos[0] = rec.event.motion.xrel;
            delta_touch_pos[1] = rec.event.motion.yrel;
        }
        
        return rec;
    }
    SDL_Event wait_event()
    {
        SDL_Event event;
        SDL_WaitEvent(&event);

        if(event.type == SDL_QUIT)
        {
            alive = false;
        }
        else if(event.type == SDL_MOUSEMOTION)
        {
            touch_pos[0] = event.motion.x;
            touch_pos[1] = event.motion.y;
            delta_touch_pos[0] = event.motion.xrel;
            delta_touch_pos[1] = event.motion.yrel;
        }
        return event;
    }
};

template <typename T, size_t SIZE>
class Stack {
    std::array<T, SIZE> data;
    size_t len;
    public:
    Stack():len(0) {}
    void clear() noexcept { len = 0; }
    bool push(const T &data) noexcept
    {
        if(len < SIZE)
        {
            this->data[len++] = data;
            return true;
        }
        return false;
    }
    bool push(const T &&data) noexcept
    {
        return push(data);
    }
    T& pop() noexcept
    {
        this->len -= this->len > 0;
        return this->data[len];
    }
    const T& top() const noexcept
    {
        return this->data[len - 1];
    }
    T& top_unconst() noexcept
    {
        return this->data[len - 1];
    }
    size_t size() const noexcept
    {
        return this->len;
    }
};
union Register {
    uint8_t ui;
    int8_t i;
};


class Chip8;
void zero_op(Chip8& chip8);
void one_op(Chip8& chip8);
void two_op(Chip8& chip8);
void three_op(Chip8& chip8);
void four_op(Chip8& chip8);
void five_op(Chip8& chip8);
void six_op(Chip8& chip8);
void seven_op(Chip8& chip8);
void eight_op(Chip8& chip8);
void nine_op(Chip8& chip8);
void ten_op(Chip8& chip8);
void eleven_op(Chip8& chip8);
void twelve_op(Chip8& chip8);
void thirteen_op(Chip8& chip8);
void fourteen_op(Chip8& chip8);
void fifteen_op(Chip8& chip8);
void no_op(Chip8& chip8)
{

}
union ScreenBuf {
    std::array<bool, 64 * 32> bit_buf;
    std::array<uint8_t, 64 * 32 / 8> byte_buf;
};
class Chip8 {
private:
    std::chrono::steady_clock::time_point last_timer_tick = std::chrono::steady_clock::now();
    std::array<Register, 16> gp_registers;
    ScreenBuf screen_buf;
    std::array<uint8_t, 4096> memory;
    Stack<uint16_t, 256> call_stack;
    constexpr static const std::array<void (*)(Chip8&), 16> nibble_opcode_lut = 
        {
            zero_op, one_op, two_op, three_op, four_op,
            five_op, six_op, seven_op, eight_op, nine_op, 
            ten_op, eleven_op, twelve_op, thirteen_op, 
            fourteen_op, fifteen_op
        };
    SDL_Texture* render_buf = nullptr;
    ViewTransformation current_view;
    Window& win;

public:
    bool paused = false;
    std::array<bool, 16> key_code { false };
    uint8_t delay_timer = 0;
    uint8_t sound_timer = 0;
    static const int FONT_ADDRESS = 0x50;
    uint16_t pc;
    uint16_t I;
    size_t instructions_executed = 0;
    const int32_t width, height, ticks_per_second;
    std::mutex state_mutex;


    Chip8(Window* win, uint32_t ticks_per_second = 60): ticks_per_second(ticks_per_second), width(64), height(32), current_view(2. / win->width, 2. / win->height, win->width >> 1, win->height >> 1), 
        win(*win)
    {
        if(win)
            this->render_buf = SDL_CreateTexture(&win->renderer(), SDL_PIXELFORMAT_BGRA32, SDL_TEXTUREACCESS_STREAMING, width, height);
        constexpr const std::array<uint8_t, 16 * 5> data = {
            0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
            0x20, 0x60, 0x20, 0x20, 0x70, // 1
            0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
            0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
            0x90, 0x90, 0xF0, 0x10, 0x10, // 4
            0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
            0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
            0xF0, 0x10, 0x20, 0x40, 0x40, // 7
            0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
            0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
            0xF0, 0x90, 0xF0, 0x90, 0x90, // A
            0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
            0xF0, 0x80, 0x80, 0x80, 0xF0, // C
            0xE0, 0x90, 0x90, 0x90, 0xE0, // D
            0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
            0xF0, 0x80, 0xF0, 0x80, 0x80  // F
        };
        memcpy(&memory[Chip8::FONT_ADDRESS], &data[0], data.size());
    }
    ~Chip8()
    {
        SDL_DestroyTexture(this->render_buf);
    }
    Chip8(Chip8&) = delete;
    Chip8(Chip8&&) = delete;
    auto& registers()
    {
        return this->gp_registers;
    }
    auto& screen()
    {
        return this->screen_buf.bit_buf;
    }
    auto& screen_bytes()
    {
        return this->screen_buf.byte_buf;
    }
    auto& mem()
    {
        return this->memory;
    }
    auto& stack()
    {
        return this->call_stack;
    }
    SDL_Texture& texture()
    {
        return *render_buf;
    }
    SDL_Rect get_view()
    {
        this->current_view.recalc();
        return this->current_view.SDL_view();
    }
    std::array<float, 2> screen_to_world(std::array<int32_t, 2> coords)
    {
        return {(1.0f * coords[0] - current_view.x_min)  / current_view.deltaX * width,
                (-1.0f * coords[1] + current_view.y_min) / current_view.deltaY * height};
    }
    std::array<float, 2> world_to_screen(std::array<float, 2> coords)
    {
        return { coords[0] / width * win.width,
                coords[1] / height * win.height };
    }
    void bresenham_world_line(std::array<int32_t, 2> touch, std::array<int32_t, 2> delta)
    {
        const auto world_touch = screen_to_world({touch[0], touch[1]});
        delta[0] = 1.0 * delta[0] * width / win.width;
        delta[1] = 1.0 * delta[1] * height / win.height;
        delta[1] *= -1;
        float error = 0;
        const auto minx = std::min(world_touch[0], world_touch[0] + delta[0]);
        const auto maxx = std::max(world_touch[0], world_touch[0] + delta[0]) + 1;
        const auto miny = std::min(world_touch[1], world_touch[1] + delta[1]);
        const auto maxy = std::max(world_touch[1], world_touch[1] + delta[1]) + 1;

        if(abs(delta[0]) > abs(delta[1]))
        {
            const float m = delta[0] ? 1.0f * delta[1] / delta[0] : 0;
            int32_t y = world_touch[1];
            for(int32_t x = minx; x < maxx; x++)
            {
                y += (error > 0.5) - (error < -0.5);
                error -= (error > 0.5) - (error < -0.5);
                set_place((-y * width) + (x), true);
                error += m;
            }
        }
        else
        {
            const float m = delta[1] ? 1.0f * delta[0] / delta[1] : 0;
            int32_t x = world_touch[0];
            for(int32_t y = miny; y < maxy; y++)
            {
                x += (error > 0.5) - (error < -0.5);
                error -= (error > 0.5) - (error < -0.5);
                set_place((-y * width) + (x), true);
                error += m;
            }
        }
    }
    void set_flag(bool val)
    {
        this->gp_registers[0x0F].ui = val;
    }
    Register& rx()
    {
        return this->gp_registers[hb() & 0x0F];
    }
    Register& ry()
    {  
        return this->gp_registers[lb() >> 4];
    }
    uint8_t hb()
    {
        return this->memory[this->pc - 2];
    }
    uint8_t lb()
    {
        return this->memory[this->pc - 1];
    }
    uint16_t get_data_at_pc()
    {
        return (hb() << 8) | lb();
    }
    void tick()
    {
        if(this->paused)
            return;
        if(this->pc + 2 > this->memory.size())
            throw std::string("Invalid memory access, out of bounds");
        this->pc += 2;
        this->nibble_opcode_lut[hb() >> 4](*this);
    }
    void dec_timers()
    {
        const auto current_time = std::chrono::steady_clock::now();
        const auto delta_time = std::chrono::duration_cast<std::chrono::microseconds>(current_time - this->last_timer_tick).count();
        if(delta_time > 16666)
        {
            this->last_timer_tick = current_time;
            for(int i = delta_time / 16666; i > 0; i--)
            {
                this->delay_timer -= this->delay_timer > 0;
                this->sound_timer -= this->sound_timer > 0;
            }
        }
    }
    bool operator[](size_t index)  
    {
        if(index < field().size())
            return (field()[index]);
        else
            throw std::string("Error accessing outside bounds" + std::to_string((int64_t) index) + "\n");
    }
    void set_place(size_t index, bool value)
    {
        if(index < field().size())
            field()[index] = value;
        //else
          //  std::cerr<<std::string("Error accessing outside bounds" + std::to_string((int64_t) index));
    }
    std::array<bool, 64*32>& field()
    {
        return this->screen_buf.bit_buf;
    }
    void randomize()
    {
        for(uint64_t i = 0; i < field().size(); i++)
        {
            srand(i);
            field()[i] = random() % 100 > 60;
        }
    }
    void render_to_texture()
    {
        int32_t pitch = 0;
        int32_t *lockedPixels = nullptr;
        if(!SDL_LockTexture(render_buf, NULL, (void**)&lockedPixels, &pitch)) {
            //we might just try to ignore pitch
            uint64_t i = 0;
            const uint64_t remainder = field().size() & 7;
            const std::array<size_t, 2> colors { 0xFF0000FF, 0xFFFFFFFF };
            const auto& screen = this->screen();
            for(; i < field().size() - remainder;)
            {
                lockedPixels[i++] = colors[screen[i]];
                lockedPixels[i++] = colors[screen[i]];
                lockedPixels[i++] = colors[screen[i]];
                lockedPixels[i++] = colors[screen[i]];
                lockedPixels[i++] = colors[screen[i]];
                lockedPixels[i++] = colors[screen[i]];
                lockedPixels[i++] = colors[screen[i]];
                lockedPixels[i++] = colors[screen[i]];
            }
            while(i < field().size())
            {
                lockedPixels[i] = colors[field()[i]];
            }
            SDL_UnlockTexture(render_buf);
        }
    }
    size_t get_key_code()
    {
        uint16_t index = -1;
        do { index++; } while(index < key_code.size() && !key_code[index]);
        return key_code[index] ? index : -1;
    }
    void translate_view(float x, float y)
    {   
        current_view.x_translation += x;
        current_view.y_translation += y;
        current_view.recalc();
    }
    void set_scale(float x_scale, float y_scale, std::array<int32_t, 2> keep_in_place)
    {
        const auto old_worldPos = this->screen_to_world(keep_in_place);

        this->current_view.x_scale += x_scale * .1 / current_view.deltaX;
        this->current_view.x_scale = clamp(this->current_view.x_scale, 1.f / win.width / 10, 5.f);
        this->current_view.y_scale += y_scale * .1 / current_view.deltaY;
        this->current_view.y_scale = clamp(this->current_view.y_scale, 1.f / win.height / 10, 5.f);


        this->current_view.recalc();
        const auto new_worldPos = this->screen_to_world(keep_in_place);
        const auto screen_deltas = world_to_screen({(new_worldPos[0] - old_worldPos[0]), (new_worldPos[1] - old_worldPos[1])});

        this->current_view.x_translation += (screen_deltas[0]);
        this->current_view.y_translation -= (screen_deltas[1]);
    }
    void load_file(std::string fp)
    {
        this->screen().fill(0);
        this->memory.fill(0);
        std::fstream file(fp);
        char current;
        const int start = 0x200;
        uint16_t i = 0;
        while(file.get(current))
        {
            this->memory[start + i++] = current;
            //std::cout<<(uint16_t)current<<"\n";
        }
        this->pc = start;
    }
};
void zero_op(Chip8& chip8)
{
    if(chip8.lb() == 0xEE)
    {
        if(chip8.stack().size())
        {
            //std::cout<<"returning from proc ending at: "<<chip8.pc<<" to: "<<chip8.stack().top();
            chip8.pc = chip8.stack().pop();
        }
        else throw std::string("Exception, popping from empty callstack!");
    }
    else if(chip8.lb() == 0xE0)
    {
        std::cout<<"Clearing screen\n";
        chip8.screen().fill(false);
    }
}
void one_op(Chip8& chip8)
{
    chip8.pc = (chip8.get_data_at_pc() & 0x0FFF);
    //std::cout<<chip8.pc<<"\n";
}
void two_op(Chip8& chip8)
{
    //std::cout<<"Calling proc from: "<< chip8.pc <<"Jumping to: "<<(chip8.get_data_at_pc() & 0x0FFF)<<"\n";
    chip8.stack().push(chip8.pc);
    chip8.pc = (chip8.get_data_at_pc() & 0x0FFF);
}
void three_op(Chip8& chip8)
{
    //std::cout<<"testing rx == lb should be 2: "<<(res << 1)<<"\n";
    chip8.pc += (chip8.rx().ui == chip8.lb()) << 1;
}
void four_op(Chip8& chip8)
{
    chip8.pc += (chip8.rx().ui != chip8.lb()) << 1;
}
void five_op(Chip8& chip8)
{
    chip8.pc += (chip8.rx().ui == chip8.ry().ui) << 1;
}
void six_op(Chip8& chip8)
{
    //std::cout<<"setting register v"<<(chip8.hb() & 0x0F)<<" to: "<<((uint16_t)chip8.lb())<<" "<<"\n";
    chip8.rx().ui = chip8.lb();
}
void seven_op(Chip8& chip8)
{
    //std::cout<<"adding v"<<(chip8.hb() & 0x0F)<<" and: "<<(uint16_t) (chip8.registers()[chip8.hb() & 0x0F].ui + chip8.lb())<<"\n";
    chip8.rx().ui += chip8.lb();
}
void eight_op(Chip8& chip8)
{
    const auto old_rx = chip8.rx().ui;
    switch(chip8.lb() & 0x0F)
    {
        case(0):
            chip8.rx().ui = chip8.ry().ui;
        break;
        case(1):
            chip8.rx().ui |= chip8.ry().ui;
        break;
        case(2):
            chip8.rx().ui &= chip8.ry().ui;
        break;
        case(3):
            chip8.rx().ui ^= chip8.ry().ui;
        break;
        case(4):
            chip8.rx().ui = old_rx + chip8.ry().ui;
            chip8.set_flag((old_rx + chip8.ry().ui) > 255);
        break;
        case(5):
            chip8.rx().ui = old_rx - chip8.ry().ui;
            chip8.set_flag(old_rx > chip8.ry().ui);
        break;
        case(7):
            chip8.rx().ui = chip8.ry().ui - old_rx;
            chip8.set_flag(old_rx > chip8.rx().ui);
        break;
        case(6):
            chip8.rx().ui = old_rx >> 1;
            chip8.set_flag(old_rx & 1);
        break;
        case(0xE):
            chip8.rx().ui = old_rx << 1;
            chip8.set_flag((old_rx & 128) > 0);
        break;
    }
}
void nine_op(Chip8& chip8)
{
    chip8.pc += (chip8.rx().ui != chip8.ry().ui) << 1;
}
void ten_op(Chip8& chip8)
{
    //std::cout<<"Setting index register to: "<<(r & 0x0FFF)<<"\n";
    chip8.I = (chip8.get_data_at_pc() & 0x0FFF);
}
void eleven_op(Chip8& chip8)
{
    chip8.pc = chip8.registers()[0].ui + (chip8.get_data_at_pc() & 0x0FFF);
}
void twelve_op(Chip8& chip8)
{
    chip8.rx().ui = rand() & chip8.lb();
}
void thirteen_op(Chip8& chip8)
{
    const auto x = (chip8.rx().ui & (chip8.width - 1));
    uint16_t y = (chip8.ry().ui & (chip8.height - 1)) * chip8.width;
    const auto n = chip8.lb() & 0xF;
    //const uint8_t mask = x + 8 > 64 ? ~(1 << (x + 8 - chip8.width)) : 0;
    //std::cout<<"Rendering instruction draw: x: "<<x<<" y: "<<y<<" height: "<< n<<"\n";
    bool collision = false;
    for(int i = 0; i < n; i++, y += chip8.width)
    {
        for(int j = 0; j < 8 & j + x < chip8.width & y < chip8.screen().size(); j++)
        {
            collision = chip8.screen()[y + x + j] | collision;
            chip8.screen()[y + x + j] ^= (chip8.mem()[chip8.I + i] >> (7 - j)) & 1;
        }
    }    
    chip8.set_flag(collision);  
}
void fourteen_op(Chip8& chip8)
{
    switch(chip8.lb())
    {
        case(0x9E):
                chip8.pc += (chip8.rx().ui == chip8.get_key_code()) << 1;
            //std::cout<<"attempting to read key code: "<<(int16_t)chip8.get_key_code()<<" == "<<(uint16_t)chip8.rx().ui<<"\n";
        break;
        case(0xA1):
                chip8.pc += (chip8.rx().ui != chip8.get_key_code()) << 1;
            //std::cout<<"attempting to read key code: "<<(int16_t)chip8.get_key_code()<<" != "<<(uint16_t)chip8.rx().ui<<"\n";
        break;
    }
}
void fifteen_op(Chip8& chip8)
{
    switch(chip8.lb())
    {
        case(7):
        chip8.rx().ui = chip8.delay_timer;
        break;
        case(0x15):
        chip8.delay_timer = chip8.rx().ui;
        break;
        case(0x18):
        chip8.sound_timer = chip8.rx().ui;
        break;
        case(0x1E):
        chip8.I += chip8.rx().ui;
        break;
        case(0x0A): 
        //std::cout<<(uint16_t)chip8.key_code<<"\n";
            if(std::find(chip8.key_code.begin(), chip8.key_code.end(), true) == chip8.key_code.end())
            {
                chip8.pc -= 2;
                std::cout<<"Waiting for input at address: "<<chip8.pc<<"\n";
            }
            else
            {
                chip8.rx().ui = chip8.get_key_code();
                chip8.key_code[chip8.rx().ui] = false;
            }
        break;
        case(0x29):
            chip8.I = Chip8::FONT_ADDRESS + 5 * (chip8.rx().ui & 0x0F);
            //std::cout<<"Setting I reg to: "<<chip8.I<<" or "<<(chip8.rx().ui & 0x0F)<<" font char";
        break;
        case(0x33):
        {
            auto remaining = chip8.rx().ui;
            //std::cout<<"bin to dec conversion: "<<(uint16_t)remaining<<"\n";
            int16_t i = 0;
            chip8.mem()[chip8.I + i++] = chip8.rx().ui / 100; 
            chip8.mem()[chip8.I + i++] = chip8.rx().ui / 10 % 10; 
            chip8.mem()[chip8.I + i++] = chip8.rx().ui % 10; 
            
            //std::cout<<(uint16_t)(chip8.mem()[chip8.I])<<(uint16_t)(chip8.mem()[chip8.I + 1])<<(uint16_t)(chip8.mem()[chip8.I + 2])<<"\n";
        }
        break;
        case(0x55):
        {
            const auto limit = chip8.hb() & 0x0F;
            for(int i = 0; i <= limit; i++)
                chip8.mem()[chip8.I + i] = chip8.registers()[i].ui;
        }
        break;
        case(0x65):
        {
            const auto limit = chip8.hb() & 0x0F;
            //std::cout<<"\n";
            for(int i = 0; i <= limit; i++)
            {
                //std::cout<<(uint16_t)chip8.mem()[chip8.I + i]<<" ";
                chip8.registers()[i].ui = chip8.mem()[chip8.I + i];
            }
            //std::cout<<"\n";
        }
        break;
    }
}
uint32_t hex_to_int(char hex)
{
    hex |= 32;
    uint32_t num = hex & 15;
    num += (hex >= 'a') ? 9 : 0;
    return num;
}
uint32_t hex_to_int(const std::string_view hex)
{
    uint32_t mag = 0;
    uint32_t result = 0;
    for(int i = hex.size() - 1; i >= 0; i--)
    {
        result += hex_to_int(hex[i]) << (mag << 2);
        mag++;
    }
    return result;
}
class Chip8Verifier {
private:
    std::vector<std::unique_ptr<Chip8> > states;
    Window& win;
public:
    Chip8Verifier(std::istream& trace_file, std::string src_file_path, Window& win): win(win)
    {
        std::string line("");
        //load trace file
        while(trace_file)
        {
            std::getline(trace_file, line);
            parse_row(line);
        }
        //create chip8 instance and load binary to be tested
        Chip8 live(&win);
        live.load_file(src_file_path);

        std::cout<<"Initial state:\n";
        std::cout<<"Live:\npc:"<<live.pc<<" I: "<<live.I<<" ";
        for(int j = 0; j < live.registers().size(); j++)
        {
            std::cout<<"r"<<j<<": "<<(uint16_t)live.registers()[j].ui<<" ";
        }
        
        std::cout<<"\nCorrect:\npc: "<<states[0]->pc<<" I: "<<states[0]->I<<" ";
        for(int j = 0; j < states[0]->registers().size(); j++)
        {
            std::cout<<"r"<<j<<": "<<(uint16_t)states[0]->registers()[j].ui<<" ";
        }
        int i = 0;
        do
        {
            live.tick();
            i++;
        } while(i < this->states.size() && compare_states(live, *this->states[i].get()));
        if(i != this->states.size())
        {
            std::cout<<"\nFailed on instruction: "<<--i<<"\n";
            //exit(0);
        }
        else
            std::cout<<"\nFinished successfully on instruction: "<<--i<<"\n";
        std::cout<<"Live:\npc:"<<live.pc<<" I: "<<live.I<<" ";
        for(int j = 0; j < live.registers().size(); j++)
        {
            std::cout<<"r"<<j<<": "<<(uint16_t)live.registers()[j].ui<<" ";
        }
        
        std::cout<<"\nCorrect:\npc: "<<states[i]->pc<<" I: "<<states[i]->I<<" ";
        for(int j = 0; j < states[i]->registers().size(); j++)
        {
            std::cout<<"r"<<j<<": "<<(uint16_t)states[i]->registers()[j].ui<<" ";
        }
        std::cout<<"\n";
    }
    bool compare_states(Chip8& s1, Chip8& s2)
    {
        //compare gp reg, I, pc, and opcode
        bool valid = true;
        for(int i = 0; valid && i < s1.registers().size(); i++)
        {
            valid = s1.registers()[i].ui == s2.registers()[i].ui;
        }
        return valid & (s1.I == s2.I) & (s1.pc == s2.pc);
    }
    void parse_row(std::string& line)
    {
        //[01:0000] V0:00 V1:00 V2:00 V3:00 V4:00 V5:00 V6:00 V7:00 V8:00 V9:00 VA:00 VB:00 VC:00 VD:00 VE:00 VF:00 I:0000 SP:0 PC:0200 O:124e
        const auto start = line.find("] ") + 2;//+ size of string
        states.push_back(std::make_unique<Chip8>(&win));
        const auto& current_state = states.back();
        auto token = parse_token(line, start);
        while(token.second < line.size())
        {
            //std::cout<<token.first<<" ";
            switch(token.first[0])
            {
                case('V'):
                    parse_register(*current_state.get(), token.first);
                break;
                case('S')://sp
                    //parse_register(*current_state.get(), token.first);
                break;
                case('I'):
                    parse_index_register(*current_state.get(), token.first);
                break;
                case('P')://pc
                    parse_program_counter(*current_state.get(), token.first);
                break;
                case('O')://opcode
                    parse_opcode(*current_state.get(), token.first);
                break;
            }
            token = parse_token(line, token.second);
            //std::cout<<"\n";
        }
    }
    void parse_register(Chip8& current_state, std::string_view tok)
    {
        const char register_index = tok[tok.find("V") + 1];
        //std::cout<<"parsing reg: "<<register_index<<"\n";
        const std::string_view register_value(tok.data()  + tok.find(":") + 1, 2);
        //std::cout<<register_value<<" "<<hex_to_int(register_value)<<"\n";
        current_state.registers()[hex_to_int(register_index)].ui = hex_to_int(register_value);
    }
    void parse_index_register(Chip8& current_state, std::string_view tok)
    {
        const std::string_view number(tok.data() + 2, 4);
        //std::cout<<"parsing I: "<<tok[0]<<": "<<number<<" == "<<hex_to_int(number)<<"\n";
        current_state.I = hex_to_int(number);
    }
    void parse_program_counter(Chip8& current_state, std::string_view tok)
    {
        const std::string_view number(tok.data() + 3, 4);
        //std::cout<<"parsing pc: "<<tok[0]<<": "<<number<<" == "<<hex_to_int(number)<<"\n";
        current_state.pc = hex_to_int(number);
    }
    void parse_opcode(Chip8& current_state, std::string_view tok)
    {
        const std::string_view number(tok.data() + 2, 4);
        //std::cout<<"parsing opcode: "<<tok[0]<<": "<<number<<" == "<<hex_to_int(number)<<"\n";
        const auto num = hex_to_int(number);
        current_state.mem()[current_state.pc] = num >> 4;
        current_state.mem()[current_state.pc + 1] = num & 0x0F;
    }
    std::pair<std::string_view, uint32_t> parse_token(std::string& line, int32_t start)
    {
        while(line[start] && line[start] == ' ') { start++; }
        auto end = start + 1;
        while(line[end] && line[end] != ' ') { end++; }
        return std::make_pair(std::string_view(line.c_str() + start, end - start), end + 1);
    }
};

void tick(Chip8* con)
{
    while(true)
    {
        const auto start = std::chrono::system_clock::now();
        //blocked scope so timer includes time to lock and unlock
        {
            std::lock_guard(con->state_mutex);
            for(int i = 0; i < con->ticks_per_second / 60 + 1; i++)
                con->tick();
            con->dec_timers();
        } 
        const auto end = std::chrono::system_clock::now();
        const double max_sleep = 1000000 / (con->ticks_per_second < 60 ? con->ticks_per_second : 60);
        const auto time_elapsed = end - start;
        if(max_sleep - time_elapsed.count() > 0)
            CrossPlatform::usleep(max_sleep - time_elapsed.count());
        else
            std::cout<<"Time elapsed: "<<time_elapsed.count()/1000000.<<"\n";
    }
}
void render_loop(Chip8* con, Window* win)
{
    while(true)
    {
        const auto start = std::chrono::system_clock::now();
        {
            con->render_to_texture();
            SDL_RenderClear(&win->renderer());
            SDL_Rect view_port = con->get_view();
            SDL_RenderCopy(&win->renderer(), &con->texture(), nullptr, &view_port);
            // Update the screen
            SDL_RenderPresent(&win->renderer());
        }
        const auto end = std::chrono::system_clock::now();
        const double max_sleep = 1000000 / 120;
        const auto time_elapsed = end - start;
        CrossPlatform::usleep(max_sleep - time_elapsed.count() > 0 ? max_sleep - time_elapsed.count() : 0);
    } 
}
void event_loop(Chip8* con, Window* win, bool* mouse_down)
{
    while(true)
    {
        SDL_Event event = win->wait_event();
        std::lock_guard(con->state_mutex);
        if(!win->is_running())
            return;
        if(event.type == SDL_KEYDOWN)
        {
            switch(event.key.keysym.sym)
            {
                case(SDLK_1):
                    con->key_code[1] = true;
                break;
                case(SDLK_2):
                    con->key_code[2] = true;
                break;
                case(SDLK_3):
                    con->key_code[3] = true;
                break;
                case(SDLK_q):
                    con->key_code[4] = true;
                break;
                case(SDLK_w):
                    con->key_code[5] = true;
                break;
                case(SDLK_e):
                    con->key_code[6] = true;
                break;
                case(SDLK_a):
                    con->key_code[7] = true;
                break;
                case(SDLK_s):
                    con->key_code[8] = true;
                break;
                case(SDLK_d):
                    con->key_code[9] = true;
                break;
                case(SDLK_z):
                    con->key_code[14] = true;
                break;
                case(SDLK_x):
                    con->key_code[0] = true;
                break;
                case(SDLK_c):
                    con->key_code[15] = true;
                break;
                case(SDLK_4):
                    con->key_code[10] = true;
                break;
                case(SDLK_r):
                    con->key_code[11] = true;
                break;
                case(SDLK_f):
                    con->key_code[12] = true;
                break;
                case(SDLK_v):
                    con->key_code[13] = true;
                break;
                case(SDLK_p):
                    con->paused = !con->paused;
                break;
                case(SDLK_o):
                    const auto paused_state = con->paused;
                    con->paused = false;
                    con->tick();
                    con->paused = paused_state;
                break;
            }
        }
        else if (event.type == SDL_KEYUP)
        {    switch(event.key.keysym.sym)
            {
                case(SDLK_1):
                    con->key_code[1] = false;
                break;
                case(SDLK_2):
                    con->key_code[2] = false;
                break;
                case(SDLK_3):
                    con->key_code[3] = false;
                break;
                case(SDLK_q):
                    con->key_code[4] = false;
                break;
                case(SDLK_w):
                    con->key_code[5] = false;
                break;
                case(SDLK_e):
                    con->key_code[6] = false;
                break;
                case(SDLK_a):
                    con->key_code[7] = false;
                break;
                case(SDLK_s):
                    con->key_code[8] = false;
                break;
                case(SDLK_d):
                    con->key_code[9] = false;
                break;
                case(SDLK_z):
                    con->key_code[14] = false;
                break;
                case(SDLK_x):
                    con->key_code[0] = false;
                break;
                case(SDLK_c):
                    con->key_code[15] = false;
                break;
                case(SDLK_4):
                    con->key_code[10] = false;
                break;
                case(SDLK_r):
                    con->key_code[11] = false;
                break;
                case(SDLK_f):
                    con->key_code[12] = false;
                break;
                case(SDLK_v):
                    con->key_code[13] = false;
                break;
                case(SDLK_p):
                    con->paused = !con->paused;
                break;
                case(SDLK_o):
                    const auto paused_state = con->paused;
                    con->paused = false;
                    con->tick();
                    con->paused = paused_state;
                break;
            }
        
        }
        else if(event.type == SDL_MOUSEMOTION)
        {
            if(!*mouse_down)
            {
                //con->bresenham_world_line(win->touch_pos, win->delta_touch_pos);
            }
            else
            {
                con->translate_view(win->delta_touch_pos[0], win->delta_touch_pos[1]);
            }
        }
        else if(event.type == SDL_MOUSEBUTTONDOWN)
        {
            *mouse_down = true;
        }
        else if(event.type == SDL_MOUSEBUTTONUP)
        {
            *mouse_down = false;
        }
        else if(event.type == SDL_MOUSEWHEEL)
        {
            con->set_scale(event.wheel.preciseY, event.wheel.preciseY, win->touch_pos);
        }
    }

}
int main(int argc, char** argv)
{
    if(argc < 3)
    {
        std::cerr<<"Usage ./a.out 'filepath' 'ticks per second' (optional)\n";
        return 1;
    }
    if(SDL_Init(SDL_INIT_VIDEO) < 0) 
    {
        std::cerr<<"Video initialization failed: "<<SDL_GetError()<<"\n";
        return 1;
    }
    const uint32_t win_dim = 750;
    Window win("Main", win_dim, win_dim * 0.5);
    Chip8 con(&win, atoi(argv[2]));
    con.load_file(argv[1]);
    std::cout<<(std::string(argv[1]) + ".trace")<<"\n";
    std::string test_code = "test_opcode.ch8";
    std::fstream trace_file(test_code + ".trace");
    Chip8Verifier ver(trace_file, test_code, win);
    bool mouse_down = false;

    auto logic_thread = std::thread(tick, &con);
    auto render_thread = std::thread(render_loop, &con, &win);
    event_loop(&con, &win, &mouse_down);
    return 0;
}
