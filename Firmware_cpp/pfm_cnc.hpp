#include <Arduino.h>

class Pfm_cnc
{

public:
    Pfm_cnc(void);
    void    init_memory(int32_t delta_steps_x, int32_t delta_steps_y, int32_t delta_steps_z);
    void    init_memory(void);
    void    init_isr(void);
    void    enable_cnc(void);
    void    disable_cnc(void);
    void    init_pins(void);
    void    set_delta_steps(uint8_t this_pfm, int32_t delta_steps);
    int32_t get_delta_steps(uint8_t this_pfm);
    void    set_freq(uint32_t freq);
    void    init(int32_t pfm_delta_steps_x, int32_t pfm_delta_steps_y, int32_t pfm_delta_steps_z);
    void    init(void);
    void    set_target_freq(uint8_t this_pfm, uint16_t pfm_target_freq, bool pfm_direction);
    void    set_target_delta(uint8_t this_pfm, uint16_t pfm_target_freq, int32_t pfm_target_delta);
};