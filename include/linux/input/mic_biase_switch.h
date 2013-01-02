#ifndef __MIC_BIASE_SWITCH__
#define __MIC_BIASE_SWITCH__

#define MIC_BIASE_SWITCH_L 0
#define MIC_BIASE_SWITCH_H 1
#define MIC_BIASE_SWITCH_INV 2

#define HEADPHONE_DETECT_VALUE 30
#define US_HEADSET_DETECT_VLAUE 800

void mic_biase_switch_set_enable(int switch_enable);

struct mic_biase_switch_platform_data {
    int gpio_switch;
};
#endif  /* __MIC_BIASE_SWITCH__ */
