
#define __REMOTECONTROL_INIT__H
#ifndef __REMOTECONTROL_INIT__H




#include "Allheaderfile.h"
#include "main.h"


extern void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
static RC_ctrl_t rc_ctrl;

#endif


