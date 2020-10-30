#ifndef __MAIN_H
#define __MAIN_H
#include "bsp.h"
#include "SCA_API.h"
#include "SCA_APP.h"
#define DEBUG_OUTPUT 1
extern unsigned char total_motor_number;
extern unsigned char if_error;
/**
 * @brief motor act function
 * @param id motor id
 * @param step step length
 * @param dir move direction, 0: positive, 1: negative
 * @return int 0: correct 1~255: error
 */
char motor_act(size_t id, float step, char dir);
/**
 * @brief update the motor status when timeout
 * 
 * @return char 0:success, 
 *              1~255: error  
 *              10+id: not online
 *              15+id: not enable
 *              20+id: position get error;
 */
char update_status(void);
#endif
