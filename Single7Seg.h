/*
 * Single7Seg.h
 *
 * Created: 01.11.2020 11:46:07
 *  Author: marti
 */ 


#ifndef SINGLE7SEG_H_
#define SINGLE7SEG_H_

void single_7seg_off(void);
uint8_t single_7seg_number(uint8_t digit);
void setDP_general(void);
void unsetDP_general(void);

#endif /* SINGLE7SEG_H_ */