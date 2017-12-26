#ifndef __MOTO_H
#define __MOTO_H

void moto_90(unsigned char SF);
void moto_front(void);
void moto_back(void);
void moto_left(unsigned char a);
void moto_right(unsigned char a);
void moto_stop(void);
void moto_jztask(void const * argument);

#endif
