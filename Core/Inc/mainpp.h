/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

double vel[3];
double x,y;
double encoder[4];

#ifdef __cplusplus
extern "C"
{
#endif

void setup(void);
void loop(void);





#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
