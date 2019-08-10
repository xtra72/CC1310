/*
 * trace.h
 *
 *  Created on: 2019. 5. 26.
 *      Author: xtra7
 */

#ifndef TRACE_H_
#define TRACE_H_


void  Trace_enable(void);
void  Trace_disable(void);
bool  Trace_isEnable(void);
void  Trace_printf(char *fmt, ...);

#endif /* TRACE_H_ */
