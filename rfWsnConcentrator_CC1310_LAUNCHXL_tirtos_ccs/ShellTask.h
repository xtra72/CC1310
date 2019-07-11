/*
 * shell.h
 *
 *  Created on: 2019. 7. 9.
 *      Author: xtra7
 */

#ifndef SHELLTASK_H_
#define SHELLTASK_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    char    name[16];
    bool    (*command)(int argc, char *argv[]);
}   ShellTaskCommand;

void ShellTask_init(ShellTaskCommand _commandList[], uint32_t _count);
bool ShellTask_addCommand(char* name, bool (*command)(int argc, char *argv[]));

void  ShellTask_output(char *fmt, ...);

#endif /* SHELLTASK_H_ */
