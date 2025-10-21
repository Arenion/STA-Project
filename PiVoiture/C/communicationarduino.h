#pragma once

#include <stdint.h>

void stopcommand();
void send_command(int fd, int8_t rpmg, int8_t rpmd);
void *lignedroite(void *arg);
void gotopoint(void *arg);