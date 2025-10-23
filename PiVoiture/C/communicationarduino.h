#pragma once

#include <stdint.h>
#include "Statemachine/map.h"

void stopcommand();
void send_command(int fd, int8_t rpmg, int8_t rpmd);
void lignedroite(struct map_node *map_node);
void gotopoint(void *arg);
void followtraj(struct map_node *);