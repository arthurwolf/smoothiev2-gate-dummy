#pragma once
#include <cstdio>
#include <sys/stat.h>

bool sim_v2_set_sd_root(const char *path);
bool sim_v2_sd_root_ready();
FILE *sim_v2_player_fopen(const char *path, const char *mode);
int sim_v2_player_stat(const char *path, struct stat *result);
