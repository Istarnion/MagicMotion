#include "config.h"

static Config _g_config;

void
ConfigInit(int num_args, char *args[])
{
    // Defaults
    _g_config.box_file = "boxes.ser";
}

const Config *
GetConfig(void)
{
    return &_g_config;
}

