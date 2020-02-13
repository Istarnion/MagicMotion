#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

typedef struct
{
    const char *box_file;
} Config;

void ConfigInit(int num_args, char *args[]);
const Config *GetConfig(void);

#endif /* end of include guard: CONFIGURATION_H_ */

