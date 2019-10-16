#include "files.h"

#include <stdlib.h>
#include <stdio.h>

char *
LoadTextFile(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if(!f) return NULL;

    fseek(f, 0, SEEK_END);
    long int length = ftell(f);
    rewind(f);

    char *text = (char *)malloc(length+1);
    fread(text, 1, length, f);
    text[length] = '\0';

    fclose(f);
    return text;
}

void
FreeTextFile(char *file_data)
{
    free(file_data);
}

