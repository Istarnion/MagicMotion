#ifndef FILES_H_
#define FILES_H_

typedef struct
{
    int width;
    int height;
    unsigned int *pixels; // 3-channel RGB
} Image;

char *LoadTextFile(const char *filename);
void FreeTextFile(char *file_data);

Image *LoadImage(Image *img, const char *filename);
void FreeImage(Image *img);

#endif /* end of include guard: FILES_H_ */
