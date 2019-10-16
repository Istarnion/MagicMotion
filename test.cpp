#include <stdio.h>
#include "magic_math.h"

#define GREEN(text) ("\033[0;32m"#text"\033[0m")
#define RED(text) ("\033[0;31m"#text"\033[0m")

static int counter = 0;
static int success_counter = 0;

void
_test(const char *test, bool pass)
{
    int test_num = ++counter;
    success_counter += !!pass;
    printf("%02d. %-69s [%s]\n", test_num, test, (pass ? GREEN(PASS) : RED(FAIL)));
}

void
_end_section()
{
    if(counter > 0)
    {
        printf("(%d of %d)\n\n", success_counter, counter);
    }

    success_counter = counter = 0;
}

#define TEST(test) _test(#test, (test))
#define HEADER(name) do { _end_section(); printf("[%s]:\n", #name); } while(0)
#define END() _end_section()

int
main(int num_args, char *args[])
{
    HEADER(Vectors);
    TEST(IsEqualV3(MakeV3(1, 2, 3), (V3){ 1, 2, 3 }));
    TEST(IsEqualV3(MakeNormalizedV3(2, 0, 0), (V3){ 1, 0, 0 }));
    TEST(IsEqualV3(NormalizeV3((V3){ 2, 0, 0 }), (V3){ 1, 0, 0 }));
    TEST(IsEqualV3(NegateV3((V3){ 2, 0, 0 }), (V3){ -2, 0, 0 }));
    TEST(IsEqualV3(AddV3((V3){ 1, 0, 0 }, (V3){ 0, 1, 0 }), (V3){ 1, 1, 0 }));
    TEST(IsEqualV3(SubV3((V3){ 1, 0, 0 }, (V3){ 1, 0, 0 }), (V3){ 0, 0, 0 }));
    TEST(IsEqualV3(ScaleV3((V3){ 1, 2, 3 }, 2), (V3){ 2, 4, 6 }));
    TEST((DotV3((V3){ 1, 2, 3 }, (V3){ 1, 2, 3 }) == 14));
    TEST(IsEqualV3(CrossV3((V3){ 1, 0, 0 }, (V3){ 0, 1, 0 }), (V3){ 0, 0, 1 }));

    HEADER(Matrices);
    TEST(IsEqualMat4(MulMat4(IdentityMat4(), IdentityMat4()), IdentityMat4()));
    TEST(IsEqualV3(MulMat4Vec3(IdentityMat4(), (V3){ 1, 1, 1 }), (V3){ 1, 1, 1 }));
    TEST(IsEqualV3(MulMat4Vec3(TranslationMat4((V3){ 2, 3, 4 }), (V3){ 1, 2, 3 }), (V3){ 3, 5, 7 }));
    TEST(IsEqualV3(MulMat4Vec3(ScaleMat4((V3){ 2, 3, 6 }), (V3){ 1, 2, 3 }), (V3){ 2, 6, 18 }));

    V3 translate = MakeV3(1, 2, 3);
    V3 scale = MakeV3(2, 4, 6);
    V3 rotation = MakeV3(0, 0, 0);
    TEST(IsEqualMat4(MulMat4(TranslationMat4(translate), TranslationMat4(translate)), TranslationMat4(ScaleV3(translate, 2))));
    TEST(IsEqualMat4(TransformMat4(translate, scale, rotation), MulMat4(TranslationMat4(translate), ScaleMat4(scale))));
    TEST(IsEqualMat4(TransformMat4(translate, scale, rotation), MulMat4(ScaleMat4(scale), TranslationMat4(translate))));

    END();
    return 0;
}

