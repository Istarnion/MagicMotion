#ifndef SCENE_H_
#define SCENE_H_

typedef bool (* SceneInitFunc)(void);
typedef void (* SceneUpdateFunc)(void);
typedef void (* SceneEndFunc)(void);

typedef struct
{
    SceneInitFunc Init;
    SceneUpdateFunc Update;
    SceneEndFunc End;
} Scene;

#endif /* end of include guard: SCENE_H_ */
