#ifndef OCTREE_H_
#define OCTREE_H_

#include "magic_math.h"

#define OCTREE_BIN_SIZE 32

typedef struct _octree_node
{
    int count;
    V3 center;
    int size;

    union
    {
        _octree_node *children[8];
        V3 points[OCTREE_BIN_SIZE];
    };
} OctreeNode;

typedef struct
{
    OctreeNode *root;
    OctreeNode *node_pool;
    size_t node_pool_size;
    size_t node_pool_index;
} Octree;

void ResetOctree(Octree *tree, size_t max_num_points, float bounding_size);
void AddPointsToOctree(V3 *points, size_t num_points, Octree *tree);
bool CheckBoxCollision(Octree *tree, V3 box_center, V3 box_size);

#endif /* end of include guard: OCTREE_H_ */

