#include "octree.h"

static void _AddPoint(Octree *tree, V3 point, OctreeNode *node);

static void
_AddPointToCorrectChild(Octree *tree, V3 point, OctreeNode *node)
{
    // 0: -x -y -z
    // 1:  x -y -z
    // 2: -x  y -z
    // 3:  x  y -z
    // 4: -x -y  z
    // 5:  x -y  z
    // 6: -x  y  z
    // 7:  x  y  z

    int index = 0;
    if(point.x > node->center.x) index |= 0x1;
    if(point.y > node->center.y) index |= 0x2;
    if(point.z > node->center.z) index |= 0x4;

    _AddPoint(tree, point, node->children[index]);
}

static void
_AddPoint(Octree *tree, V3 point, OctreeNode *node)
{
    ++node->count;

    if(node->count >= OCTREE_BIN_SIZE)
    {
        if(node->count == OCTREE_BIN_SIZE)
        {
            V3 points[OCTREE_BIN_SIZE];
            memcpy(points, node->points, sizeof(points));

            for(int i=0; i<8; ++i)
            {
                // We ensure in initialization that there will be enough
                // nodes in the pool, and they are all zero-initialized,
                // thanks to the memset in BuildOctree
                OctreeNode *child = &tree->node_pool[tree->node_pool_index++];
                child->size = node->size / 2;

                V3 offset = (V3){
                    (i & 0x1) ? child->size/2 : child->size/-2,
                    (i & 0x2) ? child->size/2 : child->size/-2,
                    (i & 0x4) ? child->size/2 : child->size/-2
                };

                child->center = AddV3(node->center, offset);

                node->children[i] = child;
            }

            for(int i=0; i<OCTREE_BIN_SIZE; ++i)
            {
                _AddPointToCorrectChild(tree, points[i], node);
            }
        }

        _AddPointToCorrectChild(tree, point, node);
    }
    else
    {
        node->points[node->count-1] = point;
    }
}

void
BuildOctree(V3 *points, size_t num_points, Octree *tree)
{
    tree->root->count = 0;
    tree->node_pool_index = 0;
    memset(tree->node_pool, 0, sizeof(OctreeNode) * tree->node_pool_size);
    for(size_t i=0; i<num_points; ++i)
    {
        _AddPoint(tree, points[i], tree->root);
    }
}

