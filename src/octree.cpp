#include "octree.h"
#include <math.h>

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

    // printf("Recursing to child %d (node %lu)\n", index, ((size_t)(node->children[index]) - (size_t)tree->node_pool)/sizeof(OctreeNode));
    _AddPoint(tree, point, node->children[index]);
}

static void
_AddPoint(Octree *tree, V3 point, OctreeNode *node)
{
    ++node->count;

    if(node->count > OCTREE_BIN_SIZE)
    {
        if(node->count == OCTREE_BIN_SIZE+1)
        {
            // printf("Splitting node %lu\n", ((size_t)node - (size_t)tree->node_pool)/sizeof(OctreeNode));
            V3 points[OCTREE_BIN_SIZE];
            memcpy(points, node->points, sizeof(points));

            for(int i=0; i<8; ++i)
            {
                // We ensure in initialization that there will be enough
                // nodes in the pool, and they are all zero-initialized,
                // thanks to the memset in ResetOctree
                OctreeNode *child = &tree->node_pool[tree->node_pool_index++];
                child->size = node->size / 2;

                V3 offset = (V3){
                    (i & 0x1) ? (child->size/2) : (child->size/-2),
                    (i & 0x2) ? (child->size/2) : (child->size/-2),
                    (i & 0x4) ? (child->size/2) : (child->size/-2)
                };

                child->center = AddV3(node->center, offset);

                node->children[i] = child;
            }

            for(int i=0; i<OCTREE_BIN_SIZE; ++i)
            {
                // printf("Redistributing point %d\n", i);
                _AddPointToCorrectChild(tree, points[i], node);
            }
        }

        // puts("Passing the current node on");
        _AddPointToCorrectChild(tree, point, node);
    }
    else
    {
        node->points[node->count-1] = point;
        // printf("Added point to leaf node %lu\n", ((size_t)node - (size_t)tree->node_pool)/sizeof(OctreeNode));
    }
}

void
ResetOctree(Octree *tree, size_t max_num_points, float bounding_size)
{
    // NOTE(istarnion): !!! max_num_points MUST be consistent during runtime.
    // we don't support reallocating the nodes ATM
    if(tree->node_pool)
    {
        memset(tree->node_pool, 0, sizeof(OctreeNode) * tree->node_pool_size);
    }
    else
    {
        if(max_num_points <= OCTREE_BIN_SIZE)
        {
            tree->node_pool_size = 1;
        }
        else
        {
            tree->node_pool_size = (size_t)8 * (max_num_points) + 1;
        }

        tree->node_pool = (OctreeNode *)calloc(tree->node_pool_size, sizeof(OctreeNode));
        printf("Allocated %lu nodes in the octree\n", tree->node_pool_size);
    }

    tree->node_pool_index = 0;
    tree->root = &tree->node_pool[tree->node_pool_index++];
    tree->root->size = bounding_size;
}

void
AddPointsToOctree(V3 *points, size_t num_points, Octree *tree)
{
    float tree_extent = tree->root->size/2.0f;
    for(size_t i=0; i<num_points; ++i)
    {
        // printf("Adding point %lu\n", i);
        if(!(abs(points[i].x) > tree_extent ||
             abs(points[i].y) > tree_extent ||
             abs(points[i].y) > tree_extent))
        {
            _AddPoint(tree, points[i], tree->root);
        }
    }
}

bool
CheckBoxCollision(Octree *tree, V3 box_min, V3 box_max)
{
    size_t touching_points = 0;

    // TODO(istarnion): Implement!
    /*
    if(!(point.x < box_center.x-box_size.x/2 ||
         point.x > box_center.x+box_size.x/2 ||
         point.y < box_center.y-box_size.y/2 ||
         point.y > box_center.y+box_size.y/2 ||
         point.z < box_center.z/2-box_size.z/2 ||
         point.z > box_center.z/2+box_size.z/2))
    {
        ++touching_points;
    }
    */

    return touching_points > 3;
}

