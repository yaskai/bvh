#include "raylib.h"
#include "num_redefs.h"

#ifndef MAP_H_
#define MAP_H_

#define MAX_TRIS_PER_NODE	8

typedef struct {
	Vector3 vertices[3];
	Vector3 normal;

} Tri;

typedef struct {
	BoundingBox bounds;		// 24

	u16 first_tri;			// 2
	u16 tri_count;			// 2

	u16 child_left;			// 2
	u16 child_right;		// 2

} BvhNode;

typedef struct {
	Model model;

	Tri *tris;
	BvhNode *bvh_nodes;

	u32 tri_count;

	u16 bvh_node_capacity;
	u16 bvh_node_count;

} Map;

void MapInit(Map *map, char *map_path);
void MapClose(Map *map);

Tri *MeshToTris(Mesh mesh, u32 *tri_count); 
Tri *ModelToTris(Model model, u32 *tri_count);

void BvhConstruct(Map *map);
void BvhNodeUpdateBounds(Map *map, u16 node_id);
void BvhNodeSubdivide(Map *map, u16 root_id);

Vector3 TriCentroid(Tri *tri);

#endif
