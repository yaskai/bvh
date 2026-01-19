#include "raylib.h"
#include "num_redefs.h"

#ifndef MAP_H_
#define MAP_H_

#define MAX_TRIS_PER_NODE	2

typedef struct {
	Vector3 vertices[3];
	Vector3 normal;

} Tri;

typedef struct {
	BoundingBox bounds;		// 24

	u16 first_tri;			// 2
	u16 tri_count;			// 2

	u16 child_lft;			// 2
	u16 child_rgt;			// 2

} BvhNode;

typedef struct {
	Model model;

	Tri *tris;
	BvhNode *bvh_nodes;

	u16 *tri_ids;

	u32 tri_count;

	u16 bvh_node_capacity;
	u16 bvh_node_count;

} Map;

void MapInit(Map *map, char *map_path);
void MapClose(Map *map);

Vector3 FaceNormal(Vector3 *vertices);
Vector3 TriCentroid(Tri *tri);

Tri *MeshToTris(Mesh mesh, u32 *tri_count); 
Tri *ModelToTris(Model model, u32 *tri_count, u16 **tri_ids);

void BvhConstruct(Map *map);
void BvhNodeUpdateBounds(Map *map, u16 node_id);
void BvhNodeSubdivide(Map *map, u16 root_id);

void BvhTraceNodes(Ray ray, u16 root_node, u16 *hits, u16 *hit_count, Map *map, u16 *tri_tests);

#endif
