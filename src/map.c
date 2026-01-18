#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include "raylib.h"
#include "raymath.h"
#include "map.h"

void MapInit(Map *map, char *map_path) {
	map->model = LoadModel(TextFormat("resources/%s", map_path));
	map->tris = ModelToTris(map->model, &map->tri_count);

	printf("mesh count: %d\n", map->model.meshCount);
	printf("tri count: %d\n", map->tri_count);

	map->bvh_node_capacity = 2048;
	map->bvh_nodes = calloc(map->bvh_node_capacity, sizeof(BvhNode));

	//map->bvh_nodes[0] = MakeBvhNode(map->tris, map->tri_count, &map->bvh_node_count);
	BvhConstruct(map);
}

void MapClose(Map *map) {
	UnloadModel(map->model);

	free(map->tris);
	free(map->bvh_nodes);
}

Tri *MeshToTris(Mesh mesh, u32 *tri_count) {
	u32 count = mesh.triangleCount;
	*tri_count = count;

	Tri *tris = calloc(count, sizeof(Tri));

	for(u32 i = 0; i < count; i++) {
		Tri tri = (Tri) {0};

		u32 indices[3] = {
			mesh.indices[i * 3 + 0],
			mesh.indices[i * 3 + 1],
			mesh.indices[i * 3 + 2]
		};

		tri.normal.x = mesh.normals[i * 3 + 0];
		tri.normal.y = mesh.normals[i * 3 + 1];
		tri.normal.z = mesh.normals[i * 3 + 2];

		for(u8 j = 0; j < 3; j++) {
			u32 id = indices[j]; 

			tri.vertices[j].x = mesh.vertices[id * 3 + 0];
			tri.vertices[j].y = mesh.vertices[id * 3 + 1];
			tri.vertices[j].z = mesh.vertices[id * 3 + 2];
		}

		tris[i] = tri;
	}

	return tris;
}

Tri *ModelToTris(Model model, u32 *tri_count) {
	u32 count = 0;
	Tri *tris = NULL;

	for(u32 i = 0; i < model.meshCount; i++) {
		u32 temp_count = 0;

		Tri *temp_tris = MeshToTris(model.meshes[i], &temp_count);
		count += temp_count;

		Tri *ptr = realloc(tris, sizeof(Tri) * count);
		tris = ptr;
		
		memcpy(tris + count - temp_count, temp_tris, sizeof(Tri) * temp_count);
		free(temp_tris);
	}

	*tri_count = count;
	return tris;
}

Vector3 TriCenter(Tri *tri) {
	return (Vector3) {
		(tri->vertices[0].x + tri->vertices[1].x + tri->vertices[2].x) / 3.0f,
		(tri->vertices[0].y + tri->vertices[1].y + tri->vertices[2].y) / 3.0f,
		(tri->vertices[0].z + tri->vertices[1].z + tri->vertices[2].z) / 3.0f
	};
}

void BvhConstruct(Map *map) {
	// Create root node
	BvhNode root_node = (BvhNode) {0};

	// Find min and max points to create bounding box
	root_node.bounds.min = Vector3Scale(Vector3One(),  FLT_MAX);
	root_node.bounds.max = Vector3Scale(Vector3One(), -FLT_MAX);

	for(u16 i = 0; i < map->tri_count; i++) {
		Tri *tri = &map->tris[i];

		for(u8 j = 0; j < 3; j++) {
			root_node.bounds.min = Vector3Min(tri->vertices[j], root_node.bounds.min);
			root_node.bounds.max = Vector3Max(tri->vertices[j], root_node.bounds.max);
		}
	}

	// Assign all triangles to root node
	root_node.first_tri = 0;
	root_node.tri_count = map->tri_count;	

	root_node.child_left = 0;
	root_node.child_right = 0;

	map->bvh_nodes[0] = root_node;
}

void BvhNodeSubdivide(Map *map, u16 root_id) {
	BvhNode *node = &map->bvh_nodes[root_id];

	// Calculate axis distances 
	float axis_length[3] = {
		node->bounds.max.x - node->bounds.min.x * 0.5f,	
		node->bounds.max.y - node->bounds.min.y * 0.5f,	
		node->bounds.max.z - node->bounds.min.z * 0.5f	
	};

	// Find longest axis distance to use for split
	short split_axis = 0;
	for(short i = 0; i < 3; i++)
		split_axis = (axis_length[i] > axis_length[split_axis]) ? i : split_axis;	

	// Calculate mid point
	Vector3 mid = (Vector3) {
		.x = node->bounds.min.x + axis_length[0] * 0.5f,	
		.y = node->bounds.min.x + axis_length[1] * 0.5f,	
		.z = node->bounds.min.x + axis_length[2] * 0.5f	
	};

	BoundingBox bounds_lft;
	BoundingBox bounds_rgt;
}

