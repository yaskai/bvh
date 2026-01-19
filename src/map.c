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

		tri.normal = (Vector3) {
			.x = mesh.normals[i * 3 + 0],
			.y = mesh.normals[i * 3 + 1],
			.z = mesh.normals[i * 3 + 2]
		};

		for(u8 j = 0; j < 3; j++) {
			u32 id = indices[j]; 

			tri.vertices[j] = (Vector3) {
				.x = mesh.vertices[id * 3 + 0],
				.y = mesh.vertices[id * 3 + 1],
				.z = mesh.vertices[id * 3 + 2]
			};
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

		tris = realloc(tris, sizeof(Tri) * count);
		
		memcpy(tris + count - temp_count, temp_tris, sizeof(Tri) * temp_count);
		free(temp_tris);
	}

	*tri_count = count;
	return tris;
}

Vector3 TriCentroid(Tri *tri) {
	return (Vector3) {
		.x = (tri->vertices[0].x + tri->vertices[1].x + tri->vertices[2].x) * 0.33f,
		.y = (tri->vertices[0].y + tri->vertices[1].y + tri->vertices[2].y) * 0.33f,
		.z = (tri->vertices[0].z + tri->vertices[1].z + tri->vertices[2].z) * 0.33f
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

void BvhNodeUpdateBounds(Map *map, u16 node_id) {
	BvhNode *node = &map->bvh_nodes[node_id];

	node->bounds.min = (Vector3) {  FLT_MAX };	
	node->bounds.max = (Vector3) { -FLT_MAX };

	for(u16 i = 0; i < node->tri_count; i++) {
		u16 tri_id = node->first_tri + i;
		Tri *tri = &map->tris[tri_id];

		for(short j = 0; j < 3; j++) {
			node->bounds.min = Vector3Min(node->bounds.min, tri->vertices[j]);
			node->bounds.max = Vector3Max(node->bounds.max, tri->vertices[j]);
		}
	}
}

void BvhNodeSubdivide(Map *map, u16 root_id) {
	BvhNode *node = &map->bvh_nodes[root_id];

	float3 vals_min = Vector3ToFloatV(node->bounds.min);

	// Calculate axis distances 
	Vector3 v_extent = (Vector3) {
		fabsf(node->bounds.max.x - node->bounds.min.x),	
		fabsf(node->bounds.max.y - node->bounds.min.y),	
		fabsf(node->bounds.max.z - node->bounds.min.z)	
	};

	float3 extent = Vector3ToFloatV(v_extent);

	// Find longest axis distance to use for split
	short split_axis = 0;
	for(short i = 0; i < 3; i++)
		split_axis = (extent.v[i] > extent.v[split_axis]) ? i : split_axis;	

	float split_pos = vals_min.v[split_axis] + (extent.v[split_axis] * 0.5f);

	u16 i = node->first_tri;
	u16 j = i + node->tri_count - 1;

	while(i <= j) {
		Tri *tri = &map->tris[i];
		
		Vector3 centroid = TriCentroid(tri);
		float3 c = Vector3ToFloatV(centroid);

		if(c.v[split_axis] < split_pos) { 
			i++;
		} else {
			// Swap with tri with tri at end of list
		}
		
	}	

}

