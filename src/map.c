#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <threads.h>
#include <stdint.h>
#include "raylib.h"
#include "raymath.h"
#include "map.h"

void SwapTriIds(u16 *a, u16 *b) {
	u16 temp = *a;
	*a = *b;
	*b = temp;
} 

void MapInit(Map *map, char *map_path) {
	map->model = LoadModel(TextFormat("resources/%s", map_path));
		
	map->tri_ids = NULL;
	map->tris = ModelToTris(map->model, &map->tri_count, &map->tri_ids);

	puts("\n--------- MAP INFO ---------");
	printf("file: %s\n", map_path);
	printf("mesh count: %d\n", map->model.meshCount);
	printf("tri count: %d\n", map->tri_count);
	puts("----------------------------\n");

	//map->bvh_node_capacity = 4;
	map->bvh_node_capacity = 512;
	map->bvh_nodes = calloc(map->bvh_node_capacity, sizeof(BvhNode));

	map->build_complete = 0;

	//BvhConstruct(map);
	//printf("bvh node count: %d\n", map->bvh_node_count);
}

void MapClose(Map *map) {
	UnloadModel(map->model);

	free(map->tris);
	free(map->bvh_nodes);
}

Vector3 TriCentroid(Tri *tri) {
	return (Vector3) {
		.x = (tri->vertices[0].x + tri->vertices[1].x + tri->vertices[2].x) * 0.33f,
		.y = (tri->vertices[0].y + tri->vertices[1].y + tri->vertices[2].y) * 0.33f,
		.z = (tri->vertices[0].z + tri->vertices[1].z + tri->vertices[2].z) * 0.33f
	};
}

Vector3 FaceNormal(Vector3 *vertices) {
	Vector3 u = Vector3Subtract(vertices[1], vertices[0]);
	Vector3 v = Vector3Subtract(vertices[2], vertices[0]);
	return Vector3Normalize(Vector3CrossProduct(u, v));
}

Vector3 BoxExtent(BoundingBox box) {
	return Vector3Subtract(box.max, box.min);
}

float BoxSurfaceArea(BoundingBox box) {
	Vector3 e = (Vector3) BoxExtent(box);
	return (e.x * e.y + e.y * e.z + e.z * e.x);
}

Vector3 BoxCenter(BoundingBox box) {
	Vector3 size = (Vector3) BoxExtent(box);
	return (Vector3) {
		box.max.x - size.x * 0.5f,
		box.max.y - size.y * 0.5f,
		box.max.z - size.z * 0.5f
	};
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

		for(u8 j = 0; j < 3; j++) {
			u32 id = indices[j]; 

			tri.vertices[j] = (Vector3) {
				.x = mesh.vertices[id * 3 + 0],
				.y = mesh.vertices[id * 3 + 1],
				.z = mesh.vertices[id * 3 + 2]
			};
		}

		tri.normal = FaceNormal(tri.vertices);
		tris[i] = tri;
	}

	return tris;
}

Tri *ModelToTris(Model model, u32 *tri_count, u16 **tri_ids) {
	u32 count = 0;

	Tri *tris = NULL;
	u16 *ids = NULL;

	for(u32 i = 0; i < model.meshCount; i++) {
		u32 temp_count = 0;

		Tri *temp_tris = MeshToTris(model.meshes[i], &temp_count);
		count += temp_count;

		tris = realloc(tris, sizeof(Tri) * count);
		ids = realloc(ids, sizeof(u16) * count);

		for(u16 j = 0; j < temp_count; j++) {
			u16 id = count - temp_count + j;
			ids[id] = id;
		}

		memcpy(tris + count - temp_count, temp_tris, sizeof(Tri) * temp_count);
		free(temp_tris);
	}

	*tri_ids = ids;

	*tri_count = count;
	return tris;
}

void BvhConstruct(Map *map) {
	float _t_start = GetTime(); 

	// Create root node
	BvhNode root_node = (BvhNode) {0};

	// Find min and max points to create bounding box
	root_node.bounds.min = Vector3Scale(Vector3One(),  FLT_MAX);
	root_node.bounds.max = Vector3Scale(Vector3One(), -FLT_MAX);

	for(u16 i = 0; i < map->tri_count; i++) {
		Tri *tri = &map->tris[map->tri_ids[i]];

		for(u8 j = 0; j < 3; j++) {
			root_node.bounds.min = Vector3Min(tri->vertices[j], root_node.bounds.min);
			root_node.bounds.max = Vector3Max(tri->vertices[j], root_node.bounds.max);
		}
	}

	// Assign all triangles to root node
	root_node.first_tri = 0;
	root_node.tri_count = map->tri_count;	

	root_node.child_lft = 0;
	root_node.child_rgt = 0;

	map->bvh_nodes[map->bvh_node_count++] = root_node;

	BvhNodeUpdateBounds(map, 0);
	//BvhNodeSubdivide(map, 0);
	//BvhNodeSubdivideSah(map, 0);
	BvhNodeSubdivideSahFast(map, 0);

	//u16 d = NodeDepth(&map->bvh_nodes[0], map);
	//printf("depth: %d\n", d);
	
	map->build_time = (GetTime() - _t_start); 
	map->build_complete = 1;

	map->bvh_node_capacity = map->bvh_node_count;
	map->bvh_nodes = realloc(map->bvh_nodes, sizeof(BvhNode) * map->bvh_node_capacity);

	size_t mem_usage = sizeof(BvhNode) * map->bvh_node_capacity;
	map->mem_use = mem_usage / 1000;

	puts("\n--------- BVH INFO ---------");
	printf("node count: %d\n", map->bvh_node_count);
	printf("memory usage: %zukb\n", mem_usage / 1000);
	printf("build time: %f\n", map->build_time);
	puts("----------------------------\n");
}

void BvhNodeUpdateBounds(Map *map, u16 node_id) {
	BvhNode *node = &map->bvh_nodes[node_id];

	node->bounds.min = (Vector3) {  FLT_MAX,  FLT_MAX,  FLT_MAX };	
	node->bounds.max = (Vector3) { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	for(u16 i = 0; i < node->tri_count; i++) {
		u16 tri_id = map->tri_ids[node->first_tri + i];
		Tri *tri = &map->tris[tri_id];

		for(short j = 0; j < 3; j++) {
			node->bounds.min = (Vector3) {
				fminf(node->bounds.min.x, tri->vertices[j].x),
				fminf(node->bounds.min.y, tri->vertices[j].y),
				fminf(node->bounds.min.z, tri->vertices[j].z),
			};

			node->bounds.max = (Vector3) {
				fmaxf(node->bounds.max.x, tri->vertices[j].x),
				fmaxf(node->bounds.max.y, tri->vertices[j].y),
				fmaxf(node->bounds.max.z, tri->vertices[j].z),
			};
		}
	}
}

void BvhNodeSubdivide(Map *map, u16 root_id) {
	BvhNode *node = &map->bvh_nodes[root_id];

	// Stop recursion
	if(node->tri_count <= MAX_TRIS_PER_NODE) return;

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

	// In-place partition
	u16 i = node->first_tri;
	u16 j = i + node->tri_count - 1;
	while(i <= j) {
		Tri *tri = &map->tris[map->tri_ids[i]];
		
		Vector3 centroid = TriCentroid(tri);
		float3 c = Vector3ToFloatV(centroid);

		if(c.v[split_axis] < split_pos) { 
			i++;
		} else {
			// Swap with tri with tri at end of list
			SwapTriIds(&map->tri_ids[i], &map->tri_ids[j--]);
		}
	}	
	
	// Cancel splitting if either side is empty 
	u16 count_lft = i - node->first_tri;  
	if(count_lft == 0 || count_lft == node->tri_count) return;

	if(map->bvh_node_count + 2 >= map->bvh_node_capacity) {
		map->bvh_node_capacity = (map->bvh_node_capacity << 1);
		map->bvh_nodes = realloc(map->bvh_nodes, sizeof(BvhNode) * map->bvh_node_capacity);
	}

	// Create child nodes
	u16 node_id_lft = map->bvh_node_count++;
	u16 node_id_rgt = map->bvh_node_count++;

	map->bvh_nodes[node_id_lft] = (BvhNode) {
		.bounds = node->bounds,
		.first_tri = node->first_tri,
		.tri_count = count_lft,
		.child_lft = 0,
		.child_rgt = 0
	};
	node->child_lft = node_id_lft;

	map->bvh_nodes[node_id_rgt] = (BvhNode) {
		.bounds = node->bounds,
		.first_tri = i,
		.tri_count = node->tri_count - count_lft,
		.child_lft = 0,
		.child_rgt = 0
	};
	node->child_rgt = node_id_rgt;

	node->tri_count = 0;

	BvhNodeUpdateBounds(map, node_id_lft);
	BvhNodeUpdateBounds(map, node_id_rgt);

	BvhNodeSubdivide(map, node_id_lft);
	BvhNodeSubdivide(map, node_id_rgt);
}

float BvhNodeCost(BvhNode *node) {
	float area = BoxSurfaceArea(node->bounds);
	return area * node->tri_count;
}

void BvhNodeSubdivideSah(Map *map, u16 root_id) {
	BvhNode *node = &map->bvh_nodes[root_id];

	// Stop recursion
	//if(node->tri_count <= MAX_TRIS_PER_NODE) return;

	// Find longest axis distance to use for split
	short best_axis = -1;
	float best_pos = 0;
	float best_cost = FLT_MAX;

	for(short axis = 0; axis < 3; axis++) {
		for(u16 i = 0; i < node->tri_count; i++) {
			u16 tri_id = map->tri_ids[node->first_tri + i];		
			Tri *tri = &map->tris[tri_id];
			
			Vector3 centroid = TriCentroid(tri);
			float3 c = Vector3ToFloatV(centroid);

			float canditate_pos = c.v[axis]; 
			float cost = SahEval(map, node, axis, canditate_pos);

			if(cost < best_cost) { 
				best_cost = cost;
				best_axis = axis;
				best_pos = canditate_pos;
			}
		}
	}

	short split_axis = best_axis;
	float split_pos = best_pos;

	float parent_area = BoxSurfaceArea(node->bounds);
	float parent_cost = node->tri_count * parent_area;
	if(best_cost >= parent_cost && node->tri_count <= MAX_TRIS_PER_NODE << 1) {
		//printf("best_cost >= parent_cost\n");
		return;
	} 

	// In-place partition
	u16 i = node->first_tri;
	u16 j = i + node->tri_count - 1;
	while(i <= j) {
		Tri *tri = &map->tris[map->tri_ids[i]];
		
		Vector3 centroid = TriCentroid(tri);
		float3 c = Vector3ToFloatV(centroid);

		if(c.v[split_axis] < split_pos) { 
			i++;
		} else {
			// Swap with tri with tri at end of list
			SwapTriIds(&map->tri_ids[i], &map->tri_ids[j--]);
		}
	}	
	
	// Cancel splitting if either side is empty 
	u16 count_lft = i - node->first_tri;  
	if(count_lft == 0 || count_lft == node->tri_count) return;

	if(map->bvh_node_count + 2 >= map->bvh_node_capacity) {
		map->bvh_node_capacity = (map->bvh_node_capacity << 1);
		map->bvh_nodes = realloc(map->bvh_nodes, sizeof(BvhNode) * map->bvh_node_capacity);
	}

	// Create child nodes
	u16 node_id_lft = map->bvh_node_count++;
	u16 node_id_rgt = map->bvh_node_count++;

	map->bvh_nodes[node_id_lft] = (BvhNode) {
		.bounds = node->bounds,
		.first_tri = node->first_tri,
		.tri_count = count_lft,
		.child_lft = 0,
		.child_rgt = 0
	};
	node->child_lft = node_id_lft;

	map->bvh_nodes[node_id_rgt] = (BvhNode) {
		.bounds = node->bounds,
		.first_tri = i,
		.tri_count = node->tri_count - count_lft,
		.child_lft = 0,
		.child_rgt = 0
	};
	node->child_rgt = node_id_rgt;

	node->tri_count = 0;

	BvhNodeUpdateBounds(map, node_id_lft);
	BvhNodeUpdateBounds(map, node_id_rgt);

	BvhNodeSubdivideSah(map, node_id_lft);
	BvhNodeSubdivideSah(map, node_id_rgt);
}

float SahEval(Map *map, BvhNode *node, short axis, float pos) {
	BoundingBox bounds_lft = (BoundingBox) {0};
	BoundingBox bounds_rgt = (BoundingBox) {0};

	u16 count_lft = 0; 
	u16 count_rgt = 0;
	
	for(u16 i = 0; i < node->tri_count; i++) {
		u16 tri_id = map->tri_ids[node->first_tri + i];
		Tri *tri = &map->tris[tri_id];

		Vector3 centroid = TriCentroid(tri);
		float3 c = Vector3ToFloatV(centroid);

		if(c.v[axis] < pos) {
			count_lft++;

			BoundsGrow(&bounds_lft, tri->vertices[0]);
			BoundsGrow(&bounds_lft, tri->vertices[1]);
			BoundsGrow(&bounds_lft, tri->vertices[2]);

		} else {
			count_rgt++;

			BoundsGrow(&bounds_rgt, tri->vertices[0]);
			BoundsGrow(&bounds_rgt, tri->vertices[1]);
			BoundsGrow(&bounds_rgt, tri->vertices[2]);
		}
	}

	float cost = count_lft * BoxSurfaceArea(bounds_lft) + count_rgt * BoxSurfaceArea(bounds_rgt);
	return (cost > 0) ? cost : FLT_MAX; 
}

float FindBestSplitPlane(Map *map, BvhNode *node, short *axis, float *split_pos) {
	float best_cost = FLT_MAX;

	BoundingBox empty_box = (BoundingBox) { .min = Vector3Scale(Vector3One(), FLT_MAX), .max = Vector3Scale(Vector3One(), -FLT_MAX) };

	for(short a = 0; a < 3; a++) {
		float bmin =  FLT_MAX;
		float bmax = -FLT_MAX;

		for(u16 i = 0; i < node->tri_count; i++) {
			Tri *tri = &map->tris[map->tri_ids[node->first_tri + i]];
	 		float3 centroid = Vector3ToFloatV(TriCentroid(tri));

			bmin = fminf(centroid.v[a], bmin);
			bmax = fmaxf(centroid.v[a], bmax);
		}

		if(bmin == bmax) continue;

		Bucket buckets[BUCKET_COUNT] = {0};
		for(short i = 0; i < BUCKET_COUNT; i++) buckets[i].bounds = empty_box;

		float scale = BUCKET_COUNT / (bmax - bmin); 

		for(u16 i = 0; i < node->tri_count; i++) {
			Tri *tri = &map->tris[map->tri_ids[node->first_tri + i]];
	 		float3 centroid = Vector3ToFloatV(TriCentroid(tri));

			int bucket_id = fmin(BUCKET_COUNT - 1, (int)((centroid.v[a] - bmin) * scale));
			buckets[bucket_id].count++;
			BoundsGrow(&buckets[bucket_id].bounds, tri->vertices[0]);
			BoundsGrow(&buckets[bucket_id].bounds, tri->vertices[1]);
			BoundsGrow(&buckets[bucket_id].bounds, tri->vertices[2]);
		}

		float area_lft[BUCKET_COUNT - 1], area_rgt[BUCKET_COUNT - 1];
		u16 count_lft[BUCKET_COUNT - 1], count_rgt[BUCKET_COUNT - 1];

		BoundingBox bounds_lft = empty_box, bounds_rgt = empty_box;

		u32 sum_lft = 0, sum_rgt = 0;		

		for(short i = 0; i < BUCKET_COUNT - 1; i++) {
			short id_lft = i;
			sum_lft += buckets[id_lft].count;
			count_lft[i] = sum_lft;
			bounds_lft.min = Vector3Min(buckets[i].bounds.min, bounds_lft.min);
			bounds_lft.max = Vector3Max(buckets[i].bounds.max, bounds_lft.max);
			area_lft[i] = BoxSurfaceArea(bounds_lft);

			short id_rgt = BUCKET_COUNT - 2 - i;
			sum_rgt += buckets[BUCKET_COUNT - 1 - i].count;
			count_rgt[id_rgt] = sum_rgt;
			bounds_rgt.min = Vector3Min(buckets[BUCKET_COUNT - 1 - i].bounds.min, bounds_rgt.min);
			bounds_rgt.max = Vector3Max(buckets[BUCKET_COUNT - 1 - i].bounds.max, bounds_rgt.max);
			area_rgt[id_rgt] = BoxSurfaceArea(bounds_rgt);
		}

		scale = (bmax - bmin) / BUCKET_COUNT;

		for(short i = 0; i < BUCKET_COUNT - 1; i++) {
			float cost = (count_lft[i] * area_lft[i] + count_rgt[i] * area_rgt[i]) / BoxSurfaceArea(node->bounds);

			if(cost < best_cost) {
				*axis = a;
				*split_pos = bmin + scale * (i + 1); 
				best_cost = cost;
			}
		}
	}

	return best_cost;
}

void BvhNodeSubdivideSahFast(Map *map, u16 root_id) {
	//WaitTime(0.1f);
	//WaitTime(0);

	BvhNode *node = &map->bvh_nodes[root_id];

	// Stop recursion
	if(node->tri_count <= MAX_TRIS_PER_NODE) return;

	// Find longest axis distance to use for split
	short best_axis = -1;
	float best_pos = 0;

	float best_cost = FindBestSplitPlane(map, node, &best_axis, &best_pos);

	float parent_cost = BvhNodeCost(node);
	if(best_cost > parent_cost) return;

	short split_axis = best_axis;
	float split_pos = best_pos;

	// In-place partition
	u16 i = node->first_tri;
	u16 j = i + node->tri_count - 1;
	while(i <= j) {
		Tri *tri = &map->tris[map->tri_ids[i]];
		
		Vector3 centroid = TriCentroid(tri);
		float3 c = Vector3ToFloatV(centroid);

		if(c.v[split_axis] < split_pos) { 
			i++;
		} else {
			// Swap with tri with tri at end of list
			SwapTriIds(&map->tri_ids[i], &map->tri_ids[j--]);
		}
	}	
	
	// Cancel splitting if either side is empty 
	u16 count_lft = i - node->first_tri;  
	if(count_lft == 0 || count_lft == node->tri_count) return;

	if(map->bvh_node_count + 2 >= map->bvh_node_capacity) {
		map->bvh_node_capacity = (map->bvh_node_capacity << 1);
		map->bvh_nodes = realloc(map->bvh_nodes, sizeof(BvhNode) * map->bvh_node_capacity);
	}

	// Create child nodes
	u16 node_id_lft = map->bvh_node_count++;
	u16 node_id_rgt = map->bvh_node_count++;

	map->bvh_nodes[node_id_lft] = (BvhNode) {
		.bounds = node->bounds,
		.first_tri = node->first_tri,
		.tri_count = count_lft,
		.child_lft = 0,
		.child_rgt = 0
	};
	node->child_lft = node_id_lft;

	map->bvh_nodes[node_id_rgt] = (BvhNode) {
		.bounds = node->bounds,
		.first_tri = i,
		.tri_count = node->tri_count - count_lft,
		.child_lft = 0,
		.child_rgt = 0
	};
	node->child_rgt = node_id_rgt;

	node->tri_count = 0;

	BvhNodeUpdateBounds(map, node_id_lft);
	BvhNodeUpdateBounds(map, node_id_rgt);

	BvhNodeSubdivideSahFast(map, node_id_lft);
	BvhNodeSubdivideSahFast(map, node_id_rgt);
}

void BoundsGrow(BoundingBox *bounds, Vector3 point) {
	for(short j = 0; j < 3; j++) {
		bounds->min = (Vector3) {
			fminf(bounds->min.x, point.x),
			fminf(bounds->min.y, point.y),
			fminf(bounds->min.z, point.z),
		};

		bounds->max = (Vector3) {
			fmaxf(bounds->max.x, point.x),
			fmaxf(bounds->max.y, point.y),
			fmaxf(bounds->max.z, point.z),
		};
	}
}

void BvhTraceNodes(Ray ray, u16 root_node, u16 *hits, u16 *hit_count, Map *map, u16 *tri_tests, u16 *branch_hits, u16 *branch_count, float *max_dist) {
	BvhNode *node = &map->bvh_nodes[root_node];

	RayCollision coll = GetRayCollisionBox(ray, node->bounds);	
	if(!coll.hit) return;

	if(coll.distance > *max_dist) return;

	bool is_leaf = ((node->child_lft + node->child_rgt) == 0);
	if(is_leaf) {
		bool tri_hit = false;
		u16 tests = 0;
		
		float best_hit_dist = *max_dist;

		for(u16 i = 0; i < node->tri_count; i++) {
			u16 tri_id = map->tri_ids[node->first_tri + i];
			Tri *tri = &map->tris[tri_id];

			tests++;

			coll = GetRayCollisionTriangle(ray, tri->vertices[0], tri->vertices[1], tri->vertices[2]);
			if(coll.distance > *max_dist) continue;

			if(coll.hit && coll.distance < best_hit_dist) {
				*max_dist = fminf(*max_dist, coll.distance + EPSILON);
				best_hit_dist = coll.distance;
				tri_hit = true;
				//break;
			}
		}

		*max_dist = best_hit_dist;
		*tri_tests += tests;

		if(tri_hit) 
			hits[(*hit_count)++] = root_node;

		return;
	}		

	branch_hits[(*branch_count)++] = root_node;

	float dist_a = FLT_MAX, dist_b = FLT_MAX;

	RayCollision coll_a = GetRayCollisionBox(ray, map->bvh_nodes[node->child_lft].bounds);
	RayCollision coll_b = GetRayCollisionBox(ray, map->bvh_nodes[node->child_rgt].bounds);

	if(coll_a.hit) dist_a = coll_a.distance;
	if(coll_b.hit) dist_b = coll_b.distance;

	if(dist_a < *max_dist) BvhTraceNodes(ray, node->child_lft, hits, hit_count, map, tri_tests, branch_hits, branch_count, max_dist);
	if(dist_b < *max_dist) BvhTraceNodes(ray, node->child_rgt, hits, hit_count, map, tri_tests, branch_hits, branch_count, max_dist);
}

u16 NodeDepth(BvhNode *node, Map *map) {
	if(!node) return 0;
	
	bool is_leaf = node->tri_count > 0;
	if(is_leaf) return 1;

	u16 l = NodeDepth(&map->bvh_nodes[node->child_lft], map);
	u16 r = NodeDepth(&map->bvh_nodes[node->child_rgt], map);

	return (u16)(1 + fmax(l, r));
}

