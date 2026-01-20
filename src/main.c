#include <math.h>
#include <float.h>
#include <stdlib.h>
#include "raylib.h"
#include "raymath.h"
#include "map.h"

#define CAM_SENSITIVITY		0.275f
#define CAM_UP				(Vector3) { 0, 1, 0 }
#define CAM_MAX_PITCH		(89.0f * DEG2RAD) 
#define CAM_SPEED			10.0f

#define VIS_NODE_CAP		512

// For first-person camera movment
// Pitch, yaw, roll
float cam_p, cam_y, cam_r;

void CameraControls(Camera3D *cam, float dt);
void VirtCameraControls(Camera3D *cam, float dt);

Material default_mat;

#define LAVENDER (Color) { 220, 208, 255, 255 }

int main() {
	SetTraceLogLevel(LOG_ERROR);
	SetConfigFlags(FLAG_VSYNC_HINT);

	InitWindow(1920, 1080, "Bvh Demo");
	DisableCursor();
	
	Camera3D camera = (Camera3D) {
		.position = (Vector3) { 30, 30, 30 },
		.target = (Vector3) { 0, 0, 0 },
		.up = (Vector3) CAM_UP,
		.fovy = 90,
		.projection = CAMERA_PERSPECTIVE
	};

	Camera3D virt_cam = camera;

	Map map = (Map) {0};

	//char *map_name = "HyruleField.glb";
	char *map_name = "e1m1.glb";
	MapInit(&map, map_name);

	default_mat = LoadMaterialDefault();

	Font font = LoadFont("resources/fonts/v5easter.ttf");

	RenderTexture rt = LoadRenderTexture(1280, 720);

	Vector3 cam_dir;
	Vector3 ray_pos;
	Vector3 ray_dest;

	u16 *vis_nodes = calloc(VIS_NODE_CAP, sizeof(u16));
	u16 vis_node_count = 0;
	u16 tri_tests = 0;

	u16 *branch_hits = calloc(VIS_NODE_CAP, sizeof(u16));
	u16 branch_hit_count = 0;

	while(!WindowShouldClose()) {
		float delta_time = GetFrameTime();

		CameraControls(&camera, delta_time);
		VirtCameraControls(&virt_cam, delta_time);
		cam_dir = Vector3Normalize(Vector3Subtract(camera.target, camera.position));

		ray_pos = Vector3Add(camera.position, Vector3Scale(cam_dir, 0));
		Ray ray = (Ray) { .position = ray_pos, .direction = cam_dir };

		vis_node_count = 0;
		tri_tests = 0;
		branch_hit_count = 0;
		float max_dist = FLT_MAX;
		BvhTraceNodes(ray, 0, vis_nodes, &vis_node_count, &map, &tri_tests, branch_hits, &branch_hit_count, &max_dist);

		BeginDrawing();

		BeginTextureMode(rt);
		ClearBackground(BLANK);

		BeginMode3D(virt_cam);

		/*
		for(u32 i = 0; i < map.tri_count; i++) { 
			Tri *tri = &map.tris[map.tri_ids[i]];

			Color norm_color = (Color) {
				.r = tri->normal.x * 255, 
				.g = tri->normal.y * 255, 
				.b = tri->normal.z * 255,
				.a = 255
			};
			
			DrawTriangle3D(
				tri->vertices[0],
				tri->vertices[1],
				tri->vertices[2],
				ColorAlpha(DARKGRAY, 1.0f)
				);

			DrawLine3D(tri->vertices[0], tri->vertices[1], SKYBLUE);
			DrawLine3D(tri->vertices[1], tri->vertices[2], SKYBLUE);
			DrawLine3D(tri->vertices[2], tri->vertices[0], SKYBLUE);

			Vector3 norm_start = TriCentroid(tri);
			Vector3 norm_dest = Vector3Add(norm_start, Vector3Scale(tri->normal, 1));
			DrawLine3D(norm_start, norm_dest, norm_color);
		}
		*/

		DrawModel(map.model, Vector3Zero(), 1, DARKGRAY);
		//DrawModelWires(map.model, Vector3Zero(), 1, SKYBLUE);

		/*
		for(u16 i = 0; i < map.bvh_node_count; i++) {
			BvhNode *node = &map.bvh_nodes[i];

			bool is_leaf = (node->tri_count > 0);
			if(!is_leaf) continue;
			Color color = DARKBLUE;

			DrawBoundingBox(node->bounds, ColorAlpha(color, 0.75f));
		}
		*/

		for(u16 i = 0; i < branch_hit_count; i++) {
			BvhNode *node = &map.bvh_nodes[branch_hits[i]];
			DrawBoundingBox(node->bounds, ColorAlpha(DARKGREEN, 0.85f));
		}
		
		for(u16 i = 0; i < vis_node_count; i++) {
			BvhNode *node = &map.bvh_nodes[vis_nodes[i]];
			DrawBoundingBox(node->bounds, GREEN);

			for(u16 i = 0; i < node->tri_count; i++) {
				u16 tri_id = map.tri_ids[node->first_tri + i];
				Tri *tri = &map.tris[tri_id];

				/*
				Color norm_color = (Color) {
					.r = tri->normal.x * 255, 
					.g = tri->normal.y * 255, 
					.b = tri->normal.z * 255,
					.a = 255
				};

				DrawTriangle3D(
					tri->vertices[0],
					tri->vertices[1],
					tri->vertices[2],
					norm_color
				);
				*/

				DrawTriangle3D(
					tri->vertices[0],
					tri->vertices[1],
					tri->vertices[2],
					ColorAlpha(GREEN, 0.5f)
				);

				DrawLine3D(tri->vertices[0], tri->vertices[1], GREEN);
				DrawLine3D(tri->vertices[1], tri->vertices[2], GREEN);
				DrawLine3D(tri->vertices[2], tri->vertices[0], GREEN);
			}
		}

		DrawRay(ray, SKYBLUE);
		DrawSphere(Vector3Add(ray.position, Vector3Scale(ray.direction, max_dist)), 0.1f, SKYBLUE);

		EndMode3D();
		EndTextureMode();

		ClearBackground(BLACK);

		BeginMode3D(camera);
		
		DrawModel(map.model, Vector3Zero(), 1, WHITE);

		//DrawMesh(map.model.meshes[i], default_mat, MatrixIdentity());	
		//DrawModelWires(map.model, Vector3Zero(), 1, BLACK);

		EndMode3D();

		DrawTexturePro(
			rt.texture,
			(Rectangle) { 0, 0, rt.texture.width, -rt.texture.height },
			(Rectangle) { 0, 0, rt.texture.width, rt.texture.height},
			Vector2Zero(), 
			0, 
			WHITE
		);

		DrawTextEx(font, TextFormat("fps: %d", GetFPS()), (Vector2) { 0, 0 }, 32, 1, LAVENDER);
		DrawTextEx(font, TextFormat("file: %s", map_name), (Vector2) { 0, 40 }, 32, 1, LAVENDER);
		DrawTextEx(font, TextFormat("leaf hits: %d", vis_node_count), (Vector2) { 0, 80 }, 32, 1, LAVENDER);
		DrawTextEx(font, TextFormat("branch hits: %d", branch_hit_count), (Vector2) { 0, 120 }, 32, 1, LAVENDER);

		float tri_percent = (tri_tests > 0) ? ((float)tri_tests / map.tri_count) * 100.0f : 0; 
		DrawTextEx(font, TextFormat("tri tests: %d/%d, %%%.2f", tri_tests, map.tri_count, tri_percent), (Vector2) { 0, 160 }, 32, 1, LAVENDER);

		EndDrawing();
	}

	CloseWindow();
	MapClose(&map);

	return 0;
}

void CameraControls(Camera3D *cam, float dt) {
	Vector2 mouse_delta = GetMouseDelta();

	cam_p -= mouse_delta.y * CAM_SENSITIVITY * dt;
	cam_y += mouse_delta.x * CAM_SENSITIVITY * dt;
	cam_p = Clamp(cam_p, -CAM_MAX_PITCH, CAM_MAX_PITCH);	

	Vector3 forward = Vector3Normalize((Vector3) { .x = cosf(cam_y) * cosf(cam_p), .y = sinf(cam_p), .z = sinf(cam_y) * cosf(cam_p) }); 
	Vector3 right = Vector3CrossProduct(forward, CAM_UP);
	
	cam->target = Vector3Add(cam->position, forward); 

	Vector3 movement = Vector3Zero();	

	if(IsKeyDown(KEY_W)) movement = Vector3Add(movement, forward);
	if(IsKeyDown(KEY_D)) movement = Vector3Add(movement, right);
	if(IsKeyDown(KEY_S)) movement = Vector3Subtract(movement, forward);
	if(IsKeyDown(KEY_A)) movement = Vector3Subtract(movement, right);

	movement = Vector3Scale(movement, CAM_SPEED * dt);

	cam->position = Vector3Add(cam->position, movement);
	cam->target = Vector3Add(cam->target, movement);
}

void VirtCameraControls(Camera3D *cam, float dt) {
	Vector3 forward = Vector3Normalize(Vector3Subtract(cam->target, cam->position)); 
	Vector3 right = Vector3CrossProduct(forward, CAM_UP);
	
	Vector3 movement = Vector3Zero();	

	movement = Vector3Add(movement, Vector3Scale(forward, GetMouseWheelMove()));

	if(IsKeyDown(KEY_UP)) 		movement = Vector3Add(movement, CAM_UP);
	if(IsKeyDown(KEY_RIGHT)) 	movement = Vector3Add(movement, right);
	if(IsKeyDown(KEY_DOWN))		movement = Vector3Subtract(movement, CAM_UP);
	if(IsKeyDown(KEY_LEFT))		movement = Vector3Subtract(movement, right);

	movement = Vector3Scale(movement, CAM_SPEED * dt);

	cam->position = Vector3Add(cam->position, movement);
	cam->target = Vector3Add(cam->target, movement);
}
