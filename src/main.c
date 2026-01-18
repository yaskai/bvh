#include <math.h>
#include "raylib.h"
#include "raymath.h"
#include "map.h"

#define CAM_SENSITIVITY		0.275f
#define CAM_UP				(Vector3) { 0, 1, 0 }
#define CAM_MAX_PITCH		(89.0f * DEG2RAD) 
#define CAM_SPEED			10.0f

// For first-person camera movment
// Pitch, yaw, roll
float cam_p, cam_y, cam_r;

void CameraControls(Camera3D *cam, float dt);

Material default_mat;

#define LAVENDER (Color) { 220, 208, 255, 255 }

int main() {
	SetTraceLogLevel(LOG_ERROR);
	SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_VSYNC_HINT);

	InitWindow(1920, 1080, "Bvh Demo");
	DisableCursor();
	
	Camera3D camera = (Camera3D) {
		.position = (Vector3) { 0, 10, 0 },
		.target = (Vector3) { 0, 0, -10 },
		.up = (Vector3) CAM_UP,
		.fovy = 90,
		.projection = CAMERA_PERSPECTIVE
	};

	Map map = (Map) {0};

	char *map_name = "HyruleField.glb";
	MapInit(&map, map_name);

	default_mat = LoadMaterialDefault();

	Font font = LoadFont("resources/fonts/v5easter.ttf");
	//Font font = LoadFontEx("resources/fonts/blex.ttf", 24, 0, 0);

	while(!WindowShouldClose()) {
		float delta_time = GetFrameTime();

		CameraControls(&camera, delta_time);

		BeginDrawing();
		ClearBackground(BLACK);

		BeginMode3D(camera);
		
		DrawModel(map.model, Vector3Zero(), 1, WHITE);

		for(u32 i = 0; i < map.tri_count; i++) { 
			Tri *tri = &map.tris[i];

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
				ColorAlpha(norm_color, 0.25f)
				);

			DrawLine3D(tri->vertices[0], tri->vertices[1], SKYBLUE);
			DrawLine3D(tri->vertices[1], tri->vertices[2], SKYBLUE);
			DrawLine3D(tri->vertices[2], tri->vertices[0], SKYBLUE);

			Vector3 norm_start = TriCenter(tri);
			Vector3 norm_dest = Vector3Add(norm_start, Vector3Scale(tri->normal, 1));
			DrawLine3D(norm_start, norm_dest, norm_color);
		}

		DrawBoundingBox(map.bvh_nodes[0].bounds, RED);
	
		//DrawMesh(map.model.meshes[i], default_mat, MatrixIdentity());	
		//DrawModelWires(map.model, Vector3Zero(), 1, BLACK);

		//DrawSphere(map.mid, 1, RED);

		EndMode3D();

		DrawTextEx(font, TextFormat("fps: %d", GetFPS()), (Vector2) { 0, 0 }, 32, 1, LAVENDER);
		DrawTextEx(font, TextFormat("file: %s", map_name), (Vector2) { 0, 40 }, 32, 1, LAVENDER);

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

