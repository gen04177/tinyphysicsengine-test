#define CAMERA_STEP (TPE_F / 2)
#define JOINT_SIZE (TPE_F / 4)
#define BALL_SIZE (5 * TPE_F / 4)

#define HEIGHTMAP_3D_RESOLUTION 8
#define HEIGHTMAP_3D_STEP (TPE_F * 2)

#include "../../helper.h"

#define ROOM_SIZE (HEIGHTMAP_3D_RESOLUTION * HEIGHTMAP_3D_STEP + JOINT_SIZE)

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envAABoxInside(p,TPE_vec3(0,0,0),TPE_vec3(ROOM_SIZE,ROOM_SIZE,ROOM_SIZE));
}

#define WATER_JOINTS (HEIGHTMAP_3D_RESOLUTION * HEIGHTMAP_3D_RESOLUTION)
#define WATER_CONNECTIONS (2 * ((HEIGHTMAP_3D_RESOLUTION - 1) * HEIGHTMAP_3D_RESOLUTION))

TPE_Joint joints[WATER_JOINTS + 1];
TPE_Connection connections[WATER_CONNECTIONS];
TPE_Body bodies[2];

void printHelp(void)
{
    printf("Water: example program for tinyphysicsengine.\n\n");

    printf("\nby Miloslav Ciz, released under CC0 1.0\n");

    printf("\n@gen04177 v0.1\n");
}

int main(void)
{
  printHelp();
  
  if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0)
  {
      printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
      return 1;
  }

  SDL_GameController *controller = SDL_GameControllerOpen(0);
  if (controller == NULL)
  {
      printf("Could not open gamecontroller! SDL_Error: %s\n", SDL_GetError());
      return 1;
  }

  helper_init();

  puts("Use the left stick to move the ball, triggers to move up/down.");

  s3l_scene.camera.transform.translation.z = -20 * TPE_F;
  s3l_scene.camera.transform.translation.y = 6 * TPE_F;
  s3l_scene.camera.transform.translation.x = ROOM_SIZE / 2 - 8 * TPE_F; 
  s3l_scene.camera.transform.rotation.y = 0;

  // build the water body:

  for (int i = 0; i < HEIGHTMAP_3D_POINTS; ++i)
    joints[i] = TPE_joint(helper_heightmapPointLocation(i),JOINT_SIZE);

  int index = 0;

  for (int j = 0; j < HEIGHTMAP_3D_RESOLUTION; ++j)
    for (int i = 0; i < HEIGHTMAP_3D_RESOLUTION - 1; ++i)
    {
      connections[index].joint1 = j * HEIGHTMAP_3D_RESOLUTION + i;
      connections[index].joint2 = connections[index].joint1 + 1;

      index++;

      connections[index].joint1 = i * HEIGHTMAP_3D_RESOLUTION + j;
      connections[index].joint2 = connections[index].joint1 + HEIGHTMAP_3D_RESOLUTION;

      index++;
    }

  TPE_bodyInit(&bodies[0],joints,WATER_JOINTS,connections,WATER_CONNECTIONS,
    2 * TPE_F);

  bodies[0].flags |= TPE_BODY_FLAG_SOFT;
  bodies[0].flags |= TPE_BODY_FLAG_ALWAYS_ACTIVE;

  // create the ball body:
  joints[WATER_JOINTS] = TPE_joint(TPE_vec3(0,0,ROOM_SIZE / 4),BALL_SIZE);
  TPE_bodyInit(&bodies[1],joints + WATER_JOINTS,1,connections,0,200);

  bodies[1].flags |= TPE_BODY_FLAG_ALWAYS_ACTIVE;

  TPE_worldInit(&tpe_world,bodies,2,environmentDistance);

  while (helper_running)
  {
    helper_frameStart();

    helper_cameraFreeMovement();

    // update the 3D model vertex positions:

    S3L_Unit *v = heightmapVertices;

    for (int i = 0; i < WATER_JOINTS; ++i)
    {
      *v = joints[i].position.x;
      v++;
      *v = joints[i].position.y;
      v++;
      *v = joints[i].position.z;
      v++;
    }

    // pin the joints at the edges of the grid:

    for (int index = 0; index < WATER_JOINTS; ++index)
      if (index % HEIGHTMAP_3D_RESOLUTION == 0 || index % HEIGHTMAP_3D_RESOLUTION == HEIGHTMAP_3D_RESOLUTION - 1 ||
        index / HEIGHTMAP_3D_RESOLUTION == 0 || index / HEIGHTMAP_3D_RESOLUTION == HEIGHTMAP_3D_RESOLUTION - 1)
        TPE_jointPin(&joints[index],helper_heightmapPointLocation(index));

    TPE_worldStep(&tpe_world);

#define G ((5 * 30) / FPS)
    TPE_bodyApplyGravity(&tpe_world.bodies[1],
      bodies[1].joints[0].position.y > 0 ? G : (-2 * G));

#define ACC ((25 * 30) / FPS)
    // SDL Game Controller inputs for movement
    int left_x = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
    int left_y = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);
    int right_trigger = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
    int left_trigger = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT);

    if (left_y < -8000) // Move forward
      TPE_bodyAccelerate(&bodies[1], TPE_vec3(0, 0, ACC));
    else if (left_y > 8000) // Move backward
      TPE_bodyAccelerate(&bodies[1], TPE_vec3(0, 0, -1 * ACC));

    if (left_x > 8000) // Move right
      TPE_bodyAccelerate(&bodies[1], TPE_vec3(ACC, 0, 0));
    else if (left_x < -8000) // Move left
      TPE_bodyAccelerate(&bodies[1], TPE_vec3(-1 * ACC, 0, 0));

    if (right_trigger > 8000) // Move up
      TPE_bodyAccelerate(&bodies[1], TPE_vec3(0, ACC, 0));
    else if (left_trigger > 8000) // Move down
      TPE_bodyAccelerate(&bodies[1], TPE_vec3(0, -1 * ACC, 0));

    helper_set3DColor(255,0,0);
    helper_draw3DSphere(bodies[1].joints[0].position,TPE_vec3(BALL_SIZE,BALL_SIZE,BALL_SIZE),TPE_vec3(0,0,0));
    helper_set3DColor(0,100,255);
    helper_drawModel(&heightmapModel,TPE_vec3(0,0,0),TPE_vec3(TPE_F,TPE_F,TPE_F),TPE_vec3(0,0,0));

    if (helper_debugDrawOn)
      helper_debugDraw(1);

    helper_frameEnd();
  }

  helper_end();
  SDL_GameControllerClose(controller);

  return 0;
}
