/** Demo showing the heightmap environment. */

#define CAMERA_STEP 200
#define HEIGHTMAP_3D_RESOLUTION 32
#define HEIGHTMAP_3D_STEP (TPE_F * 2)
#define MAP_LIMIT ((HEIGHTMAP_3D_RESOLUTION * HEIGHTMAP_3D_STEP) / 2)

#include "../../helper.h"

/** For given heightmap node at [x,y] returns its height. Here the function uses
  sines/cosines to generate a simple procedural heightmap but it could also
  retrieve the height e.g. from an image. */
TPE_Unit height(int32_t x, int32_t y)
{
  x *= 8;
  y *= 8;

  return
    TPE_sin(x + TPE_cos(y * 2)) * TPE_sin(y * 2 + TPE_cos(x * 4)) / (TPE_F / 2);
}

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envHeightmap(p,TPE_vec3(0,0,0),HEIGHTMAP_3D_STEP,height,maxD);
}

void printHelp(void)
{
    printf("Heightmap: example program for tinyphysicsengine.\n\n");

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

  helper_debugDrawOn = 0;

  // here we just set up the graphical 3D model of the heigtmap: 

  for (int y = 0; y < HEIGHTMAP_3D_RESOLUTION; ++y)
    for (int x = 0; x < HEIGHTMAP_3D_RESOLUTION; ++x)
      helper_setHeightmapPoint(x,y,height(x - HEIGHTMAP_3D_RESOLUTION / 2,y - HEIGHTMAP_3D_RESOLUTION / 2));

  tpe_world.environmentFunction = environmentDistance;

  helper_addBox(700,700,700,300,1000);

  TPE_bodyMoveTo(&helper_lastBody,TPE_vec3(0,5000,0));

  s3l_scene.camera.transform.rotation.x = -1 * TPE_F / 8;

  while (helper_running)
  {
    helper_frameStart();

    TPE_Vec3 bodyCenter = TPE_bodyGetCenterOfMass(&tpe_world.bodies[0]);

    s3l_scene.camera.transform.translation.x = bodyCenter.x;
    s3l_scene.camera.transform.translation.y = bodyCenter.y + 4000;
    s3l_scene.camera.transform.translation.z = bodyCenter.z - 4000;

    if (bodyCenter.x < -1 * MAP_LIMIT)
      TPE_bodyMoveBy(&tpe_world.bodies[0],TPE_vec3(2 * MAP_LIMIT,TPE_F,0));
    else if (bodyCenter.x > MAP_LIMIT)
      TPE_bodyMoveBy(&tpe_world.bodies[0],TPE_vec3(-2 * MAP_LIMIT,TPE_F,0));

    if (bodyCenter.z < -1 * MAP_LIMIT)
      TPE_bodyMoveBy(&tpe_world.bodies[0],TPE_vec3(0,TPE_F,2 * MAP_LIMIT));
    else if (bodyCenter.z > MAP_LIMIT)
      TPE_bodyMoveBy(&tpe_world.bodies[0],TPE_vec3(0,TPE_F,-2 * MAP_LIMIT));

    if (helper_frame % 32 == 0)
      helper_printCPU();

    helper_set3DColor(0,200,100);
    helper_drawModel(&heightmapModel,TPE_vec3(-HEIGHTMAP_3D_STEP / 2,0,-HEIGHTMAP_3D_STEP / 2),TPE_vec3(512,512,512),TPE_vec3(0,0,0));

    helper_set3DColor(100,100,200);
    helper_draw3DBox(bodyCenter,TPE_vec3(2 * TPE_F,2 * TPE_F,2 * TPE_F),
      TPE_bodyGetRotation(&tpe_world.bodies[0],0,1,2));

    for (int i = 0; i < tpe_world.bodyCount; ++i)
      TPE_bodyApplyGravity(&tpe_world.bodies[i],7);

    TPE_worldStep(&tpe_world);

    // SDL Game Controller input
    #define ACC (TPE_F / 50)
    if (SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY) < -8000)
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(0,0,ACC));
    else if (SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY) > 8000)
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(0,0,-1 * ACC));

    if (SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX) < -8000)
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(-1 * ACC,0,0));
    else if (SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX) > 8000)
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(ACC,0,0));

    if (helper_debugDrawOn)
      helper_debugDraw(1);
    #undef ACC

    helper_frameEnd();
  }

  helper_end();

  SDL_GameControllerClose(controller);

  return 0;
}