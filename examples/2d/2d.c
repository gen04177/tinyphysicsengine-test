/** Demo showing how 2D physics can be implemented. */

#define DEBUG_DRAW_DIVIDE 8

#include "../../helper.h"

#define ROOM_W (TPE_F * 10)
#define ROOM_H ((RES_Y * ROOM_W) / RES_X)

TPE_Vec3 environmentDistance(TPE_Vec3 p, TPE_Unit maxD)
{
  return TPE_envAABoxInside(p,TPE_vec3(0,0,0),TPE_vec3(ROOM_W,ROOM_H,ROOM_W));
}

int inactiveCount = 0;

void printHelp(void)
{
    printf("2D: example program for tinyphysicsengine.\n\n");

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

  tpe_world.environmentFunction = environmentDistance;

  s3l_scene.camera.transform.translation.z -= ROOM_W / 2;

  s3l_scene.camera.focalLength = 0; // set orthographic projection

  for (int i = 0; i < 4; ++i) // add bodies
  {
    if (i != 2)
    {
      helper_addCenterRectFull(TPE_F,TPE_F,TPE_F / 5,TPE_F / 5);
      TPE_bodyRotateByAxis(&helper_lastBody,TPE_vec3(TPE_F / 4,0,0));
      helper_lastBody.joints[4].sizeDivided *= 3; // make center point bigger
    }
    else
      helper_addBall(6 * TPE_F / 5,TPE_F / 5);

    helper_lastBody.friction = 4 * TPE_F / 5;
    helper_lastBody.elasticity = TPE_F / 5;

    TPE_bodyMoveBy(&helper_lastBody,TPE_vec3(-2 * TPE_F + i * 2 * TPE_F,0,0));
  }
    
  while (helper_running)
  {
    helper_frameStart();

#define ACCELERATION (TPE_F / 25)
    int leftStickX = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX);
    int leftStickY = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY);

    if (leftStickX < -8000)
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(-1 * ACCELERATION,0,0));
    else if (leftStickX > 8000)
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(ACCELERATION,0,0));
    
    if (leftStickY < -8000)
      TPE_bodyAccelerate(&tpe_world.bodies[0],TPE_vec3(0,ACCELERATION,0));
#undef ACCELERATION

    TPE_worldStep(&tpe_world);

    for (int i = 0; i < tpe_world.bodyCount; ++i)
      TPE_bodyApplyGravity(&tpe_world.bodies[i],TPE_F / 100);

    /* Here we implement our own improvement of deactivation; after some time of
       all bodies having low speed we disable them all at once. */
    TPE_Unit speed, speedMax = 0;
    int anyActive = 0;

    for (int i = 0; i < tpe_world.bodyCount; ++i)
    {
      // as we're in 2D we'll keep all joint Z positions and velocities at 0
      for (int j = 0; j < tpe_world.bodies[i].jointCount; ++j)
      {
        tpe_world.bodies[i].joints[j].position.z = 0;
        tpe_world.bodies[i].joints[j].velocity[2] = 0;
      }

      if (!(tpe_world.bodies[i].flags & TPE_BODY_FLAG_DEACTIVATED))
        anyActive = 1;

      speed = TPE_bodyGetAverageSpeed(&tpe_world.bodies[i]);

      if (speed > speedMax)
        speedMax = speed;
    }

    if (anyActive && speedMax < TPE_F / 10)
      inactiveCount++;
    else
      inactiveCount = 0;

    if (inactiveCount > 100)
    {
      TPE_worldDeactivateAll(&tpe_world);
      inactiveCount = 0;
    }

    helper_debugDraw(0);

    helper_frameEnd();
  }

  helper_end();

  SDL_GameControllerClose(controller);

  return 0;
}