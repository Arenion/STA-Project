#include <stdbool.h>
#include <pthread.h>

#include "statemachine.h"
#include "navigation.h"
#include "../communicationarduino.h"

#include "../global.h"

enum states {
    PAUSE,
    NAVIGATION,
};

bool is_new_objective()
{
    static struct position last_objective = OBJECTIF_STOP;
    pthread_mutex_lock(&MUTEX_NEXTOBJECTIF);
    if (NEXTOBJECTIF.x == last_objective.x && NEXTOBJECTIF.y == last_objective.y)
    {
        pthread_mutex_unlock(&MUTEX_NEXTOBJECTIF);
        return true;
    }
    else
    {
        last_objective = NEXTOBJECTIF;
        pthread_mutex_unlock(&MUTEX_NEXTOBJECTIF);
        return false;
    }
}

bool is_objective_stop()
{
    pthread_mutex_lock(&MUTEX_NEXTOBJECTIF);
    bool result = (NEXTOBJECTIF.x ==  OBJECTIF_STOP.x && NEXTOBJECTIF.y == OBJECTIF_STOP.y);
    pthread_mutex_unlock(&MUTEX_NEXTOBJECTIF);
    return result;
}

void step_statemachine()
{
    static enum states current_state = PAUSE;
    static bool entering = true;

    switch (current_state)
    {
    case PAUSE:
        if (entering)
        {
            stopcommand();
            entering = false;
        }
        if (is_new_objective())
        {
            if (is_objective_stop())
            {
                current_state = PAUSE;
            }
            else
            {
                current_state = NAVIGATION;
            }
            entering = true;
        }
        break;
    case NAVIGATION:
        if (is_new_objective())
        {
            if (is_objective_stop())
            {
                current_state = PAUSE;
            }
            else
            {
                current_state = NAVIGATION;
            }
            entering = true;
        }
        if (step_navigation(entering))
        {
            current_state = PAUSE;
            entering = true;
        }
        else
        {
            entering = false;
        }
        break;
    }
}