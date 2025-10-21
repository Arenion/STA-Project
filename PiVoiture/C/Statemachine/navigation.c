#include "navigation.h"
#include <stdbool.h>
#include <pthread.h>
#include <assert.h>
#include "../global.h"
#include "../communicationarduino.h"
#include "map.h"

enum states_navigation
{
    GENERE_ITINERAIRE,
    ETAPE,
};

bool is_obstacle()
{
    return false;
}

struct etape
{
    bool (*fct)(void *arg);
    void *fct_arg;
    struct polygon stop;
};

struct itineraire
{
    struct etape *etapes;
    int n_etapes;
};

bool step_navigation(bool entering)
{
    static enum states_navigation current_state = GENERE_ITINERAIRE;
    static struct itineraire current_itineraire;
    static int current_etape;
    static bool first_obstacle_seen = true;
    static struct position current_objective;
    struct position current_position;

    if (entering)
        current_state = GENERE_ITINERAIRE;

    switch (current_state)
    {
    case GENERE_ITINERAIRE:
        pthread_mutex_lock(&MUTEX_POSITION);
        current_position = POSITION;
        pthread_mutex_unlock(&MUTEX_POSITION);
        // Find closest map node to start
        struct map_node *start_map_node = closest_map_node(current_position);

        pthread_mutex_lock(&MUTEX_NEXTOBJECTIF);
        current_objective = NEXTOBJECTIF;
        pthread_mutex_unlock(&MUTEX_NEXTOBJECTIF);
        // Find closest map node to objective
        struct map_node *end_map_node = closest_map_node(current_objective);

        // Find shortest node path

        // Generate steps

        break;
    case ETAPE:
        // Check for obstacles
        if (is_obstacle())
        {
            if (first_obstacle_seen)
            {
                stopcommand();
                first_obstacle_seen = false;
            }
            break;
        }
        else if (!first_obstacle_seen)
        {
            first_obstacle_seen = true;
        }

        // Execute step function
        if (!current_itineraire.etapes[current_etape].fct(current_itineraire.etapes[current_etape].fct_arg))
            break; // We must not go to next step

        // Check if step finished
        pthread_mutex_lock(&MUTEX_POSITION);
        current_position = POSITION;
        pthread_mutex_unlock(&MUTEX_POSITION);
        if (point_inside_polygon(current_position, current_itineraire.etapes[current_etape].stop)) // are we arrived to end of step ?
        {
            current_etape++;
            if (current_etape == current_itineraire.n_etapes)
                return true; // Arrived at destination, telling state machine to go to pause state.
        }
        break;
    }
    return false; // Telling state machine we have not arrived yet.
}

// probleme : Nicolas ça marche pas ta fct lol.