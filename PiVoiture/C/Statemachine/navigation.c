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
    VERIFRESERVATION,
    ETAPE,
};

bool is_obstacle()
{
    return false;
}

bool step_navigation(bool entering)
{
    static enum states_navigation current_state = GENERE_ITINERAIRE;
    static struct map_node_list current_itineraire;
    static int current_etape;
    static bool first_reservation_demande = true;
    static bool first_obstacle_seen = true;
    static struct position current_objective;
    static struct map_node *node_array[MAX_MAP_SIZE];
    static struct map_node_list path = {
                .nodes = node_array};
    struct position current_position;
    struct map_node *start_map_node;
    struct map_node *end_map_node;

    if (entering)
        current_state = GENERE_ITINERAIRE;

    switch (current_state)
    {
    case GENERE_ITINERAIRE:
        // Find closest map node to start
        pthread_mutex_lock(&MUTEX_POSITION);
        current_position = POSITION;
        pthread_mutex_unlock(&MUTEX_POSITION);
        start_map_node = closest_map_node(current_position);

        // Find closest map node to objective
        pthread_mutex_lock(&MUTEX_NEXTOBJECTIF);
        current_objective = NEXTOBJECTIF;
        pthread_mutex_unlock(&MUTEX_NEXTOBJECTIF);
        end_map_node = closest_map_node(current_objective);

        // Find shortest node path
        find_shortest_path(start_map_node, end_map_node, &path);

        current_etape = 0;
        current_state = VERIFRESERVATION;
        break;
    case VERIFRESERVATION:
        pthread_mutex_lock(&MUTEX_RESERVATION);
        if (RESERVATION == current_itineraire.nodes[current_etape]->reservation)
        {
            first_reservation_demande = true;
            current_state = ETAPE;
        }
        else
        {
            demandereservation(current_itineraire.nodes[current_etape]->reservation);
            if (first_reservation_demande)
            {
                stopcommand();
                first_reservation_demande = false;
            }
        }
        pthread_mutex_unlock(&MUTEX_RESERVATION);
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

        // Execute step
        current_itineraire.nodes[current_etape]->passing_fct(current_itineraire.nodes[current_etape]);

        // Check if step finished
        pthread_mutex_lock(&MUTEX_POSITION);
        current_position = POSITION;
        pthread_mutex_unlock(&MUTEX_POSITION);
        if (point_after_segment(current_itineraire.nodes[current_etape]->line.vertices[current_itineraire.nodes[current_etape]->line.n_vertices-2], current_itineraire.nodes[current_etape]->line.vertices[current_itineraire.nodes[current_etape]->line.n_vertices-1], current_position)) // are we arrived to end of step ?
        {
            current_etape++;
            if (current_etape == current_itineraire.n_nodes)
                return true; // Arrived at destination, telling state machine to go to pause state.

            // Else we check the new step does not necesitate a new reservation.
            current_state = VERIFRESERVATION;
        }
        break;
    }
    return false; // Telling state machine we have not arrived yet.
}
