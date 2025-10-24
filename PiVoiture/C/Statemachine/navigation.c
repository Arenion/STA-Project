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
        printf("NAVIGATION: Début calcul du nouvel itinéraire.\n");
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

        printf("NAVIGATION: Fin calcul du nouvel itinéraire : ");
        for (int i = 0; i < path.n_nodes; ++i)
        {
            printf(path.nodes[i]->name);
            printf(" -->");
        }
        printf(" ARRIVEE;\n");

        current_etape = 0;
        printf("NAVIGATION : Début étape '%s'\n", path.nodes[current_etape]->name);
        current_state = VERIFRESERVATION;
        break;
    case VERIFRESERVATION:
        pthread_mutex_lock(&MUTEX_RESERVATION);
        if (RESERVATION == path.nodes[current_etape]->reservation || true)
        {
            printf("NAVIGATION: Réservation pour l'étape correcte.\n");
            pthread_mutex_unlock(&MUTEX_RESERVATION);
            first_reservation_demande = true;
            current_state = ETAPE;
        }
        else
        {
            pthread_mutex_unlock(&MUTEX_RESERVATION);
            demandereservation(path.nodes[current_etape]->reservation);
            if (first_reservation_demande)
            {
                printf("NAVIGATION: Réservation pour l'étape incorrecte, arrêt de la voiture.\n");
                stopcommand();
                first_reservation_demande = false;
            }
        }
        break;
    case ETAPE:
        // Check for obstacles
        if (is_obstacle())
        {
            if (first_obstacle_seen)
            {
                printf("NAVIGATION: Obstacle détécté ! Arrêt de la voiture.\n");
                stopcommand();
                first_obstacle_seen = false;
            }
            break;
        }
        else if (!first_obstacle_seen)
        {
            first_obstacle_seen = true;
        }

        printf("NAVIGATION: Execution de l'étape '%s'.\n", path.nodes[current_etape]->name);
        // Execute step
        path.nodes[current_etape]->passing_fct(path.nodes[current_etape]);

        // Check if step finished
        pthread_mutex_lock(&MUTEX_POSITION);
        current_position = POSITION;
        pthread_mutex_unlock(&MUTEX_POSITION);
        // TODO : gérer le point d'arrivé.
        // Vérification de si on est derrière le dernier segment du noeud, ou proche du dernier point.
        //printf("NAVIGATION: Vérification du fin d'étape\n");
        if (distance_between_positions(path.nodes[current_etape]->line.vertices[path.nodes[current_etape]->line.n_vertices - 1], current_position) < 10.0f || point_after_segment(path.nodes[current_etape]->line.vertices[path.nodes[current_etape]->line.n_vertices - 2], path.nodes[current_etape]->line.vertices[path.nodes[current_etape]->line.n_vertices - 1], current_position)) // are we arrived to end of step ?
        {
            printf("NAVIGATION: étape '%s' finie.\n", path.nodes[current_etape]->name);
            current_etape++;
            if (current_etape == path.n_nodes)
            {
                printf("NAVIGATION: arrivé à destination !\n");
                annoncereussiteobjectif();
                return true; // Arrived at destination, telling state machine to go to pause state.
            }
            printf("NAVIGATION: passage à l'étape '%s'\n", path.nodes[current_etape]->name);
            // TEMP
            printf("NAVIGATION: Attente temporaire pour démo\n");
            stopcommand();
            usleep(10000000);
            // TEMP
            // Else we check the new step does not necesitate a new reservation.
            current_state = VERIFRESERVATION;
        }
        break;
    }
    return false; // Telling state machine we have not arrived yet.
}
