#include "navigation.h"
#include <stdbool.h>
#include <pthread.h>
#include <assert.h>
#include "../global.h"

enum states_navigation
{
    GENERE_ITINERAIRE,
    ETAPE,
};

struct polygon
{
    int n_vertices;
    struct position *vertices;
};

int point_vertices_side(struct position p1, struct position p2, struct position pos)
{
    return (pos.x - p1.x) * (p2.y - p1.y) - (pos.y - p1.y) * (p2.x - p1.x);
}

// Ne fonctionne qu'avec des formes convexes !
bool point_inside_polygon(struct position point, struct polygon poly)
{
    assert(poly.n_vertices >= 3);
    int dir = point_vertices_side(poly.vertices[0], poly.vertices[1], point);
    for (int i = 1; i + 1 < poly.n_vertices; ++i)
    {
        if (dir * point_vertices_side(poly.vertices[i], poly.vertices[i + 1], point) < 0)
            return false;
    }
    return dir * point_vertices_side(poly.vertices[poly.n_vertices - 1], poly.vertices[0], point) >= 0; // test last segment
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
    if (entering)
        current_state = GENERE_ITINERAIRE;

    switch (current_state)
    {
    case GENERE_ITINERAIRE:
        // à faire !
        break;
    case ETAPE:
        if (!current_itineraire.etapes[current_etape].fct(current_itineraire.etapes[current_etape].fct_arg)) break; // We must not go to next step
        pthread_mutex_lock(&MUTEX_POSITION);
        struct position current_position = POSITION;
        pthread_mutex_unlock(&MUTEX_POSITION);
        if (point_inside_polygon(current_position, current_itineraire.etapes[current_etape].stop)) // are we arrived to end of step ?
        {
            current_etape++;
            if (current_etape == current_itineraire.n_etapes) return true; // Arrived at destination, telling state machine to go to pause state.
        }
        break;
    }
    return false; // Telling state machine we have not arrived yet.
}

// probleme : Nicolas ça marche pas ta fct lol.

// Detection d'obstacle ??