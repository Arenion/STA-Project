#pragma once

#include <stdbool.h>
#include "../global.h"


struct polygon
{
    int n_vertices;
    struct position *vertices;
};

struct line
{
    int n_vertices;
    struct position *vertices;
};

int point_vertices_side(struct position p1, struct position p2, struct position pos);

// Ne fonctionne qu'avec des formes convexes !
bool point_inside_polygon(struct position point, struct polygon poly);

float point_segment_distance2(struct position p1, struct position p2, struct position point);
float point_line_distance2(struct line line, struct position point, struct position return_segment[2]);

struct map_node; // Forward declaration

struct map_node_list
{
    int n_nodes;
    struct map_node **nodes;
};

struct map_node
{
    struct line line; // The line of the node. It contains the start and end points, going in the clockwise direction.
    struct polygon start_poly;
    struct polygon stop_poly;
    float weight;
    bool (*passing_fct)(void *arg);
    void *passing_fct_arg;
    struct map_node_list previous_nodes;
    struct map_node_list next_nodes;
};

extern struct map_node_list MAP;

struct map_node *closest_map_node(struct position point);

