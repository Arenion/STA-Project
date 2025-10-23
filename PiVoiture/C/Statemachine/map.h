#pragma once

#include <stdbool.h>
#include "../global.h"

#define MAX_MAP_SIZE 100

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
bool point_after_segment(struct position p1, struct position p2, struct position point);

struct map_node; // Forward declaration

struct map_node_list
{
    int n_nodes;
    struct map_node **nodes;
};

struct map_node
{
    char *name;
    struct line line; // The line of the node. It contains the start and end points, going in the clockwise direction.
    float weight;
    void (*passing_fct)(struct map_node *map_node);
    enum reservation reservation;
    struct map_node_list next_nodes;
};

extern struct map_node_list MAP;
void map_init();

struct map_node *closest_map_node(struct position point);
float find_shortest_path(struct map_node *start, struct map_node *goal,
                         struct map_node_list *path_output);

float distance_between_positions(struct position p1, struct position p2);
