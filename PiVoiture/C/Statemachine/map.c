#include "map.h"

#include <assert.h>
#include <math.h>

#include "../global.h"

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

float point_segment_distance2(struct position p1, struct position p2, struct position point)
{
    float px = (float)(p2.x - p1.x);
    float py = (float)(p2.y - p1.y);
    float norm2 = px * px + py * py;

    if (norm2 == 0.0f) // degenerate segment (single point)
        return (point.x - p1.x) * (point.x - p1.x) + (point.y - p1.y) * (point.y - p1.y);

    float u = ((point.x - p1.x) * px + (point.y - p1.y) * py) / norm2;

    u = fmaxf(0, fminf(1, u));

    float x = p1.x + u * px;
    float y = p1.y + u * py;

    float dx = x - point.x;
    float dy = y - point.y;

    return dx * dx + dy * dy;
}

float point_line_distance2(struct line line, struct position point, struct position return_segment[2])
{
    float min_dist = INFINITY;
    int min_i = -1;

    for (int i = 0; i < line.n_vertices - 1; ++i)
    {
        float new_dist = point_segment_distance2(line.vertices[i], line.vertices[i + 1], point);
        if (new_dist < min_dist)
        {
            min_dist = new_dist;
            min_i = i;
        }
    }

    // Only write if the caller provided a non-NULL pointer
    if (return_segment && min_i >= 0)
    {
        return_segment[0] = line.vertices[min_i];
        return_segment[1] = line.vertices[min_i + 1];
    }

    return min_dist;
}

bool point_after_segment(struct position p1, struct position p2, struct position point)
{
    float dx1 = p1.x - p2.x;
    float dy1 = p1.y - p2.y;
    float dx2 = point.x - p2.x;
    float dy2 = point.y - p2.y;

    return dx1 * dx2 + dy1 * dy2 < 0;
}

struct map_node *closest_map_node(struct position point)
{
    assert(MAP.n_nodes > 0);

    float min_dist = INFINITY;
    struct map_node *closest_node;

    for (int i = 1; i < MAP.n_nodes - 1; ++i)
    {
        float new_dist = point_line_distance2(MAP.nodes[i]->line, point, NULL);
        if (new_dist < min_dist)
        {
            min_dist = new_dist;
            closest_node = MAP.nodes[i];
        }
    }

    return closest_node;
}

int find_node_index(struct map_node *node)
{
    for (int i = 0; i < MAP.n_nodes; ++i)
    {
        if (MAP.nodes[i] == node)
            return i;
    }
    return -1;
}

float find_shortest_path(struct map_node *start, struct map_node *goal,
                         struct map_node_list *path_output)
{
    float dist[MAP.n_nodes];
    int prev[MAP.n_nodes];
    bool visited[MAP.n_nodes];

    for (int i = 0; i < MAP.n_nodes; ++i)
    {
        dist[i] = INFINITY;
        prev[i] = -1;
        visited[i] = false;
    }

    int start_i = find_node_index(start);
    int goal_i = find_node_index(goal);
    if (start_i < 0 || goal_i < 0)
        return INFINITY;
    dist[start_i] = 0.0f;

    while (true)
    {
        int min_dist_i = -1;
        for (int i = 0; i < MAP.n_nodes; ++i)
        {
            if (!visited[i] && dist[i] < dist[min_dist_i])
                min_dist_i = i;
        }

        if (min_dist_i == goal_i || min_dist_i == -1)
            break; // We have found the goal, or finished looking everywhere (souldn't happen ?)

        visited[min_dist_i] = true;

        // Le calcul de distance est pourri, ça prend pas en compre que le départ peut être au milieu d'un node, et l'arrivé au milieu d'un autre node. Mais plus de temps d'améliorer ça.
        for (int i = 0; i < MAP.nodes[min_dist_i]->next_nodes.n_nodes; ++i)
        {
            int child_i = find_node_index(MAP.nodes[min_dist_i]->next_nodes.nodes[i]);

            if (child_i < 0 || visited[child_i])
                continue;

            float alt = dist[min_dist_i] + MAP.nodes[min_dist_i]->weight;
            if (alt < dist[child_i])
            {
                dist[child_i] = alt;
                prev[child_i] = min_dist_i;
            }
        }
    }

    if (path_output)
    {
        int path[MAP.n_nodes];
        int len = 0;
        for (int u = goal_i; u != start_i; u = prev[u])
            path[len++] = u;

        path[len] = start_i;
        // Reverse order
        for (int i = 0; i < len; ++i)
            path_output->nodes[i] = MAP.nodes[path[len - i]];

        path_output->n_nodes = len;
    }

    return dist[goal_i];
}

// TODO: Hardcode the map here.

static void always_pass(struct map_node *map_node){};

// Geometry
static struct position line1_vertices[] = {{0, 0}, {1000, 0}};
static struct position line2_vertices[] = {{1000, 0}, {2000, 500}};

static struct line line1 = {2, line1_vertices};
static struct line line2 = {2, line2_vertices};

static struct position poly1_vertices[] = {{0, 0}, {200, -100}, {200, 100}};
static struct position poly2_vertices[] = {{1000, 0}, {1200, -100}, {1200, 100}};
static struct position poly3_vertices[] = {{2000, 500}, {2200, 400}, {2200, 600}};

static struct polygon poly1 = {3, poly1_vertices};
static struct polygon poly2 = {3, poly2_vertices};
static struct polygon poly3 = {3, poly3_vertices};

// Forward nodes and lists
static struct map_node nodeA;
static struct map_node nodeB;

static struct map_node *nodeA_next_nodes[] = {&nodeB};
static struct map_node *nodeB_prev_nodes[] = {&nodeA};

static struct map_node_list nodeA_next_list = {1, nodeA_next_nodes};
static struct map_node_list nodeB_prev_list = {1, nodeB_prev_nodes};
static struct map_node_list empty_list = {0, NULL};

// Global map node array
static struct map_node *map_nodes[] = {&nodeA, &nodeB};
struct map_node_list MAP = {2, map_nodes};

// --- Initialization function ---
__attribute__((constructor)) // optional: runs automatically at startup (GCC extension)
static void init_map(void)
{
    nodeA.line = line1;
    nodeA.weight = 5;
    nodeA.passing_fct = always_pass;
    nodeA.next_nodes = nodeA_next_list;

    nodeB.line = line2;
    nodeB.weight = 8;
    nodeB.passing_fct = always_pass;
    nodeB.next_nodes = empty_list;
}
