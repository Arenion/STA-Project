#include "map.h"

#include <assert.h>
#include <math.h>

#include "../global.h"
#include "../communicationarduino.h"
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

// ---------------------------------------------
// MAP
// ---------------------------------------------

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// ---- Points (shared coordinates) ----
#define P1 {6201, 3861}
#define P2 {5224, 1066}
#define P3 {5177, 974}
#define P4 {5088, 900}
#define P5 {4977, 855}
#define P6 {4805, 854}
#define P7 {4690, 972}
#define P8 {4554, 1030}
#define P9 {4562, 846}
#define P10 {2167, 826}
#define P11 {1960, 827}
#define P12 {1789, 890}
#define P13 {1589, 1030}
#define P14 {1503, 1182}
#define P15 {1479, 1371}
#define P16 {1644, 1373}
#define P17 {1682, 1226}
#define P18 {1774, 1122}
#define P19 {1931, 1048}
#define P20 {2105, 1045}
#define P21 {1630, 2588}
#define P22 {1456, 2588}
#define P23 {1729, 2905}
#define P24 {1520, 3020}
#define P25 {1064, 2544}
#define P26 {1060, 2790}
#define P27 {955, 2992}
#define P28 {862, 2520}
#define P29 {857, 2738}
#define P30 {959, 3115}
#define P31 {980, 3188}
#define P32 {6184, 3867}
#define P33 {1202, 3367}
#define P34 {1317, 3460}
#define P35 {1405, 3449}
#define P36 {1617, 3481}
#define P37 {1778, 3568}
#define P38 {1964, 3526}
#define P39 {2062, 3387}
#define P40 {2019, 3205}
#define P41 {1851, 3064}
#define P42 {1664, 3097}
#define P43 {1540, 3325}
#define P44 {2056, 3044}
#define P45 {2742, 2763}
#define P46 {4375, 2774}
#define P47 {2160, 3278}
#define P48 {2758, 2949}
#define P49 {4376, 2967}
#define P50 {4550, 3085}
#define P51 {4658, 3160}
#define P52 {4566, 3328}
#define P53 {4401, 3332}
#define P54 {4341, 3409}
#define P55 {4316, 3627}
#define P56 {4580, 3495}
#define P57 {4539, 3644}
#define P58 {4767, 3502}
#define P59 {4762, 3653}
#define P60 {4978, 3462}
#define P61 {4975, 3658}
#define P62 {4850, 3362}
#define P63 {4563, 2897}
#define P64 {4757, 2882}
#define P65 {4795, 2990}
#define P66 {5062, 2994}
#define P67 {5337, 3029}
#define P68 {5532, 3092}
#define P69 {5756, 2943}
#define P70 {5853, 2786}
#define P71 {5820, 2647}
#define P72 {5800, 2484}
#define P73 {5765, 2631}
#define P74 {5701, 2707}
#define P75 {5303, 2740}
#define P76 {5028, 2784}

// ---- Lines using predefined points ----
// Lignes
static struct position ligne1_vertices[] = {P10, P9};
static struct position ligne2_vertices[] = {P8, P20};
static struct position ligne3_vertices[] = {P49, P48, P47};
static struct position ligne4_vertices[] = {P44, P45, P46};
static struct position ligne5_vertices[] = {P46, P76};
static struct position ligne6_vertices[] = {P76, P75};
static struct position ligne7_vertices[] = {P66, P49};

// Ponts
static struct position pont1_vertices[] = {P24, P22, P15};
static struct position pont2_vertices[] = {P16, P21, P23};
static struct position pont3_vertices[] = {P72, P1, P2};
static struct position pont4_vertices[] = {P2, P1, P72};

// Parkings
static struct position park1_vertices[] = {P27, P29, P28};
static struct position park7_vertices[] = {P28, P29, P27};
static struct position park2_vertices[] = {P27, P26, P25};
static struct position park8_vertices[] = {P25, P26, P27};
static struct position park3_vertices[] = {P51, P62, P60, P61};
static struct position park9_vertices[] = {P61, P60, P62, P51};
static struct position park4_vertices[] = {P51, P58, P59};
static struct position park10_vertices[] = {P59, P58, P51};
static struct position park5_vertices[] = {P51, P56, P57};
static struct position park11_vertices[] = {P57, P56, P51};
static struct position park6_vertices[] = {P51, P52, P53, P54, P55};
static struct position park12_vertices[] = {P55, P54, P53, P52, P51};

// Rond-point
static struct position rp1_vertices[] = {P39, P38, P37, P36};
static struct position rp2_vertices[] = {P36, P43};
static struct position rp3_vertices[] = {P43, P42, P41};
static struct position rp4_vertices[] = {P41, P40, P39};
static struct position rp5_vertices[] = {P36, P35};
static struct position rp6_vertices[] = {P35, P43};
static struct position rp7_vertices[] = {P43, P24};
static struct position rp8_vertices[] = {P23, P41};
static struct position rp9_vertices[] = {P41, P44};
static struct position rp10_vertices[] = {P47, P39};

// Virages
static struct position virage1_vertices[] = {P15, P14, P13, P12, P11, P10};
static struct position virage2_vertices[] = {P20, P19, P18, P17, P16};
static struct position virage3_vertices[] = {P9, P6, P5, P4, P3, P2};
static struct position virage4_vertices[] = {P2, P3, P4, P5, P6, P7, P8};
static struct position virage5_vertices[] = {P35, P34, P33, P32, P31, P30, P27};
static struct position virage6_vertices[] = {P27, P30, P31, P32, P33, P34, P35};
static struct position virage7_vertices[] = {P75, P74, P73, P72};
static struct position virage8_vertices[] = {P72, P71, P70, P69, P68, P67, P66};
static struct position virage9_vertices[] = {P51, P50, P49};
static struct position virage10_vertices[] = {P66, P65, P51};
static struct position virage11_vertices[] = {P46, P63, P51};
static struct position virage12_vertices[] = {P51, P64, P76};

// TODO
void followtraj(struct map_node *map_node) {};
void lignedroite(struct map_node *map_node);

// ---- Nodes ----
// Pre-declaration
static struct map_node ligne1_node, ligne2_node, ligne3_node, ligne4_node, ligne5_node, ligne6_node, ligne7_node, pont1_node, pont2_node, pont3_node, pont4_node, park1_node, park7_node, park2_node, park8_node, park3_node, park9_node, park4_node, park10_node, park5_node, park11_node, park6_node, park12_node, rp1_node, rp2_node, rp3_node, rp4_node, rp5_node, rp6_node, rp7_node, rp8_node, rp9_node, rp10_node, virage1_node, virage2_node, virage3_node, virage4_node, virage5_node, virage6_node, virage7_node, virage8_node, virage9_node, virage10_node, virage11_node, virage12_node;

// Nextnodes
static struct map_node *ligne1_node_next[] = {&virage3_node};
static struct map_node *ligne2_node_next[] = {&virage2_node};
static struct map_node *ligne3_node_next[] = {&rp10_node};
static struct map_node *ligne4_node_next[] = {&virage11_node, &ligne5_node};
static struct map_node *ligne5_node_next[] = {&ligne6_node};
static struct map_node *ligne6_node_next[] = {&virage7_node};
static struct map_node *ligne7_node_next[] = {&ligne3_node};
static struct map_node *pont1_node_next[] = {&virage1_node};
static struct map_node *pont2_node_next[] = {&rp8_node};
static struct map_node *pont3_node_next[] = {&virage4_node};
static struct map_node *pont4_node_next[] = {&virage8_node};
static struct map_node *park1_node_next[] = {&park7_node};
static struct map_node *park7_node_next[] = {&virage6_node};
static struct map_node *park2_node_next[] = {&park8_node};
static struct map_node *park8_node_next[] = {&virage6_node};
static struct map_node *park3_node_next[] = {&park9_node};
static struct map_node *park9_node_next[] = {&virage9_node, &virage12_node};
static struct map_node *park4_node_next[] = {&park10_node};
static struct map_node *park10_node_next[] = {&virage9_node, &virage12_node};
static struct map_node *park5_node_next[] = {&park11_node};
static struct map_node *park11_node_next[] = {&virage9_node, &virage12_node};
static struct map_node *park6_node_next[] = {&park12_node};
static struct map_node *park12_node_next[] = {&virage9_node, &virage12_node};
static struct map_node *rp1_node_next[] = {&rp2_node, &rp5_node};
static struct map_node *rp2_node_next[] = {&rp3_node, &rp7_node};
static struct map_node *rp3_node_next[] = {&rp4_node, &rp9_node};
static struct map_node *rp4_node_next[] = {&rp1_node};
static struct map_node *rp5_node_next[] = {&virage5_node};
static struct map_node *rp6_node_next[] = {&rp3_node, &rp7_node};
static struct map_node *rp7_node_next[] = {&pont1_node};
static struct map_node *rp8_node_next[] = {&rp4_node, &rp9_node};
static struct map_node *rp9_node_next[] = {&ligne4_node};
static struct map_node *rp10_node_next[] = {&rp1_node};
static struct map_node *virage1_node_next[] = {&ligne1_node};
static struct map_node *virage2_node_next[] = {&pont2_node};
static struct map_node *virage3_node_next[] = {&pont4_node};
static struct map_node *virage4_node_next[] = {&ligne2_node};
static struct map_node *virage5_node_next[] = {&park1_node, &park2_node};
static struct map_node *virage6_node_next[] = {&rp6_node};
static struct map_node *virage7_node_next[] = {&pont3_node};
static struct map_node *virage8_node_next[] = {&ligne7_node, &virage10_node};
static struct map_node *virage9_node_next[] = {&ligne3_node};
static struct map_node *virage10_node_next[] = {&park3_node, &park4_node, &park5_node, &park6_node};
static struct map_node *virage11_node_next[] = {&park3_node, &park4_node, &park5_node, &park6_node};
static struct map_node *virage12_node_next[] = {&ligne6_node};

// Lignes
static struct map_node ligne1_node = {
    .line = {.n_vertices = ARRAY_SIZE(ligne1_vertices), .vertices = ligne1_vertices},
    .weight = 0.0f,
    .passing_fct = lignedroite,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(ligne1_node_next), .nodes = ligne1_node_next}};

static struct map_node ligne2_node = {
    .line = {.n_vertices = ARRAY_SIZE(ligne2_vertices), .vertices = ligne2_vertices},
    .weight = 0.0f,
    .passing_fct = lignedroite,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(ligne2_node_next), .nodes = ligne2_node_next}};

static struct map_node ligne3_node = {
    .line = {.n_vertices = ARRAY_SIZE(ligne3_vertices), .vertices = ligne3_vertices},
    .weight = 0.0f,
    .passing_fct = lignedroite,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(ligne3_node_next), .nodes = ligne3_node_next}};

static struct map_node ligne4_node = {
    .line = {.n_vertices = ARRAY_SIZE(ligne4_vertices), .vertices = ligne4_vertices},
    .weight = 0.0f,
    .passing_fct = lignedroite,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(ligne4_node_next), .nodes = ligne4_node_next}};

static struct map_node ligne5_node = {
    .line = {.n_vertices = ARRAY_SIZE(ligne5_vertices), .vertices = ligne5_vertices},
    .weight = 0.0f,
    .passing_fct = lignedroite,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(ligne5_node_next), .nodes = ligne5_node_next}};

static struct map_node ligne6_node = {
    .line = {.n_vertices = ARRAY_SIZE(ligne6_vertices), .vertices = ligne6_vertices},
    .weight = 0.0f,
    .passing_fct = lignedroite,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(ligne6_node_next), .nodes = ligne6_node_next}};

static struct map_node ligne7_node = {
    .line = {.n_vertices = ARRAY_SIZE(ligne7_vertices), .vertices = ligne7_vertices},
    .weight = 0.0f,
    .passing_fct = lignedroite,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(ligne7_node_next), .nodes = ligne7_node_next}};

static struct map_node pont1_node = {
    .line = {.n_vertices = ARRAY_SIZE(pont1_vertices), .vertices = pont1_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(pont1_node_next), .nodes = pont1_node_next}};

static struct map_node pont2_node = {
    .line = {.n_vertices = ARRAY_SIZE(pont2_vertices), .vertices = pont2_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(pont2_node_next), .nodes = pont2_node_next}};

static struct map_node pont3_node = {
    .line = {.n_vertices = ARRAY_SIZE(pont3_vertices), .vertices = pont3_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATPONT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(pont3_node_next), .nodes = pont3_node_next}};

static struct map_node pont4_node = {
    .line = {.n_vertices = ARRAY_SIZE(pont4_vertices), .vertices = pont4_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATPONT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(pont4_node_next), .nodes = pont4_node_next}};

static struct map_node park1_node = {
    .line = {.n_vertices = ARRAY_SIZE(park1_vertices), .vertices = park1_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park1_node_next), .nodes = park1_node_next}};

static struct map_node park7_node = {
    .line = {.n_vertices = ARRAY_SIZE(park7_vertices), .vertices = park7_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park7_node_next), .nodes = park7_node_next}};

static struct map_node park2_node = {
    .line = {.n_vertices = ARRAY_SIZE(park2_vertices), .vertices = park2_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park2_node_next), .nodes = park2_node_next}};

static struct map_node park8_node = {
    .line = {.n_vertices = ARRAY_SIZE(park8_vertices), .vertices = park8_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park8_node_next), .nodes = park8_node_next}};

static struct map_node park3_node = {
    .line = {.n_vertices = ARRAY_SIZE(park3_vertices), .vertices = park3_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park3_node_next), .nodes = park3_node_next}};

static struct map_node park9_node = {
    .line = {.n_vertices = ARRAY_SIZE(park9_vertices), .vertices = park9_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park9_node_next), .nodes = park9_node_next}};

static struct map_node park4_node = {
    .line = {.n_vertices = ARRAY_SIZE(park4_vertices), .vertices = park4_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park4_node_next), .nodes = park4_node_next}};

static struct map_node park10_node = {
    .line = {.n_vertices = ARRAY_SIZE(park10_vertices), .vertices = park10_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park10_node_next), .nodes = park10_node_next}};

static struct map_node park5_node = {
    .line = {.n_vertices = ARRAY_SIZE(park5_vertices), .vertices = park5_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park5_node_next), .nodes = park5_node_next}};

static struct map_node park11_node = {
    .line = {.n_vertices = ARRAY_SIZE(park11_vertices), .vertices = park11_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park11_node_next), .nodes = park11_node_next}};

static struct map_node park6_node = {
    .line = {.n_vertices = ARRAY_SIZE(park6_vertices), .vertices = park6_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park6_node_next), .nodes = park6_node_next}};

static struct map_node park12_node = {
    .line = {.n_vertices = ARRAY_SIZE(park12_vertices), .vertices = park12_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(park12_node_next), .nodes = park12_node_next}};

static struct map_node rp1_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp1_vertices), .vertices = rp1_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp1_node_next), .nodes = rp1_node_next}};

static struct map_node rp2_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp2_vertices), .vertices = rp2_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp2_node_next), .nodes = rp2_node_next}};

static struct map_node rp3_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp3_vertices), .vertices = rp3_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp3_node_next), .nodes = rp3_node_next}};

static struct map_node rp4_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp4_vertices), .vertices = rp4_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp4_node_next), .nodes = rp4_node_next}};

static struct map_node rp5_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp5_vertices), .vertices = rp5_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp5_node_next), .nodes = rp5_node_next}};

static struct map_node rp6_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp6_vertices), .vertices = rp6_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp6_node_next), .nodes = rp6_node_next}};

static struct map_node rp7_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp7_vertices), .vertices = rp7_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp7_node_next), .nodes = rp7_node_next}};

static struct map_node rp8_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp8_vertices), .vertices = rp8_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp8_node_next), .nodes = rp8_node_next}};

static struct map_node rp9_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp9_vertices), .vertices = rp9_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp9_node_next), .nodes = rp9_node_next}};

static struct map_node rp10_node = {
    .line = {.n_vertices = ARRAY_SIZE(rp10_vertices), .vertices = rp10_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(rp10_node_next), .nodes = rp10_node_next}};

static struct map_node virage1_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage1_vertices), .vertices = virage1_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage1_node_next), .nodes = virage1_node_next}};

static struct map_node virage2_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage2_vertices), .vertices = virage2_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage2_node_next), .nodes = virage2_node_next}};

static struct map_node virage3_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage3_vertices), .vertices = virage3_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage3_node_next), .nodes = virage3_node_next}};

static struct map_node virage4_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage4_vertices), .vertices = virage4_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage4_node_next), .nodes = virage4_node_next}};

static struct map_node virage5_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage5_vertices), .vertices = virage5_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage5_node_next), .nodes = virage5_node_next}};

static struct map_node virage6_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage6_vertices), .vertices = virage6_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATIONRONDPOINT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage6_node_next), .nodes = virage6_node_next}};

static struct map_node virage7_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage7_vertices), .vertices = virage7_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATPONT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage7_node_next), .nodes = virage7_node_next}};

static struct map_node virage8_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage8_vertices), .vertices = virage8_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = RESERVATPONT,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage8_node_next), .nodes = virage8_node_next}};

static struct map_node virage9_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage9_vertices), .vertices = virage9_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage9_node_next), .nodes = virage9_node_next}};

static struct map_node virage10_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage10_vertices), .vertices = virage10_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage10_node_next), .nodes = virage10_node_next}};

static struct map_node virage11_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage11_vertices), .vertices = virage11_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage11_node_next), .nodes = virage11_node_next}};

static struct map_node virage12_node = {
    .line = {.n_vertices = ARRAY_SIZE(virage12_vertices), .vertices = virage12_vertices},
    .weight = 0.0f,
    .passing_fct = followtraj,
    .reservation = PASDERESERVATION,
    .next_nodes = {.n_nodes = ARRAY_SIZE(virage12_node_next), .nodes = virage12_node_next}};

// ---- MAP ----
static struct map_node *all_nodes[] = {&ligne1_node, &ligne2_node, &ligne3_node, &ligne4_node, &ligne5_node, &ligne6_node, &ligne7_node, &pont1_node, &pont2_node, &pont3_node, &pont4_node, &park1_node, &park7_node, &park2_node, &park8_node, &park3_node, &park9_node, &park4_node, &park10_node, &park5_node, &park11_node, &park6_node, &park12_node, &rp1_node, &rp2_node, &rp3_node, &rp4_node, &rp5_node, &rp6_node, &rp7_node, &rp8_node, &rp9_node, &rp10_node, &virage1_node, &virage2_node, &virage3_node, &virage4_node, &virage5_node, &virage6_node, &virage7_node, &virage8_node, &virage9_node, &virage10_node, &virage11_node, &virage12_node};
struct map_node_list MAP = {
    .n_nodes = ARRAY_SIZE(all_nodes),
    .nodes = all_nodes};

// ---- Initialize all nodes weights ----
float calculate_segment_length(struct position p1, struct position p2)
{
    return sqrtf((float)((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y)));
}

float calculate_line_lenght(struct line line)
{
    float r = 0.0f;
    for (int i = 0; i < line.n_vertices - 1; ++i)
    {
        r += calculate_segment_length(line.vertices[i], line.vertices[i + 1]);
    }
    return r;
}

void map_init()
{
    for (int i = 0; i < MAP.n_nodes; ++i)
    {
        MAP.nodes[i]->weight = calculate_line_lenght(MAP.nodes[i]->line);
    }
}