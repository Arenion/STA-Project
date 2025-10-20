#include <stdbool.h>

enum states_navigation {
    GENERE_ITINERAIRE,
};

bool step_navigation(bool entering)
{
    static enum states_navigation current_state = GENERE_ITINERAIRE;
    if (entering) current_state = GENERE_ITINERAIRE;

    switch (current_state)
    {
        case GENERE_ITINERAIRE:
        break;
    }
}