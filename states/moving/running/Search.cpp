#include <sys/states/moving/running/Search.hpp>

namespace sys {
    namespace states {
        Search::Search() {
        }

        sc::result Search::react(const events::ReachedObjective&) {
            return forward_event();
        }
    }
}
