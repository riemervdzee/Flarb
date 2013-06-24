#ifndef _FINCH_NET_EVENT_LOOP_H
#define _FINCH_NET_EVENT_LOOP_H

#include <ev++.h>

namespace finch
{
    namespace net
    {
        class event_loop
        {
            public:
                event_loop();
                event_loop(const event_loop& other);

                virtual ~event_loop();

                void run();
                void stop();

            private:
                ev::default_loop _loop;
        };
    };
};

#endif /* finch/net/event_loop.h */

