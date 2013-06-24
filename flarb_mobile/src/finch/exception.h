#ifndef _FINCH_EXCEPTION_H
#define _FINCH_EXCEPTION_H

#include <string>

#include <exception>

namespace finch
{
    class exception: public std::exception
    {
        public:
            exception(const std::string& msg): _msg(msg) {};
            exception(): _msg() {};
            virtual ~exception() throw() {};

            virtual const char* what() const throw()
            {
                return _msg.c_str();
            }

        private:
            std::string _msg;
    };

    class missing_callback_exception: public finch::exception
    {
        public:
            missing_callback_exception(const std::string& msg):
                finch::exception(msg)
            {
            }
            missing_callback_exception():
                finch::exception()
            {
            }

            virtual ~missing_callback_exception() throw() {}
    };

    class io_exception: public finch::exception
    {
        public:
            io_exception(const std::string& msg):
                finch::exception(msg)
            {
            }

            virtual ~io_exception() throw()
            {
            }
    };

};

#endif /* finch/exception.h */

