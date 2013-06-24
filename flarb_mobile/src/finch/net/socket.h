#ifndef _FINCH_NET_SOCKET_H
#define _FINCH_NET_SOCKET_H

#include <string>
#include <vector>
#include <type_traits>

#include <ev++.h>

#include "../exception.h"

namespace finch
{
    namespace net
    {
        class server;

        template <typename T, typename = void>
        class socket_writer;

        class socket
        {
            public:
                class disconnected_exception: public finch::exception
                {
                    public:
                        disconnected_exception(const std::string& msg):
                            finch::exception(msg)
                        {
                        };

                        virtual ~disconnected_exception() throw() {};
                };

                struct io_statistics
                {
                    io_statistics(): bytes_read(0), bytes_written(0) {};
                    size_t bytes_read;
                    size_t bytes_written;
                };

                socket();
                socket(const socket& other);
                virtual ~socket();

                bool connect(const std::string& host, const int port)
                    throw(finch::exception);

                void close();
                inline void disconnect() { close(); };




                // Write integrals
                template <typename T>
                inline typename std::enable_if<
                           std::is_integral<T>::value
                        || std::is_floating_point<T>::value>::type
                    write(const T val);

                // Write raw arrays, char* has been specialized
                template <typename T>
                inline typename std::enable_if<
                           std::is_pointer<T>::value
                        && !std::is_same<char*, T>::value>::type
                    write(const T t, const size_t count);

                // Write custom types
                template <typename T>
                inline typename std::enable_if<
                           !std::is_integral<T>::value
                        && !std::is_floating_point<T>::value>::type
                    write(const T& val);

                inline void write(const std::string& t, const size_t size);




                // Read integrals
                template <typename T>
                inline typename std::enable_if<
                           std::is_integral<T>::value
                        || std::is_floating_point<T>::value>::type
                    read(T& val);

                // Read arrays, char* has been specialized
                template <typename T>
                inline typename std::enable_if<
                            std::is_pointer<T>::value>::type
                        read(T t, const size_t count);

                // Read custom types
                template <typename T>
                inline typename std::enable_if<
                           !std::is_integral<T>::value
                        && !std::is_floating_point<T>::value>::type
                    read(T& val);

                inline void read(std::string& t, const size_t size);






                template <typename T>
                inline socket& operator<<(const T t)
                {
                    write(t);
                    return *this;
                }

                template <typename T>
                inline socket& operator>>(T& t)
                {
                    read<T>(t);
                    return *this;
                }


                const io_statistics& statistics() const {return _statistics;};


                void write(const char* const data, const size_t sz);
                void read(char* const data, const size_t sz);

                const int fd() const {return _fd;};

                const size_t bytes_available() const;

            friend class finch::net::server;
            protected:
                void set_fd(const int fd);

                // throws finch::exception if not overloaded
                // and no suitable handler is found
                virtual void read_event();

                // If no handler is specified, we assume we have a
                // tcp socket, and we can read data
                virtual void data_available();

                virtual void disconnected();
                virtual void connected();

            private:
                void _callback(ev::io& watcher, int revents);

                io_statistics _statistics;

                int _fd;
                ev::io _io;
        };


        // Write integrals
        template <typename T>
        inline typename std::enable_if<
                   std::is_integral<T>::value || std::is_floating_point<T>::value>::type
            socket::write(const T v)
        {
            write(reinterpret_cast<const char*>(&v), sizeof(T));
        }

        inline void socket::write(const std::string& s, const size_t sz)
        {
            write(s.c_str(), std::min(s.length(), sz));

            if(s.length() < sz) {
                for(size_t i = s.length(); i < sz; i++) {
                    write("\0", 1);
                }
            }
        }

        // Write raw arrays
        template <typename T>
        inline typename std::enable_if<
                   std::is_pointer<T>::value
                && !std::is_same<char*, T>::value>::type
            socket::write(const T t, const size_t count)
        {
            write(reinterpret_cast<char*>(t), count*sizeof(*t));
        }





        // Read integrals
        template <typename T>
        inline typename std::enable_if<
                   std::is_integral<T>::value
                || std::is_floating_point<T>::value>::type
            socket::read(T& v)
        {
            read(reinterpret_cast<char*>(&v), sizeof(T));
        }

        // Read raw arrays
        template <typename T>
        inline typename std::enable_if<
                    std::is_pointer<T>::value>::type
            socket::read(T t, const size_t count)
        {
            read(reinterpret_cast<char*>(t), count*sizeof(*t));
        }

        inline void socket::read(std::string& s, const size_t sz)
        {
            std::vector<char> buf(sz);
            char* d = buf.data();
            read(d, sz);

            s = std::move(std::string(buf.begin(), buf.end()));
        }
    };
};

#endif /* finch/net/socket.h */

