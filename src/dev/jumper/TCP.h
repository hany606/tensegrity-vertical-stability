#ifndef TCP_G
#define TCP_G

#include <netinet/in.h>
#include <netdb.h>


class TCP{
    public:
        TCP(const char* hostname, long long int port_num);

        void error(const char *s);
        void setup();
        void write_TCP(void *buffer);
        void write_TCP(void *buffer, int buff_size);
        void read_TCP(void *buffer);
        void read_TCP(void *buffer, int buff_size);
        ~TCP();
        private:
        struct sockaddr_in serv_addr;
        struct hostent *server;
        const char* hostname;
        long long int port_num; 
        int sockfd;

};

#endif