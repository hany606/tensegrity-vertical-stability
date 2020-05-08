#include "TCP.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

TCP::TCP(const char* hostname, long long int port_num){
    TCP::hostname = hostname;
    TCP::port_num = port_num;
}

TCP::~TCP(){
    close(TCP::sockfd);

}

void TCP::error(const char *s){
    printf("ERROR\n");
    perror(s);
    exit(0);
}

void TCP::setup(){
    TCP::serv_addr.sin_family = AF_INET;
    TCP::serv_addr.sin_port = htons(TCP::port_num);

    
    TCP::sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (TCP::sockfd < 0) 
        TCP::error("ERROR opening socket");
    TCP::server = gethostbyname(TCP::hostname);
    if (TCP::server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        TCP::error("ERROR connecting");


}


void TCP::write_TCP(void *buffer, int buff_size){
    int n = write(sockfd, buffer, strlen((char*)buffer));
    // printf("-----%ld\n", strlen((char*)buffer));
    if (n < 0) 
        TCP::error("ERROR writing to socket");
}

void TCP::write_TCP(void *buffer){
    TCP::write_TCP(buffer, strlen((char*)buffer));
}

void TCP::read_TCP(void *buffer){

}
void TCP::read_TCP(void *buffer, int buff_size){
    int n = read(sockfd, buffer, buff_size);
    if (n < 0) 
        error("ERROR reading from socket");
}