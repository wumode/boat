#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define PORT 8888
#define SIZE 1000

int main(void) {
  int client_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (client_socket == -1) {
    perror("socket");
    return -1;
  }
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  inet_aton("localhost", &(addr.sin_addr));

  int addrlen = sizeof(addr);
  int listen_socket = connect(client_socket, (struct sockaddr *)&addr, addrlen);
  if (listen_socket == -1) {
    perror("connect");
    return -1;
  }
  printf("成功连接服务器\n");

  char buf[SIZE];

  memset(buf,'\0', SIZE);
  read(client_socket, buf, SIZE-1);
  printf("%s\n",buf);

  fgets(buf, SIZE, stdin);
  buf[strlen(buf)-1]='\0';
  write(client_socket, buf, strlen(buf));

  memset(buf,'\0', SIZE);
  read(client_socket, buf, SIZE-1);
  printf("%s\n",buf);

  fgets(buf, SIZE, stdin);
  buf[strlen(buf)-1]='\0';
  write(client_socket, buf, strlen(buf));

  memset(buf,'\0', SIZE);
  read(client_socket, buf, SIZE-1);
  printf("%s\n",buf);

  return 0;
}
