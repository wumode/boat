#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <json.hpp>

#define PORT 6666
#define SIZE 2048

void doCommand(char *command, char *rs) {
  printf("执行命令:%s\n", command);
  FILE *fp = popen(command, "r");
  char buf[100];
  while (fgets(buf, 99, fp)) {
    strcat(rs, buf);
  }
  printf("%s", rs);
}

int create_socket() {
  int listen_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_socket == -1) {
    perror("socket");
    return -1;
  }
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  int ret = bind(listen_socket, (struct sockaddr *)&addr, sizeof(addr));
  if (ret == -1) {
    perror("bind");
    return -1;
  }
  ret = listen(listen_socket, 5);
  if (ret == -1) {
    perror("listen");
    return -1;
  }
  return listen_socket;
}

socklen_t wait_client(socklen_t listen_socket) {
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);
  printf("等待客户端连接\n");
  socklen_t client_socket =
      accept(listen_socket, (struct sockaddr *)&client_addr, &addr_len);
  if (client_socket == -1) {
    perror("accept");
    return -1;
  }
  printf("成功接收到一个客户端:%s\n", inet_ntoa(client_addr.sin_addr));
  return client_socket;
}

void handle_client(socklen_t listen_socket, socklen_t client_socket) {
  char buf[SIZE];
  char *message;
  // 发送提示信息
  //message = "please input the service which you want:\n1. formula\n2 .string";
  //write(client_socket, message, strlen(message));

  // 读取客户端返回的结果
  memset(buf, '\0', SIZE);
  while (1){
      std::string j_r_= R"({
  "empower": 0,
  "mode": 1,
  "stop": 0,
  "route_updated": 0,
  "route_gps_positions": [
    {
      "latitude": 0.0,
      "longitude": 0.0
    },
    {
      "latitude": 0.0,
      "longitude": 0.0
    }
  ]
}
)";
      //write(client_socket, j_r_.c_str(), strlen(j_r_.c_str()));
      memset(buf, '\0', SIZE);
      read(client_socket, buf, SIZE - 1);
      //printf("%s\n", buf);
  }
//  if (strcmp(buf, "1") == 0) {
//    message = "choose formula success, please input number of x,a,n";
//    write(client_socket, message, strlen(message));
//
//    // 读取数字并执行子进程
//    memset(buf, '\0', SIZE);
//    read(client_socket, buf, SIZE - 1);
//    char command[1000] = "./formula ";
//    strcat(command, buf);
//    char rs[1000];
//    doCommand(command, rs);
//    write(client_socket, rs, strlen(rs));
//  } else if (strcmp(buf, "2") == 0) {
//    message =
//        "choose string success, please input the string you want to split";
//    write(client_socket, message, strlen(message));
//    // 读取数字并执行子进程
//    memset(buf, '\0', SIZE);
//    read(client_socket, buf, SIZE - 1);
//    char command[1000] = "./string ";
//    strcat(command, buf);
//    char rs[1000];
//    doCommand(command, rs);
//    write(client_socket, rs, strlen(rs));
//  }
}

int main(void) {
  int listen_socket = create_socket();
  while (1) {
    int client_socket = wait_client(listen_socket);
    int pid = fork();
    if(pid==0){
      handle_client(listen_socket, client_socket);
      exit(2);
      printf("final\n");
    }
  }
}
