#include <stdio.h>
#include <string.h>
int main(int argc, char *argv[])
{
  char *s = argv[1];
  size_t len = strlen(s);
  printf("{");
  for(size_t i=0;i<len;i+=2){
    if(i+1==len){
      printf("%c",s[i]);
    }else{
      if(i+1==len-1)
        printf("%c%c",s[i],s[i+1]);
      else
        printf("%c%c,",s[i],s[i+1]);
    }
  }
  printf("}\n");
  return 0;
}
