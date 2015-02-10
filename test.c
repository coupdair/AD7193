#include<stdio.h>

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define MAJOR_VER 2
#define MINOR_VER 6
#define VERSION "/home/user/.myapp" STR(MAJOR_VER) STR(MINOR_VER)

int main()
{
  printf("version=%s\n",VERSION);
  return 0;
}
