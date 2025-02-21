#include <stdio.h>

int main(int argc, char * argv[]) {
#ifdef _WRS_KERNEL
	printf("\nHello world from kernel space!\n");
	printf("argc=%d\n", argc);
	printf("argv[%d]=%s\n", argc, argv==NULL ? "NULL" : (char*)argv);
#else
	int i;
	printf("\nHello world from user space!\n");
	for(i=0; i<argc; i++) {
		printf("argv[%d]=%s\n", i, argv[i]);
	}
#endif
	return 0;
}