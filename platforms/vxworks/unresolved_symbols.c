
#include <time.h>
#include <px4_log.h>
#include <termios.h>

/** __attribute__((visibility("default"))) void free(void *ptr) */
/** { */
/**         qurt_free(ptr); */
/**         ptr = 0; */
/** } */
/**  */
/** __attribute__((visibility("default"))) void *malloc(size_t size) */
/** { */
/**         return qurt_malloc(size); */
/** } */
/**  */
/** __attribute__((visibility("default"))) void *calloc(size_t nmemb, size_t size) */
/** { */
/**         PX4_ERR("Undefined calloc called"); */
/**         return (void *) 0; */
/** } */
/**  */
/** __attribute__((visibility("default"))) void *realloc(void *ptr, size_t size) */
/** { */
/**         PX4_ERR("Undefined realloc called"); */
/**         return (void *) 0; */
/** } */
/**  */
/** __attribute__((visibility("default"))) int nanosleep(const struct timespec *req, struct timespec *rem) */
/** { */
/**         PX4_ERR("Undefined nanosleep called"); */
/**         return -1; */
/** } */
/**  */
/** __attribute__((visibility("default"))) int tcgetattr(int fd, struct termios *termiosp) */
/** { */
/**         PX4_ERR("Undefined tcgetattr called"); */
/**         return -1; */
/** } */

__attribute__((visibility("default"))) int cfsetspeed(struct termios *termiosp, speed_t speed)
{
	PX4_ERR("Undefined cfsetspeed called");
	return -1;
}
#ifdef _WRS_KERNEL
__attribute__((visibility("default"))) int dup2(int fd, int fd2)
{
	PX4_ERR("Undefined dup2 called");
	return -1;
}

__attribute__((visibility("default"))) int dup(int fd)
{
	PX4_ERR("Undefined dup called");
	return -1;
}
#endif

/** __attribute__((visibility("default"))) int tcsetattr(int fd, int options, const struct termios *termiosp) */
/** { */
/**         PX4_ERR("Undefined tcsetattr called"); */
/**         return -1; */
/** } */