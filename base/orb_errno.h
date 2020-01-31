//
// Created by fs on 2020-01-20.
//

#ifndef UORB_ORB_ERRNO_H
#define UORB_ORB_ERRNO_H

#define ENOENT 2        /* No such file or directory */
#define EINTR 4         /* Interrupted system call */
#define EIO 5           /* I/O error */
#define EBADF 9         /* Bad file number */
#define EAGAIN 11       /* Try again */
#define ENOMEM 12       /* Out of memory */
#define EFAULT 14       /* Bad address */
#define EEXIST 17       /* File exists */
#define ENODEV 19       /* No such device */
#define EINVAL 22       /* Invalid argument */
#define ENFILE 23       /* File table overflow */
#define ENOSPC 28       /* No space left on device */
#define ENAMETOOLONG 36 /* File name too long */

/*
 * This error code is special: arch syscall entry code will return
 * -ENOSYS if users try to call a syscall that doesn't exist.  To keep
 * failures of syscalls that really do exist distinguishable from
 * failures due to attempts to use a nonexistent syscall, syscall
 * implementations should refrain from returning -ENOSYS.
 */
#define ENOSYS 38 /* Invalid system call number */

#define ETIMEDOUT 110 /* Connection timed out */

/* The error code set by various library functions.  */
extern int *__orb_errno_location();

#define orb_errno (*__orb_errno_location())

#endif  // UORB_ORB_ERRNO_H
