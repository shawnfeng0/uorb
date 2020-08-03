//
// Created by fs on 2020-01-20.
//

#ifndef UORB_ORB_ERRNO_H
#define UORB_ORB_ERRNO_H

#ifndef ENOENT
#define ENOENT 2        /* No such file or directory */
#endif

#ifndef EINTR
#define EINTR 4         /* Interrupted system call */
#endif

#ifndef EIO
#define EIO 5           /* I/O error */
#endif

#ifndef EBADF
#define EBADF 9         /* Bad file number */
#endif

#ifndef EAGAIN
#define EAGAIN 11       /* Try again */
#endif

#ifndef ENOMEM
#define ENOMEM 12       /* Out of memory */
#endif

#ifndef EFAULT
#define EFAULT 14       /* Bad address */
#endif

#ifndef EEXIST
#define EEXIST 17       /* File exists */
#endif

#ifndef ENODEV
#define ENODEV 19       /* No such device */
#endif

#ifndef EINVAL
#define EINVAL 22       /* Invalid argument */
#endif

#ifndef ENFILE
#define ENFILE 23       /* File table overflow */
#endif

#ifndef ENOSPC
#define ENOSPC 28       /* No space left on device */
#endif

#ifndef ENAMETOOLONG
#define ENAMETOOLONG 36 /* File name too long */
#endif


/*
 * This error code is special: arch syscall entry code will return
 * -ENOSYS if users try to call a syscall that doesn't exist.  To keep
 * failures of syscalls that really do exist distinguishable from
 * failures due to attempts to use a nonexistent syscall, syscall
 * implementations should refrain from returning -ENOSYS.
 */
#ifndef ENOSYS
#define ENOSYS 38 /* Invalid system call number */
#endif


#ifndef ETIMEDOUT
#define ETIMEDOUT 110 /* Connection timed out */
#endif


/* The error code set by various library functions.  */
extern int *orb_errno_location();

#define orb_errno (*orb_errno_location())

#endif  // UORB_ORB_ERRNO_H
