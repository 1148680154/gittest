#!/usr/bin/env python
from follow import Follow

if __name__ == '__main__':
    follow = Follow(
        [
            #x   ,   y,   z, yaw, sleep
            [-0.3 , -1.4, 0.7, 0, 3]
        ]
    )
    follow.run()
