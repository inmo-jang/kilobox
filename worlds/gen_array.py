#!/usr/bin/env python

import sys
import math
import random

# Generate a regular array of randomly posed kilobots.
# Uses nearest square number below first arg

num = int(sys.argv[1])

per_side = int(math.sqrt(num))
separation = 0.04

count = 0
for i in xrange(per_side):
    for j in xrange(per_side):
        x = (i - (per_side - 1) / 2.0) * separation
        y = (j - (per_side - 1) / 2.0) * separation
        a = random.uniform(-180,180)
        count += 1
        print 'r0( pose [ %f %f 0 %f ])' % (x, y, a)

print 'Total generated %d' % count

