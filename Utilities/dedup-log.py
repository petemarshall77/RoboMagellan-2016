#!/usr/bin/env python

# De-duplicates the RoboMagellan log file

import sys

filename = sys.argv[1]
print "Processing file %s" % filename

last_line = ""
with open(filename) as log_file:
    for line in log_file:
      
        # If in GPS mode, remove duplicates where GPS hasn't changed.
        if line.find('LAT:') != -1 and last_line.find('LAT:') != -1:
            compare_from_line = line.find('LAT:')
            compare_to_line = line.find('BEARING:')
            compare_from_last = last_line.find('LAT:')
            compare_to_last = last_line.find('BEARING:')
        else:
            compare_from_line = 26 
            compare_from_last = 26
            compare_to_line = None
            compare_to_last = None # same as [26:]

        # If this line is the same as the last, ignore it, else
        # keep it
        if line[compare_from_line:compare_to_line] == last_line[compare_from_last:compare_to_last]:
            pass
        else:
            print line,
        
        last_line = line
        


