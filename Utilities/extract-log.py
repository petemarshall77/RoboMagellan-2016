#!/usr/bin/env python

# Extracts log data for import into spreadsheet

#---------------------------------------------------------------
# Format a line of GPS mode data
#---------------------------------------------------------------
def format_GPS_line(line):
    import re
    print 'GPS',

    # Timestamp
    matchObj = re.search(r'TIME: (\d*\.\d*)', line)
    print matchObj.group(1),

    # Latitude
    matchObj = re.search(r'LAT: (\d*\.\d*)', line)
    print matchObj.group(1),
    
    # Longitude
    matchObj = re.search(r'LAT: (\d*\.\d*)', line)
    print matchObj.group(1),

    # Bearing
    matchObj = re.search(r'BEARING: (\d*\.\d*)', line)
    print matchObj.group(1),

    # Compass
    matchObj = re.search(r'COMPASS: (\d*\.\d*)', line)
    print matchObj.group(1),

    # Delta Angle
    matchObj = re.search(r'DELTA: (-?\d*\.\d*)', line)
    print matchObj.group(1),

    # Distance
    matchObj = re.search(r'DIST: (\d*\.\d*)', line)
    print matchObj.group(1),

    # Speed
    matchObj = re.search(r'SPEED (\d*)', line)
    print matchObj.group(1),

    # Steer
    matchObj = re.search(r'STEER (-?\d*)', line)
    print matchObj.group(1),

    print
#---------------------------------------------------------------
# main()
#---------------------------------------------------------------
def main():
    import sys
    
    filename = sys.argv[1]
    print "Processing file %s" % filename
    
    with open(filename) as log_file:
        for line in log_file:
            if line.find('LAT:') > -1:
                format_GPS_line(line)
            elif line.find('Camera Mode') > -1:
                print 'CAM'
            else:
                print '???'

print __name__
if __name__ == "__main__":
    print "Hello"
    main()


