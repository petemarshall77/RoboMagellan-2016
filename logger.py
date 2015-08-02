from time import asctime

class Logger:

    def __init__(self):
        filename = "log/%s.%s" % (asctime(), 'log')
        self.file = open(filename, 'w')
        print "Created log file %s\n" % filename

    def write(self, message):
        datastring = "%s: %s\n" % (asctime(), message)
        print datastring
        self.file.write(datastring)

    def __del__(self):
        self.file.close()
        print "Closed log file"
