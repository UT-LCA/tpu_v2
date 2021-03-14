from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class pipe():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width,depth,resetvalue):
        string = '''\
        '''
        return string.format(WIDTH=width, DEPTH=depth, RESETVALUE=resetvalue)

    def write (self, width,depth,resetvalue):
        self.fp.write(self.make_str(width,depth,resetvalue))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = pipe(fp)
    uut1.write(32,1,0)
    fp.close()
