#!/usr/bin/python
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import sys
import re
time = 0   # first column in file
length = 1 # second column in file
smooth = 2 # third column in file

def makefilename(name):
  return name.strip().replace(' ','_').replace('(','').replace(')','')

class VisualizeResults:
    """"
        Visualization results of benchmarking
        input: file names
        output: store/show plot
    """
#-----------------------------------------------------------------------------------------------------------------------
    # @description: Default constructor of Plot Class
    def __init__(self):
        self.clms = []
        self.column_names=[]
        self.timeClm = []
#-----------------------------------------------------------------------------------------------------------------------
    # @description: Read data from file and separate into column wise and plot column data
    def Run(self):
        self.readDataFromFile(sys.argv[1])
        self.plot_line_graph()
        #print self.timeClm
        #+print self.clms[0]

    def plot_line_graph(self):

        plt.figure(0)
        plt.rc('axes', axisbelow=True)  # draw grid line behind graph
        plt.grid(b=True, which='major', color='darkgray', linestyle='-', alpha=0.3)

        # set scale
        plt.xscale('linear')
        plt.yscale('linear')

        x = np.linspace(0, 1, 5)
        y = self.clms[0]#
        print y[1]
        plt.xlabel(str('time'))

        #for i in range(0, len(self.clms)):
        plt.plot(x, y)
        plt.legend(self.column_names[1])

#-----------------------------------------------------------------------------------------------------------------------

    # @description: read data from file line by line and store in matrix format.
    #               self.mat[0] consist of first file data
    #               self.mat[1] consist of second file data, so on.
    # Here store in np.matrix for easy access columns and rows.
    def readDataFromFile(self,file_name):
        path = str(file_name)
        filedata = pd.read_csv(path)

        self.column_names = [key for key in filedata.keys()]
        self.timeClm.append(filedata['T'])

        for i in range(1,8):
            self.clms.append(filedata['Joint_'+ str(i)])

        #print self.column_names
        #print self.timeClm
        #print self.clms
#####################################################################################################################################

if __name__ == '__main__':
    print ('\033[94m' + "Start ploting script..." + '\033[0m')
    SCRIPT = VisualizeResults()
    SCRIPT.Run()
    print ("End ploting script...")
    print ( '\033[1m' +'\033[92m' + "######### "+ "output plots stored path: plots/XYZ.eps" + " ########### " + '\033[0m')