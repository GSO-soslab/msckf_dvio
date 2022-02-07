#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


class interpolation:

  def __init__(self):
      self.dvl_accel = np.array([ [0.0,  0.0,     0.0,     0.0],
                                  [0.25, 0.3604, -0.3528,  0.046],
                                  [0.50, 0.4952,  0.166,   0.0344],
                                  [0.75, 0.1824,  0.0732, -0.0884],
                                  [1.00, 0.1212,  0.002,  -0.012] ])
      self.main()
  
  def main(self):

      fig = plt.figure()
      ax = fig.add_subplot(111)

      ## plot DVL acceleration in x-axis
      t = self.dvl_accel[:,0]
      a_x = self.dvl_accel[:,1]
      ax.plot(t, a_x, color='b', marker="o")

      ## quadratic interpolation ##
      # plot first section
      data = np.empty((0,2), float)
      x=0.0
      y=0.0
      for i in range(100):
        x = i*0.01

        if x < 0.5: # first section: point 1~3
          y = self.quadratic(x,self.dvl_accel[0,0], self.dvl_accel[0,1],
                               self.dvl_accel[1,0], self.dvl_accel[1,1],
                               self.dvl_accel[2,0], self.dvl_accel[2,1])
        else: # second section: point 3~5
          y = self.quadratic(x,self.dvl_accel[2,0], self.dvl_accel[2,1],
                               self.dvl_accel[3,0], self.dvl_accel[3,1],
                               self.dvl_accel[4,0], self.dvl_accel[4,1])      

        data = np.append(data,[[x,y]], axis=0)

      ax.plot(data[:,0],data[:,1], color='r', marker=".")

      plt.legend( [ 'Linear '   
                  , 'quadratic'     
                  ]
                 )

      plt.tight_layout()
      plt.show()

##### examples from: https://blog.csdn.net/pipisorry/article/details/62227459
  def quadratic(self, x, x1, y1, x2, y2, x3, y3):
    return ((x-x2)*(x-x3))/((x1-x2)*(x1-x3))*y1 + ((x-x1)*(x-x3))/((x2-x1)*(x2-x3))*y2 + ((x-x1)*(x-x2))/((x3-x1)*(x3-x2))*y3

  def linear(self, x, x1, y1, x2, y2):
    return (y2-y1)/(x2-x1) * (x-x1) + y1

if __name__ == '__main__':
    example = interpolation()
