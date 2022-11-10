# -*- coding: utf-8 -*-
"""
@author: Tamara
"""
import numpy as np
import matplotlib.pyplot as plt


#Constants
v = 343 #Speed of sound in m/s

#Define time array and sample frequency
t_end = 0.01
n_steps = 1000
fs = n_steps/t_end     #100000 Hz, as in our setup
Ts = 1/fs
t = np.linspace(0, t_end, n_steps)


#-------------PART 1: SIMULATION OF THE SIGNALS----------------------

#Determine the positions 
d = 0.235  #distance between microphones (m)
r = 0.8 #distance between center of microphones and source (m)
theta_specified = 160  #angle 
thetarad = theta_specified*(np.pi/180)

xmic2 = d/2
ymic2 = 0
xmic1 = -d/2
ymic1 = 0



if theta_specified > 90:
      xsource = -r*np.sin(thetarad-0.5*np.pi)
      ysource = r*np.cos(thetarad-0.5*np.pi)
else:
    xsource = r*np.cos(thetarad)
    ysource = r*np.sin(thetarad)
    

#Plot positions
fig = plt.figure()
ax = fig.add_subplot(111)
plt.plot(xmic1, ymic1, 'ko', label = "microphone 1")
plt.plot(xmic2, ymic2, 'bo', label = "microphone 2")
theta1 = np.linspace(0, 0.5*np.pi, 1000)
plt.plot(r*np.cos(theta1),r*np.sin(theta1), 'g--')
theta2 = np.linspace(0.5*np.pi, np.pi, 1000)
plt.plot(-r*np.sin(theta2-0.5*np.pi), r*np.cos(theta2-0.5*np.pi), 'g--')
plt.plot(xsource, ysource, 'yo', label = 'source')
plt.plot(0, 0, 'ro', markersize = 2, label = 'center')
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
plt.title(f'Positions of microphones and source, d = {d} m, r = {r} m, angle = {theta_specified} degrees')
plt.grid()
plt.legend()
ax.set_aspect('equal', adjustable='box')



#Source sound 
A = 1  #Amplitude of source sound
f = 698 #Frequency of source sound (hz)
T  = 1/f
w = 2*np.pi*f #Frequency in rad/s

sourcesig = A*np.sin(w*t)

#Determine time delay
s1 = np.sqrt((xsource-xmic1)**2+(ysource-ymic1)**2)
s2 = np.sqrt((xsource-xmic2)**2+(ysource-ymic2)**2)

t1 = s1/v
t2 = s2/v

td = t1-t2
td_abs = np.abs(t1-t2)  #time delay

if t1-t2 <0:
    far = 2     #mic2 is further from source
if t1-t2>0:
    far = 1     #mic1 is further from source     
if t1-t2 == 0:
    far = 0     #both are equally far 


#Amplitude drop (using inverse square law)
A1 = A/(4*np.pi*s1**2)
A2 = A/(4*np.pi*s2**2)


#Now simulate the sound picked up by the two microphones:

if far == 2:    
    sig1 = A1*np.sin(w*t)
    sig2 = A2*np.sin(w*(t-td_abs))

if far ==1:
    sig1 = A1*np.sin(w*(t-td_abs))
    sig2 = A2*np.sin(w*t)
    
if far == 0:
    sig1 = A1*np.sin(w*t)
    sig2 = A2*np.sin(w*t)
    
    
#plot the signals 
plt.figure()
plt.plot(t, sourcesig, 'y-', label = "Source signal")
plt.plot(t, sig1, 'k-', label = "Signal mic 1")
plt.plot(t, sig2, 'b-', label = "Signal mic 2")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude (a.u.)")
plt.title(f'Simulation of the signals, d = {d} m, r = {r} m, angle = {theta_specified} degrees, Source signal: f = {f} Hz, A = {A} (a.u.)')
plt.legend()
plt.grid()



# ------------PART 2: ANALYSIS OF THE SIGNALS-------------------------

#Calculate time delay using cross correlation
cor = np.correlate(sig1-np.mean(sig1), sig2-np.mean(sig2), "same")

delay_arr = np.linspace(-0.5*n_steps/fs, 0.5*n_steps/fs, n_steps)

#Plot correlation
plt.figure()
plt.plot(delay_arr*1000000, cor)
plt.grid()
plt.xlabel("Time delay (microseconds)")
plt.ylabel('Correlation')

#Determine time delay from maximum in correlation function
tdelay = delay_arr[np.argmax(cor)]


#Print time delay results
print(f"True time delay is {td*1000000} us")
print('')
print('Calculated:')
if tdelay <0:
    print(f'Signal from mic 1 leads by {np.abs(tdelay)*1000000} us')
else:
    print((f'Signal from mic 2 leads by {np.abs(tdelay)*1000000} us'))
    
    
#Estimate direction
x =((tdelay*v)/d)
theta_estim = np.arccos(x)*(180/np.pi)

#Print direction estimation results
print(f"True angle was {theta_specified}")
print(f"Estimated angle is {theta_estim}")







