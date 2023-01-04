#!/usr/bin/env python3

import pygame
import rospy
import numpy as np
import os

# Get Path for savings
parent_dir = os.path.dirname(os.path.dirname(__file__))
filepath = os.path.join(parent_dir, 'maps')

#Create image
screen_size = tuple(map(int, (rospy.get_param('/navigation_route/screen_size')).split(','))) #(1150, 1150)
screen = pygame.display.set_mode(screen_size)
screen_color = tuple(map(int, (rospy.get_param('/navigation_route/screen_color')).split(','))) #(0,0,0)
screen.fill(screen_color) #Default
pygame.display.set_caption('Navigation Route - (c = Color, z = Clear)')

draw_on = False
last_pos = (0, 0)
 
# Radius of the Brush
radius = rospy.get_param('/navigation_route/radius')

def roundline(canvas, color, start, end, radius=1):
    Xaxis = end[0]-start[0]
    Yaxis = end[1]-start[1]
    dist = max(abs(Xaxis), abs(Yaxis))
    for i in range(dist):
        x = int(start[0]+float(i)/dist*Xaxis)
        y = int(start[1]+float(i)/dist*Yaxis)
        pygame.draw.circle(canvas, color, (x, y), radius)
 
try:
    color = (255, 255, 255)
    undo_screen = screen
    while True:
      e = pygame.event.wait()
        
      if e.type == pygame.QUIT:
          raise StopIteration  
      if e.type == pygame.MOUSEBUTTONDOWN:    
          # Draw a single circle wheneven mouse is clicked down.
          pygame.draw.circle(screen, color, e.pos, radius)
          draw_on = True
      # When mouse button released it will stop drawing   
      if e.type == pygame.MOUSEBUTTONUP:
          draw_on = False
      if e.type == pygame.KEYDOWN:
          if e.key == pygame.K_c:
            if color == (255, 255, 255):
              color = (0, 0, 0)
            else:
              color = (255, 255, 255)
          if e.key == pygame.K_z:
            screen.fill((0,0,0))
          if e.key == pygame.K_UP:
            radius +=5
          if e.key == pygame.K_DOWN:
            if radius >= 6:
              radius -=5
      # It will draw a continuous circle with the help of roundline function.   
      if e.type == pygame.MOUSEMOTION:
          if draw_on:
              pygame.draw.circle(screen, color, e.pos, radius)
              roundline(screen, color, e.pos, last_pos,  radius)
          last_pos = e.pos
      pygame.display.flip()
except StopIteration:
    pygame.image.save(screen, filepath+"/img_map_route.png")
    pass

exec(open(os.path.dirname(__file__)+"/read_image.py").read())