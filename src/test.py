#!/usr/bin/env python

#import getch
import sys
import select
#import pygame


import tty
import termios
def consoleBased():
	#this works but you need to have your cursor in the console
	def isData():
	        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

	old_settings = termios.tcgetattr(sys.stdin)
	try:
	        tty.setcbreak(sys.stdin.fileno())

	        i = 0
	        while 1:
	                #print i
	                i += 1

	                if isData():
	                        c = sys.stdin.read(1)
	                        print c 
	                        if c == '\x1b':         # x1b is ESC
	                                break
	                        if c == ' ':
	                        		print "space"

	finally:
	        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

import pygame 
from pygame.locals import *
def pygameBased():

	def display(str):

	        text = font.render(str, True, (255, 255, 255), (159, 182, 205))
	        textRect = text.get_rect()
	        textRect.centerx = screen.get_rect().centerx
	        textRect.centery = screen.get_rect().centery

	        screen.blit(text, textRect)
	        pygame.display.update()

	pygame.init()
	screen = pygame.display.set_mode( (640,480) )
	pygame.display.set_caption('Python numbers')
	screen.fill((159, 182, 205))

	#font = pygame.font.Font(None, 17)
	#font = pygame.font.SysFont()
	num = 0
	done = False
	while not done:

	        #display( str(num) )
	        num += 1

	        pygame.event.pump()
	        keys = pygame.key.get_pressed()
	        if keys[K_ESCAPE]:
	                done = True

#pygameBased()
consoleBased()

"""
def getKey():
	while True:
		z = getch.getch()
		if ord(z) == 32:
			doStuff()
		else:
			pass
		#z = "s"

def doStuff():
	print "stuff"

def something(line):
	print "THERE WAS A SPACE"

def something_else():
	print "no input"

def heardEnter():
    i,o,e = select.select([sys.stdin],[],[],0.0001)
    for s in i:
        if s == sys.stdin:
            i = sys.stdin.readline()
            print i
            return True
    return False

# If there's input ready, do something, else do something
# else. Note timeout is zero so select won't block at all.
while True:
	heardEnter()"""



"""
pygame.init()
pygame.event.pump()
while True:
	keys = pygame.key.get_pressed()
	print keys 
	raw_input()"""
"""
while True:
	events = pygame.event.get()
	for event in events:
		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_SPACE:
				print "SPACEEEE"
"""
"""
while True:
	while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
	  line = sys.stdin.readline()
	  if line:
	    something(line)
	    raw_input()
	  else: # an empty line means stdin has been closed
	    print('eof')
	    exit(0)
	else:
	  something_else()
"""

