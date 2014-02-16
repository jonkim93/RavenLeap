#!/usr/bin/env python

import getch
import sys
import select
import pygame

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
	heardEnter()
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

