#!/usr/bin/env python

import getch

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

getKey()


