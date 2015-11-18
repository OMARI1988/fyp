#!/usr/bin/env python

import copy

class Board():
	def __init__(self,x=3,y=3):
	
		self.spaces = []
		for i in range(0,x):
			row = []
			for j in range(0,y):
	
				space = Space() 

				row.append(space)
			self.spaces.append(row)

	def getCopy(self):
		
		return copy.deepcopy(self)

	def checkWinner(self,piece):
		bo = self.spaces
		return ((bo[0][0].getContent() == piece and bo[0][1].getContent() == piece and bo[0][2].getContent() == piece) or # across the top
		(bo[1][0].getContent() == piece and bo[1][1].getContent() == piece and bo[1][2].getContent() == piece) or # across the middle
		(bo[2][0].getContent() == piece and bo[2][1].getContent() == piece and bo[2][2].getContent() == piece) or # across the bottom
		(bo[0][0].getContent() == piece and bo[1][0].getContent() == piece and bo[2][0].getContent() == piece) or # down the left
		(bo[0][1].getContent() == piece and bo[1][1].getContent() == piece and bo[2][1].getContent() == piece) or # down the middle
		(bo[0][2].getContent() == piece and bo[1][2].getContent() == piece and bo[2][2].getContent() == piece) or # down the right
		(bo[0][0].getContent() == piece and bo[1][1].getContent() == piece and bo[2][2].getContent() == piece) or # diag 1 
		(bo[0][2].getContent() == piece and bo[1][1].getContent() == piece and bo[2][0].getContent() == piece))		# diag 2

	def isFull(self):
		# Return True if every space on the board has been taken. Otherwise return False.
		for row in self.spaces:
				for space in row:
					if  not space.getContent(): # if space is free 
						return False
		
		return True

	def setPosition(self, x, y, pose):
		
		self.spaces[x][y].setPose(pose)

	def getPosition(self, x, y):
		
		return self.spaces[x][y].getPose()

	def getSpaces(self):
		
		return self.spaces

	def setPiece(self, x, y, piece):
		
		self.spaces[x][y].setContent(piece)

	def getPiece(self, x, y):

		return self.spaces[x][y].getContent()

	def setPixel(self, x, y, pixel):
		
		self.spaces[x][y].setPix(pixel)

	def getPixel(self,x,y):
		
		return self.spaces[x][y].getPix()

	def printBoard(self):
		
		counter = 0
		print "  0 1 2"
		for row in self.spaces:
			print " -------"
			string = str(counter)
			for space in row:
				string += "|"
				if not space.getContent():
					string += " "
				else:
					string += space.getContent()
				#string += str((self.spaces.index(row),row.index(space)))
			print string + "|"
			counter += 1  
		print " -------"
	
  

class Space():
	def __init__(self):

		self.piece = 0
		self.pose = (0,0,0,0,0,0)
		self.pixel = (0,0)

	def setPose(self,pose):
		self.pose = pose

	def getPose(self):
		return self.pose
		
	def setContent(self,piece):
		self.piece = piece

	def getContent(self):
		return self.piece
	
	def setPix(self,pixel):
		self.pixel = pixel

	def getPix(self):
		return self.pixel


if __name__ == "__main__":
	print 'foo'
