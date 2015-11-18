#!/usr/bin/env python

import copy

class Board():
    def __init__(self,x=3,y=3,z=3):
        if x != y:
            print 'ERROR: Board X and Y must be equal'
            sys.exit()
        self.cube = []
        self.spaces = []

        for i in range(0,z):
            level = []
            for j in range(0,y):
                row = []
                for k in range(0,x):
                    space = Space() 
                    row.append(space)
                    self.spaces.append((i,j,k))
                level.append(row)
            self.cube.append(level)
        
        self.solutions = self.generateSolutions()
       
        

    def generateSolutions(self):
        
        def allEqual(x,y,z):
            return (x==y) and (y==z)

        def isSorted(x,y,z):
            return (x > y and y > z) or (z > y and y > x)
            
        solutions = []
        for a in self.spaces:
            for b in self.spaces:
                for c in self.spaces:     
                    if a != b and b != c and a != c:
                        line = [a,b,c]
                        line.sort()
                        if not line in solutions:
                            level_eq = allEqual(a[0],b[0],c[0])
                            row_eq = allEqual(a[1],b[1],c[1])
                            col_eq = allEqual(a[2],b[2],c[2])
                            level_sort = isSorted(a[0],b[0],c[0])
                            row_sort = isSorted(a[1],b[1],c[1])
                            col_sort = isSorted(a[2],b[2],c[2])
                            if ((level_eq and row_eq) or (level_eq and col_eq) or (col_eq and row_eq)) or \
                            (level_sort and row_sort and col_sort) or \
                            ((level_eq and row_sort and col_sort) or (level_sort and row_eq and col_sort) or (level_sort and row_sort and col_eq)):
                                solutions.append(line)  
        return solutions
        
    def getCopy(self):
        
        return copy.deepcopy(self)

    def checkWinner(self,letter):
        board = self.cube
        for solution in self.solutions:
            if board[solution[0][0]][solution[0][1]][solution[0][2]].getContent() == letter \
             and board[solution[1][0]][solution[1][1]][solution[1][2]].getContent() == letter \
             and board[solution[2][0]][solution[2][1]][solution[2][2]].getContent() == letter:
                    return True

    def makeMove(self, row, col, letter, circle = False):

        for level in range(len(self.cube)):
            if not self.getPiece(level,row,col):
                self.setPiece(level,row,col,letter,circle)
                return level
                #break
            
            
    def isFull(self):
        # Return True if every space on the top of the board has been taken. Otherwise return False.
        for row in self.cube[len(self.cube)-1]:
                for space in row:
                    if  not space.getContent(): # if space is free 
                        return False
    
        return True
    
    def setPosition(self, row, col, pose):
        
        self.cube[0][row][col].setPose(pose)

    def getPosition(self, row, col):
        
        return self.cube[0][row][col].getPose()

    def getSpaces(self):
        
        return self.cube    #used to be self.spaces

    def setPiece(self, lvl, row, col, piece, circle):
        
        self.cube[lvl][row][col].setContent(piece,circle)
        

    def getPiece(self, lvl, row, col):

        return self.cube[lvl][row][col].getContent()
    
    def getHighPiece(self, row, col):
        lvl = self.getHighLevel(row,col)
        if lvl != None:
            return self.getPiece(lvl,row,col)
       
            
    def getHighLevel(self, row, col):
        cube = self.getSpaces()
        if self.getPiece(0,row,col) and not self.getPiece(len(cube)-1,row,col): #make sure stack is not full or empty
            for level in range(len(cube)-2,-1,-1):
                piece = self.getPiece(level,row,col)
                if piece:
                    return level
        else:
            return None
            
    def setPixel(self, row, col, pixel):
        
        self.cube[0][row][col].setPix(pixel)

    def getPixel(self, row, col):
        
        return self.cube[0][row][col].getPix()

        
    def getCircle(self, row, col):
        
        lvl = self.getHighLevel(row,col)
        if lvl != None:
            return self.cube[lvl][row][col].getCirc()
    
    

    def printBoard(self):
        
        for level in range(len(self.cube)-1,-1,-1):
            counter = 0
            print "  0 1 2"
            for row in self.cube[level]:
                print " -------"
                string = str(counter)
                for space in row:
                    string += "|"
                    if not space.getContent():
                        string += " "
                    else:
                        string += space.getContent()
                print string + "|"
                counter += 1  
            print " -------"
  

class Space():
    def __init__(self):

        self.piece = 0
        self.pose = (0,0,0,0,0,0)
        self.pixel = (0,0)
        self.circle = False

    def setPose(self,pose):
        self.pose = pose

    def getPose(self):
        return self.pose
        
    def setContent(self,piece,circle):
        self.circle = circle
        self.piece = piece

    def getContent(self):
        return self.piece
    
    def setPix(self,pixel):
        self.pixel = pixel

    def getPix(self):
        return self.pixel
        
    def getCirc(self):
        return self.circle

if __name__ == "__main__":
    print 'foo'
